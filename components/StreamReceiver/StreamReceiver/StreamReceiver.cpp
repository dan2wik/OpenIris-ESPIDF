#include "StreamReceiver.hpp"

#ifdef CONFIG_OISTREAM_RX_MODE

#include <cstring>
#include <new>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "fec.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "stream_protocol.h"

static const char* TAG = "RX_STREAM";

// ---------------------------------------------------------------------------
// The sniffer fills one of two PSRAM slabs. When every RS block of a frame
// has >= k chunks, the slab is handed to the decode task by pointer (no
// copy) and filling continues in the other slab. Slabs are sized for the
// protocol maxima, so TX-side geometry changes never need a resize. A slab
// is fully reset on every new frame_id — chunks of different frames are
// never mixed.
// ---------------------------------------------------------------------------

#define RX_FRAME_STALE_MS 200

typedef struct
{
    volatile bool busy;  // owned by the decode task
    bool active;         // currently accumulating a frame
    uint8_t frame_id;
    uint8_t k, n, num_blocks;
    uint16_t payload_len;
    uint32_t frame_len;
    uint8_t blocks_ready;  // blocks with >= k chunks
    uint8_t chunk_present[SP_BLOCKS_MAX][SP_N_MAX];
    uint8_t block_have[SP_BLOCKS_MAX];
    int32_t rssi_sum;
    uint32_t chunks_rx;
    uint32_t t_first_ms;
    uint8_t* buf;  // SP_BLOCKS_MAX * SP_N_MAX * SP_PAYLOAD_MAX (PSRAM)
} rx_slab_t;

static rx_slab_t s_slab[2];
static int s_fill = 0;                // slab being filled, -1 = none free
static int s_last_completed_id = -1;  // suppresses late chunks of a delivered frame

static uint8_t* s_decoded = nullptr;  // SP_BLOCKS_MAX * SP_K_MAX * SP_PAYLOAD_MAX (PSRAM)
static QueueHandle_t s_decode_q = nullptr;

static JpegFrameCallback s_jpeg_callback;

static bool s_espnow_ready = false;
static const uint8_t s_bcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Feedback is rebuilt at 1 Hz but rebroadcast ~5x/s for control-plane
// robustness under loss.
static sp_feedback_t s_fb_cached;
static bool s_fb_valid = false;

// Receive statistics, printed at 1 Hz by the decode task
typedef struct
{
    uint32_t frames_decoded;
    uint32_t frames_incomplete;  // abandoned: next frame started or stale
    uint32_t chunks_stored;      // on completed frames
    uint32_t chunks_expected;    // on completed frames
    uint32_t chunks_late;        // chunks of an already-delivered frame
    uint32_t chunks_nofill;      // dropped because both slabs were busy
    uint32_t blocks_repaired;    // blocks that needed parity reconstruction
    uint32_t bad_jpeg;
    int64_t rssi_sum;
    uint32_t rssi_cnt;
} rx_stats_t;
static rx_stats_t s_st;

static fec_t* s_fec = nullptr;
static uint8_t s_fec_k = 0, s_fec_n = 0;

static bool ensure_rx_fec(uint8_t k, uint8_t n)
{
    if (s_fec && s_fec_k == k && s_fec_n == n)
    {
        return true;
    }
    if (s_fec)
    {
        fec_free(s_fec);
        s_fec = nullptr;
    }
    init_fec();
    s_fec = fec_new(k, n);
    s_fec_k = k;
    s_fec_n = n;
    return s_fec != nullptr;
}

// ---------------------------------------------------------------------------
// Decode task: receives a completed slab by index, reconstructs missing data
// chunks per RS block, trims to the frame length from the header and hands
// the JPEG to the callback. Also prints statistics and broadcasts feedback.
// ---------------------------------------------------------------------------
static void rx_decode_task(void* arg)
{
    (void)arg;
    int64_t last_log = esp_timer_get_time();
    int64_t last_fb_send = 0;

    while (true)
    {
        int idx;
        bool got = xQueueReceive(s_decode_q, &idx, pdMS_TO_TICKS(250)) == pdTRUE;

        // Rebroadcast the cached feedback ~5x/s
        if (s_espnow_ready && s_fb_valid)
        {
            int64_t t = esp_timer_get_time();
            if (t - last_fb_send >= 190000)
            {
                last_fb_send = t;
                esp_now_send(s_bcast_mac, (const uint8_t*)&s_fb_cached, sizeof(s_fb_cached));
            }
        }

        if (got)
        {
            rx_slab_t* s = &s_slab[idx];
            const uint8_t k = s->k, n = s->n;
            const uint16_t payload = s->payload_len;
            bool ok = ensure_rx_fec(k, n);

            for (uint8_t b = 0; ok && b < s->num_blocks; b++)
            {
                uint8_t* out_base = s_decoded + (size_t)b * k * payload;
                const uint8_t* blk = s->buf + (size_t)b * n * payload;

                bool all_data = true;
                for (uint8_t c = 0; c < k; c++)
                {
                    if (!s->chunk_present[b][c])
                    {
                        all_data = false;
                        break;
                    }
                }

                if (all_data)
                {
                    // data chunks 0..k-1 are a contiguous prefix of the block
                    memcpy(out_base, blk, (size_t)k * payload);
                    continue;
                }

                s_st.blocks_repaired++;

                // Primary chunks at their natural slots, parity filling holes
                const gf* in[SP_K_MAX];
                unsigned index[SP_K_MAX];
                gf* outp[SP_K_MAX];
                uint8_t outix = 0;
                uint8_t next_par = k;

                for (uint8_t c = 0; c < k; c++)
                {
                    if (s->chunk_present[b][c])
                    {
                        in[c] = blk + (size_t)c * payload;
                        index[c] = c;
                        memcpy(out_base + (size_t)c * payload, in[c], payload);
                    }
                    else
                    {
                        while (next_par < n && !s->chunk_present[b][next_par]) next_par++;
                        // guaranteed by blocks_ready: >= k chunks present
                        in[c] = blk + (size_t)next_par * payload;
                        index[c] = next_par;
                        outp[outix++] = out_base + (size_t)c * payload;
                        next_par++;
                    }
                }
                fec_decode(s_fec, in, outp, index, payload);
            }

            if (ok)
            {
                uint32_t cap = (uint32_t)s->num_blocks * k * payload;
                uint32_t flen = s->frame_len;
                if (flen == 0 || flen > cap) flen = cap;

                if (s_decoded[0] != 0xFF || s_decoded[1] != 0xD8)
                {
                    s_st.bad_jpeg++;
                }
                s_st.frames_decoded++;

                if (s_jpeg_callback)
                {
                    s_jpeg_callback(s_decoded, flen);
                }
            }
            s->busy = false;  // release slab back to the sniffer
        }

        int64_t now = esp_timer_get_time();
        if (now - last_log >= 1000000)
        {
            // Frames complete before their tail chunks arrive (interleaved
            // order), so stragglers counted in chunks_late are received too.
            uint32_t loss_pm = 0;  // permille
            uint32_t received = s_st.chunks_stored + s_st.chunks_late;
            if (s_st.chunks_expected)
            {
                if (received > s_st.chunks_expected) received = s_st.chunks_expected;
                loss_pm = (uint32_t)(1000ULL * (s_st.chunks_expected - received) / s_st.chunks_expected);
            }
            int rssi = s_st.rssi_cnt ? (int)(s_st.rssi_sum / (int64_t)s_st.rssi_cnt) : 0;

            // Refresh the cached feedback snapshot for the rebroadcaster
            s_fb_cached.magic[0] = 'O';
            s_fb_cached.magic[1] = 'I';
            s_fb_cached.version = SP_FEEDBACK_VERSION;
            s_fb_cached.rssi_avg = (int8_t)rssi;
            s_fb_cached.chunks_expected = (uint16_t)(s_st.chunks_expected > 0xFFFF ? 0xFFFF : s_st.chunks_expected);
            s_fb_cached.chunks_received = (uint16_t)(received > 0xFFFF ? 0xFFFF : received);
            s_fb_cached.blocks_repaired = (uint16_t)(s_st.blocks_repaired > 0xFFFF ? 0xFFFF : s_st.blocks_repaired);
            s_fb_cached.frames_decoded = (uint8_t)(s_st.frames_decoded > 0xFF ? 0xFF : s_st.frames_decoded);
            s_fb_cached.frames_incomplete = (uint8_t)(s_st.frames_incomplete > 0xFF ? 0xFF : s_st.frames_incomplete);
            s_fb_valid = true;

            ESP_LOGI(TAG, "rx: %lu fps, loss %lu.%lu%%, repaired %lu blk, incomplete %lu, late %lu, nofill %lu, badjpeg %lu, rssi %d",
                     (unsigned long)s_st.frames_decoded, (unsigned long)(loss_pm / 10), (unsigned long)(loss_pm % 10),
                     (unsigned long)s_st.blocks_repaired, (unsigned long)s_st.frames_incomplete, (unsigned long)s_st.chunks_late,
                     (unsigned long)s_st.chunks_nofill, (unsigned long)s_st.bad_jpeg, rssi);
            memset(&s_st, 0, sizeof(s_st));
            last_log = now;
        }
    }
}

// ---------------------------------------------------------------------------
// Promiscuous sniffer: validates v2 chunks, accumulates them into the fill
// slab and hands completed frames to the decode task.
// ---------------------------------------------------------------------------
static void slab_begin_frame(rx_slab_t* s, const sp_parsed_t* p, uint32_t now_ms)
{
    s->active = true;
    s->frame_id = p->hdr->frame_id;
    s->k = p->k;
    s->n = p->n;
    s->num_blocks = p->hdr->num_blocks;
    s->payload_len = p->chunk_len;
    s->frame_len = p->frame_len;
    s->blocks_ready = 0;
    s->rssi_sum = 0;
    s->chunks_rx = 0;
    s->t_first_ms = now_ms;
    memset(s->chunk_present, 0, sizeof(s->chunk_present));
    memset(s->block_have, 0, sizeof(s->block_have));
}

static void sniffer_cb(void* buf, wifi_promiscuous_pkt_type_t t)
{
    (void)t;
    const wifi_promiscuous_pkt_t* ppkt = (wifi_promiscuous_pkt_t*)buf;
    if (ppkt->rx_ctrl.rx_state != 0) return;  // reception error

    sp_parsed_t p;
    if (!sp_parse_frame(ppkt->payload, ppkt->rx_ctrl.sig_len, &p)) return;

    // (Re)claim a fill slab if the last completion left us without one
    if (s_fill < 0)
    {
        if (!s_slab[0].busy)
            s_fill = 0;
        else if (!s_slab[1].busy)
            s_fill = 1;
        else
        {
            s_st.chunks_nofill++;
            return;
        }
        s_slab[s_fill].active = false;
    }
    rx_slab_t* s = &s_slab[s_fill];

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    if (s->active)
    {
        bool same = (s->frame_id == p.hdr->frame_id) && (s->k == p.k) && (s->n == p.n) && (s->num_blocks == p.hdr->num_blocks) &&
                    (s->payload_len == p.chunk_len) && (s->frame_len == p.frame_len);
        if (!same || (now_ms - s->t_first_ms > RX_FRAME_STALE_MS))
        {
            // A new frame began or accumulation went stale. The old frame is
            // abandoned whole — its chunks are never attached to a new frame.
            if (s->chunks_rx) s_st.frames_incomplete++;
            s->active = false;
        }
    }

    if (!s->active)
    {
        if ((int)p.hdr->frame_id == s_last_completed_id)
        {
            s_st.chunks_late++;  // straggler of a frame already delivered
            return;
        }
        slab_begin_frame(s, &p, now_ms);
    }

    const uint8_t b = p.hdr->rs_block_id;
    const uint8_t c = p.hdr->chunk_id;
    if (s->chunk_present[b][c]) return;  // duplicate
    s->chunk_present[b][c] = 1;
    s->chunks_rx++;
    s->rssi_sum += ppkt->rx_ctrl.rssi;

    if (s->block_have[b] < s->k)
    {  // block still needs chunks
        memcpy(s->buf + ((size_t)b * s->n + c) * s->payload_len, p.payload, s->payload_len);
        if (++s->block_have[b] == s->k)
        {
            s->blocks_ready++;
        }
    }

    if (s->blocks_ready >= s->num_blocks)
    {
        // Frame complete — hand this slab off, continue in the other one
        s->busy = true;
        s->active = false;
        s_last_completed_id = s->frame_id;
        s_st.chunks_stored += s->chunks_rx;
        s_st.chunks_expected += (uint32_t)s->num_blocks * s->n;
        s_st.rssi_sum += s->rssi_sum;
        s_st.rssi_cnt += s->chunks_rx;

        int done = s_fill;
        int other = 1 - s_fill;
        s_fill = s_slab[other].busy ? -1 : other;
        if (s_fill >= 0)
        {
            s_slab[s_fill].active = false;
        }
        xQueueSend(s_decode_q, &done, 0);
    }
}

esp_err_t stream_receiver_begin(JpegFrameCallback callback)
{
    s_jpeg_callback = callback;

    esp_netif_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK)
    {
        return err;
    }
    // STA (unconnected) instead of NULL so ESP-NOW can transmit feedback;
    // promiscuous reception works the same in STA mode. Power save must stay
    // off — the sniffer needs the receiver at 100%.
    esp_wifi_set_mode(WIFI_MODE_STA);
    err = esp_wifi_start();
    if (err != ESP_OK)
    {
        return err;
    }
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(CONFIG_OISTREAM_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Receive slabs and decode output live in PSRAM, sized for the protocol
    // maxima so runtime FEC geometry changes never need a resize.
    const size_t slab_sz = (size_t)SP_BLOCKS_MAX * SP_N_MAX * SP_PAYLOAD_MAX;
    for (int i = 0; i < 2; i++)
    {
        s_slab[i].buf = (uint8_t*)heap_caps_malloc(slab_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    s_decoded = (uint8_t*)heap_caps_malloc((size_t)SP_BLOCKS_MAX * SP_K_MAX * SP_PAYLOAD_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_slab[0].buf || !s_slab[1].buf || !s_decoded)
    {
        ESP_LOGE(TAG, "Failed to allocate RX stream buffers in PSRAM");
        return ESP_ERR_NO_MEM;
    }

    ensure_rx_fec(SP_K_DEFAULT, SP_N_DEFAULT);

    // ESP-NOW feedback channel back to the camera (broadcast)
    if (esp_now_init() == ESP_OK)
    {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, s_bcast_mac, 6);
        peer.channel = 0;  // current channel
        peer.ifidx = WIFI_IF_STA;
        peer.encrypt = false;
        if (esp_now_add_peer(&peer) == ESP_OK)
        {
            s_espnow_ready = true;
        }
    }
    if (!s_espnow_ready)
    {
        ESP_LOGW(TAG, "ESP-NOW init failed - running without feedback (camera uses defaults)");
    }

    s_decode_q = xQueueCreate(2, sizeof(int));
    xTaskCreatePinnedToCore(rx_decode_task, "rx_decode", 4096, nullptr, 5, nullptr, 1);

    wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_DATA};
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(sniffer_cb));
    esp_wifi_set_promiscuous(true);

    ESP_LOGI(TAG, "802.11 RX started on channel %d", CONFIG_OISTREAM_WIFI_CHANNEL);
    return ESP_OK;
}

#endif  // CONFIG_OISTREAM_RX_MODE
