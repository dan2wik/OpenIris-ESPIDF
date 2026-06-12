#include "TXStream.hpp"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

TXStream::TXStream(const int WIFI_CHANNEL)
    : _wifi_channel(WIFI_CHANNEL)
{
}

void TXStream::startStream()
{
    while (true)
    {
        fb = esp_camera_fb_get();

        if (!fb)
        {
            ESP_LOGE(TAG, "Camera capture failed");
            response = ESP_FAIL;
        }
        else
        {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
            TXStream::send_jpeg_frame(_jpg_buf, _jpg_buf_len);

            esp_camera_fb_return(fb);
        }

        vTaskDelay(pdMS_TO_TICKS(1000/500));
    }
}

// ---------------------------------------------------------------------------
// Runtime FEC geometry. Phase 3 adaptation may change these between frames;
// the v2 header tells the receiver, no other synchronization is needed.
// ---------------------------------------------------------------------------
static uint8_t  s_k       = SP_K_DEFAULT;
static uint8_t  s_n       = SP_N_DEFAULT;
static uint16_t s_payload = SP_PAYLOAD_DEFAULT;

static fec_t  *s_fec = nullptr;
static uint8_t s_fec_k = 0, s_fec_n = 0;

// Parity staging for a whole frame (interleaving needs every block encoded
// before the first column is transmitted). Sized for the protocol maxima of
// parity chunks so geometry changes never need a realloc.
static uint8_t *s_parity_buf = nullptr; // SP_BLOCKS_MAX * (SP_N_MAX - SP_K_MAX) * SP_PAYLOAD_MAX
static uint8_t  s_tail_chunk[SP_PAYLOAD_MAX];   // zero-padded final partial data chunk
static uint8_t  s_zero_chunk[SP_PAYLOAD_MAX];   // shared all-padding data chunk

static SemaphoreHandle_t s_tx_done_sem = nullptr;

// 1 Hz transmit statistics
static uint32_t st_frames = 0, st_frames_oversize = 0, st_chunks = 0, st_tx_fail = 0;
static int64_t  st_last_log_us = 0;

// ---------------------------------------------------------------------------
// Link adaptation, driven by the dongle's 1 Hz ESP-NOW feedback.
// Degrade (loss > 12%): step the PHY rate down first (more SNR margin per
// airtime than parity), then add parity. Recover (loss < 3% for 5 s): shed
// parity first, then step the rate back up. No feedback for 5 s -> defaults,
// so a feedback-less dongle still works. Thresholds from the RF stress
// characterization: <10% loss is fully absorbed by RS(8,12), the frame-loss
// cliff starts ~15%.
// ---------------------------------------------------------------------------
static const wifi_phy_rate_t k_rate_ladder[] = {WIFI_PHY_RATE_24M, WIFI_PHY_RATE_36M, WIFI_PHY_RATE_54M};
static const char *k_rate_names[]            = {"24M", "36M", "54M"};
static constexpr const char *ADAPT_TAG       = "TX_SERVER";
#define RATE_IDX_DEFAULT 2

// TX power ladder in quarter-dBm: 2..10 dBm. The cap (10 dBm) is 50% of the
// chip's 20 dBm maximum by request; the default matches CONFIG_WIFI_TX_POWER.
// Power is the FIRST escalation rung (costs no airtime) and the LAST thing
// restored, so the camera idles at minimum power on a clean link.
static const int8_t k_power_ladder[] = {8, 16, 24, 32, 40};
#define POWER_IDX_DEFAULT 3  // 8 dBm = current CONFIG_WIFI_TX_POWER
#define POWER_IDX_CAP     4  // 10 dBm
#define POWER_DBM(idx)    (k_power_ladder[idx] / 4)

static volatile int64_t s_fb_time_us = 0;
static sp_feedback_t    s_fb_latest;
static int s_rate_idx     = RATE_IDX_DEFAULT;
static int s_power_idx    = POWER_IDX_DEFAULT;
static int s_good_seconds = 0;   // clean ticks toward coarse recovery (parity/rate)
static int s_clean_seconds = 0;  // clean ticks toward the fast power trim
static int s_probe_window  = 5;  // clean seconds required before a rate-up probe; doubles on failed probes
static int64_t s_last_rate_probe_us = 0;

void txstream_feedback(const sp_feedback_t *fb)
{
    s_fb_latest = *fb;                    // single writer (WiFi task)
    s_fb_time_us = esp_timer_get_time();  // written after payload: reader checks time first
}

static void apply_rate(int idx)
{
    s_rate_idx = idx;
    esp_wifi_config_80211_tx_rate(WIFI_IF_STA, k_rate_ladder[idx]);
}

static void apply_power(int idx)
{
    s_power_idx = idx;
    esp_wifi_set_max_tx_power(k_power_ladder[idx]);
}

static void adapt_tick(int64_t now)
{
    const bool have_fb = (now - s_fb_time_us) < 5000000 && s_fb_time_us != 0;

    if (!have_fb) {
        if (s_rate_idx != RATE_IDX_DEFAULT || s_n != SP_N_DEFAULT || s_power_idx != POWER_IDX_DEFAULT) {
            apply_rate(RATE_IDX_DEFAULT);
            apply_power(POWER_IDX_DEFAULT);
            s_n = SP_N_DEFAULT;
            ESP_LOGW(ADAPT_TAG, "adapt: feedback lost, reverting to defaults (%s, RS(%u,%u), %ddBm)",
                     k_rate_names[s_rate_idx], s_n, s_k, POWER_DBM(s_power_idx));
        }
        s_good_seconds = 0;
        return;
    }

    sp_feedback_t fb = s_fb_latest;
    if (fb.chunks_expected == 0) {
        return; // no measurement this second
    }
    uint32_t received = fb.chunks_received > fb.chunks_expected ? fb.chunks_expected : fb.chunks_received;
    uint32_t loss_pm = 1000u * (fb.chunks_expected - received) / fb.chunks_expected;

    // Dropped frames are the real failure (chunk loss is only its proxy):
    // any incomplete frame triggers an immediate degrade step.
    if (loss_pm > 120 || fb.frames_incomplete > 0) {
        s_good_seconds = s_clean_seconds = 0;
        // A degrade right after a rate-up probe means that rung failed —
        // back off before trying it again.
        if (s_last_rate_probe_us && (now - s_last_rate_probe_us) < 5000000) {
            s_probe_window = s_probe_window >= 30 ? 60 : s_probe_window * 2;
            s_last_rate_probe_us = 0;
        }
        // Energy-ordered escalation: power (~free) -> parity (x1.33 airtime)
        // -> rate (x1.5-2 airtime). Exception: above ~35% raw loss the SNR is
        // below the current rate's decode threshold and parity can't help —
        // go straight to a rate drop.
        if (s_power_idx < POWER_IDX_CAP) {
            apply_power(s_power_idx + 1);
        } else if (loss_pm > 350 && s_rate_idx > 0) {
            apply_rate(s_rate_idx - 1);
        } else if (s_n + 2 <= SP_N_MAX) {
            s_n += 2;
        } else if (s_rate_idx > 0) {
            apply_rate(s_rate_idx - 1);
        } else {
            return; // already at maximum robustness
        }
        ESP_LOGW(ADAPT_TAG, "adapt[degrade]: loss %lu.%lu%% drops %u rssi %d -> %s RS(%u,%u) %ddBm",
                 (unsigned long)(loss_pm / 10), (unsigned long)(loss_pm % 10),
                 fb.frames_incomplete, fb.rssi_avg,
                 k_rate_names[s_rate_idx], s_n, s_k, POWER_DBM(s_power_idx));
        return;
    }

    if (loss_pm >= 30) {
        s_good_seconds = s_clean_seconds = 0; // comfort band (3-12%): hold
        return;
    }

    // Clean link (< 3%). Coarse recovery first (parity, then power-boosted
    // rate probes), power trim as the fast independent loop.
    s_good_seconds++;
    s_clean_seconds++;

    // Recovery mirrors the escalation order: restore rate first (it's the
    // most expensive rung held, and probing while parity is still wide means
    // the probe is cushioned — a failed probe loses packets, not frames),
    // then shed parity, then trim power.
    if (s_rate_idx < RATE_IDX_DEFAULT && s_good_seconds >= s_probe_window) {
        // Power-boosted rate probe: borrow up to +4dB of headroom to cover
        // the higher rate's SNR requirement; the trim loop reclaims the
        // excess afterwards. High rate at modest power beats low rate at
        // low power on both airtime and energy.
        s_good_seconds = 0;
        int boosted = s_power_idx + 2 > POWER_IDX_CAP ? POWER_IDX_CAP : s_power_idx + 2;
        apply_power(boosted);
        apply_rate(s_rate_idx + 1);
        s_last_rate_probe_us = now;
        ESP_LOGI(ADAPT_TAG, "adapt[probe]: loss %lu.%lu%% -> %s RS(%u,%u) %ddBm (window %ds)",
                 (unsigned long)(loss_pm / 10), (unsigned long)(loss_pm % 10),
                 k_rate_names[s_rate_idx], s_n, s_k, POWER_DBM(s_power_idx), s_probe_window);
        return;
    }

    if (s_rate_idx == RATE_IDX_DEFAULT && s_probe_window != 5 &&
        s_last_rate_probe_us && (now - s_last_rate_probe_us) > 30000000) {
        s_probe_window = 5; // fully recovered and stable: forgive past probe failures
    }

    if (s_n > SP_N_DEFAULT && s_good_seconds >= 5) {
        s_good_seconds = 0;
        s_n -= 2;
        ESP_LOGI(ADAPT_TAG, "adapt[shed]: loss %lu.%lu%% -> %s RS(%u,%u) %ddBm",
                 (unsigned long)(loss_pm / 10), (unsigned long)(loss_pm % 10),
                 k_rate_names[s_rate_idx], s_n, s_k, POWER_DBM(s_power_idx));
        return;
    }

    // Fast power trim: one rung down per 2 clean seconds, independent of
    // where rate/parity sit.
    if (s_power_idx > 0 && s_clean_seconds >= 2) {
        s_clean_seconds = 0;
        apply_power(s_power_idx - 1);
        ESP_LOGI(ADAPT_TAG, "adapt[trim]: loss %lu.%lu%% -> %s RS(%u,%u) %ddBm",
                 (unsigned long)(loss_pm / 10), (unsigned long)(loss_pm % 10),
                 k_rate_names[s_rate_idx], s_n, s_k, POWER_DBM(s_power_idx));
    }
}

static void tx_done_callback(const esp_80211_tx_info_t *tx_info)
{
    (void)tx_info;
    xSemaphoreGive(s_tx_done_sem);
}

static bool ensure_fec(uint8_t k, uint8_t n)
{
    if (s_fec && s_fec_k == k && s_fec_n == n) {
        return true;
    }
    if (s_fec) {
        fec_free(s_fec);
        s_fec = nullptr;
    }
    init_fec();
    s_fec = fec_new(k, n);
    s_fec_k = k;
    s_fec_n = n;
    return s_fec != nullptr;
}

static bool ensure_buffers()
{
    if (s_parity_buf) {
        return true;
    }
    const size_t sz = (size_t)SP_BLOCKS_MAX * (SP_N_MAX - SP_K_MAX) * SP_PAYLOAD_MAX;
    s_parity_buf = (uint8_t *)heap_caps_malloc(sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_parity_buf) {
        s_parity_buf = (uint8_t *)heap_caps_malloc(sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    return s_parity_buf != nullptr;
}

void TXStream::send_jpeg_frame(const uint8_t *jpeg, size_t len)
{
    static uint8_t  frame_id = 0;
    static uint16_t seq = 0;
    static bool     init_done = false;
    static uint8_t  src_mac[6];

    if (!init_done) {
        s_tx_done_sem = xSemaphoreCreateBinary();
        if (s_tx_done_sem) {
            esp_wifi_register_80211_tx_cb(tx_done_callback);
        }
        esp_wifi_get_mac(WIFI_IF_STA, src_mac);
        memset(s_zero_chunk, 0, sizeof(s_zero_chunk));
        init_done = true;
    }

    if (!jpeg || len == 0) return;
    if (!ensure_fec(s_k, s_n) || !ensure_buffers()) {
        ESP_LOGE(TAG, "FEC init failed");
        return;
    }

    const uint8_t  k       = s_k;
    const uint8_t  n       = s_n;
    const uint8_t  parity  = n - k;
    const uint16_t payload = s_payload;

    // Frames that don't fit are dropped, never truncated.
    if (len > SP_MAX_FRAME_BYTES(k, payload)) {
        st_frames_oversize++;
        ESP_LOGW(TAG, "Frame %u too large (%u bytes > %u), dropped",
                 frame_id, (unsigned)len, (unsigned)SP_MAX_FRAME_BYTES(k, payload));
        return;
    }

    const uint16_t numDataChunks = (uint16_t)((len + payload - 1) / payload);
    const uint8_t  numBlocks     = (uint8_t)((numDataChunks + k - 1) / k);

    // Data chunk pointers go straight into the camera JPEG buffer; only the
    // final partial chunk (zero-padded) and fully-padding chunks use scratch.
    const uint16_t tail_idx = (uint16_t)(len / payload); // index of the partial chunk, if any
    const size_t   tail_len = len % payload;
    if (tail_len) {
        memcpy(s_tail_chunk, jpeg + (size_t)tail_idx * payload, tail_len);
        memset(s_tail_chunk + tail_len, 0, payload - tail_len);
    }

    auto data_chunk_ptr = [&](uint16_t idx) -> const uint8_t * {
        if (idx < tail_idx)               return jpeg + (size_t)idx * payload;
        if (idx == tail_idx && tail_len)  return s_tail_chunk;
        return s_zero_chunk;
    };

    // Encode parity for every block before transmitting anything.
    unsigned block_nums[SP_N_MAX - SP_K_MAX];
    for (uint8_t p = 0; p < parity; p++) {
        block_nums[p] = (unsigned)(k + p);
    }

    auto parity_chunk_ptr = [&](uint8_t block, uint8_t p) -> uint8_t * {
        return s_parity_buf + ((size_t)block * parity + p) * payload;
    };

    for (uint8_t b = 0; b < numBlocks; b++) {
        const gf *src[SP_K_MAX];
        gf *dst[SP_N_MAX - SP_K_MAX];
        for (uint8_t c = 0; c < k; c++) {
            src[c] = data_chunk_ptr((uint16_t)(b * k + c));
        }
        for (uint8_t p = 0; p < parity; p++) {
            dst[p] = parity_chunk_ptr(b, p);
        }
        fec_encode(s_fec, src, dst, block_nums, parity, payload);
    }

    // Column-interleaved transmit: chunk c of every block, then chunk c+1.
    // A burst of lost packets then costs each block ~burst/numBlocks chunks
    // instead of wiping out a single block.
    sp_chunk_hdr_t hdr;
    memcpy(hdr.oui, SP_OUI, 3);
    hdr.version    = SP_VERSION;
    hdr.frame_id   = frame_id;
    hdr.geometry   = sp_geometry_pack(k, n);
    hdr.num_blocks = numBlocks;
    sp_hdr_set_chunk_len(&hdr, payload);
    sp_hdr_set_frame_len(&hdr, (uint32_t)len);

    uint8_t air[SP_AIR_FRAME_MAX];

    for (uint8_t c = 0; c < n; c++) {
        for (uint8_t b = 0; b < numBlocks; b++) {
            const uint8_t *chunk = (c < k) ? data_chunk_ptr((uint16_t)(b * k + c))
                                           : parity_chunk_ptr(b, (uint8_t)(c - k));
            hdr.rs_block_id = b;
            hdr.chunk_id    = c;

            size_t air_len = sp_build_frame(air, src_mac, seq, &hdr, chunk, payload);
            seq = (seq + 1) & 0x0FFF;

            // Clear stale completion signal before queuing
            xSemaphoreTake(s_tx_done_sem, 0);

            esp_err_t result = esp_wifi_80211_tx(WIFI_IF_STA, air, air_len, false);

            if (result == ESP_OK) {
                // Wait for hardware to finish (natural backpressure)
                xSemaphoreTake(s_tx_done_sem, pdMS_TO_TICKS(50));
            } else if (result == ESP_ERR_NO_MEM) {
                // Descriptor ring full — wait for current TX to drain, retry once
                xSemaphoreTake(s_tx_done_sem, pdMS_TO_TICKS(50));
                result = esp_wifi_80211_tx(WIFI_IF_STA, air, air_len, false);
                if (result == ESP_OK) {
                    xSemaphoreTake(s_tx_done_sem, pdMS_TO_TICKS(50));
                }
            }
            if (result != ESP_OK) {
                st_tx_fail++;
            } else {
                st_chunks++;
            }
        }
    }

    st_frames++;
    frame_id++;

    int64_t now = esp_timer_get_time();
    if (now - st_last_log_us >= 1000000) {
        ESP_LOGI(TAG, "tx: %lu fps, %lu chunks/s, %lu tx_fail, %lu oversize, %s RS(%u,%u) %ddBm payload=%u",
                 (unsigned long)st_frames, (unsigned long)st_chunks,
                 (unsigned long)st_tx_fail, (unsigned long)st_frames_oversize,
                 k_rate_names[s_rate_idx], n, k, POWER_DBM(s_power_idx), payload);
        st_frames = st_chunks = st_tx_fail = 0;
        st_last_log_us = now;
        adapt_tick(now);
    }
}
