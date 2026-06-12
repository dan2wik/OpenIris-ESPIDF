#include "wifiManager.hpp"
#include "fec.h"
#include "stream_protocol.h"
#include "esp_heap_caps.h"
#include "esp_now.h"
#include "usb_cdc_serial.h"
#include <new>

static bool s_espnow_ready = false;
static const uint8_t s_bcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Feedback is rebuilt at 1 Hz but rebroadcast ~5x/s so the camera's
// duty-cycled receiver (300ms wake window per second) catches a copy.
static sp_feedback_t s_fb_cached;
static bool s_fb_valid = false;

static auto WIFI_MANAGER_TAG = "[WIFI_MANAGER]";

// ---------------------------------------------------------------------------
// 802.11 stream receiver (protocol v2, see stream_protocol.h)
//
// The sniffer fills one of two PSRAM slabs. When every RS block of a frame
// has >= k chunks, the slab is handed to the decode task by pointer (no
// copy) and filling continues in the other slab. Slabs are sized for the
// protocol maxima, so TX-side geometry changes never need a resize. A slab
// is fully reset on every new frame_id — chunks of different frames are
// never mixed.
// ---------------------------------------------------------------------------

#define RX_FRAME_STALE_MS 200

typedef struct {
    volatile bool busy;        // owned by the decode task
    bool     active;           // currently accumulating a frame
    uint8_t  frame_id;
    uint8_t  k, n, num_blocks;
    uint16_t payload_len;
    uint32_t frame_len;
    uint8_t  blocks_ready;     // blocks with >= k chunks
    uint8_t  chunk_present[SP_BLOCKS_MAX][SP_N_MAX];
    uint8_t  block_have[SP_BLOCKS_MAX];
    int32_t  rssi_sum;
    uint32_t chunks_rx;
    uint32_t t_first_ms;
    uint8_t *buf;              // SP_BLOCKS_MAX * SP_N_MAX * SP_PAYLOAD_MAX (PSRAM)
} rx_slab_t;

static rx_slab_t s_slab[2];
static int s_fill = 0;                 // slab being filled, -1 = none free
static int s_last_completed_id = -1;   // suppresses late chunks of a delivered frame

static uint8_t *s_decoded = nullptr;   // SP_BLOCKS_MAX * SP_K_MAX * SP_PAYLOAD_MAX (PSRAM)
static QueueHandle_t s_decode_q = nullptr;

// Receive statistics, printed at 1 Hz by the decode task
typedef struct {
    uint32_t frames_decoded;
    uint32_t frames_incomplete;  // abandoned: next frame started or stale
    uint32_t chunks_stored;      // on completed frames
    uint32_t chunks_expected;    // on completed frames
    uint32_t chunks_late;        // chunks of an already-delivered frame
    uint32_t chunks_nofill;      // dropped because both slabs were busy
    uint32_t blocks_repaired;    // blocks that needed parity reconstruction
    uint32_t bad_jpeg;
    int64_t  rssi_sum;
    uint32_t rssi_cnt;
} rx_stats_t;
static rx_stats_t s_st;

static JpegFrameCallback g_jpegFrameCallback = nullptr;

void WiFiManagerHelpers::event_handler(void *arg, esp_event_base_t event_base,
                                       int32_t event_id, void *event_data)
{
  ESP_LOGI(WIFI_MANAGER_TAG, "Trying to connect, got event: %d", (int)event_id);
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    if (const auto err = esp_wifi_connect(); err != ESP_OK)
    {
      ESP_LOGI(WIFI_MANAGER_TAG, "esp_wifi_connect() failed: %s", esp_err_to_name(err));
    }
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
    {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(WIFI_MANAGER_TAG, "retry to connect to the AP");
    }
    else
    {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(WIFI_MANAGER_TAG, "connect to the AP fail");
  }

  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    const auto *event = static_cast<ip_event_got_ip_t *>(event_data);
    ESP_LOGI(WIFI_MANAGER_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

WiFiManager::WiFiManager(std::shared_ptr<ProjectConfig> deviceConfig, QueueHandle_t eventQueue, StateManager *stateManager) : deviceConfig(deviceConfig), eventQueue(eventQueue), stateManager(stateManager) {}

void WiFiManager::SetCredentials(const char *ssid, const char *password)
{
  memcpy(_wifi_cfg.sta.ssid, ssid, std::min(strlen(ssid), sizeof(_wifi_cfg.sta.ssid)));
  memcpy(_wifi_cfg.sta.password, password, std::min(strlen(password), sizeof(_wifi_cfg.sta.password)));
}

void WiFiManager::ConnectWithHardcodedCredentials()
{
  SystemEvent event = {EventSource::WIFI, WiFiState_e::WiFiState_ReadyToConnect};
  this->SetCredentials(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &_wifi_cfg));

  xQueueSend(this->eventQueue, &event, 10);
  esp_wifi_start();

  event.value = WiFiState_e::WiFiState_Connecting;
  xQueueSend(this->eventQueue, &event, 10);

  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE,
                                         pdFALSE,
                                         portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT)
  {
    ESP_LOGI(WIFI_MANAGER_TAG, "connected to ap SSID:%p password:%p",
             _wifi_cfg.sta.ssid, _wifi_cfg.sta.password);

    event.value = WiFiState_e::WiFiState_Connected;
    xQueueSend(this->eventQueue, &event, 10);
  }
  else if (bits & WIFI_FAIL_BIT)
  {
    ESP_LOGE(WIFI_MANAGER_TAG, "Failed to connect to SSID:%p, password:%p",
             _wifi_cfg.sta.ssid, _wifi_cfg.sta.password);

    event.value = WiFiState_e::WiFiState_Error;
    xQueueSend(this->eventQueue, &event, 10);
  }
  else
  {
    ESP_LOGE(WIFI_MANAGER_TAG, "UNEXPECTED EVENT");
  }
}

void WiFiManager::ConnectWithStoredCredentials()
{
  SystemEvent event = {EventSource::WIFI, WiFiState_e::WiFiState_ReadyToConnect};

  auto const networks = this->deviceConfig->getWifiConfigs();

  if (networks.empty())
  {
    event.value = WiFiState_e::WiFiState_Disconnected;
    xQueueSend(this->eventQueue, &event, 10);
    ESP_LOGE(WIFI_MANAGER_TAG, "No networks stored, cannot connect");
    return;
  }

  for (const auto& network : networks)
  {
    xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
    this->SetCredentials(network.ssid.c_str(), network.password.c_str());

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &_wifi_cfg));
    xQueueSend(this->eventQueue, &event, 10);

    esp_wifi_start();

    event.value = WiFiState_e::WiFiState_Connecting;
    xQueueSend(this->eventQueue, &event, 10);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT)
    {
      ESP_LOGI(WIFI_MANAGER_TAG, "connected to ap SSID:%p password:%p",
               _wifi_cfg.sta.ssid, _wifi_cfg.sta.password);

      event.value = WiFiState_e::WiFiState_Connected;
      xQueueSend(this->eventQueue, &event, 10);
      return;
    }
    ESP_LOGE(WIFI_MANAGER_TAG, "Failed to connect to SSID:%p, password:%p, trying next stored network",
             _wifi_cfg.sta.ssid, _wifi_cfg.sta.password);
  }

  event.value = WiFiState_e::WiFiState_Error;
  xQueueSend(this->eventQueue, &event, 10);
  ESP_LOGE(WIFI_MANAGER_TAG, "Failed to connect to all saved networks");
}

void WiFiManager::SetupAccessPoint()
{
  ESP_LOGI(WIFI_MANAGER_TAG, "Connection to stored credentials failed, starting AP");

  esp_netif_create_default_wifi_ap();
  wifi_init_config_t esp_wifi_ap_init_config = WIFI_INIT_CONFIG_DEFAULT();

  ESP_ERROR_CHECK(esp_wifi_init(&esp_wifi_ap_init_config));

  wifi_config_t ap_wifi_config = {
      .ap = {
          .ssid = CONFIG_AP_WIFI_SSID,
          .password = CONFIG_AP_WIFI_PASSWORD,
          .max_connection = 1,
      },
  };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_LOGI(WIFI_MANAGER_TAG, "AP started.");
}

static fec_t  *s_fec = nullptr;
static uint8_t s_fec_k = 0, s_fec_n = 0;

static bool ensure_rx_fec(uint8_t k, uint8_t n)
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

void WiFiManager::setJpegFrameCallback(JpegFrameCallback callback) {
    g_jpegFrameCallback = callback;
}

// ---------------------------------------------------------------------------
// Decode task: receives a completed slab by index, reconstructs missing data
// chunks per RS block, trims to the frame length from the header and hands
// the JPEG to the callback. Also prints receive statistics at 1 Hz.
// ---------------------------------------------------------------------------
static void rs_decode_task_fn(void* arg)
{
    (void)arg;
    int64_t last_log = esp_timer_get_time();
    int64_t last_fb_send = 0;

    while (true) {
        int idx;
        bool got = xQueueReceive(s_decode_q, &idx, pdMS_TO_TICKS(250)) == pdTRUE;

        // Rebroadcast the cached feedback ~5x/s for the camera's wake window
        if (s_espnow_ready && s_fb_valid) {
            int64_t t = esp_timer_get_time();
            if (t - last_fb_send >= 190000) {
                last_fb_send = t;
                esp_now_send(s_bcast_mac, (const uint8_t *)&s_fb_cached, sizeof(s_fb_cached));
            }
        }

        if (got) {
            rx_slab_t *s = &s_slab[idx];
            const uint8_t  k = s->k, n = s->n;
            const uint16_t payload = s->payload_len;
            bool ok = ensure_rx_fec(k, n);

            for (uint8_t b = 0; ok && b < s->num_blocks; b++) {
                uint8_t *out_base = s_decoded + (size_t)b * k * payload;
                const uint8_t *blk = s->buf + (size_t)b * n * payload;

                bool all_data = true;
                for (uint8_t c = 0; c < k; c++) {
                    if (!s->chunk_present[b][c]) { all_data = false; break; }
                }

                if (all_data) {
                    // data chunks 0..k-1 are a contiguous prefix of the block
                    memcpy(out_base, blk, (size_t)k * payload);
                    continue;
                }

                s_st.blocks_repaired++;

                // Primary chunks at their natural slots, parity filling holes
                const gf *in[SP_K_MAX];
                unsigned  index[SP_K_MAX];
                gf       *outp[SP_K_MAX];
                uint8_t   outix = 0;
                uint8_t   next_par = k;

                for (uint8_t c = 0; c < k; c++) {
                    if (s->chunk_present[b][c]) {
                        in[c] = blk + (size_t)c * payload;
                        index[c] = c;
                        memcpy(out_base + (size_t)c * payload, in[c], payload);
                    } else {
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

            if (ok) {
                uint32_t cap = (uint32_t)s->num_blocks * k * payload;
                uint32_t flen = s->frame_len;
                if (flen == 0 || flen > cap) flen = cap;

                if (s_decoded[0] != 0xFF || s_decoded[1] != 0xD8) {
                    s_st.bad_jpeg++;
                }
                s_st.frames_decoded++;

                if (g_jpegFrameCallback) {
                    g_jpegFrameCallback(s_decoded, flen);
                }
            }
            s->busy = false; // release slab back to the sniffer
        }

        // USB liveness watchdog: a wedged TinyUSB task doesn't panic on its
        // own, so force a coredump (captures all task backtraces) when the
        // heartbeat stalls for 5 seconds.
        static uint32_t last_hb = 0;
        static uint8_t  hb_stalled = 0;
        int64_t now = esp_timer_get_time();
        if (now - last_log >= 1000000) {
            if (usb_cdc_active()) {
                uint32_t hb = g_tud_task_heartbeat;
                if (hb == last_hb) {
                    if (++hb_stalled >= 5) {
                        ESP_LOGE(WIFI_MANAGER_TAG, "TinyUSB task wedged (heartbeat stalled 5s) — aborting for coredump");
                        abort();
                    }
                } else {
                    hb_stalled = 0;
                }
                last_hb = hb;
            }
            // Frames complete before their tail chunks arrive (interleaved
            // order), so stragglers counted in chunks_late are received too.
            uint32_t loss_pm = 0; // permille
            if (s_st.chunks_expected) {
                uint32_t received = s_st.chunks_stored + s_st.chunks_late;
                if (received > s_st.chunks_expected) received = s_st.chunks_expected;
                loss_pm = (uint32_t)(1000ULL * (s_st.chunks_expected - received) / s_st.chunks_expected);
            }
            int rssi = s_st.rssi_cnt ? (int)(s_st.rssi_sum / (int64_t)s_st.rssi_cnt) : 0;

            // Refresh the cached link-quality feedback snapshot. It is
            // re-broadcast ~5x/second (below) so the camera's duty-cycled
            // receiver catches at least one copy per wake window.
            {
                uint32_t received = s_st.chunks_stored + s_st.chunks_late;
                s_fb_cached.magic[0] = 'O';
                s_fb_cached.magic[1] = 'I';
                s_fb_cached.version  = SP_FEEDBACK_VERSION;
                s_fb_cached.rssi_avg = (int8_t)rssi;
                s_fb_cached.chunks_expected   = (uint16_t)(s_st.chunks_expected > 0xFFFF ? 0xFFFF : s_st.chunks_expected);
                s_fb_cached.chunks_received   = (uint16_t)(received > 0xFFFF ? 0xFFFF : received);
                s_fb_cached.blocks_repaired   = (uint16_t)(s_st.blocks_repaired > 0xFFFF ? 0xFFFF : s_st.blocks_repaired);
                s_fb_cached.frames_decoded    = (uint8_t)(s_st.frames_decoded > 0xFF ? 0xFF : s_st.frames_decoded);
                s_fb_cached.frames_incomplete = (uint8_t)(s_st.frames_incomplete > 0xFF ? 0xFF : s_st.frames_incomplete);
                s_fb_valid = true;
            }
            ESP_LOGI(WIFI_MANAGER_TAG,
                     "rx: %lu fps, loss %lu.%lu%%, repaired %lu blk, incomplete %lu, late %lu, nofill %lu, badjpeg %lu, rssi %d",
                     (unsigned long)s_st.frames_decoded,
                     (unsigned long)(loss_pm / 10), (unsigned long)(loss_pm % 10),
                     (unsigned long)s_st.blocks_repaired,
                     (unsigned long)s_st.frames_incomplete,
                     (unsigned long)s_st.chunks_late,
                     (unsigned long)s_st.chunks_nofill,
                     (unsigned long)s_st.bad_jpeg,
                     rssi);
            memset(&s_st, 0, sizeof(s_st));
            last_log = now;
        }
    }
}

// ---------------------------------------------------------------------------
// Promiscuous sniffer: validates v2 chunks, accumulates them into the fill
// slab and hands completed frames to the decode task.
// ---------------------------------------------------------------------------
static void slab_begin_frame(rx_slab_t *s, const sp_parsed_t *p, uint32_t now_ms)
{
    s->active      = true;
    s->frame_id    = p->hdr->frame_id;
    s->k           = p->k;
    s->n           = p->n;
    s->num_blocks  = p->hdr->num_blocks;
    s->payload_len = p->chunk_len;
    s->frame_len   = p->frame_len;
    s->blocks_ready = 0;
    s->rssi_sum    = 0;
    s->chunks_rx   = 0;
    s->t_first_ms  = now_ms;
    memset(s->chunk_present, 0, sizeof(s->chunk_present));
    memset(s->block_have, 0, sizeof(s->block_have));
}

static void sniffer_cb(void *buf, wifi_promiscuous_pkt_type_t t)
{
    (void)t;
    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
    if (ppkt->rx_ctrl.rx_state != 0) return; // reception error

    sp_parsed_t p;
    if (!sp_parse_frame(ppkt->payload, ppkt->rx_ctrl.sig_len, &p)) return;

    // (Re)claim a fill slab if the last completion left us without one
    if (s_fill < 0) {
        if (!s_slab[0].busy)      s_fill = 0;
        else if (!s_slab[1].busy) s_fill = 1;
        else { s_st.chunks_nofill++; return; }
        s_slab[s_fill].active = false;
    }
    rx_slab_t *s = &s_slab[s_fill];

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    if (s->active) {
        bool same = (s->frame_id == p.hdr->frame_id) &&
                    (s->k == p.k) && (s->n == p.n) &&
                    (s->num_blocks == p.hdr->num_blocks) &&
                    (s->payload_len == p.chunk_len) &&
                    (s->frame_len == p.frame_len);
        if (!same || (now_ms - s->t_first_ms > RX_FRAME_STALE_MS)) {
            // A new frame began or accumulation went stale (frame_id wrap /
            // TX restart). The old frame is abandoned whole — its chunks are
            // never attached to the new frame.
            if (s->chunks_rx) s_st.frames_incomplete++;
            s->active = false;
        }
    }

    if (!s->active) {
        if ((int)p.hdr->frame_id == s_last_completed_id) {
            s_st.chunks_late++; // straggler of a frame already delivered
            return;
        }
        slab_begin_frame(s, &p, now_ms);
    }

    const uint8_t b = p.hdr->rs_block_id;
    const uint8_t c = p.hdr->chunk_id;
    if (s->chunk_present[b][c]) return;          // duplicate
    s->chunk_present[b][c] = 1;
    s->chunks_rx++;
    s->rssi_sum += ppkt->rx_ctrl.rssi;

    if (s->block_have[b] < s->k) {               // block still needs chunks
        memcpy(s->buf + ((size_t)b * s->n + c) * s->payload_len, p.payload, s->payload_len);
        if (++s->block_have[b] == s->k) {
            s->blocks_ready++;
        }
    }

    if (s->blocks_ready >= s->num_blocks) {
        // Frame complete — hand this slab off, continue in the other one
        s->busy = true;
        s->active = false;
        s_last_completed_id = s->frame_id;
        s_st.chunks_stored   += s->chunks_rx;
        s_st.chunks_expected += (uint32_t)s->num_blocks * s->n;
        s_st.rssi_sum += s->rssi_sum;
        s_st.rssi_cnt += s->chunks_rx;

        int done = s_fill;
        int other = 1 - s_fill;
        s_fill = s_slab[other].busy ? -1 : other;
        if (s_fill >= 0) {
            s_slab[s_fill].active = false;
        }
        xQueueSend(s_decode_q, &done, 0);
    }
}

void WiFiManager::Begin()
{
  #ifdef CONFIG_TX_MODE
    ESP_LOGI(WIFI_MANAGER_TAG, "Beginning TX Startup");
    esp_netif_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_err_t err = esp_wifi_config_80211_tx_rate(WIFI_IF_STA,  WIFI_PHY_RATE_54M); //WIFI_PHY_RATE_2M_L
    esp_wifi_start();
    esp_wifi_set_channel(CONFIG_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Receive the dongle's link-quality feedback (drives rate/parity/power
    // adaptation in TXStream; absent feedback = static defaults).
    //
    // NOTE: do NOT enable modem power save / ESP-NOW wake windows here. It
    // was tried (WIFI_PS_MIN_MODEM + 300ms/1s wake window) and the sleep
    // transitions mangled the injected stream: chunk loss rose ~10x at the
    // same RSSI. At 67fps the radio transmits every ~15ms, so there is no
    // idle to harvest anyway — the receiver must stay on while streaming.
    esp_wifi_set_ps(WIFI_PS_NONE);
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb([](const esp_now_recv_info_t *info, const uint8_t *data, int len) {
            (void)info;
            if (len == (int)sizeof(sp_feedback_t)) {
                const sp_feedback_t *fb = (const sp_feedback_t *)data;
                if (fb->magic[0] == 'O' && fb->magic[1] == 'I' && fb->version == SP_FEEDBACK_VERSION) {
                    txstream_feedback(fb);
                }
            }
        });
        ESP_LOGI(WIFI_MANAGER_TAG, "Feedback receiver ready (ESP-NOW)");
    } else {
        ESP_LOGW(WIFI_MANAGER_TAG, "ESP-NOW init failed - adaptation disabled, using static defaults");
    }

    ESP_LOGI(WIFI_MANAGER_TAG, "TX started on channel %d", CONFIG_WIFI_CHANNEL);
  #endif

  #ifdef CONFIG_RX_MODE
    ESP_LOGI(WIFI_MANAGER_TAG, "Beginning RX Startup");
    esp_netif_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    // STA (unconnected) instead of NULL so ESP-NOW can transmit feedback;
    // promiscuous reception works the same in STA mode.
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(CONFIG_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
        // Configure promiscuous filter to only receive data packets
    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(sniffer_cb));
    esp_wifi_set_promiscuous(true);

    // Receive slabs and decode output live in PSRAM, sized for the protocol
    // maxima so runtime FEC geometry changes never need a resize.
    const size_t slab_sz = (size_t)SP_BLOCKS_MAX * SP_N_MAX * SP_PAYLOAD_MAX;
    for (int i = 0; i < 2; i++) {
        s_slab[i].buf = (uint8_t *)heap_caps_malloc(slab_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    s_decoded = (uint8_t *)heap_caps_malloc((size_t)SP_BLOCKS_MAX * SP_K_MAX * SP_PAYLOAD_MAX,
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_slab[0].buf || !s_slab[1].buf || !s_decoded) {
        ESP_LOGE(WIFI_MANAGER_TAG, "Failed to allocate RX stream buffers in PSRAM");
        return;
    }

    ensure_rx_fec(SP_K_DEFAULT, SP_N_DEFAULT);

    // ESP-NOW feedback channel back to the camera (broadcast, 1 Hz)
    if (esp_now_init() == ESP_OK) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, s_bcast_mac, 6);
        peer.channel = 0; // current channel
        peer.ifidx   = WIFI_IF_STA;
        peer.encrypt = false;
        if (esp_now_add_peer(&peer) == ESP_OK) {
            s_espnow_ready = true;
            ESP_LOGI(WIFI_MANAGER_TAG, "Feedback channel ready (ESP-NOW broadcast)");
        }
    }
    if (!s_espnow_ready) {
        ESP_LOGW(WIFI_MANAGER_TAG, "ESP-NOW init failed - running without feedback (camera uses defaults)");
    }

    s_decode_q = xQueueCreate(2, sizeof(int));
    xTaskCreatePinnedToCore(rs_decode_task_fn, "rs_decode", 4096, nullptr, 5, nullptr, 1);

    ESP_LOGI(WIFI_MANAGER_TAG, "RX started on channel %d", CONFIG_WIFI_CHANNEL);
  #endif
    
  #if !defined(CONFIG_TX_MODE) && !defined(CONFIG_RX_MODE)
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  auto netif = esp_netif_create_default_wifi_sta();

  wifi_init_config_t esp_wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&esp_wifi_init_config));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &WiFiManagerHelpers::event_handler,
                                                      nullptr,
                                                      &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &WiFiManagerHelpers::event_handler,
                                                      nullptr,
                                                      &instance_got_ip));

  _wifi_cfg = {};
  _wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WEP;
  _wifi_cfg.sta.pmf_cfg.capable = true;
  _wifi_cfg.sta.pmf_cfg.required = false;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

  ESP_LOGI(WIFI_MANAGER_TAG, "Beginning setup");
  const auto hasHardcodedCredentials = strlen(CONFIG_WIFI_SSID) > 0;
  if (hasHardcodedCredentials)
  {
    ESP_LOGI(WIFI_MANAGER_TAG, "Detected hardcoded credentials, trying them out");
    this->ConnectWithHardcodedCredentials();
  }

  if (this->stateManager->GetWifiState() != WiFiState_e::WiFiState_Connected || !hasHardcodedCredentials)
  {
    ESP_LOGI(WIFI_MANAGER_TAG, "Hardcoded credentials failed or missing, trying stored credentials");
    xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
    this->ConnectWithStoredCredentials();
  }

  if (this->stateManager->GetWifiState() != WiFiState_e::WiFiState_Connected)
  {
    ESP_LOGI(WIFI_MANAGER_TAG, "Stored netoworks failed or hardcoded credentials missing, starting AP");
    xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
    esp_netif_destroy(netif);
    this->SetupAccessPoint();
  }
  #endif
}