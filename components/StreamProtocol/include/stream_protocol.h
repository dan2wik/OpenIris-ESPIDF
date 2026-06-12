// Wire protocol shared between the camera (TXStream) and the dongle
// (wifiManager RX). Single source of truth — both sides must build from the
// same copy of these definitions.
//
// v2: the header is self-describing. Every chunk carries the FEC geometry
// (k, n), chunk payload size and total JPEG length, so the transmitter may
// change any of them between frames and the receiver follows with no
// negotiation. Chunks are transmitted column-interleaved across RS blocks
// so burst losses spread over many blocks instead of killing one.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SP_VERSION        2
#define SP_ETHERTYPE      0x88B5

// Hard protocol maxima. RX buffers are sized from these; TX must never
// exceed them regardless of runtime tuning.
#define SP_K_MAX          8     // data chunks per RS block
#define SP_N_MAX          16    // total chunks per RS block (k + parity)
#define SP_BLOCKS_MAX     16    // RS blocks per frame
#define SP_PAYLOAD_MAX    800   // chunk payload bytes

// Current defaults (TX side; may be tuned at runtime within the maxima)
#define SP_K_DEFAULT      8
#define SP_N_DEFAULT      12    // 50% overhead: any 4-of-12 losses per block recoverable
#define SP_PAYLOAD_DEFAULT 400

#define SP_MAX_FRAME_BYTES(k, payload) ((uint32_t)SP_BLOCKS_MAX * (k) * (payload))

static const uint8_t SP_OUI[3] = {0xAC, 0xDE, 0x47};

typedef struct __attribute__((packed)) {
    uint8_t frame_control[2];
    uint8_t duration[2];
    uint8_t addr1[6];   // destination (broadcast)
    uint8_t addr2[6];   // source
    uint8_t addr3[6];   // BSSID (= source)
    uint8_t seq_ctrl[2];
} sp_ieee80211_hdr_t;

typedef struct __attribute__((packed)) {
    uint8_t  oui[3];
    uint8_t  version;       // SP_VERSION
    uint8_t  frame_id;
    uint8_t  geometry;      // (k << 4) | (n - k)
    uint8_t  num_blocks;    // RS blocks in this frame
    uint8_t  rs_block_id;
    uint8_t  chunk_id;      // 0..k-1 = data, k..n-1 = parity
    uint8_t  chunk_len[2];  // payload bytes per chunk in this frame, big endian
    uint8_t  frame_len[3];  // total JPEG bytes, big endian 24-bit
} sp_chunk_hdr_t;

#define SP_LLC_SNAP_LEN   8
#define SP_AIR_OVERHEAD   (sizeof(sp_ieee80211_hdr_t) + SP_LLC_SNAP_LEN + sizeof(sp_chunk_hdr_t))
#define SP_AIR_FRAME_MAX  (SP_AIR_OVERHEAD + SP_PAYLOAD_MAX)

static inline uint8_t  sp_geometry_pack(uint8_t k, uint8_t n) { return (uint8_t)((k << 4) | ((n - k) & 0x0F)); }
static inline uint8_t  sp_geometry_k(uint8_t g)               { return g >> 4; }
static inline uint8_t  sp_geometry_n(uint8_t g)               { return (uint8_t)((g >> 4) + (g & 0x0F)); }

static inline void sp_hdr_set_chunk_len(sp_chunk_hdr_t *h, uint16_t v)
{
    h->chunk_len[0] = (uint8_t)(v >> 8);
    h->chunk_len[1] = (uint8_t)(v & 0xFF);
}

static inline void sp_hdr_set_frame_len(sp_chunk_hdr_t *h, uint32_t v)
{
    h->frame_len[0] = (uint8_t)(v >> 16);
    h->frame_len[1] = (uint8_t)(v >> 8);
    h->frame_len[2] = (uint8_t)(v & 0xFF);
}

// Build a complete 802.11 air frame around one chunk. Returns total length.
static inline size_t sp_build_frame(uint8_t *out, const uint8_t src_mac[6], uint16_t seq,
                                    const sp_chunk_hdr_t *hdr,
                                    const uint8_t *payload, size_t payload_len)
{
    sp_ieee80211_hdr_t *mac = (sp_ieee80211_hdr_t *)out;
    mac->frame_control[0] = 0x08; // Type=Data, Subtype=Data
    mac->frame_control[1] = 0x00;
    mac->duration[0] = 0x00;
    mac->duration[1] = 0x00;
    memset(mac->addr1, 0xFF, 6);
    memcpy(mac->addr2, src_mac, 6);
    memcpy(mac->addr3, src_mac, 6);
    mac->seq_ctrl[0] = (uint8_t)((seq & 0x0F) << 4);
    mac->seq_ctrl[1] = (uint8_t)(seq >> 4);

    uint8_t *p = out + sizeof(sp_ieee80211_hdr_t);
    // LLC/SNAP
    *p++ = 0xAA; *p++ = 0xAA; *p++ = 0x03;
    *p++ = 0x00; *p++ = 0x00; *p++ = 0x00;
    *p++ = (uint8_t)(SP_ETHERTYPE >> 8);
    *p++ = (uint8_t)(SP_ETHERTYPE & 0xFF);

    memcpy(p, hdr, sizeof(sp_chunk_hdr_t));
    p += sizeof(sp_chunk_hdr_t);
    memcpy(p, payload, payload_len);
    return SP_AIR_OVERHEAD + payload_len;
}

// ---------------------------------------------------------------------------
// Control plane: 1 Hz link-quality feedback broadcast by the dongle over
// ESP-NOW. The camera adapts PHY rate and FEC parity from it; if feedback
// stops for 5 s the camera reverts to compile-time defaults, so one-way
// operation always keeps working.
// ---------------------------------------------------------------------------
#define SP_FEEDBACK_VERSION 1

typedef struct __attribute__((packed)) {
    uint8_t  magic[2];          // 'O','I'
    uint8_t  version;           // SP_FEEDBACK_VERSION
    int8_t   rssi_avg;          // mean chunk RSSI over the last second
    uint16_t chunks_expected;   // last second, clamped
    uint16_t chunks_received;   // stored + late stragglers, clamped
    uint16_t blocks_repaired;
    uint8_t  frames_decoded;
    uint8_t  frames_incomplete;
    uint8_t  proposed_channel;  // reserved for channel negotiation, 0 = none
} sp_feedback_t;

// Implemented in TXStream.cpp; called from the ESP-NOW receive callback on
// the camera to feed the adaptation state machine.
void txstream_feedback(const sp_feedback_t *fb);

typedef struct {
    const sp_chunk_hdr_t *hdr;
    const uint8_t *payload;
    uint8_t  k, n;
    uint16_t chunk_len;
    uint32_t frame_len;
} sp_parsed_t;

// Parse and validate a sniffed 802.11 payload. Returns false for anything
// that is not a structurally valid v2 stream chunk.
static inline bool sp_parse_frame(const uint8_t *pkt, size_t len, sp_parsed_t *out)
{
    if (len < SP_AIR_OVERHEAD) return false;
    if ((pkt[0] & 0xFC) != 0x08) return false; // plain data frames only

    const uint8_t *llc = pkt + sizeof(sp_ieee80211_hdr_t);
    if (llc[0] != 0xAA || llc[1] != 0xAA || llc[2] != 0x03) return false;
    if (llc[3] != 0x00 || llc[4] != 0x00 || llc[5] != 0x00) return false;
    if (((llc[6] << 8) | llc[7]) != SP_ETHERTYPE) return false;

    const sp_chunk_hdr_t *hdr = (const sp_chunk_hdr_t *)(llc + SP_LLC_SNAP_LEN);
    if (memcmp(hdr->oui, SP_OUI, 3) != 0) return false;
    if (hdr->version != SP_VERSION) return false;

    uint8_t k = sp_geometry_k(hdr->geometry);
    uint8_t n = sp_geometry_n(hdr->geometry);
    uint16_t chunk_len = (uint16_t)((hdr->chunk_len[0] << 8) | hdr->chunk_len[1]);
    if (k == 0 || k > SP_K_MAX || n <= k || n > SP_N_MAX) return false;
    if (hdr->num_blocks == 0 || hdr->num_blocks > SP_BLOCKS_MAX) return false;
    if (hdr->rs_block_id >= hdr->num_blocks) return false;
    if (hdr->chunk_id >= n) return false;
    if (chunk_len == 0 || chunk_len > SP_PAYLOAD_MAX) return false;
    if (len < SP_AIR_OVERHEAD + chunk_len) return false;

    out->hdr = hdr;
    out->payload = (const uint8_t *)hdr + sizeof(sp_chunk_hdr_t);
    out->k = k;
    out->n = n;
    out->chunk_len = chunk_len;
    out->frame_len = ((uint32_t)hdr->frame_len[0] << 16) |
                     ((uint32_t)hdr->frame_len[1] << 8) |
                     hdr->frame_len[2];
    return true;
}

#ifdef __cplusplus
}
#endif
