#pragma once
#ifndef STREAMRECEIVER_HPP
#define STREAMRECEIVER_HPP

#include "sdkconfig.h"

#ifdef CONFIG_OISTREAM_RX_MODE

#include <functional>
#include "esp_err.h"

// 802.11 broadcast video receiver (dongle side of the TX/RX link).
//
// Sniffs the camera's raw broadcast stream in promiscuous mode, reassembles
// and Reed-Solomon-repairs frames (see stream_protocol.h) and hands complete
// JPEGs to the callback. Broadcasts link-quality feedback over ESP-NOW
// (~5x/s) that drives the camera's power/parity/rate adaptation.
//
// Owns its WiFi bring-up: unassociated STA on CONFIG_OISTREAM_WIFI_CHANNEL
// with promiscuous reception. Call once, before/independent of UVC setup.

using JpegFrameCallback = std::function<void(uint8_t*, size_t)>;

esp_err_t stream_receiver_begin(JpegFrameCallback callback);

#endif  // CONFIG_OISTREAM_RX_MODE
#endif  // STREAMRECEIVER_HPP
