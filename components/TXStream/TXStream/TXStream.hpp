#pragma once
#ifndef TXSTREAM_HPP
#define TXSTREAM_HPP

#include "sdkconfig.h"

#ifdef CONFIG_OISTREAM_TX_MODE

#include <cstring>
#include <stdio.h>
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "fec.h"
#include "stream_protocol.h"

// 802.11 broadcast video transmitter (camera side of the TX/RX dongle link).
//
// JPEG frames are Reed-Solomon encoded (see stream_protocol.h) and injected
// as raw broadcast data frames — no association, no ACKs, no retransmission;
// erasure coding absorbs packet loss. A 1 Hz ESP-NOW feedback channel from
// the dongle drives adaptation of TX power, FEC parity and PHY rate; without
// feedback the transmitter runs at static defaults.
class TXStream
{
   public:
    // Initializes WiFi (unassociated STA on CONFIG_OISTREAM_WIFI_CHANNEL),
    // the injection rate and the ESP-NOW feedback receiver.
    static esp_err_t begin();

    // Spawns the camera -> encode -> inject loop task.
    static void startTask();

    // FEC-encode and transmit one JPEG frame.
    static void send_jpeg_frame(const uint8_t* jpeg, size_t len);

   private:
    static void streamTask(void* arg);
    static constexpr const char* TAG = "TX_STREAM";
};

#endif  // CONFIG_OISTREAM_TX_MODE
#endif  // TXSTREAM_HPP
