#pragma once
#ifndef TXSTREAM_HPP
#define TXSTREAM_HPP

#include "esp_log.h"
#include "esp_camera.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_err.h"
#include "fec.h"
#include "stream_protocol.h"
#include <stdio.h>
#include <cstring>

class TXStream {
public:
    explicit TXStream(int wifi_channel);

    // kicks off the continuous camera->WiFi loop
    void startStream();

    // FEC-encode and transmit one JPEG frame (protocol v2, column-interleaved)
    static void send_jpeg_frame(const uint8_t *jpeg, size_t len);

private:
    int _wifi_channel;

    camera_fb_t*    fb        = nullptr;
    esp_err_t       response  = ESP_OK;
    const uint8_t*  _jpg_buf  = nullptr;
    size_t          _jpg_buf_len = 0;

    static constexpr const char* TAG = "TX_SERVER";
};

#endif // TXSTREAM_HPP
