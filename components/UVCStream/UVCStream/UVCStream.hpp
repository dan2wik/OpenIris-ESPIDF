#pragma once
#ifndef UVCSTREAM_HPP
#define UVCSTREAM_HPP
#include "esp_timer.h"
#include "esp_camera.h"
#include <CameraManager.hpp>
#include <StateManager.hpp>
#include "esp_log.h"
#include "usb_device_uvc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// we need access to the camera manager
// in order to update the frame settings
extern std::shared_ptr<CameraManager> cameraHandler;

// we also need a way to inform the rest of the system of what's happening
extern QueueHandle_t eventQueue;

namespace UVCStreamHelpers
{
  // TODO move the camera handling code to the camera manager and have the uvc manager initialize it in wired mode

  extern uint16_t frameWidth;
  extern uint16_t frameHeight;

  typedef struct
  {
    camera_fb_t *cam_fb_p;
    uvc_fb_t uvc_fb;
  } fb_t;

  static fb_t s_fb;

  #ifdef CONFIG_RX_MODE
  typedef struct{
    uint8_t *buf;
    size_t len;
  } jpeg_fb_t;

  // Preallocated in PSRAM by UVCStreamManager::setup():
  // jpeg_pp[2] is a ping-pong pair filled by provide_jpeg_frame (writers flip
  // jpeg_front after the copy); uvc_frame_buf is the stable copy handed to
  // the UVC stack. Definitions live in UVCStream.cpp.
  extern jpeg_fb_t jpeg_pp[2];
  extern volatile int jpeg_front;
  extern SemaphoreHandle_t frame_ready_sem;
  extern uint8_t *uvc_frame_buf;
  #endif

  static esp_err_t camera_start_cb(uvc_format_t format, int width, int height, int rate, void *cb_ctx);
  static void camera_stop_cb(void *cb_ctx);
  static uvc_fb_t *camera_fb_get_cb(void *cb_ctx);
  static void camera_fb_return_cb(uvc_fb_t *fb, void *cb_ctx);
}

class UVCStreamManager
{
  uint8_t *uvc_buffer = nullptr;

public:
  esp_err_t setup();
  #ifdef CONFIG_RX_MODE
  void provide_jpeg_frame(uint8_t *jpeg_data, size_t jpeg_len);
  #endif
};

#endif // UVCSTREAM_HPP
