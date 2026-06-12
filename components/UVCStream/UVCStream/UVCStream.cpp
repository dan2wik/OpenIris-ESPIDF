#include "UVCStream.hpp"
#include "esp_heap_caps.h"
constexpr int UVC_MAX_FRAMESIZE_SIZE(75 * 1024);

static const char *UVC_STREAM_TAG = "[UVC DEVICE]";

namespace UVCStreamHelpers
{
  uint16_t frameWidth = 0;
  uint16_t frameHeight = 0;

  #ifdef CONFIG_RX_MODE
  // All frame buffers live in PSRAM (allocated once in setup()); see hpp.
  jpeg_fb_t jpeg_pp[2] = {};
  volatile int jpeg_front = 0;
  SemaphoreHandle_t frame_ready_sem = nullptr;
  uint8_t *uvc_frame_buf = nullptr;
  #endif
}

static esp_err_t UVCStreamHelpers::camera_start_cb(uvc_format_t format, int width, int height, int rate, void *cb_ctx)
{
  (void)cb_ctx;
  ESP_LOGI(UVC_STREAM_TAG, "camera_start_cb CALLED: format=%d, width=%d, height=%d, rate=%d", format, width, height, rate);
  frameWidth = width;
  frameHeight = height;
  ESP_LOGI(UVC_STREAM_TAG, "Camera Start");
  ESP_LOGI(UVC_STREAM_TAG, "Format: %d, width: %d, height: %d, rate: %d", format, width, height, rate);
  #ifndef CONFIG_RX_MODE
  framesize_t frame_size = FRAMESIZE_240X240;
  #endif

  if (format != UVC_FORMAT_JPEG)
  {
    ESP_LOGE(UVC_STREAM_TAG, "Only support MJPEG format");
    return ESP_ERR_NOT_SUPPORTED;
  }

  if (width == frameWidth && height == frameHeight)
  {
    #ifndef CONFIG_RX_MODE
    frame_size = FRAMESIZE_240X240; // Default resolution for UVC Stream
    #endif
  }
  else
  {
    ESP_LOGE(UVC_STREAM_TAG, "Unsupported frame size %dx%d", width, height);
    return ESP_ERR_NOT_SUPPORTED;
  }
  #ifndef CONFIG_RX_MODE
  cameraHandler->setCameraResolution(frame_size);
  cameraHandler->resetCamera(false);
  #endif

  constexpr SystemEvent event = {EventSource::STREAM, StreamState_e::Stream_ON};
  xQueueSend(eventQueue, &event, 10);

  return ESP_OK;
}

static void UVCStreamHelpers::camera_stop_cb(void *cb_ctx)
{
  ESP_LOGI(UVC_STREAM_TAG, "camera_stop_cb CALLED");
  (void)cb_ctx;
  #ifndef CONFIG_RX_MODE
  if (s_fb.cam_fb_p)
  {
    esp_camera_fb_return(s_fb.cam_fb_p);
    s_fb.cam_fb_p = nullptr;
  }
  #endif

  constexpr SystemEvent event = {EventSource::STREAM, StreamState_e::Stream_OFF};
  xQueueSend(eventQueue, &event, 10);
}

static uvc_fb_t *UVCStreamHelpers::camera_fb_get_cb(void *cb_ctx)
{
  (void)cb_ctx;
  #ifndef CONFIG_RX_MODE
  s_fb.cam_fb_p = esp_camera_fb_get();
  if (!s_fb.cam_fb_p)
  {
    ESP_LOGE(UVC_STREAM_TAG, "No camera FB");
    return nullptr;
  }
  s_fb.uvc_fb.buf = s_fb.cam_fb_p->buf;
  s_fb.uvc_fb.len = s_fb.cam_fb_p->len;
  s_fb.uvc_fb.width = s_fb.cam_fb_p->width;
  s_fb.uvc_fb.height = s_fb.cam_fb_p->height;
  s_fb.uvc_fb.format = UVC_FORMAT_JPEG; // we gotta make sure we're ALWAYS using JPEG
  s_fb.uvc_fb.timestamp = s_fb.cam_fb_p->timestamp;
  #else
  // Check if a frame is already available (decode finished before we were called)
  if (uxSemaphoreGetCount(frame_ready_sem) == 0) {
    // No frame yet — wait up to 200ms for RS decode to finish
    if (xSemaphoreTake(frame_ready_sem, pdMS_TO_TICKS(200)) != pdTRUE)
    {
      ESP_LOGW(UVC_STREAM_TAG, "FB_CB: semaphore timeout, no frame ready");
      return nullptr;
    }
  } else {
    // Frame already decoded — take immediately
    xSemaphoreTake(frame_ready_sem, 0);
  }
  // Snapshot the front (stable) ping-pong buffer; provide_jpeg_frame only
  // ever writes the back buffer, then flips.
  const int front = jpeg_front;
  const size_t flen = jpeg_pp[front].len;
  if (jpeg_pp[front].buf == nullptr || flen == 0) {
    ESP_LOGW(UVC_STREAM_TAG, "FB_CB: no frame available");
    return nullptr;
  }
  memcpy(uvc_frame_buf, jpeg_pp[front].buf, flen);
  s_fb.uvc_fb.buf = uvc_frame_buf;
  s_fb.uvc_fb.len = flen;
  s_fb.uvc_fb.width = frameWidth;
  s_fb.uvc_fb.height = frameHeight;
  s_fb.uvc_fb.format = UVC_FORMAT_JPEG; // we gotta make sure we're ALWAYS using JPEG
  s_fb.uvc_fb.timestamp = static_cast<timeval>(esp_timer_get_time());
  #endif

  if (s_fb.uvc_fb.len > UVC_MAX_FRAMESIZE_SIZE)
  {
    ESP_LOGE(UVC_STREAM_TAG, "Frame size %d is larger than max frame size %d", s_fb.uvc_fb.len, UVC_MAX_FRAMESIZE_SIZE);
    #ifndef CONFIG_RX_MODE
    esp_camera_fb_return(s_fb.cam_fb_p);
    #endif
    return nullptr;
  }

  return &s_fb.uvc_fb;
}

static void UVCStreamHelpers::camera_fb_return_cb(uvc_fb_t *fb, void *cb_ctx)
{
  (void)cb_ctx;
  assert(fb == &s_fb.uvc_fb);
  #ifndef CONFIG_RX_MODE
  esp_camera_fb_return(s_fb.cam_fb_p);
  #endif
}

esp_err_t UVCStreamManager::setup()
{
  ESP_LOGI(UVC_STREAM_TAG, "Setting up UVC Stream");

   #ifdef CONFIG_RX_MODE
   UVCStreamHelpers::frame_ready_sem = xSemaphoreCreateCounting(255, 0);
   if (UVCStreamHelpers::frame_ready_sem == nullptr)
   {
     ESP_LOGE(UVC_STREAM_TAG, "Failed to create frame ready semaphore");
     return ESP_FAIL;
   }

   // Frame buffers in PSRAM, allocated once — no per-frame heap churn
   UVCStreamHelpers::uvc_frame_buf = static_cast<uint8_t *>(
       heap_caps_malloc(UVC_MAX_FRAMESIZE_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
   for (int i = 0; i < 2; i++)
   {
     UVCStreamHelpers::jpeg_pp[i].buf = static_cast<uint8_t *>(
         heap_caps_malloc(UVC_MAX_FRAMESIZE_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
     UVCStreamHelpers::jpeg_pp[i].len = 0;
   }
   if (!UVCStreamHelpers::uvc_frame_buf ||
       !UVCStreamHelpers::jpeg_pp[0].buf || !UVCStreamHelpers::jpeg_pp[1].buf)
   {
     ESP_LOGE(UVC_STREAM_TAG, "Failed to allocate UVC frame buffers in PSRAM");
     return ESP_FAIL;
   }
   #endif

  uvc_buffer = static_cast<uint8_t *>(malloc(UVC_MAX_FRAMESIZE_SIZE));
  if (uvc_buffer == nullptr)
  {
    ESP_LOGE(UVC_STREAM_TAG, "Allocating buffer for UVC Device failed");
    return ESP_FAIL;
  }

  uvc_device_config_t config = {
      .uvc_buffer = uvc_buffer,
      .uvc_buffer_size = UVC_MAX_FRAMESIZE_SIZE,
      .start_cb = UVCStreamHelpers::camera_start_cb,
      .fb_get_cb = UVCStreamHelpers::camera_fb_get_cb,
      .fb_return_cb = UVCStreamHelpers::camera_fb_return_cb,
      .stop_cb = UVCStreamHelpers::camera_stop_cb,
  };

  esp_err_t ret = uvc_device_config(0, &config);
  if (ret != ESP_OK)
  {
    ESP_LOGE(UVC_STREAM_TAG, "Configuring UVC Device failed: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(UVC_STREAM_TAG, "Configured UVC Device");

  ESP_LOGI(UVC_STREAM_TAG, "Initializing UVC Device");
  ret = uvc_device_init();
  if (ret != ESP_OK)
  {
    ESP_LOGE(UVC_STREAM_TAG, "Initializing UVC Device failed: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(UVC_STREAM_TAG, "Initialized UVC Device");

  return ESP_OK;
}

#ifdef CONFIG_RX_MODE
void UVCStreamManager::provide_jpeg_frame(uint8_t *jpeg_data, size_t jpeg_len)
{
  if (!jpeg_data || jpeg_len == 0 || jpeg_len > UVC_MAX_FRAMESIZE_SIZE)
  {
    ESP_LOGW(UVC_STREAM_TAG, "Invalid JPEG frame input");
    return;
  }

  // Write the back buffer of the ping-pong pair, then flip. Readers only
  // ever touch the front buffer, so no allocation or locking is needed.
  const int back = 1 - UVCStreamHelpers::jpeg_front;
  if (!UVCStreamHelpers::jpeg_pp[back].buf)
  {
    return; // setup() failed or not run yet
  }
  memcpy(UVCStreamHelpers::jpeg_pp[back].buf, jpeg_data, jpeg_len);
  UVCStreamHelpers::jpeg_pp[back].len = jpeg_len;
  UVCStreamHelpers::jpeg_front = back;

  ESP_LOGD(UVC_STREAM_TAG, "Submitted JPEG frame (%zu bytes)", jpeg_len);

  // Signal that a frame is ready
  xSemaphoreGive(UVCStreamHelpers::frame_ready_sem);
}
#endif
