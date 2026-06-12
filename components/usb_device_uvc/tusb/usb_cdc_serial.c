/*
 * CDC-ACM serial channel for the UVC composite device.
 *
 * Provides:
 *  - a simple read/write API for the command channel (usb_cdc_serial.h)
 *  - a console mirror: ESP_LOG output is buffered from boot and replayed to
 *    the CDC port when the host opens it, then streamed live
 *  - the esptool reset handshake so the board can be reflashed over the CDC
 *    port without holding the BOOT button
 */
#include "tusb.h"

#if CFG_TUD_CDC

#include <stdarg.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "sdkconfig.h"
#include "usb_cdc_serial.h"

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32S2
#include "soc/rtc_cntl_reg.h"
#endif

// Holds boot output until the host opens the COM port. PSRAM if available.
#define LOG_RING_SIZE_PSRAM    (16 * 1024)
#define LOG_RING_SIZE_INTERNAL (4 * 1024)
#define LOG_LINE_MAX           512

// One mutex serializes the log ring and every tud_cdc_write producer
// (log drain vs command responses). Never block while holding it: the
// TinyUSB task is the FIFO consumer and must keep running.
static SemaphoreHandle_t s_cdc_mutex;
static vprintf_like_t s_orig_vprintf;

static char *s_ring;
static size_t s_ring_size;
static size_t s_ring_head; // next write position
static size_t s_ring_len;  // bytes currently buffered

bool usb_cdc_active(void)
{
    return tusb_inited();
}

static bool cdc_lock(TickType_t timeout)
{
    return s_cdc_mutex && xSemaphoreTake(s_cdc_mutex, timeout) == pdTRUE;
}

static void cdc_unlock(void)
{
    xSemaphoreGive(s_cdc_mutex);
}

//--------------------------------------------------------------------+
// Console mirror (ESP_LOG -> CDC)
//--------------------------------------------------------------------+

static void ring_append(const char *data, size_t len)
{
    if (!s_ring) {
        return;
    }
    if (len >= s_ring_size) { // keep only the newest window
        data += len - (s_ring_size - 1);
        len = s_ring_size - 1;
    }
    for (size_t i = 0; i < len; i++) {
        s_ring[s_ring_head] = data[i];
        s_ring_head = (s_ring_head + 1) % s_ring_size;
    }
    s_ring_len += len;
    if (s_ring_len >= s_ring_size) { // overwrote oldest data
        s_ring_len = s_ring_size - 1;
    }
}

// Push buffered output into the CDC FIFO without blocking; whatever doesn't
// fit stays in the ring for the next call. Caller must hold the mutex.
static void ring_drain_locked(void)
{
    if (!usb_cdc_active() || !tud_cdc_connected()) {
        return;
    }
    while (s_ring_len > 0) {
        size_t tail = (s_ring_head + s_ring_size - s_ring_len) % s_ring_size;
        size_t contiguous = (tail + s_ring_len <= s_ring_size) ? s_ring_len : s_ring_size - tail;
        uint32_t written = tud_cdc_write(s_ring + tail, contiguous);
        if (written == 0) { // FIFO full — try again on the next log line
            break;
        }
        s_ring_len -= written;
    }
    tud_cdc_write_flush();
}

// Invoked by the TinyUSB task whenever the host drains the CDC TX FIFO —
// keeps pushing buffered backlog out without waiting for the next log line.
void tud_cdc_tx_complete_cb(uint8_t itf)
{
    (void)itf;
    if (cdc_lock(0)) {
        ring_drain_locked();
        cdc_unlock();
    }
}

static int console_mirror_vprintf(const char *fmt, va_list args)
{
    static char line[LOG_LINE_MAX];

    if (cdc_lock(pdMS_TO_TICKS(20))) {
        va_list copy;
        va_copy(copy, args);
        int n = vsnprintf(line, sizeof(line), fmt, copy);
        va_end(copy);
        if (n > 0) {
            ring_append(line, (size_t)n < sizeof(line) ? (size_t)n : sizeof(line) - 1);
            ring_drain_locked();
        }
        cdc_unlock();
    }

    // keep the original console (UART0 / USB-Serial/JTAG) working
    return s_orig_vprintf ? s_orig_vprintf(fmt, args) : 0;
}

void usb_cdc_console_mirror_init(void)
{
    if (s_ring) {
        return;
    }
    if (!s_cdc_mutex) {
        s_cdc_mutex = xSemaphoreCreateMutex();
    }

    s_ring_size = LOG_RING_SIZE_PSRAM;
    s_ring = heap_caps_malloc(s_ring_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_ring) {
        s_ring_size = LOG_RING_SIZE_INTERNAL;
        s_ring = malloc(s_ring_size);
    }
    if (!s_ring) {
        return; // no buffer, no mirror — console still works
    }

    s_orig_vprintf = esp_log_set_vprintf(console_mirror_vprintf);
}

//--------------------------------------------------------------------+
// Command channel read/write
//--------------------------------------------------------------------+

int usb_cdc_read(uint8_t *buf, size_t buf_size, uint32_t timeout_ms)
{
    if (!usb_cdc_active()) {
        return -1;
    }

    uint32_t waited_ms = 0;
    while (tud_cdc_available() == 0) {
        if (waited_ms >= timeout_ms) {
            return 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        waited_ms += 10;
    }
    return (int)tud_cdc_read(buf, buf_size);
}

int usb_cdc_write(const void *buf, size_t len)
{
    if (!usb_cdc_active() || !tud_cdc_connected()) {
        return -1;
    }
    if (!s_cdc_mutex) {
        s_cdc_mutex = xSemaphoreCreateMutex();
    }

    const uint8_t *p = (const uint8_t *)buf;
    size_t total = 0;
    while (total < len) {
        uint32_t written = 0;
        if (cdc_lock(pdMS_TO_TICKS(50))) {
            written = tud_cdc_write(p + total, len - total);
            tud_cdc_write_flush();
            cdc_unlock();
        }
        total += written;
        if (written == 0) {
            // FIFO full — yield (outside the lock) so the TinyUSB task can drain it
            vTaskDelay(1);
            if (!tud_cdc_connected()) {
                break;
            }
        }
    }
    return (int)total;
}

//--------------------------------------------------------------------+
// esptool reset-to-bootloader support
//--------------------------------------------------------------------+

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32S2
static void reset_to_bootloader(void)
{
    // Request ROM download mode for the next boot and restart. Without USB
    // persist flags the ROM comes up on the USB-Serial/JTAG port — the same
    // COM port as the BOOT-button path — so flashing always targets one
    // known bootloader port.
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    esp_restart();
}
#else
static void reset_to_bootloader(void)
{
    esp_restart();
}
#endif

// Bootloader-touch baud rate. Deliberately NOT the Arduino-style 1200:
// Windows' serial-mouse enumeration (sermouse) probes new COM ports at
// 1200 baud with DTR toggling, which kept rebooting the device into the
// bootloader on enumeration. 12345 is never probed by anything.
#define BOOTLOADER_TOUCH_BAUD 12345

static uint32_t s_last_bit_rate = 115200;

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *coding)
{
    (void)itf;
    s_last_bit_rate = coding->bit_rate;
}

// esptool's classic reset toggles DTR/RTS through (0,1)->(1,1)->(1,0)->(0,0);
// entering the final state from the sequence triggers the bootloader (same
// state machine as arduino-esp32). A "touch" (open at BOOTLOADER_TOUCH_BAUD,
// then drop DTR) triggers it as well — see scripts/flash-rx.ps1.
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)itf;
    enum { LINE_IDLE, LINE_1, LINE_2, LINE_3 };
    static int state = LINE_IDLE;

    if (!dtr && s_last_bit_rate == BOOTLOADER_TOUCH_BAUD) {
        reset_to_bootloader();
    }

    if (dtr) {
        // Host opened the port — replay buffered boot output.
        // Zero timeout: this runs on the TinyUSB task, which must not block.
        if (cdc_lock(0)) {
            ring_drain_locked();
            cdc_unlock();
        }
    }

    if (!dtr && rts) {
        state = (state == LINE_IDLE) ? LINE_1 : LINE_IDLE;
    } else if (dtr && rts) {
        state = (state == LINE_1) ? LINE_2 : LINE_IDLE;
    } else if (dtr && !rts) {
        state = (state == LINE_2) ? LINE_3 : LINE_IDLE;
    } else { // !dtr && !rts
        if (state == LINE_3) {
            reset_to_bootloader();
        }
        state = LINE_IDLE;
    }
}

#endif // CFG_TUD_CDC
