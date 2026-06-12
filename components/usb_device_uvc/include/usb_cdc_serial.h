#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// True once TinyUSB has been started (uvc_device_init), i.e. the CDC COM port
// exists on the bus. Before that, the chip's USB-Serial/JTAG port is active
// instead and callers should fall back to it.
bool usb_cdc_active(void);

// Increments every TinyUSB task pass (>=10Hz when healthy). If it stops
// advancing while usb_cdc_active(), the USB stack is wedged — used by the
// RX watchdog to force a coredump for diagnosis.
extern volatile uint32_t g_tud_task_heartbeat;

// Mirror ESP_LOG output (incl. the boot logo and startup logs) to the CDC
// port. Output is buffered from the moment this is called and replayed when
// the host opens the port; the original console keeps working. Call once,
// early in app_main, before other tasks start logging.
void usb_cdc_console_mirror_init(void);

// Read up to buf_size bytes from the CDC port, waiting up to timeout_ms for
// data to arrive. Returns the number of bytes read, 0 on timeout, or -1 if
// the CDC port is not active.
int usb_cdc_read(uint8_t *buf, size_t buf_size, uint32_t timeout_ms);

// Write len bytes to the CDC port. Data is dropped if the host does not have
// the port open. Returns the number of bytes written, or -1 if the CDC port
// is not active or not open on the host side.
int usb_cdc_write(const void *buf, size_t len);

#ifdef __cplusplus
}
#endif
