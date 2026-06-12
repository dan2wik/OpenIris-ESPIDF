idf_component_register(SRCS usb_device_uvc.c
                    INCLUDE_DIRS "include"
                    REQUIRES usb esp_timer)

idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)

target_include_directories(${tusb_lib} PUBLIC "${COMPONENT_DIR}/tusb" "${COMPONENT_DIR}/include")
target_sources(${tusb_lib} PUBLIC "${COMPONENT_DIR}/tusb/usb_descriptors.c"
                                  "${COMPONENT_DIR}/tusb/usb_cdc_serial.c")

# Local override of espressif/usb_device_uvc 1.1.0 that adds a CDC-ACM interface
# to the composite device. Version macros were set by cmake_utilities upstream.
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    USB_DEVICE_UVC_VER_MAJOR=1
    USB_DEVICE_UVC_VER_MINOR=1
    USB_DEVICE_UVC_VER_PATCH=0)
