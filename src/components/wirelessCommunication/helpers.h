#pragma once
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)

#include <Arduino.h>
#include <esp_now.h>

namespace wirelessCommunication {
namespace helpers {

char *mac_addr_to_string(const uint8_t *mac_addr);

const char *esp_err_to_string(int32_t err);

}  // namespace helpers
}  // namespace wirelessCommunication
#endif