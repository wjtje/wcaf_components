#include "helpers.h"

namespace wirelessCommunication {
namespace helpers {

char mac_addr_buff_[18];

char *mac_addr_to_string(const uint8_t *mac_addr) {
  snprintf(mac_addr_buff_, sizeof(mac_addr_buff_),
           "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1],
           mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  return mac_addr_buff_;
}

const char *esp_err_to_string(int32_t err) {
  switch (err) {
    case ESP_OK:
      return "OK";

    case ESP_ERR_NO_MEM:
    case ESP_ERR_ESPNOW_NO_MEM:
      return "Out of memory";

    case ESP_ERR_INVALID_ARG:
    case ESP_ERR_ESPNOW_ARG:
      return "Invalid argument";

    case ESP_ERR_INVALID_STATE:
      return "Invalid state";

    case ESP_ERR_INVALID_SIZE:
      return "Invalid size";

    case ESP_ERR_NOT_FOUND:
      return "Requested resource not found";

    case ESP_ERR_NOT_SUPPORTED:
      return "Operation or feature not supported";

    case ESP_ERR_TIMEOUT:
      return "Operation timed out";

    case ESP_ERR_INVALID_RESPONSE:
      return "Received response was invalid";

    case ESP_ERR_INVALID_CRC:
      return "CRC or checksum was invalid";

    case ESP_ERR_INVALID_VERSION:
      return "Version was invalid";

    case ESP_ERR_INVALID_MAC:
      return "MAC address was invalid";

    case ESP_ERR_ESPNOW_NOT_INIT:
      return "ESPNOW is not initialized.";

    case ESP_ERR_ESPNOW_FULL:
      return "ESPNOW peer list is full";

    case ESP_ERR_ESPNOW_NOT_FOUND:
      return "ESPNOW peer is not found";

    case ESP_ERR_ESPNOW_EXIST:
      return "ESPNOW peer has existed";

    default:
      return "Unknown error";
  }
}

}  // namespace helpers
}  // namespace wirelessCommunication