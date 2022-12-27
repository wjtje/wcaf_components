#pragma once
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <wcaf/components/interval/interval.h>
#include <wcaf/core/component.h>
#include <wcaf/core/log.h>
#include <wcaf/helpers/list.h>

#include "helpers.h"

using namespace wcaf;

namespace wirelessCommunication {

class WirelessCommunication;

extern WirelessCommunication *global_wireless_communication;

class WirelessCommunication : public Component {
 public:
  enum PairingStatus : uint8_t { UNPAIRED, PAIR_REQUESTED, PAIRED };
  enum DeviceClass : uint8_t { RECEIVER, SENDER };

  void setup();
  void loop();
  const char *get_tag() { return TAG; }
  static const char *TAG;

  void set_device_class(DeviceClass device_class) {
    this->device_class_ = device_class;
  }

  void on_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int length);
  void on_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {}

  // Static function
  static void on_recv_cb_st(const uint8_t *mac_addr, const uint8_t *data,
                            int length) {
    global_wireless_communication->on_recv_cb(mac_addr, data, length);
  }
  static void on_send_cb_st(const uint8_t *mac_addr,
                            esp_now_send_status_t status) {
    global_wireless_communication->on_send_cb(mac_addr, status);
  }

 protected:
  interval::Interval interval_;
  list::List<esp_now_peer_info_t *> peers_;

  DeviceClass device_class_{DeviceClass::SENDER};
  PairingStatus pairing_status_{PairingStatus::UNPAIRED};

  int channel_{0};
  uint8_t counter_{0};

  uint8_t buffer_[128];
  uint8_t buffer_pos_{0};

  void esp_now_init_();
};

}  // namespace wirelessCommunication
#endif
