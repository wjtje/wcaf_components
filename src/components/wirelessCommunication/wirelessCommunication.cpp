#include "wirelessCommunication.h"

#include <esp_wifi.h>

namespace wirelessCommunication {

const char *WirelessCommunication::TAG = "WireComm";
WirelessCommunication *global_wireless_communication = nullptr;
uint8_t broadcastAdress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void WirelessCommunication::setup() {
  global_wireless_communication = this;

  // Setup WIFI
  WiFi.mode(WIFI_STA);
  this->channel_ = WiFi.channel();
  WCAF_LOG("Active on channel: %i", this->channel_);

  // Setup ESP-NOW
  this->esp_now_init_();

  // Create peer data
  auto peer = (esp_now_peer_info_t *)malloc(sizeof(esp_now_peer_info_t));
  memset(peer, 0, sizeof(esp_now_peer_info_t));
  this->peers_.push_back(peer);

  for (int ii = 0; ii < 6; ++ii) {
    peer->peer_addr[ii] = (uint8_t)0xff;
  }

  peer->channel = 12;
  peer->encrypt = 0;

  auto err = esp_now_add_peer(peer);
  WCAF_LOG("State: %s", helpers::esp_err_to_string(err));

  // Create interval
  this->interval_.set_interval(1000);
  this->interval_.set_callback([this]() {
    WCAF_LOG("Sending message");
    auto err = esp_now_send((*this->peers_.begin())->peer_addr, &this->counter_,
                            sizeof(this->counter_));
    this->counter_++;
    WCAF_LOG("State: %s", helpers::esp_err_to_string(err));
  });
}

void WirelessCommunication::loop() { this->interval_.loop(); }

void WirelessCommunication::esp_now_init_() {
  if (esp_now_init() != 0) {
    WCAF_LOG("Failed to init ESP-NOW");
    return;
  }

  // ESP8266 - set self role

  esp_now_register_recv_cb(WirelessCommunication::on_recv_cb_st);
  esp_now_register_send_cb(WirelessCommunication::on_send_cb_st);
}

void WirelessCommunication::on_recv_cb(const uint8_t *mac_addr,
                                       const uint8_t *data, int length) {
  WCAF_LOG("Got message from: %s", helpers::mac_addr_to_string(mac_addr));
}

}  // namespace wirelessCommunication
