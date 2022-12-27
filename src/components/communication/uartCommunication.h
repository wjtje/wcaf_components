#pragma once
#include "communication.h"

namespace communication {
namespace uart {

class Uart : public CommunicationInterface {
 public:
  void setup();
  void loop();
  const char *get_tag() { return TAG; }
  static const char *TAG;

  void set_buffer_size(uint8_t size) { this->buffer_size_ = size; }

  void send(const uint8_t *data, const uint8_t *addr);
  void on_data(
      std::function<void(const uint8_t *data, const uint8_t *addr)> &&lambda) {
    this->on_data_ = lambda;
  }
  void on_error(std::function<void(uint8_t error)> &&lambda) {
    this->on_error_ = lambda;
  }

 protected:
  void reset_buffer_();

  HardwareSerial *serial_{&Serial2};
  unsigned long speed_;

  // Buffer
  uint8_t buffer_size_;
  uint8_t *buffer_;
  uint8_t buffer_pos_{0};

  bool is_receiving_{false};
  uint32_t last_time_{0};
  uint16_t data_timeout_{10};

  // Callbacks
  std::function<void(const uint8_t *data, const uint8_t *addr)> on_data_;
  std::function<void(uint8_t error)> on_error_;
};

}  // namespace uart
}  // namespace communication