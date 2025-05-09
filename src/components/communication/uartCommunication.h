#pragma once
#include "communication.h"

namespace communication {
namespace uart {

class Uart : public CommunicationInterface {
 public:
  virtual void setup() override;
  virtual void loop() override;
  virtual const char *get_tag() override { return TAG; }
  static const char *TAG;

  virtual void set_buffer_size(uint8_t size) override {
    this->buffer_size_ = size;
  }
  void set_serial(HardwareSerial *serial) { this->serial_ = serial; }
  void set_speed(unsigned long speed) { this->speed_ = speed; }
  void set_init_serial(bool init_serial) { this->init_serial_ = init_serial; }

  virtual void send(const uint8_t *data) override;
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  virtual void on_data(void (*lambda)(const uint8_t *data,
                                      void *argument)) override {
    this->on_data_ = lambda;
  }
  virtual void on_error(void (*lambda)(uint8_t error,
                                       void *argument)) override {
    this->on_error_ = lambda;
  }
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  void on_data(std::function<void(const uint8_t *data)> &&lambda) {
    this->on_data_ = lambda;
  }
  void on_error(std::function<void(uint8_t error)> &&lambda) {
    this->on_error_ = lambda;
  }
#endif

 protected:
  void reset_buffer_();

  HardwareSerial *serial_;
  unsigned long speed_{115200};
  bool init_serial_{true};
  uint8_t invalid_byte_count_{0};

  // Buffer
  uint8_t buffer_size_;
  uint8_t *buffer_;
  uint8_t buffer_pos_{0};

  bool is_receiving_{false};
  uint32_t last_time_{0};
  uint16_t data_timeout_{10};

  // Callbacks
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  void (*on_data_)(const uint8_t *data, void *argument);
  void (*on_error_)(uint8_t error, void *argument);
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  std::function<void(const uint8_t *data)> on_data_;
  std::function<void(uint8_t error)> on_error_;
#endif
};

}  // namespace uart
}  // namespace communication