#pragma once

#include <Arduino.h>
#include <wcaf/core/component.h>
#include <wcaf/core/log.h>
#include <wcaf/helpers/list.h>

#if defined(ARDUINO_ESP32_DEV)
#include <functional>
#endif

using namespace wcaf;

namespace uartCommunication {

class UartCommunication : public Component {
 public:
  void setup();
  void loop();
  const char *get_tag() { return TAG; }
  static const char *TAG;

  void set_serial(HardwareSerial *serial) { this->serial_ = serial; }
  void set_speed(unsigned long speed) { this->speed_ = speed; }

  void send(uint16_t type, uint8_t *data, uint8_t length);
#if defined(ARDUINO_AVR_UNO)
  void on_message(uint16_t type, void (*lambda)(uint8_t *, uint8_t)) {
    auto tmp =
        (receive_callback_struct_ *)malloc(sizeof(receive_callback_struct_));
    tmp->type = type;
    tmp->lambda = lambda;
    this->receive_callback_.push_back(tmp);
  }
  void on_error(void (*lambda)(uint8_t)) {
    this->error_callback_.push_back(lambda);
  }
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  void on_message(uint16_t type,
                  std::function<void(uint8_t *, uint8_t)> &&lambda) {
    auto tmp =
        (receive_callback_struct_ *)malloc(sizeof(receive_callback_struct_));
    tmp->type = type;
    tmp->lambda = lambda;
    this->receive_callback_.push_back(tmp);
  }
  void on_error(std::function<void(uint8_t)> &&lambda) {
    this->error_callback_.push_back(lambda);
  }
#endif

 protected:
  void reset_buffer_();
  uint16_t crc_(uint8_t *data, uint8_t length);

  HardwareSerial *serial_;
  unsigned long speed_;

  bool is_receiving_{false};
  uint32_t last_time_{0};
  uint16_t data_timeout_{10};

  uint8_t buffer_[128];
  uint8_t buffer_pos_{0};

  static uint8_t header_length_;
  static uint8_t start_byte_;
  // 0 = Start bytes (0x55)
  // 1 = Message length
  // 2 = CRC top 8 bits
  // 3 = CRC bottom 8 bits
  // 4 = Type top 8 bits
  // 5 = Type bottom 8 bits

#if defined(ARDUINO_AVR_UNO)
  struct receive_callback_struct_ {
    uint16_t type;
    void (*lambda)(uint8_t *, uint8_t);
  };
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  struct receive_callback_struct_ {
    uint16_t type;
    std::function<void(uint8_t *, uint8_t)> lambda;
  };
#endif
  list::List<receive_callback_struct_ *> receive_callback_;

  enum error_type_ : uint8_t { TIMEOUT, READ_ERR, CRC, LENGTH_ERR };
#if defined(ARDUINO_AVR_UNO)
  list::List<void (*)(uint8_t)> error_callback_;
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  list::List<std::function<void(uint8_t)> > error_callback_;
#endif
};

}  // namespace uartCommunication