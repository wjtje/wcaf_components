#include "uartCommunication.h"

namespace communication {
namespace uart {

const char *Uart::TAG = "Uart";

void Uart::setup() {
  this->buffer_ = (uint8_t *)malloc(this->buffer_size_);
  if (this->buffer_ == nullptr) {
    WCAF_LOG_ERROR("Couldn't allocate buffer, out of memory");
    return;
  }

  if (this->init_serial_) this->serial_->begin(this->speed_);
}

void Uart::loop() {
  // Check last time data read
  if (this->is_receiving_ &&
      millis() - this->last_time_ > this->data_timeout_) {
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
    this->on_error_(data_error::TIMEOUT, this->argument_);
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
    this->on_error_(data_error::TIMEOUT);
#endif
    this->reset_buffer_();
  }

  // Check if data is available
  unsigned long end_time = millis() + 150;
  while (this->serial_->available()) {
    // Stop after 150ms
    if (millis() > end_time) break;

    // Read byte
    int incomming_byte = this->serial_->read();
    if (incomming_byte < 0) {
      WCAF_LOG_WARNING(
          "Tried to read bytes, but there wasn't anything to read");
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
      this->on_error_(data_error::READ_ERROR, this->argument_);
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
      this->on_error_(data_error::READ_ERROR);
#endif
      return;
    }

    // Update state
    this->is_receiving_ = true;
    this->last_time_ = millis();

    // Save to buffer
    this->buffer_[this->buffer_pos_] = (uint8_t)(unsigned int)incomming_byte;
    this->buffer_pos_++;

    // Check for REQ and ACK byes
    if (this->buffer_[0] == REQ_BYTE || this->buffer_[0] == ACK_BYTE) {
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
      this->on_data_(this->buffer_, BROADCAST_ADDRESS, this->argument_);
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
      this->on_data_(this->buffer_, BROADCAST_ADDRESS);
#endif
      this->reset_buffer_();
      continue;
    }

    // Check if start byte is correct
    if (this->buffer_[0] != START_BYTE) {
      this->reset_buffer_();
      continue;
    }

    if (this->buffer_pos_ <= 1) continue;

    // Check if the correct amount of bytes are received
    if (this->buffer_[1] == this->buffer_pos_ && this->buffer_pos_ > 0) {
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
      this->on_data_(this->buffer_, BROADCAST_ADDRESS, this->argument_);
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
      this->on_data_(this->buffer_, BROADCAST_ADDRESS);
#endif
      this->reset_buffer_();
      continue;
    }

    // Check for buffer overflow
    if (this->buffer_pos_ >= this->buffer_size_) {
      WCAF_LOG_ERROR("To much data");
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
      this->on_error_(data_error::LENGTH_ERROR, this->argument_);
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
      this->on_error_(data_error::LENGTH_ERROR);
#endif
      this->reset_buffer_();
      continue;
    }
  }
}

void Uart::send(const uint8_t *data, const uint8_t *addr) {
  this->serial_->write(data, data[1]);
  this->serial_->flush();
}

void Uart::reset_buffer_() {
  this->buffer_pos_ = 0;
  this->is_receiving_ = false;
}

}  // namespace uart
}  // namespace communication
