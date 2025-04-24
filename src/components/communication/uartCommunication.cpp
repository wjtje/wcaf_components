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

  if (this->init_serial_) {
    this->serial_->begin(this->speed_);
    delay(10);  // This decreases the change of a out of sync situation
  }
}

void Uart::loop() {
  const uint32_t start_time = millis();

  // Check last time data read
  if (this->is_receiving_ &&
      start_time - this->last_time_ > this->data_timeout_) {
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
    this->on_error_(data_error::TIMEOUT, this->argument_);
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
    this->on_error_(data_error::TIMEOUT);
#endif
    this->reset_buffer_();
  }

  while (this->serial_->available() && (millis() - start_time < 50)) {
    const uint32_t now = millis();

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
    this->last_time_ = now;

    // Save to buffer
    this->buffer_[this->buffer_pos_] = (uint8_t)(unsigned int)incomming_byte;
    this->buffer_pos_++;

    // Check for REQ and ACK byes
    if (this->buffer_[0] == REQ_BYTE || this->buffer_[0] == ACK_BYTE) {
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
      this->on_data_(this->buffer_, this->argument_);
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
      this->on_data_(this->buffer_);
#endif
      this->reset_buffer_();
      continue;
    }

    // Check if start byte is correct
    if (this->buffer_[0] != START_BYTE) {
      this->reset_buffer_();

      if (++invalid_byte_count_ > 10) {
        invalid_byte_count_ = 0;
        // There is a chance that the UART is out of sync.
        // This should not happen, but we restart the serial just is case.
        this->serial_->end();
        delay(1);
        this->serial_->begin(this->speed_);
      }

      continue;
    }

    if (this->buffer_pos_ <= 1) continue;

    // Check if the correct amount of bytes are received
    if (this->buffer_[1] == this->buffer_pos_ && this->buffer_pos_ > 0) {
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
      this->on_data_(this->buffer_, this->argument_);
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
      this->on_data_(this->buffer_);
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

void Uart::send(const uint8_t *data) {
  // Remove all REQ bytes that are in the recv buffer
  if (data[1] == 1 && data[0] == ACK_BYTE) {
    while (this->serial_->peek() == REQ_BYTE) this->serial_->read();
  }

  this->serial_->write(data, data[1]);
  this->serial_->flush();
}

void Uart::reset_buffer_() {
  this->buffer_pos_ = 0;
  this->is_receiving_ = false;
}

}  // namespace uart
}  // namespace communication
