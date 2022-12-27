#include "uartCommunication.h"

namespace communication {
namespace uart {

const char *Uart::TAG = "Uart";

void Uart::setup() {
  this->buffer_ = (uint8_t *)malloc(this->buffer_size_);
  if (this->buffer_ == nullptr) {
    WCAF_LOG("Couldn't allocate buffer, out of memory");
    return;
  }

  this->serial_->begin(115200);
}

void Uart::loop() {
  // Check last time data read
  if (this->is_receiving_ &&
      millis() - this->last_time_ > this->data_timeout_) {
    this->on_error_(data_error::TIMEOUT);
    this->reset_buffer_();
  }

  // Check if data is available
  if (this->serial_->available() <= 0) return;

  // Read byte
  int incomming_byte = this->serial_->read();
  if (incomming_byte < 0) {
    WCAF_LOG("Tried to read bytes, but there wasn't anything to read");
    this->on_error_(data_error::READ_ERROR);
    return;
  }

  // Update state
  this->is_receiving_ = true;
  this->last_time_ = millis();

  // Save to buffer
  this->buffer_[this->buffer_pos_] = (uint8_t)(unsigned int)incomming_byte;
  this->buffer_pos_++;

  // Check if start byte is correct
  if (this->buffer_[0] != START_BYTE) {
    this->reset_buffer_();
    return;
  }

  // DEBUG Serial communication
  // Serial.printf("%u: %02X\n", this->buffer_pos_ - 1,
  //               (unsigned int)incomming_byte);

  if (this->buffer_pos_ <= 1) return;

  // Check if the correct amount of bytes are received
  if (this->buffer_[1] == this->buffer_pos_ && this->buffer_pos_ > 0) {
    this->on_data_(this->buffer_, BROADCAST_ADDRESS);
    this->reset_buffer_();
    return;
  }

  // Check for buffer overflow
  if (this->buffer_pos_ >= 128) {
    WCAF_LOG("To much data");
    this->on_error_(data_error::LENGTH_ERROR);
    this->reset_buffer_();
    return;
  }
}

void Uart::send(const uint8_t *data, const uint8_t *addr) {
  this->serial_->write(data, data[1]);
}

void Uart::reset_buffer_() {
  this->buffer_pos_ = 0;
  this->is_receiving_ = false;
}

}  // namespace uart
}  // namespace communication
