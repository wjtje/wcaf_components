#include "uartCommunication.h"

namespace uartCommunication {

const char* UartCommunication::TAG = "UartComm";
uint8_t UartCommunication::header_length_{8};
uint8_t UartCommunication::start_byte_{0x55};

void UartCommunication::setup() { this->serial_->begin(this->speed_); }

void UartCommunication::loop() {
  // Check last time data read
  if (this->is_receiving_ &&
      millis() - this->last_time_ > this->data_timeout_) {
    WCAF_LOG("Data timeout");
    for (auto callback : this->error_callback_) {
      callback(error_type_::TIMEOUT);
    }
    this->reset_buffer_();
  }

  // Check if data is available
  if (this->serial_->available() <= 0) return;

  // Read byte
  int incomming_byte = this->serial_->read();
  if (incomming_byte < 0) {
    WCAF_LOG("Tried to read bytes, but there wasn't anything to read");
    for (auto callback : this->error_callback_) {
      callback(error_type_::READ_ERR);
    }
    return;
  }

  // Update state
  this->is_receiving_ = true;
  this->last_time_ = millis();

  // Save to buffer
  this->buffer_[this->buffer_pos_] = (uint8_t)(unsigned int)incomming_byte;
  this->buffer_pos_++;

  // Check if start byte is correct
  if (this->buffer_[0] != this->start_byte_) {
    this->reset_buffer_();
    return;
  }

  // DEBUG Serial communication
  // Serial.printf("%u: %02X\n", this->buffer_pos_ - 1,
  //               (unsigned int)incomming_byte);

  if (this->buffer_pos_ <= 1) return;

  // Check if the correct amount of bytes are received
  if (this->buffer_[1] == this->buffer_pos_ && this->buffer_pos_ > 0) {
    uint8_t length = this->buffer_[1];
    uint16_t crc_r = this->buffer_[2] << 8 | this->buffer_[3];
    uint16_t type = this->buffer_[4] << 8 | this->buffer_[5];

    // Calculate crc
    auto crc = this->crc_(this->buffer_ + this->header_length_,
                          length - this->header_length_);

    // Check crc
    if (crc != crc_r) {
      WCAF_LOG("Invalid CRC: Received '%02X', Calculated '%02X'", crc_r, crc);
      for (auto callback : this->error_callback_) {
        callback(error_type_::CRC);
      }
    } else {
      WCAF_LOG("Received %u bytes with type %u", length, type);

      // Send data to callbacks
      for (auto callback : this->receive_callback_) {
        if (callback->type == type) {
          callback->lambda(this->buffer_ + this->header_length_,
                           length - this->header_length_);
        }
      }
    }

    this->reset_buffer_();
    return;
  }

  // Check for buffer overflow
  if (this->buffer_pos_ >= 128) {
    WCAF_LOG("To much data");
    for (auto callback : this->error_callback_) {
      callback(error_type_::LENGTH_ERR);
    }
    this->reset_buffer_();
    return;
  }
}

void UartCommunication::send(uint16_t type, uint8_t* data, uint8_t length) {
  // Check if buffer is currectly not used
  uint8_t* buffer = nullptr;

  if (this->is_receiving_) {
    buffer = (uint8_t*)malloc(128);
  } else {
    buffer = this->buffer_;
  }

  if (buffer == nullptr) return;

  auto crc = this->crc_(data, length);

  // Create message header
  buffer[0] = this->start_byte_;
  buffer[1] = length + this->header_length_;
  buffer[2] = crc >> 8;
  buffer[3] = crc;
  buffer[4] = type >> 8;
  buffer[5] = type;

  // Copy data
  memcpy(buffer + this->header_length_, data, length);

  // Send the data
  this->serial_->write(buffer, buffer[0]);
  if (this->is_receiving_) delete buffer;
}

void UartCommunication::reset_buffer_() {
  this->buffer_pos_ = 0;
  this->is_receiving_ = false;
}

uint16_t UartCommunication::crc_(uint8_t* data, uint8_t length) {
  uint8_t x;
  uint16_t crc = 0xFFFF;

  while (length--) {
    x = crc >> 8 ^ *data++;
    x ^= x >> 4;
    crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x << 5)) ^
          ((uint16_t)x);
  }

  return crc;
}

}  // namespace uartCommunication
