#include "communication.h"

namespace communication {

const char *Communication::TAG = "Communication";

void Communication::setup() {
  this->send_buffer_ = (uint8_t *)malloc(this->buffer_size_);

  if (this->send_buffer_ == nullptr) {
    WCAF_LOG("Couldn't allocate buffer, out of memory");
    return;
  }

  this->interface_->set_buffer_size(this->buffer_size_);
  this->interface_->setup();
  this->interface_->on_data([this](const uint8_t *data, const uint8_t *addr) {
    // Get information from header
    uint8_t length = data[1];
    uint16_t crc_r = data[2] << 8 | data[3];
    uint16_t type = data[4] << 8 | data[5];

    // Check CRC
    auto crc = helpers::crc(data + HEADER_SIZE, length - HEADER_SIZE);

    if (crc != crc_r) {
      WCAF_LOG("Invalid CRC: Received '%02X', Calculated '%02X'", crc_r, crc);
      this->on_error_(data_error::CRC_ERROR);
      return;
    }

    WCAF_LOG("Received %u bytes from %s with type %u", length,
             helpers::mac_addr_to_string(addr), type);

    for (auto callback : this->recv_callbacks_) {
      if (callback->type == type || callback->type == 0)
        callback->lambda(data + HEADER_SIZE, length, addr);
    }
  });
  this->interface_->on_error([this](uint8_t error) { this->on_error_(error); });
}

void Communication::loop() { this->interface_->loop(); }

void Communication::send_message(const uint16_t type, const uint8_t *data,
                                 const uint8_t length, const uint8_t *addr) {
  if (length > this->buffer_size_ - HEADER_SIZE) {
    WCAF_LOG("Message is to large");
    return;
  }
  auto crc = helpers::crc(data, length);

  // Create header
  this->send_buffer_[0] = START_BYTE;
  this->send_buffer_[1] = length + HEADER_SIZE;
  this->send_buffer_[2] = crc >> 8;
  this->send_buffer_[3] = crc;
  this->send_buffer_[4] = type >> 8;
  this->send_buffer_[5] = type;

  memcpy(this->send_buffer_ + HEADER_SIZE, data, length);

  this->interface_->send(this->send_buffer_, addr);
}

void Communication::on_message(
    std::function<void(const uint8_t *data, const uint8_t length,
                       const uint8_t *addr)> &&lambda,
    uint16_t type) {
  auto tmp = (recv_callback_struct_ *)malloc(sizeof(recv_callback_struct_));

  if (tmp == nullptr) {
    WCAF_LOG("Could not register on_message callback, out of memory");
    return;
  }

  tmp->type = type;
  tmp->lambda = lambda;

  this->recv_callbacks_.push_back(tmp);
}

void Communication::on_error(std::function<void(uint8_t error)> &&lambda) {
  this->err_callbacks_.push_back(lambda);
}

void Communication::on_error_(uint8_t error) {
  WCAF_LOG("Communication error %u", error);
}

namespace helpers {

uint16_t crc(const uint8_t *data, uint8_t length) {
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

char mac_addr_buff_[18];

char *mac_addr_to_string(const uint8_t *mac_addr) {
  snprintf(mac_addr_buff_, sizeof(mac_addr_buff_),
           "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1],
           mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  return mac_addr_buff_;
}

}  // namespace helpers

}  // namespace communication