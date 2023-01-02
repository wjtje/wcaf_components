#include "communication.h"

namespace communication {

const char *Communication::TAG = "Communication";

void Communication::setup() {
  this->send_buffer_ = (uint8_t *)malloc(this->buffer_size_);

  if (this->send_buffer_ == nullptr) {
    WCAF_LOG("Couldn't allocate buffer, out of memory");
    return;
  }

  // Setup interface
  this->interface_->set_buffer_size(this->buffer_size_);
  this->interface_->setup();
#ifdef ARDUINO_AVR_UNO
  this->interface_->set_argument(this);
  this->interface_->on_data([](const uint8_t *data, const uint8_t *addr,
                               void *argument) {
    auto comm = (Communication *)argument;
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  this->interface_->on_data([this](const uint8_t *data, const uint8_t *addr) {
    auto comm = this;
#endif
    // Check for REQ and ACK
    if (data[0] == REQ_BYTE) {
      comm->send_byte_(ACK_BYTE);
      comm->is_receiving_ = true;
      return;
    } else if (data[0] == ACK_BYTE) {
      comm->is_sending_ = false;
      comm->interface_->send(comm->send_buffer_, comm->send_addr_);
      return;
    }

    comm->is_receiving_ = false;

    // Get information from header
    uint8_t length = data[1];
    uint16_t crc_r = data[2] << 8 | data[3];
    uint16_t type = data[4] << 8 | data[5];

    // Check CRC
    auto crc = helpers::crc(data + HEADER_SIZE, length - HEADER_SIZE);

    if (crc != crc_r) {
      WCAF_LOG("Invalid CRC: Received '%02X', Calculated '%02X'", crc_r, crc);
      comm->on_error_(data_error::CRC_ERROR);
      return;
    }

    WCAF_LOG("Received %u bytes from %s with type %u", length,
             helpers::mac_addr_to_string(addr), type);

    for (auto callback : comm->recv_callbacks_) {
      if (callback->type == type || callback->type == 0)
#ifdef ARDUINO_AVR_UNO
        callback->lambda(data + HEADER_SIZE, length, addr, callback->argument);
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
        callback->lambda(data + HEADER_SIZE, length, addr);
#endif
    }
  });

#ifdef ARDUINO_AVR_UNO
  this->interface_->on_error([](uint8_t error, void *argument) {
    auto comm = (Communication *)argument;
    comm->on_error_(error);
  });
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  this->interface_->on_error([this](uint8_t error) { this->on_error_(error); });
#endif

  // Setup REQ interval
  this->req_interval_ = new interval::Interval();
  this->req_interval_->set_interval(1);
#ifdef ARDUINO_AVR_UNO
  this->req_interval_->set_argument(this);
  this->req_interval_->set_callback([](void *argument) {
    auto comm = (Communication *)argument;
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  this->req_interval_->set_callback([this]() {
    auto comm = this;
#endif
    comm->send_byte_(REQ_BYTE);
  });
}

void Communication::loop() {
  this->interface_->loop();
  if (this->is_sending_) this->req_interval_->loop();
}

bool Communication::send_message(const uint16_t type, const uint8_t *data,
                                 const uint8_t length, const uint8_t *addr) {
  if (length > this->buffer_size_ - HEADER_SIZE) {
    WCAF_LOG("Message is to large");
    return false;
  }

  if (this->is_sending_) {
    WCAF_LOG("Still sending message");
    return false;
  }

  this->is_sending_ = true;

  auto crc = helpers::crc(data, length);

  // Create header
  this->send_buffer_[0] = START_BYTE;
  this->send_buffer_[1] = length + HEADER_SIZE;
  this->send_buffer_[2] = crc >> 8;
  this->send_buffer_[3] = crc;
  this->send_buffer_[4] = type >> 8;
  this->send_buffer_[5] = type;

  // Store data in buffers
  memcpy(this->send_buffer_ + HEADER_SIZE, data, length);
  memcpy(this->send_addr_, addr, 6);

  return true;
}

#ifdef ARDUINO_AVR_UNO
void Communication::on_message(uint16_t type, void *argument,
                               void (*lambda)(const uint8_t *data,
                                              const uint8_t length,
                                              const uint8_t *addr,
                                              void *argument)) {
  auto tmp = (recv_callback_struct_ *)malloc(sizeof(recv_callback_struct_));

  if (tmp == nullptr) {
    WCAF_LOG("Could not register on_message callback, out of memory");
    return;
  }

  tmp->argument = argument;
  tmp->type = type;
  tmp->lambda = lambda;

  this->recv_callbacks_.push_back(tmp);
}

void Communication::on_error(void (*lambda)(uint8_t error)) {
  this->err_callbacks_.push_back(lambda);
}
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
void Communication::on_message(
    uint16_t type, std::function<void(const uint8_t *data, const uint8_t length,
                                      const uint8_t *addr)> &&lambda) {
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
#endif

void Communication::on_error_(uint8_t error) {
  WCAF_LOG("Communication %s", helpers::error_to_string(error));

  for (auto callback : this->err_callbacks_) {
    callback(error);
  }
}

void Communication::send_byte_(const uint8_t byte) {
  auto message = (uint8_t *)malloc(2);
  message[0] = byte;
  message[1] = 1;

  this->interface_->send(message, this->send_addr_);

  delete message;
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

const char *error_to_string(uint8_t error) {
  switch (error) {
    case 0:
      return "data timeout";
    case 1:
      return "read error";
    case 2:
      return "crc error";
    case 3:
      return "incorrect length";

    default:
      return "unknown error";
  }
}

}  // namespace helpers

}  // namespace communication