#include "communication.h"

namespace communication {

const char *Communication::TAG = "Communication";
// char buff_[512];  // Buffer is used for debug

void Communication::setup() {
  // Setup interface
  this->interface_->set_buffer_size(sizeof(t_Message::data));
  this->interface_->setup();
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  this->interface_->set_argument(this);
  this->interface_->on_data([](const uint8_t *data, const uint8_t *addr,
                               void *argument) {
    Communication *comm = (Communication *)argument;
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  this->interface_->on_data([this](const uint8_t *data, const uint8_t *addr) {
    Communication *comm = this;
#endif
    comm->last_time_receiving_ = millis();
    // Calculate message length
    uint8_t length = (data[0] == REQ_BYTE || data[0] == ACK_BYTE) ? 1 : data[1];

    // DEBUG
    // for (int i = 0; i < length; i++) {
    //   snprintf(buff_ + (3 * i), 4, "%02x ", data[i]);
    // }
    // WCAF_LOG_DEFAULT("%s", buff_);

    // Check for REQ and ACK
    if (data[0] == REQ_BYTE) {
      if (!(comm->last_time_receiving_ - comm->is_receiving_ >
            comm->receive_timeout_))
        return;
      // WCAF_LOG_DEFAULT("REQ");
      comm->send_byte_(ACK_BYTE);
      comm->is_receiving_ = comm->last_time_receiving_;
      return;
    } else if (data[0] == ACK_BYTE) {
      if (!comm->message_queue_.empty() && !comm->is_receiving()) {
        const t_Message &message = comm->message_queue_.dpop();
        comm->interface_->send(message.data, message.addr);
      } else {
        // WCAF_LOG_WARNING("Received ACK, but not sending anything");
      }

      return;
    }

    comm->is_receiving_ = 0;

    // Get information from header
    uint16_t crc_r = data[2] << 8 | data[3];
    uint32_t id = (uint32_t)(data[4]) << 24 | (uint32_t)(data[5]) << 16 |
                  data[6] << 8 | data[7];

    // Check CRC
    uint16_t crc = helpers::crc(data + HEADER_SIZE, length - HEADER_SIZE);

    if (crc != crc_r) {
      WCAF_LOG_ERROR("Invalid CRC: Received '%02X', Calculated '%02X'", crc_r,
                     crc);
      comm->on_error_(data_error::CRC_ERROR);
      return;
    }

    // WCAF_LOG_DEFAULT("Received %u bytes from %s with id %lu", length,
    //                  helpers::mac_addr_to_string(addr), id);

    for (recv_callback_struct_ &callback : comm->recv_callbacks_) {
      if (callback.id == id || callback.id == 0) {
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
        callback.lambda(data + HEADER_SIZE, length - HEADER_SIZE, addr,
                        callback.argument);
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
        callback.lambda(data + HEADER_SIZE, length - HEADER_SIZE, addr);
#endif
      }
    }
  });

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  this->interface_->on_error([](uint8_t error, void *argument) {
    Communication *comm = (Communication *)argument;
    comm->on_error_(error);
  });
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  this->interface_->on_error([this](uint8_t error) { this->on_error_(error); });
#endif
}

void Communication::loop() {
  const uint32_t now = millis();
  this->interface_->loop();

  if (!this->message_queue_.empty() && (now - this->last_req_ >= 1)) {
    this->send_byte_(REQ_BYTE);
  }
}

bool Communication::send_message(const uint32_t id, const uint8_t *data,
                                 const uint8_t length, const uint8_t *addr) {
  if (length > sizeof(t_Message::data) - HEADER_SIZE) {
    WCAF_LOG_WARNING("Message is to large");
    return false;
  }

  if (this->message_queue_.full()) {
    WCAF_LOG_WARNING("Message queue is full");
    return false;
  }

  t_Message message;
  memcpy(message.addr, addr, sizeof(message.addr));

  // Create header
  uint16_t crc = helpers::crc(data, length);

  message.data[0] = START_BYTE;
  message.data[1] = length + HEADER_SIZE;
  message.data[2] = crc >> 8;
  message.data[3] = crc;
  message.data[4] = id >> 24;
  message.data[5] = id >> 16;
  message.data[6] = id >> 8;
  message.data[7] = id;

  // Store data in buffers
  memcpy(message.data + HEADER_SIZE, data, length);

  this->message_queue_.push(message);

  return true;
}

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
void Communication::on_message(uint32_t id, void *argument,
                               void (*lambda)(const uint8_t *data,
                                              const uint8_t length,
                                              const uint8_t *addr,
                                              void *argument)) {
  recv_callback_struct_ tmp = {
      .id = id, .argument = argument, .lambda = lambda};
  this->recv_callbacks_.push_back(tmp);
}

void Communication::on_error(void (*lambda)(uint8_t error)) {
  this->err_callbacks_.push_back(lambda);
}
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
void Communication::on_message(
    uint32_t id, std::function<void(const uint8_t *data, const uint8_t length,
                                    const uint8_t *addr)> &&lambda) {
  recv_callback_struct_ tmp = {.id = id, .lambda = lambda};
  this->recv_callbacks_.push_back(tmp);
}

void Communication::on_error(std::function<void(uint8_t error)> &&lambda) {
  this->err_callbacks_.push_back(lambda);
}
#endif

void Communication::on_error_(uint8_t error) {
  WCAF_LOG_ERROR("Communication %s", helpers::error_to_string(error));

  for (auto callback : this->err_callbacks_) {
    callback(error);
  }
}

void Communication::send_byte_(const uint8_t byte) {
  uint8_t message[2] = {byte, 1};
  uint8_t addr[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

  // Try to take the addr of the first device in the queue
  if (!this->message_queue_.empty())
    memcpy(addr, this->message_queue_.front().addr, sizeof(addr));

  this->interface_->send(message, addr);
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