#pragma once
#include <wcaf/core/component.h>
#include <wcaf/core/log.h>
#include <wcaf/helpers/list.h>

#include <functional>

using namespace wcaf;

namespace communication {

static const uint8_t BROADCAST_ADDRESS[8] = {0xFF, 0xFF, 0xFF,
                                             0xFF, 0xFF, 0xFF};
static const uint8_t HEADER_SIZE = 8;
static const uint8_t START_BYTE = 0x55;

enum data_error : uint8_t { TIMEOUT, READ_ERROR, CRC_ERROR, LENGTH_ERROR };

class CommunicationInterface;

class Communication : public Component {
 public:
  void setup();
  void loop();
  const char *get_tag() { return TAG; }
  static const char *TAG;

  /**
   * @brief Set the size of the receive and send buffer.
   * The max message length is calculate by the buffer size minus 8 bytes
   * for the header.
   *
   * @param size
   */
  void set_buffer_size(uint8_t size) { this->buffer_size_ = size; };

  void set_communication_interface(CommunicationInterface *interface) {
    this->interface_ = interface;
  }

  void send_message(const uint16_t type, const uint8_t *data,
                    const uint8_t length,
                    const uint8_t *addr = BROADCAST_ADDRESS);
  void on_message(std::function<void(const uint8_t *data, const uint8_t length,
                                     const uint8_t *addr)> &&lambda,
                  uint16_t type = 0);
  void on_error(std::function<void(const uint8_t error)> &&lambda);

 protected:
  void on_error_(uint8_t error);

  CommunicationInterface *interface_;

  // Buffers
  uint8_t buffer_size_{128};

  uint8_t *send_buffer_;
  uint8_t send_buffer_pos_{0};

  // Callbacks
  struct recv_callback_struct_ {
    uint16_t type;
    std::function<void(const uint8_t *data, const uint8_t length,
                       const uint8_t *addr)>
        lambda;
  };
  list::List<recv_callback_struct_ *> recv_callbacks_;

  list::List<std::function<void(uint8_t error)>> err_callbacks_;
};

class CommunicationInterface : public Component {
 public:
  virtual void set_buffer_size(uint8_t size);
  virtual void send(const uint8_t *data, const uint8_t *addr);
  virtual void on_data(
      std::function<void(const uint8_t *data, const uint8_t *addr)> &&lambda);
  virtual void on_error(std::function<void(uint8_t error)> &&lambda);
};

namespace helpers {

uint16_t crc(const uint8_t *data, uint8_t length);
char *mac_addr_to_string(const uint8_t *mac_addr);

}  // namespace helpers

}  // namespace communication
