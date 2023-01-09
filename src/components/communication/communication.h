#pragma once
#include <wcaf/components/interval/interval.h>
#include <wcaf/core/component.h>
#include <wcaf/core/log.h>
#include <wcaf/helpers/list.h>

#if defined(ARDUINO_ARCH_ESP8266)
#include <list>
#endif

#if defined(ARDUINO_ESP32_DEV)
#include <functional>
#include <list>
#endif

using namespace wcaf;

namespace communication {

static const uint8_t BROADCAST_ADDRESS[8] = {0xFF, 0xFF, 0xFF,
                                             0xFF, 0xFF, 0xFF};
static const uint8_t HEADER_SIZE = 8;
static const uint8_t START_BYTE = 0x55;
static const uint8_t REQ_BYTE = 0x56;
static const uint8_t ACK_BYTE = 0x57;

enum data_error : uint8_t { TIMEOUT, READ_ERROR, CRC_ERROR, LENGTH_ERROR };

class CommunicationInterface;

class Communication : public Component {
 public:
  void setup();
  void loop();
  const char *get_tag() { return TAG; }
  static const char *TAG;

  /**
   * @brief Sets the maximum size of a single message (the first 8 bytes are
   * used by the header).
   *
   * @param size
   */
  void max_message_length(uint8_t size) { this->max_message_length_ = size; };

  /**
   * @brief Set the maximum millisecondes between the REQ and the actual
   * message.
   *
   * @param timeout
   */
  void set_receive_timeout(uint32_t timeout) {
    this->receive_timeout_ = timeout;
  }

  /**
   * @brief Sets the maximum amount of message inside the queue at any given
   * moment.
   *
   * @param size
   */
  void set_message_queue(uint8_t size) { this->max_queue_length_ = size; }

  void set_communication_interface(CommunicationInterface *interface) {
    this->interface_ = interface;
  }

  /**
   * @brief Send data using the communication interface to an other device.
   *
   * @param type 2 bytes of data indicating the type of data you're sending,
   * this can be anything you want (excluding 0)
   * @param data A pointer to data you want to send
   * @param length The length of data. The maximum is buffer size - header size
   * @param addr The address receiving the data, defaults to broadcast
   * @return true The message is being send
   * @return false Couldn't send your message
   */
  bool send_message(const uint16_t type, const uint8_t *data,
                    const uint8_t length,
                    const uint8_t *addr = BROADCAST_ADDRESS);
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  void on_message(uint16_t type, void *argument,
                  void (*lambda)(const uint8_t *data, const uint8_t length,
                                 const uint8_t *addr, void *argument));
  void on_error(void (*lambda)(uint8_t error));
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  void on_message(uint16_t type,
                  std::function<void(const uint8_t *data, const uint8_t length,
                                     const uint8_t *addr)> &&lambda);
  void on_error(std::function<void(const uint8_t error)> &&lambda);
#endif

  bool is_receiving() {
    if (!(millis() - this->is_receiving_ > this->receive_timeout_))
      this->is_receiving_ = 0;
    return this->is_receiving_ != 0;
  }
  bool is_sending() { return this->message_queue_.size() > 0; }

 protected:
  void on_error_(uint8_t error);
  /**
   * @brief Send a single byte to the communication interface
   *
   * @param byte
   */
  void send_byte_(const uint8_t byte);

  CommunicationInterface *interface_;
  interval::Interval *req_interval_;

  // Buffers
  uint8_t max_message_length_{128};
  uint8_t max_queue_length_{5};

  struct message_ {
    uint8_t addr[6];
    uint8_t *data;
  };
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  list::List<message_ *> message_queue_;
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  std::list<message_ *> message_queue_;
#endif

  // Communication states
  uint32_t is_receiving_{0};

  uint32_t receive_timeout_{10};
// Callbacks
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  struct recv_callback_struct_ {
    uint16_t type;
    void *argument;
    void (*lambda)(const uint8_t *data, const uint8_t length,
                   const uint8_t *addr, void *argument);
  };
  list::List<recv_callback_struct_ *> recv_callbacks_;

  list::List<void (*)(uint8_t error)> err_callbacks_;
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  struct recv_callback_struct_ {
    uint16_t type;
    std::function<void(const uint8_t *data, const uint8_t length,
                       const uint8_t *addr)>
        lambda;
  };
  list::List<recv_callback_struct_ *> recv_callbacks_;

  list::List<std::function<void(uint8_t error)> > err_callbacks_;
#endif
};

class CommunicationInterface : public Component {
 public:
  virtual void set_buffer_size(uint8_t size);
  virtual void send(const uint8_t *data, const uint8_t *addr);
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  void set_argument(void *argument) { this->argument_ = argument; }
  virtual void on_data(void (*lambda)(const uint8_t *data, const uint8_t *addr,
                                      void *argument));
  virtual void on_error(void (*lambda)(uint8_t error, void *argument));

 protected:
  void *argument_;
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  virtual void on_data(
      std::function<void(const uint8_t *data, const uint8_t *addr)> &&lambda);
  virtual void on_error(std::function<void(uint8_t error)> &&lambda);
#endif
};

namespace helpers {

uint16_t crc(const uint8_t *data, uint8_t length);
char *mac_addr_to_string(const uint8_t *mac_addr);
const char *error_to_string(uint8_t error);

}  // namespace helpers

}  // namespace communication
