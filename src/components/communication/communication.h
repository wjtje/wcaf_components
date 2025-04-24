#pragma once
#include <wcaf/components/interval/interval.h>
#include <wcaf/core/component.h>
#include <wcaf/core/log.h>
#include <wcaf/helpers/circular_buffer.h>
#include <wcaf/helpers/vector.h>

#if defined(ARDUINO_ESP32_DEV)
#include <functional>
#endif

using namespace wcaf;

namespace communication {

static const uint8_t HEADER_SIZE = 8;
static const uint8_t START_BYTE = 0x55;
static const uint8_t REQ_BYTE = 0x56;
static const uint8_t ACK_BYTE = 0x57;

enum data_error : uint8_t { TIMEOUT, READ_ERROR, CRC_ERROR, LENGTH_ERROR };

class CommunicationInterface;

class Communication : public Component {
 public:
  struct Message {
    uint8_t data[128];
  };
  using MessageQueue = CircularBuffer<Message, 5>;

  virtual void setup() override;
  virtual void loop() override;
  virtual const char *get_tag() override { return TAG; }
  static const char *TAG;

  /**
   * @brief Set the maximum millisecondes between the REQ and the actual
   * message.
   *
   * @param timeout
   */
  void set_receive_timeout(uint32_t timeout) {
    this->receive_timeout_ = timeout;
  }

  void set_communication_interface(CommunicationInterface *interface) {
    this->interface_ = interface;
  }

  MessageQueue &get_queue() { return this->message_queue_; }

  /**
   * @brief Send data using the communication interface to an other device.
   *
   * @param id A 4 byte number to indicate what kind of message you are sending.
   * @param data A pointer to data you want to send
   * @param length The length of data. The maximum is buffer size - header size
   * @return true The message is being send
   * @return false Couldn't send your message
   */
  bool send_message(const uint32_t id, const uint8_t *data,
                    const uint8_t length);
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  void on_message(uint32_t id, void *argument,
                  void (*lambda)(const uint8_t *data, const uint8_t length,
                                 void *argument));
  void on_error(void (*lambda)(uint8_t error));
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  void on_message(uint32_t id,
                  std::function<void(const uint8_t *data,
                                     const uint8_t length, )> &&lambda);
  void on_error(std::function<void(const uint8_t error)> &&lambda);
#endif

  bool is_receiving() {
    if (millis() - this->is_receiving_ > this->receive_timeout_)
      this->is_receiving_ = 0;
    return this->is_receiving_ != 0;
  }
  bool is_sending() { return !this->message_queue_.empty(); }
  uint32_t get_last_time_receiving() { return this->last_time_receiving_; }

 protected:
  void on_error_(uint8_t error);
  /**
   * @brief Send a single byte to the communication interface
   *
   * @param byte
   */
  void send_byte_(const uint8_t byte);

  CommunicationInterface *interface_;

  uint32_t last_req_{0};
  uint32_t last_time_receiving_{0};

  // Buffer
  MessageQueue message_queue_;

  // Communication states
  uint32_t is_receiving_{0};

  uint32_t receive_timeout_{10};
// Callbacks
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  struct recv_callback_struct_ {
    uint32_t id;
    void *argument;
    void (*lambda)(const uint8_t *data, const uint8_t length, void *argument);
  };
  Vector<recv_callback_struct_> recv_callbacks_;

  Vector<void (*)(uint8_t error)> err_callbacks_;
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  struct recv_callback_struct_ {
    uint32_t id;
    std::function<void(const uint8_t *data, const uint8_t length)> lambda;
  };
  Vector<recv_callback_struct_> recv_callbacks_;

  Vector<std::function<void(uint8_t error)> > err_callbacks_;
#endif
};

class CommunicationInterface : public Component {
 public:
  virtual void set_buffer_size(uint8_t size) = 0;
  virtual void send(const uint8_t *data) = 0;
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  void set_argument(void *argument) { this->argument_ = argument; }
  virtual void on_data(void (*lambda)(const uint8_t *data, void *argument)) = 0;
  virtual void on_error(void (*lambda)(uint8_t error, void *argument)) = 0;

 protected:
  void *argument_;
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ESP32_DEV)
  virtual void on_data(std::function<void(const uint8_t *data)> &&lambda) = 0;
  virtual void on_error(std::function<void(uint8_t error)> &&lambda) = 0;
#endif
};

namespace helpers {

uint16_t crc(const uint8_t *data, uint8_t length);
const char *error_to_string(uint8_t error);

}  // namespace helpers

}  // namespace communication
