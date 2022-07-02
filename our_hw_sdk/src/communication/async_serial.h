/**
* @file        AsyncSerialDevice.h
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-5-30
* @description serial asynchronous communication
*
* copy from https://github.com/westonrobot/wrp_sdk
*/

#ifndef OUR_HW_SDK_ASYNC_SERIAL_H
#define OUR_HW_SDK_ASYNC_SERIAL_H

#include <cstdint>
#include <memory>
#include <array>
#include "communication/async_listener_interface.hpp"
#include "communication/ring_buffer.hpp"
#include "hal/protocol/serial_port_common_def.h"

namespace communication {
struct SerialDataFrame {
  uint8_t *bytes;
  size_t  length;
};

class AsyncSerialDevice : public AsyncListenerInterface, public std::enable_shared_from_this<AsyncSerialDevice> {
 public:
  using ReceiveCallback =
      std::function<void(uint8_t *data, const size_t bufsize, size_t len)>;

 public:
  explicit AsyncSerialDevice(const SerialConfig serial_config);

  void CloseDevice() override;
  bool IsOpened() override;

  void SetReceiveCallback(ReceiveCallback const &cb) { rcv_cb_ = cb; }
  bool SendBytes(const uint8_t *bytes, size_t length);

  void SetSerialConfig(const SerialConfig& serial_config);
  void SetBaudRate(unsigned baudrate);
  void SetHardwareFlowControl(bool enabled) { serial_config_.hardware_flow_control = enabled; }
//  void SetSerialConfig(SerialConfig serial_config) { serial_config_ = serial_config; }

  // 设置虚假发送模式，不向物理总线发送数据
  void SetFakeMode(bool fake_mode = true);
  SerialConfig      serial_config_;

 private:
  asio::serial_port serial_port_;

  ReceiveCallback rcv_cb_ = nullptr;

  // tx/rx buffering
  static constexpr uint32_t             rxtx_buffer_size = 1024 * 8;
  // rx buffer
  std::array<uint8_t, rxtx_buffer_size> rx_buf_{};
  // tx buffer
  uint8_t                               tx_buf_[rxtx_buffer_size]{};
  RingBuffer<uint8_t, rxtx_buffer_size> tx_rbuf_;
  std::recursive_mutex                  tx_mutex_;
  bool                                  tx_in_progress_  = false;

  bool SetupDevice() override;
  void DefaultReceiveCallback(uint8_t *data, const size_t bufsize, size_t len);
  void ReadFromPort();
  void WriteToPort(bool check_if_busy);
  bool SendDataImpl(const std::any &send_data) override;
  std::any ReceiveDataImpl() const override;
};

}
#endif //OUR_HW_SDK_ASYNC_SERIAL_H
