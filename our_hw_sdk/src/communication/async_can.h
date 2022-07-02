/**
* @file        AsyncCanDevice.h
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-5-30
* @description 

* Copyright (c) 2022 Nan Lin
*/

#ifndef OUR_HW_SDK_INCLUDE_COMMUNICATION_ASYNC_CAN_H_
#define OUR_HW_SDK_INCLUDE_COMMUNICATION_ASYNC_CAN_H_

#include <linux/can.h>
#include <memory>
#include "asio/posix/basic_stream_descriptor.hpp"
#include "communication/async_listener_interface.hpp"

namespace communication {
class AsyncCanDevice : public AsyncListenerInterface, public std::enable_shared_from_this<AsyncCanDevice> {
 public:
  using ReceiveCallback = std::function<void(can_frame *rx_frame)>;

 public:
  explicit AsyncCanDevice(const std::string &can_port = "can0", const std::string &device_name = "");
  bool IsOpened() override;
  void CloseDevice() override;
  void SetReceiveCallback(ReceiveCallback cb) { rcv_cb_ = cb; }
  bool SendFrame(const can_frame &frame);

 private:
  int                                    can_fd_{};
  std::string                            can_port_;
  asio::posix::basic_stream_descriptor<> socketcan_stream_;

  struct can_frame rcv_frame_{};
  ReceiveCallback  rcv_cb_ = nullptr;

  bool SetupDevice() override;
  void DefaultReceiveCallback(can_frame *rx_frame);
  void ReadFromPort(struct can_frame &rec_frame, asio::posix::basic_stream_descriptor<> &stream);
  bool SendDataImpl(const std::any &send_data) override;
  std::any ReceiveDataImpl() const override;
};
}
#endif //OUR_HW_SDK_INCLUDE_COMMUNICATION_ASYNC_CAN_H_
