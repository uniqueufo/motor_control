/**
* @file        AsyncListenerInterface.h
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-5-30
* @description communication device interface for receiving data from field bus.
*
* edited from https://github.com/westonrobot/wrp_sdk
* Copyright (c) 2022 Nan Lin
*/

#ifndef OUR_HW_SDK_ASYNC_LISTENER_INTERFACE_HPP
#define OUR_HW_SDK_ASYNC_LISTENER_INTERFACE_HPP

#include "communication/communication_device.hpp"
#include "asio.hpp"

namespace communication {
/// if your communication device requires receiving data from field-bus, please inherit from this class
class AsyncListenerInterface : public CommunicationDevice {
 public:
  AsyncListenerInterface(std::string device_name)
      : CommunicationDevice(device_name){}
  virtual ~AsyncListenerInterface() = default;
  //do not allow copy
  AsyncListenerInterface() = delete;
  AsyncListenerInterface(const AsyncListenerInterface &other) = delete;
  bool StartListening() {
    if (is_opened_) {
      listener_thread_ = std::thread([this]() { io_context_.run(); });
      return true;
    }
    std::cerr << "Failed to setup port, please check if specified port exits "
                 "or if you have proper permissions to access it"
              << std::endl;
    return false;
  };
  void StopListening() {
    io_context_.stop();
    if (listener_thread_.joinable()) listener_thread_.join();
    io_context_.reset();
  }

 protected:
  std::thread listener_thread_;
  asio::io_context io_context_;
};
}

#endif //OUR_HW_SDK_ASYNC_LISTENER_INTERFACE_HPP
