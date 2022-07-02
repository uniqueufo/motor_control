/**
* @file        CommunicationDevice.hpp
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-5-29
* @description 

* Copyright (c) 2022 Nan Lin
*/

#ifndef OUR_HW_SDK_COMMUNICATION_DEVICE_HPP
#define OUR_HW_SDK_COMMUNICATION_DEVICE_HPP
#include <iostream>
#include <mutex>
#include <memory>
#include <functional>
#include <thread>
#include <any>

namespace communication {
class CommunicationDevice {
 public:
  explicit CommunicationDevice(const std::string &name) : device_name_(name), is_opened_(false) {}
  std::string GetDeviceName() const { return device_name_; }

  bool OpenDevice() { return SetupDevice(); }
  virtual bool IsOpened() { return is_opened_; }
  virtual void CloseDevice() {}

  // send data interface
  template<typename T>
  void SendData(T value) {
    SendDataImpl(std::any(value));
  }
  //receive data interface
  template<typename T>
  T ReceiveData() const {
    std::any res = ReceiveDataImpl();
    return std::any_cast<T>(res);
  }

 protected:
  virtual bool SetupDevice() = 0;
  virtual bool SendDataImpl(std::any const &value) = 0;
  virtual std::any ReceiveDataImpl() const = 0;

  std::string device_name_;
  bool        is_opened_;
};
}

#endif //OUR_HW_SDK_COMMUNICATION_DEVICE_HPP
