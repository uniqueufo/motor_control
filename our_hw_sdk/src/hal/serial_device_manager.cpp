/**
* @file        serial_motor_manager.cpp
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-6-8
* @description 

* Copyright (c) 2022 Nan Lin
*/

#include "serial_device_manager.h"

namespace hal {

bool SerialDeviceManager::Connect() {
  assert(! serial_device_->serial_config_.port_name.empty());
  ConfigureSerial();
  if (!serial_connected_) {
    std::cerr << "ERROR: Failed to connect to serial port" << std::endl;
    return false;
  }
  return true;
}

void SerialDeviceManager::ConfigureSerial() {
  if (serial_device_== nullptr){
    serial_device_ = std::make_shared<communication::AsyncSerialDevice>( serial_device_->serial_config_);
  }
  serial_device_->OpenDevice();
  serial_device_->SetReceiveCallback(
      std::bind(&SerialDeviceManager::ParseUARTBuffer, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  serial_device_->StartListening();
  if (serial_device_->IsOpened()) serial_connected_ = true;
}

void SerialDeviceManager::Disconnect() {
  if (serial_connected_ && serial_device_->IsOpened()) {
    serial_device_->StopListening();
    serial_connected_ = false;
  }
}
void SerialDeviceManager::Terminate() {
  try {
    Disconnect();
  }
  catch (...){std::cout << "Serial device is not closed normally!" << std::endl;}
  device_ctl_cfg_.keep_running_       = false;
  device_ctl_cfg_.cmd_thread_started_ = false;
  std::terminate();
}
void SerialDeviceManager::StartCmdThread() {
  device_ctl_cfg_.keep_running_ = true;
  device_ctl_cfg_.cmd_thread_ =
      std::thread(std::bind(&SerialDeviceManager::ControlLoop, this));
  device_ctl_cfg_.cmd_thread_started_ = true;
}
void SerialDeviceManager::ControlLoop() {
  StopWatch ctrl_sw;
  uint32_t  timeout_iter_num;

  if (device_ctl_cfg_.enable_timeout_) {
    if (device_ctl_cfg_.timeout_ms_ < device_ctl_cfg_.cmd_thread_period_ms_)
      device_ctl_cfg_.timeout_ms_ = device_ctl_cfg_.cmd_thread_period_ms_;
    timeout_iter_num = static_cast<uint32_t>(device_ctl_cfg_.timeout_ms_ / device_ctl_cfg_.cmd_thread_period_ms_);
    //std::cout << "Timeout iteration number: " << timeout_iter_num << std::endl;
    std::cout << "function ControlLoop==>Timeout iteration time: " << device_ctl_cfg_.timeout_ms_ << " ms " << std::endl;
  }

  while (device_ctl_cfg_.keep_running_) {
    ctrl_sw.tic();
    if (device_ctl_cfg_.enable_timeout_) {
      // 当超过看门狗计数时，不再允许发送新数据，通过DeviceControlConfig.FeedCmdTimeoutWatchdog清零看门狗
      if (device_ctl_cfg_.watchdog_counter_ < timeout_iter_num) {
        SendRobotCmd();
        ++device_ctl_cfg_.watchdog_counter_;
        device_ctl_cfg_.cmd_type = device_ctl_cfg_.CTL_STATE_RESET_RUN;
      } else if (device_ctl_cfg_.cmd_type == device_ctl_cfg_.CTL_STATE_RESET_RUN) {
        std::cout << "Warning: cmd timeout, old cmd not sent to serial device" << std::endl;
        device_ctl_cfg_.cmd_type = device_ctl_cfg_.CTL_STATE_RESET_STOP;
      }
    } else {
      SendRobotCmd();
    }
    ctrl_sw.sleep_until_ms(device_ctl_cfg_.cmd_thread_period_ms_);
    if (device_ctl_cfg_.print_loop_freq)
      std::cout << "serial device control loop frequency: " << 1.0 / ctrl_sw.toc()
                << std::endl;
  }

}
SerialDeviceManager::~SerialDeviceManager() {
  Disconnect();
  if (device_ctl_cfg_.cmd_thread_.joinable()) device_ctl_cfg_.cmd_thread_.join();
  device_ctl_cfg_.cmd_thread_started_ = false;
}
void SerialDeviceManager::SetSerialConfig(SerialConfig serial_config) {
  serial_device_->SetSerialConfig(serial_config);
}
}