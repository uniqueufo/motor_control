/**
* @file        serial_motor_manager.h
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-6-8
* @description manager for serial-bus motors.

* Copyright (c) 2022 Nan Lin
*/

#ifndef OUR_HW_SDK_SRC_HAL_SERIAL_DEVICE_MANAGER_H_
#define OUR_HW_SDK_SRC_HAL_SERIAL_DEVICE_MANAGER_H_

#include <thread>
#include <mutex>
#include "communication/async_serial.h"
#include "hal/stopwatch.h"

namespace hal {

/// \brief Device manager configuration and running state
struct DeviceControlConfig {
  // send command thread
  std::thread       cmd_thread_;
  int32_t           cmd_thread_period_ms_ = 10;
  bool              cmd_thread_started_   = false;
  std::atomic<bool> keep_running_{};

  // timeout to be implemented in each user's device manager
  bool     enable_timeout_   = true;
  uint32_t timeout_ms_       = 500;
  uint32_t watchdog_counter_ = 0;
  void FeedCmdTimeoutWatchdog() { watchdog_counter_ = 0; };

  enum CmdType {
    CTL_STATE_RESET      = -1,
    CTL_STATE_RESET_RUN  = 0,
    CTL_STATE_RESET_STOP = 1
  };
  CmdType  cmd_type          = CTL_STATE_RESET;

  // debug information
  bool print_loop_freq = false;
};

/// \brief 用于管理单条串口总线上所有设备的收发和数据处理
class SerialDeviceManager {
 public:
  SerialDeviceManager() {};
  SerialDeviceManager(const SerialConfig &serial_cfg) {
    serial_device_ = std::make_shared<communication::AsyncSerialDevice>(serial_cfg);
    serial_device_->SetSerialConfig(serial_cfg);
  }

  SerialDeviceManager(const std::string &serial_device_name,
                      const std::string &port_name,
                      BaudRate baud_rate = BR115200) {
    serial_device_ = std::make_shared<communication::AsyncSerialDevice>(
        SerialConfig({.serial_device_name=serial_device_name,
                         .port_name=port_name, .baud_rate=baud_rate}));
  }

  ~SerialDeviceManager();
  // do not allow copy or assignment
  SerialDeviceManager(const SerialDeviceManager &other) = delete;
  SerialDeviceManager &operator=(const SerialDeviceManager &other) = delete;

  void SetSerialConfig(SerialConfig serial_config);
  // connect to robot from serial
  bool Connect();
  // disconnect from robot, only valid for serial port
  void Disconnect();
  // ask background thread to shutdown properly,强制退出
  void Terminate();
  // cmd thread runs at 100Hz (10ms) by default
  void SetCmdThreadPeriodMs(int32_t period_ms) {
    device_ctl_cfg_.cmd_thread_period_ms_ = period_ms;
  };
  // 设置虚假发送模式，不向物理总线发送数据
  void SetFakeMode(bool fake_mode) {
    serial_device_->SetFakeMode(fake_mode);
  }
  // send完命令后记得喂狗
  void FeedCmdTimeoutWatchdog() { device_ctl_cfg_.watchdog_counter_ = 0; };

 protected:
  // serial-bus config
  std::shared_ptr<communication::AsyncSerialDevice> serial_device_    = nullptr;
//  SerialConfig                                      serial_cfg_;
  bool                                              serial_connected_ = false;
  // device control-process config and state
  DeviceControlConfig                               device_ctl_cfg_;

  // interal functions
  void StartCmdThread();
  void ControlLoop();

  // functions that must/may be implemented by child classes
  virtual void SendRobotCmd() {};
  virtual void ParseUARTBuffer(uint8_t *buf, const size_t bufsize, size_t bytes_received) {}

 private:
  void ConfigureSerial();
};
}
#endif //OUR_HW_SDK_SRC_HAL_SERIAL_DEVICE_MANAGER_H_
