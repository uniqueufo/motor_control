/**
* @file        RMD_def.h
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-5-30
* @description myactuator RMD_L motor protocol definition
* www.myactuator.com

* Copyright (c) 2022 Nan Lin
*/

#ifndef OUR_HW_SDK_INCLUDE_COMMUNICATION_PROTOCOL_RS485_DEF_H_
#define OUR_HW_SDK_INCLUDE_COMMUNICATION_PROTOCOL_RS485_DEF_H_

#include <iostream>
#include "asio.hpp"
#include <cstdint>

//#ifdef __cplusplus
//extern "C" {
//#endif

///////////// rs-485 setup configs
using SPB = asio::serial_port_base;

enum BaudRate {
  BR9600   = (uint32_t) 9600,
  BR19200  = (uint32_t) 19200,
  BR57600  = (uint32_t) 57600,
  BR115200 = (uint32_t) 115200
};

struct SerialConfig {
  std::string                             serial_device_name; // 管理的串口总线别名，仅用于调试
  std::string                             port_name;
  BaudRate                                baud_rate;
  size_t                                  character_size        = 8;
  asio::serial_port_base::parity::type    party                 = SPB::parity::none;
  asio::serial_port_base::stop_bits::type stop_bits             = SPB::stop_bits::one;
  bool                                    hardware_flow_control = false;
  bool                                    fake_mode             = false; //设置虚假总线通讯模式，不需要物理串口设备
};

#endif //OUR_HW_SDK_INCLUDE_COMMUNICATION_PROTOCOL_RS485_DEF_H_
