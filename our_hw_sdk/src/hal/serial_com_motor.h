/**
* @file        SerialComMotor.h
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-6-1
* @description com-serial motor implement

* Copyright (c) 2022 Nan Lin
*/

#ifndef OUR_HW_SDK_SRC_HAL_SERIAL_COM_MOTOR_H_
#define OUR_HW_SDK_SRC_HAL_SERIAL_COM_MOTOR_H_

#include "hal/motor.h"

#include <utility>
#include "communication/async_serial.h"

namespace hal {
// serial motor
class SerialComMotor: public Motor{
 public:
  SerialComMotor(const std::string name, int id,
                 std::shared_ptr<communication::AsyncSerialDevice> com_device): Motor(name, id), com_device_(std::move(com_device)) {}
 protected:
  // serial communication device
  std::shared_ptr<communication::AsyncSerialDevice> com_device_;
};
}

#endif //OUR_HW_SDK_SRC_HAL_SERIAL_COM_MOTOR_H_
