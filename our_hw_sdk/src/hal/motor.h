/**
* @file        motor.h
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-6-1
* @description abstract motor class

* Copyright (c) 2022 Nan Lin
*/
#ifndef OUR_HW_SDK_SRC_HAL_MOTOR_H_
#define OUR_HW_SDK_SRC_HAL_MOTOR_H_

#include <string>
#include "communication/communication_device.hpp"

namespace hal {
class Motor {
 public:
  Motor(const std::string name, int id) : name_(name), id_(id), enable_(true) {}
  virtual bool init() =0;

 protected:
  std::string name_;
   int   id_;
  bool        enable_; //上电使能
};
}

#endif //OUR_HW_SDK_SRC_HAL_MOTOR_H_
