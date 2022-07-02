#include <iostream>
#include <unistd.h>

#include "../../hal/user_hw/pan_tilt_motor.h"
#include "../../hal/user_hw/pan_tilt_motor_manager.h"
#include "../../hal/serial_com_motor.h"

using namespace hal;

int main(int argc, char *argv[]) {
  std::string device_id = "/dev/ttyUSB0";
  BaudRate    baud_rate = BR115200;


  PanTiltMotorManager motorManager = PanTiltMotorManager(device_id);
  motorManager.GetParamFromFile(std::string("motor_manager.yaml"));
//  motorManager.SetFakeMode(false); //设置为调试模式，不发送真实总线数据
  motorManager.InitAllDevices();
//  motorManager.SendSetZeroPointToROMCmd(1);
  //  motorManager.SendStopControlCmd(1);
//  motorManager.SendSpeedControldCmd(1, 0);
//    motorManager.SendStopControlCmd(1);
//  motorManager.SendCircleCPositionControlCmd(1, 100, 40);
//    motorManager.SendStopControlCmd(1);
//    motorManager.SendReadPIDFromBusCmd(1);
//  motorManager.SetSpeedControl(1, 10);
//  motorManager.SetCircleCPosition(1, 100, 40);
  while (true)
  {
//        motorManager.SendReadPIDFromBusCmd(1);
      sleep(1);
  }
//   motorManager.SendReadPIDFromBusCmd(1);
//    motorManager.SendCircleCPositionControlCmd(1, 100, 20);
  return 0;
}