/**
* @file        PanTiltMotorManager.h
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-6-3
* @description 

* Copyright (c) 2022 Nan Lin
*/

#ifndef OUR_HW_SDK_SRC_HAL_USER_HW_CFG_PAN_TILT_MOTOR_MANAGER_H_
#define OUR_HW_SDK_SRC_HAL_USER_HW_CFG_PAN_TILT_MOTOR_MANAGER_H_
#include <thread>
#include <mutex>
#include "communication/async_serial.h"
#include "hal/user_hw/motor_cfg/RMDL_RS485_def.h"
#include "hal/user_hw/motor_cfg/pan_tilt_motor_msg_process.h"
#include "hal/serial_device_manager.h"
#include "hal/user_hw/pan_tilt_motor.h"
#include "yaml-cpp/yaml.h"

#include <fstream>
#include <iostream>

namespace hal {

//RS485_FRAME_MACRO(FRAME_MAX_DATA_LEN, TEMP_PARSING_DATA, SERIAL_FRAME_HEADER_ID, 0x00)

class PanTiltMotorManager : public SerialDeviceManager {
 public:
  //默认构造时，之后必须通过setSerialConfig设置串口参数
  PanTiltMotorManager() = default;
  explicit PanTiltMotorManager(const std::string &serial_name, BaudRate baud_rate = BR115200,
                               const std::string &serial_device_name = "PanTiltSerialCard")
      : SerialDeviceManager(serial_device_name, serial_name, baud_rate) {}
  explicit PanTiltMotorManager(const SerialConfig &serial_cfg) : SerialDeviceManager(serial_cfg) {}
   ~PanTiltMotorManager();

  void SetMaxSpeedControl(std::string config_file_name, std::string motor_name, double max_speed);
  void SetCircleCPosition(uint8_t motor_id, double circle_pose, double max_speed_limit);
  void SetSpeedControl(uint8_t motor_id, double speed);
  // 读取配置文件，不初始化设备
  void GetParamFromFile(std::string config_file_name);
  bool InitAllDevices();
  //  std::shared_ptr<communication::AsyncSerialDevice> com_device_;

  // SpeedControlCmd speed_control_speed_;
  // CirclePositionControlCmd circle_position_control_;

  void SendRobotCmd() override;
  void SendReadPIDFromBusCmd(uint8_t motor_id);
  void SendSetPIDToRAMCmd(uint8_t motor_id);
  void SendSetZeroPointToROMCmd(uint8_t motor_id);
  // void SendSetMaxSpeedCmd(uint8_t motor_id, double max_speed);
  void SendCircleCPositionControlCmd(uint8_t motor_id, double circle_pose, double max_speed_limit);
  void SendSpeedControldCmd(uint8_t motor_id, double speed);
  void SendStopControlCmd(uint8_t motor_id);

  PanTiltMotorStateFeedback GetMotorState(uint8_t motor_id);

 private:
  //interal functions
  pan_tilt_motor *pan_motor_;
  pan_tilt_motor *tilt_motor_;
  std::map<uint8_t, pan_tilt_motor*> motor_map_; // map node-id to motor
  void RegisterMotor(uint8_t motor_id, pan_tilt_motor *motor);

  // manager config
  DeviceConfiguration device_config_;


  // receiving data process
  static TEMP_PARSING_DATA uart_parsing_data; //串口帧解析临时变量
  void ParseUARTBuffer(uint8_t *buf, const size_t bufsize, size_t bytes_received) override;
  static bool DecodeMsgFromUART(uint8_t c); //分割串口帧，提取数据存于变量uart_parsing_data中。
  void UserNewStateMsgReceivedCallback(); //当收到数据时，用户自定义的处理函数

};

}
#endif //OUR_HW_SDK_SRC_HAL_USER_HW_CFG_PAN_TILT_MOTOR_MANAGER_H_
