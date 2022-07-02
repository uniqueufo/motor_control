/**
* @file        pan_tilt_motor.h
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-6-2
* @description 

* Copyright (c) 2022 Nan Lin
*/

#ifndef OUR_HW_SDK_SRC_HAL_USER_HW_CFG_PAN_TILT_MOTOR_H_
#define OUR_HW_SDK_SRC_HAL_USER_HW_CFG_PAN_TILT_MOTOR_H_
#include <utility>

#include "hal/serial_com_motor.h"
#include "hal/user_hw/motor_cfg/pan_tilt_motor_msg_process.h"
#include <mutex>

namespace hal {
class pan_tilt_motor : public SerialComMotor {
 public:
  pan_tilt_motor(const std::string name, int id, std::shared_ptr<communication::AsyncSerialDevice> com_device) :
      SerialComMotor(name, id, std::move(com_device)) {}
  ~pan_tilt_motor() {StopControl();}

  bool init();
  // 从串口总线中读取PID参数，不需要频繁读取
  void ReadPID_fromBus();
  // 从串口总线中读取状态参数，需要定时频繁读取
  void ReadCurrentState_fromBus();
  // 写入PID参数到RAM中
  // void SetPID_RAM(const PID_cfg& pid_cfg);
  void SetPID_RAM();
  // 设置当前点为零点，写入固定的ROM中，注意需要重启才能生效
  void SetZeroPoint_ROM();
  // 设置最大速度
  void SetMaxSpeed(double max_speed);
  // 设置单圈位置控制，带最大速度限制
  // void CirclePositionControl(double circle_pose, double max_speed);
  void CirclePositionControl(double circle_pose, double max_speed_limit);
  // 设置速度控制
  // void SpeedControl(double speed);
  void SpeedControl(double speed);
  // 停止电机，清除所有控制命令
  void StopControl();
  PanTiltMotorStateFeedback GetCurState();


  // 分割完成的串口帧serial frame解码到电机物理状态，uart_frame_feedback -> cur_s
  bool DecodeFrameFeedback2CurrentState(const TEMP_PARSING_DATA& uart_frame_feedback);
  
  // setter&getter frame_control_cmd
  void setFrameControlCmd(PanTiltMotorControlCmd frame_control_cmd);
  PanTiltMotorControlCmd getFrameControlCmd();

  void setMotorConfig(PanTiltMotorConfig motor_config);
  PanTiltMotorConfig getMotorConfig();

 private:
  //电机实时状态, motor current state
  PanTiltMotorStateFeedback cur_s;
  //电机配置参数， motor config
  PanTiltMotorConfig motor_cfg;
  std::mutex motor_cfg_mutex;

  //  PanTiltMotorStateFeedback frame_state_feedback_; //用户选择的哪些反馈变量
  PanTiltMotorControlCmd frame_control_cmd_;  //用户选择的控制模式和控制命令
  std::mutex motor_control_cmd_mutex;

};
}
#endif //OUR_HW_SDK_SRC_HAL_USER_HW_CFG_PAN_TILT_MOTOR_H_
