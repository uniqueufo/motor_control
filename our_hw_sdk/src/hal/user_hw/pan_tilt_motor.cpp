/**
* @file        pan_tilt_motor.cpp
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-6-2
* @description 

* Copyright (c) 2022 Nan Lin
*/

#include "pan_tilt_motor.h"
#include "hal/user_hw/motor_cfg/RMDL_RS485_def.h"
#include "communication/utils/communication_typecasting.hpp"
#include <cmath>
namespace hal {
bool pan_tilt_motor::init() {
  // read the configuration file, and set corresponding  parameters;
  // com_device_->SetReceiveCallback(hal::DataProcessCallBack);
  return true;
}
void pan_tilt_motor::ReadPID_fromBus() {
  ReadPIDFrame read_pid_frame(id_);
  com_device_->SendData<communication::SerialDataFrame>(read_pid_frame.GetFrameData_with_Check());
}
void pan_tilt_motor::ReadCurrentState_fromBus() {
  ReadState2Frame read_motor_state_frame(id_);
  com_device_->SendData<communication::SerialDataFrame>(read_motor_state_frame.GetFrameData_with_Check());
}
void pan_tilt_motor::SetPID_RAM() {
  const PID_cfg &pid_cfg = motor_cfg.pid_cfg;
  WritePIDFrame write_pid_frame(id_);
  write_pid_frame.frame_data[0] = pid_cfg.anglePidKp;
  write_pid_frame.frame_data[1] = pid_cfg.anglePidKi;
  write_pid_frame.frame_data[2] = pid_cfg.speedPidKp;
  write_pid_frame.frame_data[3] = pid_cfg.speedPidKi;
  write_pid_frame.frame_data[4] = pid_cfg.iqPidKp;
  write_pid_frame.frame_data[5] = pid_cfg.iqPidKi;
  com_device_->SendData<communication::SerialDataFrame>(write_pid_frame.GetFrameData_with_Check());
}
void pan_tilt_motor::SetZeroPoint_ROM() {
  WriteZeroPointFrame write_zero_point_frame(id_);
  com_device_->SendData<communication::SerialDataFrame>(write_zero_point_frame.GetFrameData_with_Check());
}
void pan_tilt_motor::SetMaxSpeed(double max_speed) {
  motor_cfg.max_speed = max_speed;
}
// 设置单圈位置控制，角度范围0~359.99
// 带最大速度限制36000=360dps degree per second
void pan_tilt_motor::CirclePositionControl(double circle_pose, double max_speed_limit) {
    double target_pos =  (circle_pose * 100) / 16383.0 * 2 * M_PI;
    double cur_pos = cur_s.position;
    // 6.3 is the max angle
    if(cur_pos > 6)  cur_pos = 0;
    uint8_t direc = 0x01;       // default counterclockwise is positive
    if(target_pos >= cur_pos) {
        direc = 0x00;
    } else {
        direc = 0x01;
    }
    std::cout << "cur target:" << (circle_pose * 100) / 16383.0 * 2 * M_PI  << std::endl;
    std::cout << "cur direc:" << direc  << std::endl;
        //  uint16_t circle_pose_frame = (circle_pose - motor_cfg.offset) / (2 * M_PI) * 16383.0;
  uint16_t circle_pose_frame = (uint16_t) (circle_pose - motor_cfg.offset) * 100;
  //  uint32_t max_speed_frame = max_speed_limit * 180 / M_PI;
  uint32_t max_speed_frame = (uint32_t) max_speed_limit * 100;
  CirclePositionControlFrame_SpeedLimit write_circle_pos_speedlimit(id_);
  // 转动方向 0x00代表顺时针，0x01代表逆时针
  write_circle_pos_speedlimit.frame_data[0] = direc;
  write_circle_pos_speedlimit.frame_data[1] = *(uint8_t *) (&circle_pose_frame);
  write_circle_pos_speedlimit.frame_data[2] = *((uint8_t *) (&circle_pose_frame) + 1);
  write_circle_pos_speedlimit.frame_data[3] = 0x00;
  write_circle_pos_speedlimit.frame_data[4] = *(uint8_t *) (&max_speed_frame);
  write_circle_pos_speedlimit.frame_data[5] = *((uint8_t *) (&max_speed_frame) + 1);
  write_circle_pos_speedlimit.frame_data[6] = *((uint8_t *) (&max_speed_frame) + 2);
  write_circle_pos_speedlimit.frame_data[7] = *((uint8_t *) (&max_speed_frame) + 3);
//  printf("%02x ",  write_circle_pos_speedlimit.frame_data);
//  std::cout << "FRAME_DATA_LEN is "<< write_circle_pos_speedlimit.FRAME_DATA_LEN <<std::endl;
//  std::cout << "DATA_LEN_ is "<< write_circle_pos_speedlimit.DATA_LEN_ <<std::endl;
  com_device_->SendData<communication::SerialDataFrame>(write_circle_pos_speedlimit.GetFrameData_with_Check());
}
void pan_tilt_motor::SpeedControl(double speed) {
  //  int32_t speed_frame = speed * 180 / M_PI;
  int32_t speed_frame = speed * 100;
  SpeedControllerFrame write_speed_control(id_);
  write_speed_control.frame_data[0] = *(uint8_t *) (&speed_frame);
  write_speed_control.frame_data[1] = *((uint8_t *) (&speed_frame) + 1);
  write_speed_control.frame_data[2] = *((uint8_t *) (&speed_frame) + 2);
  write_speed_control.frame_data[3] = *((uint8_t *) (&speed_frame) + 3);
  com_device_->SendData<communication::SerialDataFrame>(write_speed_control.GetFrameData_with_Check());
}
void pan_tilt_motor::StopControl() {
  CloseMotorFrame close_motor_control(id_);
  auto temp_data= close_motor_control.GetFrameData_with_Check();
  com_device_->SendData<communication::SerialDataFrame>(temp_data);
}
PanTiltMotorStateFeedback pan_tilt_motor:: GetCurState(){
    return cur_s;
}
bool pan_tilt_motor::DecodeFrameFeedback2CurrentState(const TEMP_PARSING_DATA &uart_frame_feedback) {
  switch (uart_frame_feedback.frame_cmd) {
    case 0x00: return false;
    case SERIAL_READ_PID_CMD: {
      assert(uart_frame_feedback.frame_data_len == 6);
      PID_cfg &pid_cfg = motor_cfg.pid_cfg;
      pid_cfg.anglePidKp = uart_frame_feedback.frame_data[0];
      pid_cfg.anglePidKi = uart_frame_feedback.frame_data[1];
      pid_cfg.speedPidKp = uart_frame_feedback.frame_data[2];
      pid_cfg.speedPidKi = uart_frame_feedback.frame_data[3];
      pid_cfg.iqPidKp    = uart_frame_feedback.frame_data[4];
      pid_cfg.iqPidKi    = uart_frame_feedback.frame_data[5];
        printf("hehe \n");
      return true;
    }
    case SERIAL_READ_STATE2_CMD: {
      assert(uart_frame_feedback.frame_data_len == 7);
      cur_s.temperature            = (int8_t) uart_frame_feedback.frame_data[0];
      int16_t torque_current_frame = communication::bytes_to_int16(&uart_frame_feedback.frame_data[1]);
      cur_s.torque_current = torque_current_frame / 2048.0 * 33.0;
      int16_t speed_frame = communication::bytes_to_int16(&uart_frame_feedback.frame_data[3]);
      cur_s.speed = speed_frame / 180.0 * M_PI;
      uint16_t encoder_frame = communication::bytes_to_uint16(&uart_frame_feedback.frame_data[5]);
      cur_s.position = encoder_frame / 16383.0 * 2 * M_PI + motor_cfg.offset; //TODO: plus offset?? or reduce??
        std::cout << "cur position" <<  cur_s.position << std::endl;
      return true;
    }
    default: {
      std::cout << "[Warning]: Command " << std::hex << static_cast<int>(uart_frame_feedback.frame_cmd)
                << std::dec << " decode frame serial feedback not realized yet!" << std::endl;
      return false;
    }
  }

}
void pan_tilt_motor::setFrameControlCmd(PanTiltMotorControlCmd frame_control_cmd){
  std::lock_guard<std::mutex> guard(motor_control_cmd_mutex);
  frame_control_cmd_ = frame_control_cmd;
}
PanTiltMotorControlCmd pan_tilt_motor::getFrameControlCmd(){
  std::lock_guard<std::mutex> guard(motor_control_cmd_mutex);
  return frame_control_cmd_;
}
void pan_tilt_motor::setMotorConfig(PanTiltMotorConfig motor_config){
  std::lock_guard<std::mutex> guard(motor_cfg_mutex);
  motor_cfg = motor_config;
}
PanTiltMotorConfig pan_tilt_motor::getMotorConfig(){
  std::lock_guard<std::mutex> guard(motor_cfg_mutex);
  return motor_cfg;
}
}