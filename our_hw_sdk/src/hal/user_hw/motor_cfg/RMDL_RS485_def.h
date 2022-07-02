/**
* @file        RMD_def.h
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-5-30
* @description myactuator RMD_L motor protocol definition
* www.myactuator.com

* Copyright (c) 2022 Nan Lin
*/

#ifndef OUR_HW_SDK_INCLUDE_COMMUNICATION_PROTOCOL_RMDL_RS485_DEF_H_
#define OUR_HW_SDK_INCLUDE_COMMUNICATION_PROTOCOL_RMDL_RS485_DEF_H_

#include <iostream>
#include "asio.hpp"
#include <cstdint>
#include "communication/async_serial.h"
#include "hal/protocol/serial_port_common_def.h"

//#ifdef __cplusplus
//extern "C" {
//#endif


////////// rs485 communication frame definition

//  // 帧命令
//  uint8_t FRAME_ID;
//  uint8_t cmd;
//  uint8_t motor_id;
//  uint8_t data_len = DATA_LEN;
//  uint8_t header_check_sum;
//  // 帧数据
//  uint8_t data[DATA_LEN];
//  uint8_t data_check_sum;
template<size_t DATA_LEN, uint8_t FRAME_ID, uint8_t CMD>
struct RS485Frame {
  explicit RS485Frame(uint8_t motor_id) {
    assert(DATA_LEN >= 0 && DATA_LEN <= 60);
    total_frame_data[0] = FRAME_ID;
    total_frame_data[1] = CMD;
    total_frame_data[2] = motor_id;
    total_frame_data[3] = DATA_LEN;
    total_frame_data[4] = HeaderCheckSum2(FRAME_ID, CMD, motor_id);
  }

  communication::SerialDataFrame GetFrameData_with_Check() {
    total_frame_data[4] = HeaderCheckSum();
    if (DATA_LEN_ != 0) {
      total_frame_data[FRAME_DATA_LEN - 1] = DataCheckSum();
    }
    return sdf;
  }

//  #if(DATA_LEN != 0)
//    static constexpr size_t FRAME_DATA_LEN = DATA_LEN + 6;
//  #else
//    static constexpr size_t FRAME_DATA_LEN = 5;
//  #endif
//  static constexpr size_t DATA_LEN_ = DATA_LEN;
  static constexpr uint8_t DATA_LEN_ = DATA_LEN;
  static constexpr uint8_t FRAME_DATA_LEN = (DATA_LEN == 0) ? 5 :  DATA_LEN + 6;
  uint8_t                 total_frame_data[FRAME_DATA_LEN]{}; //整个待发送命令帧的数据

  uint8_t                        &frame_header_id       = total_frame_data[0];
  uint8_t                        &frame_cmd             = total_frame_data[1];
  uint8_t                        &frame_motor_id        = total_frame_data[2];
  uint8_t                        &frame_data_len        = total_frame_data[3];
  uint8_t                        &frame_header_checksum = total_frame_data[4];
  uint8_t                        *frame_data            = &total_frame_data[5];
  uint8_t                        &frame_data_checksum   = total_frame_data[FRAME_DATA_LEN - 1];
  communication::SerialDataFrame sdf                    = {.bytes=total_frame_data, .length=FRAME_DATA_LEN};

  inline uint8_t HeaderCheckSum2(uint8_t frame_id, uint8_t cmd, uint8_t motor_id) {
    return (uint8_t) (frame_id + cmd + motor_id + uint8_t(frame_data_len));
  }
  inline uint8_t HeaderCheckSum() {
      return HeaderCheckSum2(frame_header_id, frame_cmd, frame_motor_id);
  }
  inline uint8_t DataCheckSum(){
    uint8_t     data_sum = 0;
    assert(DATA_LEN_ != 0);
      for (size_t i = 0; i < DATA_LEN_; ++i) {
        data_sum += (uint8_t) frame_data[i];
      }
    return data_sum;
  }
//  inline void SetData_CheckSum(const uint8_t *data) {
////    assert(FRAME_DATA_LEN != 5);
//    uint8_t     data_sum = 0;
//    for (size_t i        = 0; i < DATA_LEN; ++i) {
//      total_frame_data[i] = data[i];
//      data_sum += (uint8_t) total_frame_data[5 + i];
//    }
//    total_frame_data[FRAME_DATA_LEN - 1] = data_sum;
//  }

};

#define RS485_FRAME_MACRO(data_len, frame_name, frame_id, frame_cmd) \
typedef struct RS485Frame<data_len, frame_id, frame_cmd> (frame_name);

#define SERIAL_FRAME_HEADER_ID 0x3E
#define FRAME_MAX_DATA_LEN  (size_t)13

#define SERIAL_READ_PID_CMD 0x30
#define SERIAL_WRITE_PID_RAM_CMD 0x31
#define SERIAL_WRITE_PID_ROM_CMD 0x32
#define SERIAL_READ_STATE2_CMD 0x9C // 读取电机状态2：温度、转矩电流、转速、编码器位置 ReadState2Frame

// 读取PID
RS485_FRAME_MACRO(0, ReadPIDFrame, SERIAL_FRAME_HEADER_ID, SERIAL_READ_PID_CMD)
// 驱动回复PID
RS485_FRAME_MACRO(6, RetReadPIDFrame, SERIAL_FRAME_HEADER_ID, SERIAL_READ_PID_CMD)
//写入PID到RAM，断电后失效
RS485_FRAME_MACRO(6, WritePIDFrame, SERIAL_FRAME_HEADER_ID, SERIAL_WRITE_PID_RAM_CMD)
//写入PID到RAM驱动回复
RS485_FRAME_MACRO(6, RetWritePIDFrame, SERIAL_FRAME_HEADER_ID, SERIAL_WRITE_PID_RAM_CMD)
//写入PID到ROM，断电后仍然有效，尽量少使用！！！！
RS485_FRAME_MACRO(6, WritePIDFrame_ROM, SERIAL_FRAME_HEADER_ID, SERIAL_WRITE_PID_ROM_CMD)
//写入PID到ROM驱动回复
RS485_FRAME_MACRO(6, RetWritePIDFrame_ROM, SERIAL_FRAME_HEADER_ID, SERIAL_WRITE_PID_ROM_CMD)

// 读取加速度
RS485_FRAME_MACRO(0, ReadAccFrame, SERIAL_FRAME_HEADER_ID, 0x33)
// 驱动回复加速度
RS485_FRAME_MACRO(4, RetReadAccFrame, SERIAL_FRAME_HEADER_ID, 0x33)
// 写入加速度
RS485_FRAME_MACRO(4, WriteAccFrame, SERIAL_FRAME_HEADER_ID, 0x34)
// 驱动回复加速度
RS485_FRAME_MACRO(4, RetWriteAccFrame, SERIAL_FRAME_HEADER_ID, 0x34)

// 写入当前位置到ROM作为电机零点命令，需要重新上电才能生效！不要频繁使用
RS485_FRAME_MACRO(0, WriteZeroPointFrame, SERIAL_FRAME_HEADER_ID, 0x19)
// 驱动回复写入零点
RS485_FRAME_MACRO(0, RetWriteZeroPointFrame, SERIAL_FRAME_HEADER_ID, 0x19)

// 读取多圈角度
RS485_FRAME_MACRO(0, ReadMultiCircleAngleFrame, SERIAL_FRAME_HEADER_ID, 0x92)
// 驱动回复多圈角度
RS485_FRAME_MACRO(8, RetReadMultiCircleAngleFrame, SERIAL_FRAME_HEADER_ID, 0x92)
// 读取单圈角度
RS485_FRAME_MACRO(0, ReadCircleAngleFrame, SERIAL_FRAME_HEADER_ID, 0x94)
// 驱动回复单圈角度
RS485_FRAME_MACRO(2, RetReadCircleAngleFrame, SERIAL_FRAME_HEADER_ID, 0x94)

// 读取电机状态1和错误标志
RS485_FRAME_MACRO(0, ReadState1Frame, SERIAL_FRAME_HEADER_ID, 0x9A)
// 驱动回复电机状态1
RS485_FRAME_MACRO(7, RetReadState1Frame, SERIAL_FRAME_HEADER_ID, 0x9A)
// 读取电机状态2：温度、转矩电流、转速、编码器位置
RS485_FRAME_MACRO(0, ReadState2Frame, SERIAL_FRAME_HEADER_ID, SERIAL_READ_STATE2_CMD)
// 驱动回复电机状态2
RS485_FRAME_MACRO(7, RetReadState2Frame, SERIAL_FRAME_HEADER_ID, SERIAL_READ_STATE2_CMD)

// 清除电机错误标志
RS485_FRAME_MACRO(0, ClearErrorFrame, SERIAL_FRAME_HEADER_ID, 0x9B)
// 驱动回复清除电机错误标志
RS485_FRAME_MACRO(7, RetClearErrorFrame, SERIAL_FRAME_HEADER_ID, 0x9B)
// 关闭电机,清除运行状态和之前接收的控制指令
RS485_FRAME_MACRO(0, CloseMotorFrame, SERIAL_FRAME_HEADER_ID, 0x80)
// 驱动回复关闭电机
RS485_FRAME_MACRO(0, RetCloseMotorFrame, SERIAL_FRAME_HEADER_ID, 0x80)
// 停止电机,但是不清除运行状态和之前接收的控制指令
RS485_FRAME_MACRO(0, StopMotorFrame, SERIAL_FRAME_HEADER_ID, 0x81)
// 驱动回复关闭电机
RS485_FRAME_MACRO(0, RetStopMotorFrame, SERIAL_FRAME_HEADER_ID, 0x81)
// 从电机停止命令中恢复电机运行
RS485_FRAME_MACRO(0, RecoverMotorFrame, SERIAL_FRAME_HEADER_ID, 0x88)
// 驱动回复恢复电机运行
RS485_FRAME_MACRO(0, RetRecoverMotorFrame, SERIAL_FRAME_HEADER_ID, 0x88)

//////////////控制模式
// 转矩闭环控制命令
RS485_FRAME_MACRO(2, TorqueControlFrame, SERIAL_FRAME_HEADER_ID, 0xA1)
// 驱动回复转矩闭环控制
RS485_FRAME_MACRO(7, RetTorqueControlFrame, SERIAL_FRAME_HEADER_ID, 0xA1)
// 速度闭环控制命令
RS485_FRAME_MACRO(4, SpeedControllerFrame, SERIAL_FRAME_HEADER_ID, 0xA2)
// 驱动回复转矩闭环控制
RS485_FRAME_MACRO(7, RetSpeedControlFrame, SERIAL_FRAME_HEADER_ID, 0xA2)
// 多圈位置闭环控制命令
RS485_FRAME_MACRO(8, MultiCirclePositionControlFrame, SERIAL_FRAME_HEADER_ID, 0xA3)
// 驱动回复多圈位置闭环控制
RS485_FRAME_MACRO(7, RetMultiCirclePositionControlFrame, SERIAL_FRAME_HEADER_ID, 0xA3)
// 多圈位置闭环控制命令,带速度限制
RS485_FRAME_MACRO(12, MultiCirclePositionControlFrame_SpeedLimit, SERIAL_FRAME_HEADER_ID, 0xA4)
// 驱动回复多圈位置闭环控制带速度限制
RS485_FRAME_MACRO(7, RetMultiCirclePositionControlFrame_SpeedLimit, SERIAL_FRAME_HEADER_ID, 0xA4)
// 单圈位置闭环控制命令
RS485_FRAME_MACRO(4, CirclePositionControlFrame, SERIAL_FRAME_HEADER_ID, 0xA5)
// 驱动回复多圈位置闭环控制
RS485_FRAME_MACRO(7, RetCirclePositionControlFrame, SERIAL_FRAME_HEADER_ID, 0xA5)
// 单圈位置闭环控制命令,带速度限制
RS485_FRAME_MACRO(8, CirclePositionControlFrame_SpeedLimit, SERIAL_FRAME_HEADER_ID, 0xA6)
// 驱动回复单圈位置闭环控制,带速度限制
RS485_FRAME_MACRO(7, RetCirclePositionControlFrame_SpeedLimit, SERIAL_FRAME_HEADER_ID, 0xA6)

struct PID_cfg {
  uint8_t anglePidKp; //位置环P参数
  uint8_t anglePidKi; //位置环I参数
  uint8_t speedPidKp; //速度环P参数
  uint8_t speedPidKi; //速度环I参数
  uint8_t iqPidKp;    //转矩环P参数
  uint8_t iqPidKi;    //转矩环I参数
};
////////// MYACTUATOR RMD-motor full state information
typedef struct RMDMotorState {
  PID_cfg  pid_cfg;     //三环PI
  int32_t  accel;      //加速度，单位1dps/s
  uint16_t encoder_pos;    //14位编码器范围 0~16383,为编码器原始位置减去零偏后的位置
  uint16_t encoderRaw;     //编码器原始位置
  uint16_t encoderOffset;  //编码器零偏
  uint64_t motorAngle;     //多圈绝对角度值，正值表示顺时针累计角度，负值表示逆时针累计角度。单位0.01度/LSB
  uint16_t circleAngle;    //单圈角度值，以编码器零点为起始点，顺时针增加，再到达零点时数值回0，单位0.01度/LSB。范围0~35999
  int8_t   temperature;    //电机温度，单位1℃/LSB
  uint16_t voltage;        //电机电压，单位0.1V/LSB
  uint8_t  errorState;     //错误标志，[bit 0] 电压状态，0代表正常，1代表低压; [bit 3]温度状态，0代表温度正常，1代表过温保护
  int16_t  iq;             //转矩电流值，-2048~2048,对应-33A~33A
  int16_t  power;          //输出功率， -1000~1000
  int16_t  speed;          //电机转速, 1dps/LSB
  int16_t  iPhase[3];      //ABC三相电流， 单位1A/64LSB
} MotorState;

// struct PanTiltMotorState {
//   int8_t temperature;   // 温度   data[5]
//   int16_t iq;           // 转矩电流值低字节 data[6]*(uint8_t *)(&iq)  高字节:data[7] *(uint8_t *)((&iq)+1)
//   int16_t speed;        // 电机速度   data[8] *(uint8_t *)(&speed)/data[9]:*((uint8_t *)(&speed+)1)
//   uint16_t encoder;     // 编码器位置 data[10] *(uint8_t *)(&encoder)  data[11]*((uint8_t *)(&encoder)+1)
// }

//#ifdef __cplusplus
//}
//#endif

#endif //OUR_HW_SDK_INCLUDE_COMMUNICATION_PROTOCOL_RMDL_RS485_DEF_H_

// receive data types
//#define RET_READ_PID_FRAME         ((uint8_t)0x30)
//#define RET_WRITE_PID_FRAME         ((uint8_t)0x31)
//#define RET_WRITE_PID_FRAME_ROM         ((uint8_t)0x32)
//#define RET_READ_ACC_FRAME         ((uint8_t)0x33)
//#define RET_WRITE_ACC_FRAME         ((uint8_t)0x34)
//#define READ_ENCODER         ((uint8_t)0x90)
//
//#define WRITE_ENCODER_AS_ZERO_POINT         ((uint8_t)0x91)
//#define RET_WRITE_ZERO_POINT_FRAME         ((uint8_t)0x19)
//#define RET_READ_MULTI_CIRCLE_ANGLE         ((uint8_t)0x92)
//#define RET_READ_CIRCLE_ANGLE         ((uint8_t)0x94)
//
//#define RET_READ_STATE1         ((uint8_t)0x9A)
//#define RET_READ_STATE2         ((uint8_t)0x9C)
//#define RET_READ_STATE3         ((uint8_t)0x9D)
//
//#define RET_CLEAR_ERROR         ((uint8_t)0x9B)
//#define RET_CLOSE_MOTOR         ((uint8_t)0x80)
//#define RET_STOP_MOTOR         ((uint8_t)0x81)
//#define RET_RECOVER_MOTOR         ((uint8_t)0x88)
//
//#define RET_TORQUE_CONTROL         ((uint8_t)0xA1)
//#define RET_SPEED_CONTROL         ((uint8_t)0xA2)
//#define RET_MULTI_CIRCLE_POSITION_CONTROL         ((uint8_t)0xA3)
//#define RET_MULTI_CIRCLE_POSITION_CONTROL_SPEEDLIMIT         ((uint8_t)0xA4)
//#define RET_CIRCLE_POSITION_CONTROL         ((uint8_t)0xA5)
//#define RET_CIRCLE_POSITION_CONTROL_SPEEDLIMIT         ((uint8_t)0xA6)
