/**
* @file        PanTiltMotorManager.cpp
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-6-3
* @description 

* Copyright (c) 2022 Nan Lin
*/

#include "pan_tilt_motor_manager.h"
#include "hal/user_hw/motor_cfg/pan_tilt_motor_msg_process.h"
#include "unistd.h"
#define PATH_SIZE 255

namespace hal {

TEMP_PARSING_DATA PanTiltMotorManager::uart_parsing_data = TEMP_PARSING_DATA(0x00);

void PanTiltMotorManager::ParseUARTBuffer(uint8_t *buf, const size_t bufsize, size_t bytes_received) {
// std::cout << "bytes received from serial，receive " << bytes_received << " bytes." << std::endl;
  // serial_parser_.PrintStatistics();
  // serial_parser_.ParseBuffer(buf, bytes_received);
//  PanTiltMotorStateFeedback status_msg{};

  for (int i = 0; i < bytes_received; ++i) {
    if (DecodeMsgFromUART(buf[i])) //解码成功获得串口解析数据status_msg，还未转换到物理数据
      UserNewStateMsgReceivedCallback(); //转换成实际物理数据，如: 编码器线数->电机angle
  }
}
void PanTiltMotorManager::UserNewStateMsgReceivedCallback() {
//    std::cout << "get new state" << std::endl;
//  std::lock_guard<std::mutex> guard(scout_state_mutex_);
  uint8_t motor_id = uart_parsing_data.frame_motor_id;
  motor_map_[motor_id]->DecodeFrameFeedback2CurrentState(uart_parsing_data);
}

PanTiltMotorManager::~PanTiltMotorManager() {
pan_motor_->StopControl();
tilt_motor_->StopControl();

}
void PanTiltMotorManager::SendRobotCmd() {
  // 定时读取运行时电机状态
//  for (auto it = motor_map_.begin(); it != motor_map_.end(); ++it) {
//      it->second->ReadCurrentState_fromBus();
//      it->second->ReadPID_fromBus();
//      PanTiltMotorControlCmd cmd_temp = it->second->getFrameControlCmd();
//      it->second->SpeedControl(cmd_temp.speed_);
//      it->second->CirclePositionControl(cmd_temp.circle_pose_, cmd_temp.max_speed_limit_);
//      it->second->StopControl();
//  }

  // only test motor1
  // 读取电机状态
  motor_map_[1]->ReadCurrentState_fromBus();
//  motor_map_[1]->ReadPID_fromBus();
  // 获取控制命令
//  PanTiltMotorControlCmd cmd_temp = motor_map_[1]->getFrameControlCmd();
  // SpeedControl和CirclePositionControl控制命令单独发送的时其他命令的参数要设置为0
  // 否则会默认发送CirclePositionControl
//  if(cmd_temp.speed_ >= 0) {
//    motor_map_[1]->SpeedControl(cmd_temp.speed_);
//  }
//  if(cmd_temp.circle_pose_ >= 0) {
//    motor_map_[1]->CirclePositionControl(cmd_temp.circle_pose_, cmd_temp.max_speed_limit_);
//  }
  // motor_map_[1]->StopControl();

  // feed dog
  FeedCmdTimeoutWatchdog();
}

void PanTiltMotorManager::SendReadPIDFromBusCmd(uint8_t motor_id) {
  if (serial_connected_) {
    motor_map_[motor_id]->ReadPID_fromBus();
  }
}
void PanTiltMotorManager::SendSetPIDToRAMCmd(uint8_t motor_id) {
  if (serial_connected_) {
    motor_map_[motor_id]->SetPID_RAM();
  }
}
void PanTiltMotorManager::SendSetZeroPointToROMCmd(uint8_t motor_id) {
  if (serial_connected_) {
    motor_map_[motor_id]->SetZeroPoint_ROM();
  }
}
void PanTiltMotorManager::SendCircleCPositionControlCmd(uint8_t motor_id, double circle_pose, double max_speed_limit) {
  if (serial_connected_) {
    // position limit
    PanTiltMotorConfig cur_motor_cfg =  motor_map_[motor_id]->getMotorConfig();
    circle_pose = circle_pose > cur_motor_cfg.max_angle ? cur_motor_cfg.max_angle
                : circle_pose < cur_motor_cfg.min_angle ? cur_motor_cfg.min_angle
                : circle_pose;
    // speed limit
    max_speed_limit = max_speed_limit > cur_motor_cfg.max_speed ? cur_motor_cfg.max_speed
                : max_speed_limit;
    motor_map_[motor_id]->CirclePositionControl(circle_pose, max_speed_limit);
  }
}
// void PanTiltMotorManager::SendSetMaxSpeedCmd(uint8_t motor_id, double max_speed) {
//   if (serial_connected_) {
//     motor_map_[motor_id]->SetMaxSpeed(max_speed);
//   }
// };
void PanTiltMotorManager::SendSpeedControldCmd(uint8_t motor_id, double speed) {
  // limit the max speed
  PanTiltMotorConfig cur_motor_cfg =  motor_map_[motor_id]->getMotorConfig();
  speed = speed > cur_motor_cfg.max_speed ? cur_motor_cfg.max_speed : speed;
  if (serial_connected_) {
    motor_map_[motor_id]->SpeedControl(speed);
  }
}
void PanTiltMotorManager::SendStopControlCmd(uint8_t motor_id) {
  if (serial_connected_) {
    motor_map_[motor_id]->StopControl();
  }
}

PanTiltMotorStateFeedback PanTiltMotorManager::GetMotorState(uint8_t motor_id){
    if (serial_connected_) {
        return motor_map_[motor_id]->GetCurState();
    }
    else {
        return PanTiltMotorStateFeedback();
    }
}

void PanTiltMotorManager::SetMaxSpeedControl(std::string config_file_name, std::string motor_name, double max_speed) {
  YAML::Node cfgs = YAML::LoadFile(config_file_name);
  cfgs[motor_name]["maxSpeed"] = max_speed;

  // 更新配置文件
  std::ofstream fout(config_file_name);
  fout << cfgs <<std::endl; 
  fout.close();
}

void PanTiltMotorManager::SetCircleCPosition(uint8_t motor_id, double circle_pose, double max_speed_limit) {
  PanTiltMotorControlCmd cur_motor_cmd = motor_map_[motor_id]->getFrameControlCmd();
  cur_motor_cmd.circle_pose_ = circle_pose;
  cur_motor_cmd.max_speed_limit_ = max_speed_limit;
  motor_map_[motor_id]->setFrameControlCmd(cur_motor_cmd);
}
void PanTiltMotorManager::SetSpeedControl(uint8_t motor_id, double speed) {
  PanTiltMotorControlCmd cur_motor_cmd = motor_map_[motor_id]->getFrameControlCmd();
  cur_motor_cmd.speed_ = speed;
  motor_map_[motor_id]->setFrameControlCmd(cur_motor_cmd);
}
void PanTiltMotorManager::RegisterMotor(uint8_t motor_id, pan_tilt_motor *motor) {
  motor_map_[motor_id] = motor;
}
void PanTiltMotorManager::GetParamFromFile(std::string config_file_name){
//    std::string config_path = "";
//    char path[PATH_SIZE];
//    if(!getcwd(path, PATH_SIZE)) {
//        std::cout << "yaml file path get failed!" << std::endl;
//        return;
//    }
//    std::string realPath(path);
//    std::cout <<"realPath:"<< realPath << std::endl;
//    config_path = realPath.append("/../../../../src/hal/user_hw/motor_cfg/")
//            .append(config_file_name);
//    std::cout << config_path << std::endl;
  YAML::Node cfgs = YAML::LoadFile(config_file_name);

  device_config_.runBase = cfgs["runBase"].as<bool>();
  device_config_.runArm = cfgs["runArm"].as<bool>();
  device_config_.runPanTilt = cfgs["runPanTilt"].as<bool>();
  device_config_.runStopButton = cfgs["runStopButton"].as<bool>();

  device_config_.PanTiltComName = cfgs["PanTiltComName"].as<std::string>();
  device_config_.StopButtonComName = cfgs["StopButtonComName"].as<std::string>();
  device_config_.Baudrate = BaudRate(cfgs["BaudRate"].as<uint32_t>());

  device_config_.baseLoopRate = cfgs["baseLoopRate"].as<double>();
  device_config_.armLoopRate = cfgs["armLoopRate"].as<double>();
  device_config_.panTiltLoopRate = cfgs["panTiltLoopRate"].as<double>();
  device_config_.stopButtonLoopRate = cfgs["stopButtonLoopRate"].as<double>();

  device_config_.wheelConfiguration.maxLinearSpeed = cfgs["stopButtonLoopRate"].as<double>();
  device_config_.wheelConfiguration.maxTurnSpeed = cfgs["maxTurnSpeed"].as<double>();
  device_config_.wheelConfiguration.wheelDist = cfgs["wheelDist"].as<double>();
  device_config_.wheelConfiguration.wheelPerimeter = cfgs["wheelPerimeter"].as<double>();

  device_config_.elevator.elevatorUnitHeight = cfgs["elevatorUnitHeight"].as<double>();
  device_config_.elevator.minElevatorHeight = cfgs["minElevatorHeight"].as<double>();
  device_config_.elevator.maxElevatorHeight = cfgs["maxElevatorHeight"].as<double>();

//  device_config_.stopButton.stopChar = cfgs["stopChar"].as<uint8_t>();
//  device_config_.stopButton.normalChar = cfgs["normalChar"].as<uint8_t>();

  // set serial-config, and connect serial-bus-card
  SerialConfig serialConfig={.serial_device_name="com_serial_device",
                             .port_name=device_config_.PanTiltComName,
                             .baud_rate = device_config_.Baudrate};
  SetSerialConfig(serialConfig);

  int pan_motor_id = cfgs["pan"]["serial_id"].as<int>();
  int tilt_motor_id = cfgs["tilt"]["serial_id"].as<int>();

  pan_motor_ = new pan_tilt_motor("pan", pan_motor_id, serial_device_);
  tilt_motor_ = new pan_tilt_motor("tilt", tilt_motor_id, serial_device_);

  RegisterMotor(pan_motor_id, pan_motor_);
  RegisterMotor(tilt_motor_id, tilt_motor_);

  // 启动控制环
//  if(! device_ctl_cfg_.cmd_thread_started_) StartCmdThread();

  // 加载电机配置
  // pan_motor_config
  PanTiltMotorConfig pan_motor_cfg;
  pan_motor_cfg.offset = cfgs["panOffset"].as<double>();
  pan_motor_cfg.max_speed = cfgs["pan"]["maxSpeed"].as<double>();
  pan_motor_cfg.max_aspeed = cfgs["pan"]["maxASpeed"].as<double>();
  pan_motor_cfg.max_angle = cfgs["pan"]["maxAngle"].as<double>();
  pan_motor_cfg.min_angle = cfgs["pan"]["minAngle"].as<double>();
  pan_motor_cfg.coefficient = cfgs["pan"]["coefficient"].as<int>();
  pan_motor_cfg.pid_cfg.anglePidKp = cfgs["pan"]["position_Kp"].as<int>();
  pan_motor_cfg.pid_cfg.anglePidKi = cfgs["pan"]["position_Ki"].as<int>();
  pan_motor_cfg.pid_cfg.speedPidKp = cfgs["pan"]["velocity_Kp"].as<int>();
  pan_motor_cfg.pid_cfg.speedPidKi = cfgs["pan"]["velocity_Ki"].as<int>();
  pan_motor_->setMotorConfig(pan_motor_cfg);

  // tilt_motor_config
  PanTiltMotorConfig tilt_motor_cfg;
  tilt_motor_cfg.offset = cfgs["tiltOffset"].as<double>();
  tilt_motor_cfg.max_speed = cfgs["tilt"]["maxSpeed"].as<double>();
  tilt_motor_cfg.max_aspeed = cfgs["tilt"]["maxASpeed"].as<double>();
  tilt_motor_cfg.max_angle = cfgs["tilt"]["maxAngle"].as<double>();
  tilt_motor_cfg.min_angle = cfgs["tilt"]["minAngle"].as<double>();
  tilt_motor_cfg.coefficient = cfgs["tilt"]["coefficient"].as<int>();
  tilt_motor_cfg.pid_cfg.anglePidKp = cfgs["tilt"]["position_Kp"].as<int>();
  tilt_motor_cfg.pid_cfg.anglePidKi = cfgs["tilt"]["position_Ki"].as<int>();
  tilt_motor_cfg.pid_cfg.speedPidKp = cfgs["tilt"]["velocity_Kp"].as<int>();
  tilt_motor_cfg.pid_cfg.speedPidKi = cfgs["tilt"]["velocity_Ki"].as<int>();
  tilt_motor_->setMotorConfig(tilt_motor_cfg);
}

bool PanTiltMotorManager::DecodeMsgFromUART(uint8_t c) {
  static PanTiltSerialDecodeState decode_state = WAIT_FOR_HEADER_ID;
  static uint8_t receiving_frame_data_count = 0; //当前接收到的帧数据数量
  bool           new_frame_parsed           = false;
  switch (decode_state) {
    case WAIT_FOR_HEADER_ID: {
      if (c == SERIAL_FRAME_HEADER_ID) {
        uart_parsing_data.frame_cmd             = (uint8_t) 0x00;
        uart_parsing_data.frame_header_id       = (uint8_t) SERIAL_FRAME_HEADER_ID;
        uart_parsing_data.frame_motor_id        = (uint8_t) 0x00;
        uart_parsing_data.frame_data_len        = 0;
        uart_parsing_data.frame_header_checksum = (uint8_t) 0x00;
        memset(uart_parsing_data.frame_data, 0, FRAME_MAX_DATA_LEN);
        uart_parsing_data.frame_data_checksum = (uint8_t) 0x00;

        decode_state = WAIT_FOR_CMD;
//        SERIAL_DEBUG_PRINT<std::string>(" ",  "found sof");
      }
      break;
    }
    case WAIT_FOR_CMD: {
      uart_parsing_data.frame_cmd = c;
      decode_state = WAIT_FOR_NODE_ID;
//      SERIAL_DEBUG_PRINT<uint8_t>("frame cmd: ", uart_parsing_data.frame_cmd);
      break;
    }
    case WAIT_FOR_NODE_ID: {
      uart_parsing_data.frame_motor_id = c;
      decode_state = WAIT_FOR_DATA_LEN;
      break;
    }
    case WAIT_FOR_DATA_LEN: {
      uart_parsing_data.frame_data_len = c;
      decode_state = WAIT_FOR_HEADER_CHECKSUM;
      break;
    }
    case WAIT_FOR_HEADER_CHECKSUM: {
      uart_parsing_data.frame_header_checksum = c;
      if (uart_parsing_data.frame_data_len == 0) {
        new_frame_parsed           = true;
        receiving_frame_data_count = 0;
        decode_state               = WAIT_FOR_HEADER_ID;
        break;
      } else {
        decode_state = WAIT_FOR_SPECIFIC_DATA;
        break;
      }
    }
    case WAIT_FOR_SPECIFIC_DATA: {
      uart_parsing_data.frame_data[receiving_frame_data_count++] = c;
//#ifdef PRINT_CPP_DEBUG_INFO
//      std::cout << "1 byte added: " << std::hex << static_cast<int>(c) << std::dec << std::endl;
//#endif
      if (uart_parsing_data.frame_data_len == receiving_frame_data_count) {
        decode_state = WAIT_FOR_DATA_CHECKSUM;
      }
      break;
    }
    case WAIT_FOR_DATA_CHECKSUM: {
      uart_parsing_data.frame_data_checksum = c;
      new_frame_parsed           = true;
      receiving_frame_data_count = 0;
      decode_state               = WAIT_FOR_HEADER_ID;
//#ifdef PRINT_CPP_DEBUG_INFO
//      std::cout << "--- frame checksum: " << std::hex << static_cast<int>(frame_checksum) << std::dec << std::endl;
//        std::cout << "--- internal frame checksum: " << std::hex << static_cast<int>(internal_checksum) << std::dec << std::endl;
//#endif
      break;
    }
    default:break;
  }

  // parse segmented serial-data-frame
  if (new_frame_parsed) {
      uint8_t tmp =  uart_parsing_data.HeaderCheckSum();
    bool check_correct = uart_parsing_data.frame_header_checksum == uart_parsing_data.HeaderCheckSum();
//    check_correct = true;
    if (uart_parsing_data.frame_data_len != 0) {
      check_correct &= (uart_parsing_data.frame_data_checksum == uart_parsing_data.DataCheckSum());
    }
    if (check_correct) {
      ++uart_parsing_stats.frame_parsed;
//      ExtractStateMessage(feedback_msg);
    } else {
      ++uart_parsing_stats.frame_with_wrong_checksum;
    }
    new_frame_parsed &= check_correct;
  }
  return new_frame_parsed;
}

bool PanTiltMotorManager::InitAllDevices() {
  if (! Connect()) return false;
  // 启动控制环
  if(! device_ctl_cfg_.cmd_thread_started_) StartCmdThread();
  return true;
}

}