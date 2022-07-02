#include "ros/ros.h"
#include "std_msgs/String.h"
#include "motor_control/MotorState.h"
#include "motor_control/MotorControlCmd.h"
#include "motor_control/MotorCommonMsg.h"

// import
 #include "../our_hw_sdk/src/hal/user_hw/pan_tilt_motor_manager.h"
 #include "../our_hw_sdk/src/hal/user_hw/pan_tilt_motor.h"
 #include "../our_hw_sdk/src/hal/serial_com_motor.h"
 #include "../our_hw_sdk/src/hal/user_hw/motor_cfg/pan_tilt_motor_msg_process.h"

#include <sstream>
 #define DEVICE_NAME "/dev/ttyUSB0"
 hal::PanTiltMotorManager motorManager = hal::PanTiltMotorManager(DEVICE_NAME);


motor_control::MotorState parseStateFeedBack(uint8_t motor_id, PanTiltMotorStateFeedback motorStateFeedback) {
    motor_control::MotorState motorState;
 	motorState.motor_id = motor_id;
 	motorState.position = motorStateFeedback.position;
 	motorState.temperature = motorStateFeedback.temperature;
 	motorState.speed = motorStateFeedback.temperature;
 	motorState.torque_current = motorStateFeedback.temperature;
 	motorState.torque = motorStateFeedback.temperature;
 	return motorState;
 }

 // send command
 void setCmdCallback(const motor_control::MotorControlCmd cmd)
 {
    if(cmd.speed > 0) {
        motorManager.SendSpeedControldCmd(cmd.motor_id, cmd.speed);
    } else if(cmd.circle_pose > 0){
        motorManager.SendCircleCPositionControlCmd(cmd.motor_id, cmd.circle_pose, cmd.max_speed_limit);
    }
 	ROS_INFO("motor %d set cmd_config", 1);
 }

 // set zero
 void setZeroCallback(const motor_control::MotorCommonMsg motor_cm_msg)
 {
     motorManager.SendSetZeroPointToROMCmd(motor_cm_msg.motor_id);
     ros::shutdown();
     exit(0);
 }

 // stop motor
 void stopMotorCallback(const motor_control::MotorCommonMsg motor_cm_msg)
 {
    motorManager.SendStopControlCmd(motor_cm_msg.motor_id);
 }

 void SigintHandler(int sig) {
    ROS_INFO("shutting down!");
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
	 ros::init(argc, argv, "motor_manager");
	 ros::NodeHandle n;
	 // ====publish====
	 ros::Publisher state_pub = n.advertise<motor_control::MotorState>("motor_state", 1000);
	 
	 // ====subscribe====
	 // control
	 ros::Subscriber cmd_sub = n.subscribe("motor_cmd", 20, setCmdCallback);
     ros::Subscriber set_zero_sub = n.subscribe("set_zero", 1, setZeroCallback);
     ros::Subscriber stop_motor = n.subscribe("stop_motor", 1, stopMotorCallback);

	 // loop rate
     ros::Rate loop_rate(10);
	
	 std::string device_name = "/dev/ttyUSB0";
	 motorManager.GetParamFromFile(std::string("/home/lql/catkin_ws/src/motor_control/config/motor_manager.yaml"));
	 motorManager.InitAllDevices();

    signal(SIGINT, SigintHandler);

	 int count = 0;
	 while(ros::ok())
	 {
        motor_control::MotorState motorFirstState;
//        motor_control::MotorState motorSecondState;

	 	motorFirstState = parseStateFeedBack(1, motorManager.GetMotorState(1));
//	 	motorSecondState = parseStateFeedBack(2, motorManager.GetMotorState(2));
        state_pub.publish(motorFirstState);
//         state_pub.publish(motorSecondState);

//	 	ROS_INFO("motor %d data send", 1);
	 	ros::spinOnce();
	 	// sleep
	 	loop_rate.sleep();
	 	++count;
	 }
	
	 motorManager.SendStopControlCmd(1);
//	 motorManager.SendStopControlCmd(2);
	return 0;
}
