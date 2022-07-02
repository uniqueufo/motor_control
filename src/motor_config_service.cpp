#include <ros/ros.h>
#include <motor_control/MaxSpeed.h>
#include <motor_control/MotorConfig.h>
#include <motor_control/ManagerConfig.h>
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <iostream>

#define CONFIG_FILE_NAME "/home/lql/catkin_ws/src/motor_control/config/motor_manager.yaml"

//bool pubCommand = false;
//bool commandCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
//{
//    pubCommand = !pubCommand;
//    //显示请求数据
//    ROS_INFO("Publish turtle velocity command [%s]",pubCommand == true ? "Yes" : "No");
//    //设置反馈数据
//    res.success = true;
//    res.message = "Change turtle command state";
//    return true;
//}

bool motorConfigCallback(motor_control::MotorConfig::Request &req,
                         motor_control::MotorConfig::Response &res){
    YAML::Node cfgs = YAML::LoadFile(CONFIG_FILE_NAME);

    cfgs[req.motor_name]["position_offset"] = req.offset;
    cfgs[req.motor_name]["maxSpeed"] = req.max_speed;
    cfgs[req.motor_name]["maxASpeed"] = req.max_aspeed;
    cfgs[req.motor_name]["maxAngle"] = req.max_angle;
    cfgs[req.motor_name]["minAngle"] = req.min_angle;
    cfgs[req.motor_name]["coefficient"] = req.coefficient;
    cfgs[req.motor_name]["position_Kp"] = req.anglePidKp;
    cfgs[req.motor_name]["position_Ki"] = req.speedPidKi;
    cfgs[req.motor_name]["velocity_Kp"] = req.speedPidKp;
    cfgs[req.motor_name]["velocity_Ki"] = req.speedPidKi;
    cfgs[req.motor_name]["current_Kp"] = req.iqPidKp;
    cfgs[req.motor_name]["current_Ki"] = req.iqPidKi;

    std::ofstream fout(CONFIG_FILE_NAME);
    fout << cfgs <<std::endl;
    fout.close();
    res.result = "Update Motor config Success!";
    return true;
}

bool speedConfigCallback(motor_control::MaxSpeed::Request &req,
                         motor_control::MaxSpeed::Response &res){
    YAML::Node cfgs = YAML::LoadFile(CONFIG_FILE_NAME);
    cfgs[req.motor_name]["maxSpeed"] = req.maxSpeed;
    // update config file
    std::ofstream fout(CONFIG_FILE_NAME);
    fout << cfgs <<std::endl;
    fout.close();
    res.result = "Update Motor max_speed Success!";

    return true;
}

bool manageConfigCallback(motor_control::ManagerConfig::Request &req,
                         motor_control::ManagerConfig::Response &res){
    YAML::Node cfgs = YAML::LoadFile(CONFIG_FILE_NAME);
    cfgs["runBase"] = req.runBase;
    cfgs["runArm"] = req.runArm;
    cfgs["runPanTilt"] = req.runPanTilt;
    cfgs["runStopButton"] = req.runStopButton;
    cfgs["PanTiltComName"] = req.PanTiltComName;
    cfgs["baseLoopRate"] = req.baseLoopRate;
    cfgs["armLoopRate"] = req.armLoopRate;
    cfgs["panTiltLoopRate"] = req.panTiltLoopRate;
    cfgs["stopButtonLoopRate"] = req.stopButtonLoopRate;
    cfgs["maxLinearSpeed"] = req.maxLinearSpeed;
    cfgs["maxTurnSpeed"] = req.maxTurnSpeed;
    cfgs["wheelDist"] = req.wheelDist;
    cfgs["wheelPerimeter"] = req.wheelPerimeter;
    cfgs["elevatorUnitHeight"] = req.elevatorUnitHeight;
    cfgs["minElevatorHeight"] = req.minElevatorHeight;
    cfgs["maxElevatorHeight"] = req.maxElevatorHeight;
    cfgs["panOffset"] = req.panOffset;
    cfgs["tiltOffset"] = req.tiltOffset;

    // update config file
    std::ofstream fout(CONFIG_FILE_NAME);
    fout << cfgs <<std::endl;
    fout.close();
    res.result = "Update Motor max_speed Success!";

    return true;
    return true;
}

int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc, argv, "motor_config_node");
    //创建节点句柄
    ros::NodeHandle n;
    //创建一个名为/XXX 的server，注册回调函数Callback
    ros::ServiceServer motor_config_service = n.advertiseService("/motor_config_service", motorConfigCallback);
    ros::ServiceServer motor_speed_config_service = n.advertiseService("/motor_speed_config_service", speedConfigCallback);
    ros::ServiceServer manager_config_service = n.advertiseService("/manager_config_service", manageConfigCallback);
    //创建一个Publisher, 发布名为/turtle/cmd_vel的topic,消息类型为geometry_msgs::Twsit,队列长度为10
//    turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    //循环等待回调函数
    ROS_INFO("Ready to receive turtle command");
    //设置循环频率
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        //查看一次回调函数列表
        ros::spinOnce();
//        //如果标志为true，则发布速度质量
//        if(pubCommand)
//        {
//            geometry_msgs::Twist vel_msg;
//            vel_msg.linear.x = 2.0;
//            vel_msg.angular.z = 2.0;
//            turtle_vel_pub.publish(vel_msg);
//        }
        //按照循环频率延时
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}
