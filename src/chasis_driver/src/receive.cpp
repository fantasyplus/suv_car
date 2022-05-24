 
#include <ros/ros.h>
#include "mpc_msgs/VehicleStatus.h"

// 接收到订阅的消息后，会进入消息回调函数
void vehicleInfoCallback(const mpc_msgs::VehicleStatus::ConstPtr& msg)
{
    // 将接收到的消息打印出来
    ROS_INFO("Subcribe  Info: car_mode:%d  error_level:%d", 
			 msg->car_mode, msg->error_level);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "receive");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    ros::Subscriber person_info_sub = n.subscribe("/vehicle_status", 10, vehicleInfoCallback);

    // 循环等待回调函数
    ros::spin();

    return 0;
}
