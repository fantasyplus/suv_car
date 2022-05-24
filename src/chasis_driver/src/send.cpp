#include <ros/ros.h>
#include "mpc_msgs/ControlCommand.h"

int main(int argc, char **argv)
{
    // ROS节点初始化
    
    ros::init(argc, argv, "send");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/info的topic
    ros::Publisher person_info_pub = n.advertise<mpc_msgs::ControlCommand>("/info", 10);

    // 设置循环的频率
    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok())
    {
        mpc_msgs::ControlCommand Vehicle_cmd;
        Vehicle_cmd.linear_acceleration = 3;
        Vehicle_cmd.linear_velocity = 2;
        Vehicle_cmd.steering_angle = 10; //转向角
        // 发布消息
		person_info_pub.publish(Vehicle_cmd);

       // 按照循环频率延时
        loop_rate.sleep();
    }

    return 0;
}