
/**
 * 订阅/received_messages话题，转换消息类型后，发布名为vehicle_status的话题
 */
 
#include <ros/ros.h>
#include "socketcan_bridge/topic_to_socketcan.h"
#include "can_msgs/Frame.h"
#include "mpc_msgs/ControlCommand.h"



mpc_msgs::ControlCommand Vehicle_cmd;
can_msgs::Frame Can_vehicle_cmd;
// 接收到订阅的消息后，会进入消息回调函数
void Callback_vehicle_cmd(const mpc_msgs::ControlCommand& msg)
{
    float speed,acc; //m/s m/s^2
    float steer; //方向盘转角
    uint8_t gear;
    uint8_t check = 0;
    Vehicle_cmd = msg;
    speed = Vehicle_cmd.linear_velocity * 3.6;
    acc = Vehicle_cmd.linear_acceleration;
    steer = -(Vehicle_cmd.steering_angle/3.1415*180)*14.6;
    ROS_INFO("speed:%f , direction:%f , steer:%f",speed,acc,steer);

    Can_vehicle_cmd.header.frame_id = "";
    Can_vehicle_cmd.header.stamp = ros::Time::now();
    
    Can_vehicle_cmd.id = 0x18125A1F;
    Can_vehicle_cmd.dlc = 8;
    Can_vehicle_cmd.is_rtr = 0;
    Can_vehicle_cmd.is_extended = 1;
    Can_vehicle_cmd.is_error = 0;
    Can_vehicle_cmd.data = {0};
    Can_vehicle_cmd.data[0] = speed+50;
    Can_vehicle_cmd.data[1] = (acc+12.8)*10;
    Can_vehicle_cmd.data[2] = ((uint16_t)((steer+1080)*10))&0xFF;
    Can_vehicle_cmd.data[3] = (((uint16_t)((steer+1080)*10))>>8)&0xFF;
    
    //异或校验
    for(int i=0;i<7;i++)
    {

       check ^= Can_vehicle_cmd.data[i];

    }

    Can_vehicle_cmd.data[7] = check;

}



int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "chasis_control_node");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber
    ros::Subscriber pose_sub = n.subscribe("/ctrl_raw", 10, Callback_vehicle_cmd);

    //创建一个Publisher,发布名为vehicle_status的topic，该话题的message需要定义
    ros::Publisher turtle_vel_pub = n.advertise<can_msgs::Frame>("/sent_messages", 10);

    //设置循环的频率
    ros::Rate loop_rate(50); //50Hz(20ms)chasis_control_node
    while(ros::ok())
    {

    turtle_vel_pub.publish(Can_vehicle_cmd);
    //ROS_INFO("Sending...");
    ros::spinOnce();

    //按照循环频率延时
    loop_rate.sleep();

    }

    return 0;
}
