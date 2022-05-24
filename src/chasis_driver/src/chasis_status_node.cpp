
/**
 * 订阅/received_messages话题，转换消息类型后，发布名为vehicle_status的话题
 */
 
#include <ros/ros.h>
#include "socketcan_bridge/socketcan_to_topic.h"
#include "can_msgs/Frame.h"
#include "mpc_msgs/VehicleStatus.h"



mpc_msgs::VehicleStatus Vehicle_ST;
// 接收到订阅的消息后，会进入消息回调函数
//回调函数进行协议解析
void Callback_vehicle_status(const can_msgs::Frame::ConstPtr& msg)
{
    
    if(msg->id == 0x18105A2F) //0x1802A0B0
    {
        //初步协议，后期需要调整
        Vehicle_ST.car_mode = msg->data[0]&0x03;
        Vehicle_ST.error_level = (msg->data[0]>>4)&0x03;
        Vehicle_ST.steer =  (msg->data[2]<<8)+msg->data[1];
        Vehicle_ST.steer =  Vehicle_ST.steer*0.1-1080;
        //ROS_INFO("True");
    
    }
    else if(msg->id == 0x18125A2F)
    {
        Vehicle_ST.hand_brake = msg->data[0]&0x03;
        Vehicle_ST.gear = (msg->data[0]>>2)&0x0F;
        Vehicle_ST.stop = (msg->data[0]>>6)&0x01;
        Vehicle_ST.acc = msg->data[1]*0.1-12.8;
        Vehicle_ST.speed = msg->data[2]-50;
    }

    // 将接收到的消息打印出来
    //ROS_INFO("Frame: ID:%x, Data:%d", msg->id, msg->data[0]);
    //ROS_INFO("Frame: ID:%x", msg->id);

}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "chasis_status_node");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数Callback
    ros::Subscriber pose_sub = n.subscribe("/received_messages", 10, Callback_vehicle_status);

    //创建一个Publisher,发布名为vehicle_status的topic，该话题的message需要定义
    ros::Publisher turtle_vel_pub = n.advertise<mpc_msgs::VehicleStatus>("/vehicle_status", 10);

    //设置循环的频率
    ros::Rate loop_rate(50); //50Hz(20ms)chasis_status_node
    while(ros::ok())
    {


    turtle_vel_pub.publish(Vehicle_ST);
    //ROS_INFO("Sending...");
    ros::spinOnce();

    //按照循环频率延时
    loop_rate.sleep();

    }

    return 0;
}
