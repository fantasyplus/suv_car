#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Header.h>
#include <iostream>

#include "serial_util.h"
#include "nmea_parser.h"
#include "status_protocol.h"

#include <string.h>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

const size_t max_buf_len = 512u;
//sentence: "$GPCHC,2133,116814.48,177.79,0.83,3.05,0.06,0.16,-0.02,-0.0530,0.0144,0.9949,32.17453255,118.48081929,18.42,-0.012,0.004,0.007,0.013,14,14,51,1,2*7D\r
int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmea_driver_node");

    std::string nmea_head, port_name;
    int baud_rate;
    double org_lat, org_lon;
    bool is_pub_sentence, is_pub_pose, is_broadcast_tf;
    std::string sentence_topic, pose_topic;
    std::string send_ip;
    int send_port;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_("~");

    private_nh_.param<std::string>("port_name", port_name, "/dev/ttyUSB0");
    private_nh_.param<int>("baud_rate", baud_rate, 115200);
    private_nh_.param<std::string>("nmea_head", nmea_head, "GPGGA");
    private_nh_.param<double>("org_lat", org_lat, 32.17209607);
    private_nh_.param<double>("org_lon", org_lon, 118.47587092);
    private_nh_.param<bool>("is_pub_sentence", is_pub_sentence, true);
    private_nh_.param<std::string>("sentence_topic", sentence_topic, "nmea_sentence");
    private_nh_.param<bool>("is_pub_pose", is_pub_pose, true);
    private_nh_.param<std::string>("pose_topic", pose_topic, "gnss_pose");
    private_nh_.param<bool>("is_broadcast_tf", is_broadcast_tf, true);

    //发送nav信息给笔记本的参数
    private_nh_.param<std::string>("send_ip", send_ip, "10.168.1.111");
    private_nh_.param<int>("send_port", send_port, 51001);

    const std::string MAP_FRAME_("map");
    const std::string GPS_FRAME_("gps");

    org_lat = org_lat * M_PI / 180;
    org_lon = org_lon * M_PI / 180;

    ros::Publisher pub_sentence;
    if (is_pub_sentence)
    {
        pub_sentence = nh_.advertise<nmea_msgs::Sentence>(sentence_topic, 10);
    }
    ros::Publisher pub_pose;
    if (is_pub_pose)
    {
        pub_pose = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic, 10);
    }

    tf::TransformBroadcaster broadcast_tf;

    // init serial
    int sp_fd = open_serial_port(port_name.c_str(), baud_rate, 8, 'N', 1);
    if (-1 == sp_fd)
    {
        std::cerr << "ERROR while open serial port!" << std::endl;
        exit(-1);
    }

    char recv_buf[max_buf_len] = {'0'};
    stamped_pose sp;
    uint32_t seq = 0;

    //配置发送状态信息的socket
    int send_sock = socket(PF_INET, SOCK_DGRAM, 0);
    if (send_sock < 0)
    {
        perror("nmea driver: open send_socket failed!\n");
        exit(-1);
    }
    struct sockaddr_in send_hostaddr;
    memset(&send_hostaddr, 0, sizeof(send_hostaddr));
    send_hostaddr.sin_family = AF_INET;
    send_hostaddr.sin_port = htons(static_cast<unsigned short>(send_port));
    send_hostaddr.sin_addr.s_addr = inet_addr(send_ip.c_str());

    NavStatus nav_diagram; //发送NavStatus状态信息
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();

        read_until(sp_fd, recv_buf, max_buf_len, "\r\n");
        auto stamp = ros::Time::now();
        std::string line(recv_buf);
        std::string valid_data;

        if (parse_nmea(line, nmea_head, valid_data, sp))
        {
            if (is_pub_sentence)
            {
                nmea_msgs::Sentence ns = nmea_msgs::Sentence();
                ns.header.stamp = stamp;
                ns.header.frame_id = GPS_FRAME_;
                ns.header.seq = seq;
                ns.sentence = valid_data;
                pub_sentence.publish(ns);
            }

            double x, y, z;
            blh2xyz(sp.lat, sp.lon, sp.alt, org_lat, org_lon, x, y, z);

            if (is_pub_pose)
            {
                geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped();
                pose.header.frame_id = MAP_FRAME_;
                pose.header.stamp = stamp;
                pose.header.seq = seq;
                pose.pose.position.x = y;
                pose.pose.position.y = x;
                pose.pose.position.z = z;
                pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(sp.roll, sp.pitch, sp.yaw);
                pub_pose.publish(pose);
            }

            if (is_broadcast_tf)
            {
                tf::Transform transform;
                tf::Quaternion quaternion;
                quaternion.setRPY(sp.roll, sp.pitch, sp.yaw);
                transform.setOrigin(tf::Vector3(y, x, z));
                transform.setRotation(quaternion);
                broadcast_tf.sendTransform(tf::StampedTransform(transform, stamp, MAP_FRAME_, GPS_FRAME_));
            }

            //发送NavStatus状态信息
            memset(&nav_diagram, 0, sizeof(nav_diagram));
            nav_diagram.stamp = seq;
            nav_diagram.nav_status = static_cast<unsigned char>(1);
            nav_diagram.gps_status = static_cast<unsigned char>(2);
            nav_diagram.yaw = static_cast<float>(sp.yaw / M_PI * 180.0);
            nav_diagram.lon = sp.lon / M_PI * 180.0;
            nav_diagram.lat = sp.lat / M_PI * 180.0;

            if (-1 == sendto(send_sock, &nav_diagram, sizeof(nav_diagram), 0, (struct sockaddr *)&send_hostaddr, sizeof(send_hostaddr)))
            {
                perror("nmea driver: send vehicle_show message error!\n");
            }
        }
        seq++;
        loop_rate.sleep();
    }

    close(sp_fd);

    return 0;
}