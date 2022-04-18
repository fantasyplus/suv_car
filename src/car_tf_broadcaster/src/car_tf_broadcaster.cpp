#include <nmea_msgs/Sentence.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Header.h>
#include <iostream>

geometry_msgs::PoseStamped _gnss_pose;
void callbackGnssPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    _gnss_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_tf_broadcaster");

    ros::NodeHandle _nh;
    ros::NodeHandle _private_nh;

    tf::TransformBroadcaster _tf_broadcaster;

    double lidar_trans_x, lidar_trans_y, lidar_trans_z;
    double lidar_rotation_roll, lidar_rotation_pitch, lidar_rotation_yaw;
    std::string map_frame, base_link_frame, lidar_frame;

    _private_nh.param<double>("lidar_trans_x", lidar_trans_x, 0.0);
    _private_nh.param<double>("lidar_trans_y", lidar_trans_y, 0.0);
    _private_nh.param<double>("lidar_trans_z", lidar_trans_z, 0.0);
    _private_nh.param<double>("lidar_rotation_roll", lidar_rotation_roll, 0.0);
    _private_nh.param<double>("lidar_rotation_pitch", lidar_rotation_pitch, 0.0);
    _private_nh.param<double>("lidar_rotation_yaw", lidar_rotation_yaw, 0.0);
    _private_nh.param<std::string>("map_frame", map_frame, "map");
    _private_nh.param<std::string>("base_link_frame", base_link_frame, "base_link");
    _private_nh.param<std::string>("lidar_frame_id", lidar_frame, "rslidar");

    ros::Subscriber sub_gnss_pose;
    sub_gnss_pose = _nh.subscribe<geometry_msgs::PoseStamped>("gnss_pose", 1, &callbackGnssPose);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();

        //广播map->base_link和base_link->lidar_rs的tf

        // map->base_link
        geometry_msgs::TransformStamped base_link_transform;

        base_link_transform.header.frame_id = map_frame;
        base_link_transform.header.stamp = ros::Time::now();

        base_link_transform.child_frame_id = base_link_frame;

        base_link_transform.transform.translation.x = _gnss_pose.pose.position.x;
        base_link_transform.transform.translation.y = _gnss_pose.pose.position.y;
        base_link_transform.transform.translation.z = _gnss_pose.pose.position.z;
        base_link_transform.transform.rotation = _gnss_pose.pose.orientation;

        _tf_broadcaster.sendTransform(base_link_transform);

        // base_link->lidar_rs
        geometry_msgs::TransformStamped lidar_transform;

        lidar_transform.header.frame_id = base_link_frame;
        lidar_transform.header.stamp = base_link_transform.header.stamp;

        lidar_transform.child_frame_id = lidar_frame;

        lidar_transform.transform.translation.x = lidar_trans_x;
        lidar_transform.transform.translation.y = lidar_trans_y;
        lidar_transform.transform.translation.z = lidar_trans_z;

        geometry_msgs::Quaternion q;
        q = tf::createQuaternionMsgFromRollPitchYaw(lidar_rotation_roll,
                                                    lidar_rotation_pitch,
                                                    lidar_rotation_yaw);
        lidar_transform.transform.rotation.w = q.w;
        lidar_transform.transform.rotation.x = q.x;
        lidar_transform.transform.rotation.y = q.y;
        lidar_transform.transform.rotation.z = q.z;

        _tf_broadcaster.sendTransform(lidar_transform);

        //广播结束

        loop_rate.sleep();
    }

    return 0;
}