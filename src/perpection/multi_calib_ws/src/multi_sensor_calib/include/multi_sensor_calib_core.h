#pragma once
 
#include <ros/ros.h>
 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
 

#include <sensor_msgs/PointCloud2.h>

using namespace sensor_msgs;
using namespace message_filters;

typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                       sensor_msgs::PointCloud2>
    syncPolicy;

 
class MultiCalibCore
{
 
private:
    message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_point_cloud_left;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_point_cloud_right;
 
    message_filters::Synchronizer<syncPolicy> *sync_;

    ros::Publisher pub_transform_points_;
 
 
public:

    
    MultiCalibCore(ros::NodeHandle &nh);
    ~MultiCalibCore();

    void Callback(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr,
                  const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr1);
    void Spin();
};