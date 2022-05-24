#include "nodelet/nodelet.h"

#include "pluginlib/class_list_macros.h"
#include "ros/ros.h"


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
 
#include <pcl/filters/voxel_grid.h>
 
#include <sensor_msgs/PointCloud2.h>

// #include "std_msgs/Float64.h"

namespace rslidar_pre {
    class VoxelFilter: public nodelet::Nodelet {

        private:
            ros::Subscriber sub_point_cloud_;
 
            ros::Publisher pub_filtered_points_;

            // double value;

        public:
            VoxelFilter(){}
            void onInit(){
               
               ros::NodeHandle nh = getPrivateNodeHandle();

               
               
                //原本 nh.getParam("/节点名称/value")
                //但是 有了PrivateNodeHandle 不需要加节点名称了
                // nh.getParam("value",value);

                sub_point_cloud_ = nh.subscribe("/rslidar_points",10, &VoxelFilter::point_cb, this);
                // sub_point_cloud_ = nh.subscribe("/rslidar_points_recover",10, &VoxelFilter::point_cb, this);

    
                pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);
            
                // ros::spin();
                // while(ros::ok()){
                //     ros::spinOnce();
                // }

            }

            void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr){

                // ROS_INFO("hello -------");
                pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            
                pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
            
                pcl::VoxelGrid<pcl::PointXYZI> vg;
            
                vg.setInputCloud(current_pc_ptr);
                vg.setLeafSize(0.1f, 0.1f, 0.1f);
                vg.filter(*filtered_pc_ptr);
            
                sensor_msgs::PointCloud2 pub_pc;
                pcl::toROSMsg(*filtered_pc_ptr, pub_pc);
            
                pub_pc.header = in_cloud_ptr->header;
            
                pub_filtered_points_.publish(pub_pc);
            }

    };
};

PLUGINLIB_EXPORT_CLASS(rslidar_pre::VoxelFilter,nodelet::Nodelet)