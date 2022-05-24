#include "multi_sensor_calib_core.h"

MultiCalibCore::MultiCalibCore(ros::NodeHandle &nh)
{
    sub_point_cloud_right = new message_filters::Subscriber<sensor_msgs::PointCloud2>(
        nh, "/rslidar_points_right", 10, ros::TransportHints().tcpNoDelay());

    sub_point_cloud_left = new message_filters::Subscriber<sensor_msgs::PointCloud2>(
        nh, "/rslidar_points_left", 10, ros::TransportHints().tcpNoDelay());

    sync_ = new message_filters::Synchronizer<syncPolicy>(
        syncPolicy(10), *sub_point_cloud_right, *sub_point_cloud_left);
    sync_->registerCallback(boost::bind(&MultiCalibCore::Callback, this, _1, _2));

    pub_transform_points_ = nh.advertise<sensor_msgs::PointCloud2>("/rslidar_points_recover", 10);


    ros::spin();

}

MultiCalibCore::~MultiCalibCore(){}

void MultiCalibCore::Spin()
{

}

void MultiCalibCore::Callback(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr_r,
                            const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr_l){
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr_r(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr_l(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*in_cloud_ptr_r, *current_pc_ptr_r);

    pcl::fromROSMsg(*in_cloud_ptr_l, *current_pc_ptr_l);

    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    // transform_1 (0,0) = 0.0866609;
    // transform_1 (0,1) = -0.995835;
    // transform_1 (0,2) = -0.0283214;
    // transform_1 (0,3) = -1.440667;
    // transform_1 (1,0) = 0.996095;
    // transform_1 (1,1) = 0.0861311;
    // transform_1 (1,2) = 0.0194209;
    // transform_1 (1,3) = 1.484906;
    // transform_1 (2,0) = -0.0169007;
    // transform_1 (2,1) = -0.0298938;
    // transform_1 (2,2) = 0.99941;
    // transform_1 (2,3) = -0.092793;

    transform_1 (0,0) = 0.0973484;
    transform_1 (0,1) = -0.995177;
    transform_1 (0,2) = -0.0120874;
    transform_1 (0,3) = -1.579430;
    transform_1 (1,0) = 0.99525;
    transform_1 (1,1) = 0.097329;
    transform_1 (1,2) = 0.0021848;
    transform_1 (1,3) = 1.666694;
    transform_1 (2,0) = -0.000997812;
    transform_1 (2,1) = -0.0122427;
    transform_1 (2,2) = 0.999925;
    transform_1 (2,3) = -0.070700;


    // std::cout << "transform_1" << transform_1 << std::endl;

    pcl::transformPointCloud (*current_pc_ptr_r, *transformed_cloud, transform_1);


    *current_pc_ptr_l += *transformed_cloud;

    
    sensor_msgs::PointCloud2 pub_pc;
    pcl::toROSMsg(*current_pc_ptr_l, pub_pc);
 
    pub_pc.header = in_cloud_ptr_l->header;
 
    pub_transform_points_.publish(pub_pc);

}

// void MultiCalibCore::point_cb_left(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr)
// {
//     std::cout << "into left" << std::endl;
//     pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
//     // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
 
//     pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

//     *current_pc_ptr += *rslidar_right;
 
//     // pcl::VoxelGrid<pcl::PointXYZI> vg;
 
//     // vg.setInputCloud(current_pc_ptr);
//     // vg.setLeafSize(0.2f, 0.2f, 0.2f);
//     // vg.filter(*filtered_pc_ptr);
 
//     sensor_msgs::PointCloud2 pub_pc;
//     pcl::toROSMsg(*current_pc_ptr, pub_pc);
 
//     pub_pc.header = in_cloud_ptr->header;
 
//     pub_transform_points_.publish(pub_pc);
// }

// void MultiCalibCore::point_cb_right(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr)
// {

//     std::cout << "into right" << std::endl;
//     Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
//     transform_1 (0,0) = 0.0866609;
//     transform_1 (0,1) = -0.995835;
//     transform_1 (0,2) = -0.0283214;
//     transform_1 (0,3) = -1.440667;
//     transform_1 (1,0) = 0.996095;
//     transform_1 (1,1) = 0.0861311;
//     transform_1 (1,2) = 0.0194209;
//     transform_1 (1,3) = 1.484906;
//     transform_1 (2,0) = -0.0169007;
//     transform_1 (2,1) = -0.0298938;
//     transform_1 (2,2) = 0.99941;
//     transform_1 (2,3) = -0.092793;

//     std::cout << "transform_1" << transform_1 << std::endl;


//     pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

//     pcl::transformPointCloud (*current_pc_ptr, *transformed_cloud, transform_1);

//     std::cout << "end" << std::endl;

//     *rslidar_right = *transformed_cloud;

// }