#include <ros/ros.h>
// #include <mpc_msgs/Lane.h>
// #include <mpc_msgs/Waypoint.h>
// #include <mpc_msgs/Waypoint.h>
// #include "turtlesim/Pose.h"
//#include "chasis_driver/VehicleStatus.h"
#include "std_msgs/String.h"
// #include "mpc_follower/qp_solver/qp_solver_unconstr_fast.h"
// #include "mpc_follower/mpc_trajectory.h"
// #include <mpc_follower/amathutils_lib/amathutils.hpp>
//#include "mpc_follower/mpc_utils.h"
#include "mpc_follower/mpc_follower_core.h"
#include <qpOASES.hpp>
#include <fstream>

 

void callbackRefPath(const geometry_msgs::PoseStamped &msg)
{
  // mpc_msgs::Lane current_waypoints_;  
  // current_waypoints_ = *msg;
  // //DEBUG_INFO("[MPC] path callback: received path size = %lu", current_waypoints_.waypoints.size());


  // //ROS_INFO("traj: %lf",current_waypoints_.wa);

  // MPCTrajectory traj;
  
  // //计算相对时间，设置路径起点的时间为0.之后根据每个路径点的目标速度以及点与点之间的距离计算出车辆到达每个路径点时相对于起点的时间
  // /* calculate relative time */
  // std::vector<double> relative_time;
  // MPCUtils::calcPathRelativeTime(current_waypoints_, relative_time);
  // // DEBUG_INFO("[MPC] path callback: relative_time.size() = %lu, front() = %f, back() = %f",
  // //            relative_time.size(), relative_time.front(), relative_time.back());
  

  // //按照设置的均分距离traj_resample_dist_使用线性插值法对路径进行均分重置
  // /* resampling */
  // MPCUtils::convertWaypointsToMPCTrajWithDistanceResample(current_waypoints_, relative_time, 0.1, traj);
  // //此函数是对路径点的航向角进行修改，保证相邻路径点的航向角的差值处于-pi～pi之间
  // MPCUtils::convertEulerAngleToMonotonic(traj.yaw);
  // //DEBUG_INFO("[MPC] path callback: resampled traj size() = %lu", traj.relative_time.size());
  
  // //对重置的路径进行光滑处理
  // /* path smoothing */
  // bool enable_path_smoothing_ = 1;
  // int path_filter_moving_ave_num_ = 35;
  // int path_smoothing_times_ = 1;
  // if (enable_path_smoothing_)
  // {
  //   for (int i = 0; i < path_smoothing_times_; ++i)
  //   {
  //     if (!MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.x) ||
  //         !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.y) ||
  //         !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.yaw) ||
  //         !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.vx))
  //     {
  //       ROS_WARN("[MPC] path callback: filtering error. stop filtering");
  //       return;
  //     }
  //   }
  // }
  // bool enable_yaw_recalculation_ = 0;
  // if (enable_yaw_recalculation_)
  // {
  //   MPCUtils::calcTrajectoryYawFromXY(traj);
  //   MPCUtils::convertEulerAngleToMonotonic(traj.yaw);
  // }
  
  // MPCUtils::calcTrajectoryYawFromXY(traj);
  // for (uint i = 0; i < traj.size(); ++i)
  // {
  //   ROS_INFO("traj: %lf",traj.yaw[i]);
  //   //ROS_INFO("NEW");

  // }
  geometry_msgs::PoseStamped veh_pose;
  veh_pose = msg;
  const double current_yaw = tf2::getYaw(veh_pose.pose.orientation);
  ROS_INFO("current_yaw :%lf",current_yaw/M_PI*180);

}

void callbackVehicleStatus(const mpc_msgs::VehicleStatus &msg)
{
  //天美SUV，转向角：方向盘转角=1：14.6
  double x;
  x = msg.steer/14.6/180*3.1415;  //当前转向角

  ROS_INFO("x: %lf",x);
}

int main(int argc, char **argv)
{
    // ROS节点初始化
    
    ros::init(argc, argv, "test_node");



    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/info的topic
    // ros::Publisher person_info_pub = n.advertise<std_msgs::String>("/info", 10);

    // ros::Subscriber sub_ref_path_ = n.subscribe("/vehicle_status", 1, callbackVehicleStatus);
    // ros::spin();
    
    // std::ifstream myfile("mpc_traj.txt");
    // if(myfile.fail()){
    //     ROS_INFO("read failed");
    //     return ;

    // }
    // 设置循环的频率
    ros::Rate loop_rate(1);


    // int count = 0;
    // while (ros::ok())
    // {
    //     std_msgs::String msg;
    //     msg.data = "Hello World!";
    //     // 发布消息
		// person_info_pub.publish(msg);

    //    	ROS_INFO("Hello World!");
    //     //ROS_INFO_STREAM("Hello World!");
    //    // 按照循环频率延时
    //     loop_rate.sleep();
    // }
    
    //amathutils::normalizeRadian();
    // double n_angle ;
    // n_angle = 3.14;
    // n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;
  

    ROS_INFO("Hello!");
    return 0;
}