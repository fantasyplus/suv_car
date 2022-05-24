#include "mpc_follower/mpc_follower_core.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_follower");
  MPCFollower obj;
  ros::spin();
  ROS_INFO("Hello World!");
  return 0;
}
