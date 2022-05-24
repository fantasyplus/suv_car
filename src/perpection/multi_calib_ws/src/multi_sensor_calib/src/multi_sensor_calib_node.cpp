#include "multi_sensor_calib_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_sensor_calib");

    ros::NodeHandle nh;

    MultiCalibCore core(nh);

    return 0;
}