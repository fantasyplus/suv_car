#pragma once

#include <string>

struct stamped_pose
{
    double utc_time;
    double lat;
    double lon;
    double alt;
    double yaw;
    double pitch;
    double roll;
    double velo;
};

void reset_stamped_pose(stamped_pose & sp);

bool parse_nmea(const std::string &nmea_line, const std::string &nmea_head, std::string &valid_line, stamped_pose &sp);

void blh2xyz(double lat, double lon, double alt, double lat_org, double lon_org, double &x, double &y, double &z);
