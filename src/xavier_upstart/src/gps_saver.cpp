#include <iostream>
#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <fstream>
#include <boost/xpressive/xpressive.hpp>
#include <boost/lexical_cast.hpp>
#include <iomanip>

static auto gpchc = boost::xpressive::sregex::compile(R"--(^\$GPCHC,(\d+),([\d\.]+),([\d\.]+),([-\d\.]+),([-\d\.]+),[-\d\.]+,[-\d\.]+,[-\d\.]+,[-\d\.]+,[-\d\.]+,[-\d\.]+,([-\d\.]+),([-\d\.]+),([-\d\.]+),[-\d\.]+,[-\d\.]+,[-\d\.]+,([-\d\.]+),\d+,\d+,(\d+),\d+,\d+\*([0-9A-F]+)\r\n)--");
std::ofstream out("/home/ros/suv_car/src/xavier_upstart/script/gps_path.txt"/*, std::ios::out | std::ios::app*/);

void callbackNmeaSentence(const nmea_msgs::SentenceConstPtr &msg)
{
    std::string line = msg->sentence;

    boost::xpressive::smatch what;
    bool is_valid_sentence = boost::xpressive::regex_match(line, what, gpchc);
    if (is_valid_sentence)
    {
        double lat = boost::lexical_cast<double>(what[6]);
        double lon = boost::lexical_cast<double>(what[7]);

        out << std::fixed << std::setprecision(10) << lat << " " << lon << std::endl;

        ROS_INFO("save lat:%.10f,lon:%.10f", lat, lon);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_saver");

    ros::NodeHandle _nh;
    ros::Subscriber _sub_nmea = _nh.subscribe("/nmea_sentence", 1, &callbackNmeaSentence);

    while (ros::ok())
    {
        ros::spin();
    }
    out.close();
    return 0;
}