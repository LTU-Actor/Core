#include <cstdlib>
#include <ros/ros.h>

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "router");
    ros::start();

    ROS_INFO_STREAM("Hello ROS!");

    ros::spin();
    ros::shutdown();
}
