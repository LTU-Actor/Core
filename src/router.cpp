#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/Int64.h>

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "router");
    ros::start();

    ROS_INFO_STREAM("Hello ROS!");

    ros::NodeHandle nh{"~"};

    std_msgs::Int64 msg;
    msg.data = 0;

    ros::Publisher publisher = nh.advertise<std_msgs::Int64>("/router", 1);

    ros::Rate rate(200);
    while(ros::ok())
    {
        msg.data++;
        publisher.publish(msg);
        rate.sleep();
    }

    ros::shutdown();
}
