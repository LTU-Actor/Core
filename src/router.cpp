#include <cstdlib>
#include <ros/ros.h>
#include <sol/sol.hpp>
#include <std_msgs/Int64.h>

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "router");
    ros::start();

    ros::NodeHandle nh{"~"};

    sol::state lua;
    std_msgs::Int64 msg;
    msg.data = 0;
    lua.set_function("increment", [&msg] { ++msg.data; });

    ros::Publisher publisher = nh.advertise<std_msgs::Int64>("/router", 1);

    ros::Rate rate(200);
    while (ros::ok())
    {
        lua.script("increment()");
        publisher.publish(msg);
        rate.sleep();
    }

    ros::shutdown();
}
