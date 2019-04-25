#include <cstdlib>
#include <ros/ros.h>
#include <sol/sol.hpp>
#include <std_msgs/Int64.h>
#include <std_srvs/Empty.h>
#include <thread>
#include <chrono>

#include <ltu_actor_core/GetCurrentRoute.h>
#include <ltu_actor_core/GetRouteList.h>
#include <ltu_actor_core/LoadRoute.h>
#include <ltu_actor_core/SaveRoute.h>
#include <ltu_actor_core/SetTemporaryRoute.h>

sol::state lua;
sol::environment env(lua);
sol::load_result route;

bool
get_current_route_cb(ltu_actor_core::GetCurrentRoute::Request &req,
                     ltu_actor_core::GetCurrentRoute::Response &res)
{
    res.filename = "basic filename";
    res.content = "some content";
    return true;
}

bool
get_route_list_cb(ltu_actor_core::GetRouteList::Request &req,
                     ltu_actor_core::GetRouteList::Response &res)
{
    res.routes.push_back("route1.lua");
    res.routes.push_back("route2.lua");
    res.routes.push_back("route3.lua");
    res.routes.push_back("route4.lua");
    res.routes.push_back("route5.lua");
    res.routes.push_back("route6.lua");
    res.routes.push_back("route7.lua");
    return true;
}

bool
load_route_cb(ltu_actor_core::LoadRoute::Request &req,
                     ltu_actor_core::LoadRoute::Response &res)
{
    env = sol::environment(lua, sol::create);
    route = lua.load_file(req.filename);
    res.success = true;
    return true;
}

bool
save_route_cb(ltu_actor_core::SaveRoute::Request &req,
                     ltu_actor_core::SaveRoute::Response &res)
{
    res.success = false;
    return true;
}

bool
set_temporary_route_cb(ltu_actor_core::SetTemporaryRoute::Request &req,
                     ltu_actor_core::SetTemporaryRoute::Response &res)
{
    env = sol::environment(lua, sol::create);
    route = lua.load(req.content);
    res.success = true;
    return true;
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "router");
    ros::start();

    ros::NodeHandle nh{"~"};

    std_msgs::Int64 msg;
    msg.data = 0;

    ros::Publisher publisher = nh.advertise<std_msgs::Int64>("/router", 1);
    ros::ServiceServer get_current_route = nh.advertiseService("get_current_route", &get_current_route_cb);
    ros::ServiceServer get_route_list = nh.advertiseService("get_route_list", &get_route_list_cb);
    ros::ServiceServer load_route = nh.advertiseService("load_route", &load_route_cb);
    ros::ServiceServer save_route = nh.advertiseService("save_route", &save_route_cb);
    ros::ServiceServer set_temporary_route = nh.advertiseService("set_temporary_route", &set_temporary_route_cb);
    ros::ServiceClient estop = nh.serviceClient<std_srvs::Empty>("/estop/stop");

    lua.open_libraries();
    lua.set_function("heartbeat", [&msg] { ++msg.data; });
    lua.set_function("spin_once", [] { ros::spinOnce(); });
    lua.set_function("estop", [&estop] { std_srvs::Empty e; estop.call(e); });
    lua.set_function("spin_for", [](int ms) {
        auto begin = std::chrono::high_resolution_clock::now();
        ros::Rate r(50);
        while(true)
        {
            auto elapsed = std::chrono::high_resolution_clock::now() - begin;
            if (elapsed >= std::chrono::milliseconds(ms)) break;
            ros::spinOnce();
            r.sleep();
        }
    });

    env = sol::environment(lua, sol::create);

    route = lua.load("heartbeat();spin_for(500);");

    ros::Rate rate(30);
    while (ros::ok())
    {
        route(env);
        publisher.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
}
