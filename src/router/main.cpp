#include <chrono>
#include <cstdlib>
#include <ros/ros.h>
#include <sol/sol.hpp>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <thread>

// file io
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <ltu_actor_core/GetCurrentRoute.h>
#include <ltu_actor_core/GetRouteList.h>
#include <ltu_actor_core/LoadRoute.h>
#include <ltu_actor_core/SaveRoute.h>
#include <ltu_actor_core/SetTemporaryRoute.h>

sol::state lua;
sol::environment env(lua);
sol::load_result route;

bool load_next = false;
sol::environment next_env;
sol::load_result next_route;

std::string script_folder;

// https://stackoverflow.com/a/20446239
bool
ends_with(const std::string &str, const std::string &suffix)
{
    return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::vector<std::string>
get_directory_contents(const std::string &dir)
{
    DIR *dp;
    struct dirent *dirp;
    if ((dp = opendir(dir.c_str())) == NULL)
    {
        ROS_ERROR_STREAM("Failed to open folder " << dir);
        return {};
    }

    std::vector<std::string> ret;
    while ((dirp = readdir(dp)) != NULL) ret.push_back(std::string(dirp->d_name));

    closedir(dp);
    return ret;
}

bool
save_file(const std::string &filename, const std::string &contents)
{
    int f = open(filename.c_str(), O_CREAT | O_TRUNC);
    if (f < 0) return false;

    size_t count = contents.length();
    const char *ptr = contents.data();

    while (count > 0)
    {
        size_t num_write = write(f, ptr, count);
        if (num_write <= 0) break;
        ptr += num_write;
        count -= num_write;
    }

    close(f);
    return count == 0;
}

bool
get_current_route_cb(ltu_actor_core::GetCurrentRoute::Request &req, ltu_actor_core::GetCurrentRoute::Response &res)
{
    res.filename = "basic filename";
    res.content = "some content";
    return true;
}

bool
get_route_list_cb(ltu_actor_core::GetRouteList::Request &req, ltu_actor_core::GetRouteList::Response &res)
{
    res.routes = get_directory_contents(script_folder);
    for (auto it = res.routes.begin(); it != res.routes.end();)
    {
        if (!ends_with(*it, ".lua"))
            it = res.routes.erase(it);
        else
            ++it;
    }

    return true;
}

bool
load_route_cb(ltu_actor_core::LoadRoute::Request &req, ltu_actor_core::LoadRoute::Response &res)
{
    next_route = lua.load_file(script_folder + req.filename);
    if (next_route.valid())
    {
        next_env = sol::environment(lua, sol::create);
        load_next = true;
        res.success = true;
    }
    else
    {
        res.success = false;
    }
    return true;
}

bool
save_route_cb(ltu_actor_core::SaveRoute::Request &req, ltu_actor_core::SaveRoute::Response &res)
{
    // don't save an invalid route!
    sol::load_result lr = lua.load(req.content);
    if (lr.valid())
        res.success = save_file(req.filename, req.content);
    else
        res.success = false;
    return true;
}

bool
set_temporary_route_cb(ltu_actor_core::SetTemporaryRoute::Request &req,
                       ltu_actor_core::SetTemporaryRoute::Response &res)
{
    next_route = lua.load(req.content);
    if (next_route.valid())
    {
        next_env = sol::environment(lua, sol::create);
        load_next = true;
        res.success = true;
    }
    else
    {
        res.success = false;
    }
    return true;
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "router");
    ros::start();

    ros::NodeHandle nh{"~"};

    nh.getParam("script_folder", script_folder);
    if (!ends_with(script_folder, "/")) script_folder += "/";

    // Hartbeat
    std_msgs::Bool hb_msg;
    hb_msg.data = false;

    ros::Publisher hb = nh.advertise<std_msgs::Bool>("heartbeat", 1);
    ros::ServiceServer get_current_route = nh.advertiseService("get_current_route", &get_current_route_cb);
    ros::ServiceServer get_route_list = nh.advertiseService("get_route_list", &get_route_list_cb);
    ros::ServiceServer load_route = nh.advertiseService("load_route", &load_route_cb);
    ros::ServiceServer save_route = nh.advertiseService("save_route", &save_route_cb);
    ros::ServiceServer set_temporary_route = nh.advertiseService("set_temporary_route", &set_temporary_route_cb);
    ros::ServiceClient estop = nh.serviceClient<std_srvs::Empty>("/estop/stop");

    lua.open_libraries();
    lua.set_function("heartbeat", [&] {
        hb_msg.data = !hb_msg.data;
        hb.publish(hb_msg);
    });
    lua.set_function("spin_once", [] { ros::spinOnce(); });
    lua.set_function("estop", [&estop] {
        std_srvs::Empty e;
        estop.call(e);
    });
    lua.set_function("spin_for", [](int ms) {
        auto begin = std::chrono::high_resolution_clock::now();
        ros::Rate r(50);
        while (true)
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
        ros::spinOnce();

        // replace the route here so the route dosen't replace itself from a spin callback
        if (load_next)
        {
            env = next_env;
            route = next_route;
            load_next = false;
        }

        rate.sleep();
    }

    ros::shutdown();
}
