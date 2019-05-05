// stdlib
#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>
#include <unordered_map>

// file io
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

// Lua
#include <sol/sol.hpp>

// Ros
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Char.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>

// Internal
#include <ltu_actor_core/GetCurrentRoute.h>
#include <ltu_actor_core/GetRouteList.h>
#include <ltu_actor_core/LoadRoute.h>
#include <ltu_actor_core/SaveRoute.h>
#include <ltu_actor_core/SetTemporaryRoute.h>

// Local
#include "subpub.h"

bool load_next = false;
std::string route_filename = "init";
std::string route_contents =
    "--[[\n"
    "  heartbeat()                          toggle the heartbeat status topic to show the route is running\n"
    "  spin_once()                          allow ROS topics/services to be serviced\n"
    "  spin_for(int ms)                     delay for milliseconds while also servicing ROS\n"
    "  send(float linear, float angular)    continuously send cmd commands (forward/reverse & steering)\n"
    "  send_topic(string topic)             continuously forward a Twist topic as for cmd instead of constants\n"
    "  estop()                              trigger an estop\n"
    "]]\n\n"
    "send(0.0, 0.0);\nheartbeat();\nspin_for(500);";

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
    int f = open(filename.c_str(), O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR);
    if (f == -1) return false;

    size_t count = contents.length();
    const char *ptr = contents.data();

    while (count > 0)
    {
        ssize_t num_write = write(f, ptr, count);
        if (num_write <= 0) break;
        ptr += num_write;
        count -= num_write;
    }

    close(f);
    return count == 0;
}

std::string
load_file(const std::string &filename)
{
    int f = open(filename.c_str(), 0);
    if (f < 0) return {};

    int buf_size = getpagesize();
    char *buf = new char[buf_size];

    std::string ret;

    ssize_t num_read;
    while ((num_read = read(f, buf, buf_size - 1)) > 0)
    {
        buf[num_read] = '\0';
        ret += std::string(buf);
    }

    // errno, dont return partial file.
    if (num_read < 0) ret = std::string();

    delete[] buf;
    close(f);
    return ret;
}

bool
get_current_route_cb(ltu_actor_core::GetCurrentRoute::Request &req, ltu_actor_core::GetCurrentRoute::Response &res)
{
    res.filename = route_filename;
    res.content = route_contents;
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

    if (std::find(res.routes.begin(), res.routes.end(), route_filename) == res.routes.end())
        res.routes.push_back(route_filename);

    return true;
}

bool
load_route_cb(ltu_actor_core::LoadRoute::Request &req, ltu_actor_core::LoadRoute::Response &res)
{
    std::string filename = script_folder + req.filename;
    std::string contents = load_file(filename);

    if (contents.empty())
    {
        ROS_ERROR_STREAM("Failed to load file " << filename << "!");
        res.success = false;
        return true;
    }

    load_next = true;
    res.success = true;
    route_contents = contents;
    route_filename = req.filename;

    return true;
}

bool
save_route_cb(ltu_actor_core::SaveRoute::Request &req, ltu_actor_core::SaveRoute::Response &res)
{
    if (!ends_with(req.filename, ".lua"))
    {
        res.success = false;
        ROS_ERROR_STREAM("Not a Lua file!");
    }
    else
    {
        res.success = save_file(script_folder + req.filename, req.content);
        if (!res.success) ROS_ERROR_STREAM("Failed to write file!");
    }
    return true;
}

bool
set_temporary_route_cb(ltu_actor_core::SetTemporaryRoute::Request &req,
                       ltu_actor_core::SetTemporaryRoute::Response &res)
{
    load_next = true;
    res.success = true;
    route_contents = req.content;
    return true;
}

int
main(int argc, char **argv)
{
    /*
     * Init
     ******************************************************************************************************************/
    ros::init(argc, argv, "router");
    ros::start();
    ros::NodeHandle nh{"~"};

    // ROS Params
    std::string estop_service;
    nh.getParam("script_folder", script_folder);
    if (!ends_with(script_folder, "/")) script_folder += "/";
    nh.getParam("estop_service", estop_service);

    // Pubs & Subs
    ros::Publisher twist_out = nh.advertise<geometry_msgs::Twist>("cmd", 1);
    ros::Subscriber twist_in;
    ros::Publisher hb = nh.advertise<std_msgs::Bool>("heartbeat", 1);
    ros::ServiceServer get_current_route = nh.advertiseService("get_current_route", &get_current_route_cb);
    ros::ServiceServer get_route_list = nh.advertiseService("get_route_list", &get_route_list_cb);
    ros::ServiceServer load_route = nh.advertiseService("load_route", &load_route_cb);
    ros::ServiceServer save_route = nh.advertiseService("save_route", &save_route_cb);
    ros::ServiceServer set_temporary_route = nh.advertiseService("set_temporary_route", &set_temporary_route_cb);
    ros::ServiceClient estop = nh.serviceClient<std_srvs::Empty>(estop_service);

    // Heartbeat
    std_msgs::Bool hb_msg;
    hb_msg.data = false;

    // Constant twist pub
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = cmd_msg.linear.y = cmd_msg.linear.z = 0;
    cmd_msg.angular.x = cmd_msg.angular.y = cmd_msg.angular.z = 0;
    ros::Timer cmd_timer =
        nh.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent &e) { twist_out.publish(cmd_msg); });
    cmd_timer.start();



    /*
     * Setup Lua environment
     ******************************************************************************************************************/

    sol::state lua;
    std::unordered_map<std::string, std::shared_ptr<RouteSub_>> route_subs;
    std::unordered_map<std::string, std::shared_ptr<RoutePub_>> route_pubs;

    // Might as well open a decent amount of stuff since we don't create many Lua states
    lua.open_libraries(sol::lib::base, sol::lib::string, sol::lib::math, sol::lib::table, sol::lib::os);

    // Used in wrappers that will exit from Lua on return true
    lua.set_function("__spin_once", []() -> bool {
        ros::spinOnce();
        return load_next;
    });
    lua.set_function("__spin_for", [](int ms) -> bool {
        auto begin = std::chrono::high_resolution_clock::now();
        ros::Rate r(50);
        while (true)
        {
            auto elapsed = std::chrono::high_resolution_clock::now() - begin;
            if (elapsed >= std::chrono::milliseconds(ms)) break;
            ros::spinOnce();
            if (load_next) return true; // Return as early as possible if needed
            r.sleep();
        }
        return load_next;
    });

    lua.set_function("heartbeat", [&] {
        hb_msg.data = !hb_msg.data;
        hb.publish(hb_msg);
    });
    lua.set_function("estop", [&estop] {
        std_srvs::Empty e;
        estop.call(e);
    });
    lua.set_function("send_topic", [&](std::string topic) {
        // Stop anything caused by send()
        cmd_timer.stop();

        // Only screw with the subscriber if it is different
        if (twist_in.getTopic() != topic)
        {
            boost::function<void(const geometry_msgs::Twist &)> callback = [&](const geometry_msgs::Twist &msg) {
                twist_out.publish(msg);
            };
            twist_in = nh.subscribe<geometry_msgs::Twist>(topic, 1, callback);
        }
    });
    lua.set_function("send", [&](float linear, float angular) {
        // Stop anything caused by send_topic, only if it is running
        if (!twist_in.getTopic().empty()) twist_in = {};

        // Our car only listens for linear.x and angular.z
        cmd_msg.linear.x = linear;
        cmd_msg.linear.y = 0;
        cmd_msg.linear.z = 0;
        cmd_msg.angular.x = 0;
        cmd_msg.angular.y = 0;
        cmd_msg.angular.z = angular;

        cmd_timer.start();
    });

    lua_add_sub(lua, nh, route_subs, Bool, bool, data);
    lua_add_sub(lua, nh, route_subs, Byte, unsigned char, data);
    lua_add_sub(lua, nh, route_subs, Char, char, data);
    lua_add_sub(lua, nh, route_subs, Duration, double, data.toSec());
    lua_add_sub(lua, nh, route_subs, Float32, float, data);
    lua_add_sub(lua, nh, route_subs, Float64, double, data);
    lua_add_sub(lua, nh, route_subs, Int16, int, data);
    lua_add_sub(lua, nh, route_subs, Int32, int, data);
    lua_add_sub(lua, nh, route_subs, Int8, int, data);
    lua_add_sub(lua, nh, route_subs, String, std::string, data);
    lua_add_sub(lua, nh, route_subs, Time, double, data.toSec());
    lua_add_sub(lua, nh, route_subs, UInt16, unsigned int, data);
    lua_add_sub(lua, nh, route_subs, UInt32, unsigned int, data);
    lua_add_sub(lua, nh, route_subs, UInt8, unsigned int, data);

    lua_add_pub(lua, nh, route_pubs, Bool, bool, data);
    lua_add_pub(lua, nh, route_pubs, Byte, unsigned char, data);
    lua_add_pub(lua, nh, route_pubs, Char, char, data);
    // lua_add_pub(lua, nh, route_pubs, Duration, double, data.toSec());
    lua_add_pub(lua, nh, route_pubs, Float32, float, data);
    lua_add_pub(lua, nh, route_pubs, Float64, double, data);
    lua_add_pub(lua, nh, route_pubs, Int16, int, data);
    lua_add_pub(lua, nh, route_pubs, Int32, int, data);
    lua_add_pub(lua, nh, route_pubs, Int8, int, data);
    lua_add_pub(lua, nh, route_pubs, String, std::string, data);
    // lua_add_pub(lua, nh, route_pubs, Time, double, data.);
    lua_add_pub(lua, nh, route_pubs, UInt16, unsigned int, data);
    lua_add_pub(lua, nh, route_pubs, UInt32, unsigned int, data);
    lua_add_pub(lua, nh, route_pubs, UInt8, unsigned int, data);


    // wrappers to exit the script to allow replacing it
    lua.script(R"(
function spin_once()
    if __spin_once() then
        error("reloading script")
    end
end
function spin_for(ms)
    if __spin_for(ms) then
        error("reloading script")
    end
end
        )");



    /*
     * Main looping
     ******************************************************************************************************************/

    sol::environment env = sol::environment(lua, sol::create);
    sol::load_result route = lua.load(route_contents);

    ros::Rate rate(30);
    while (ros::ok())
    {
        ros::spinOnce();

        try
        {
            route(env);
            // replace the route here so the route doesn't replace itself from a spin callback
            if (load_next)
            {
                env = sol::environment(lua, sol::create); // reset the route sandbox
                route = lua.load(route_contents);
                route_subs.clear();
                route_pubs.clear();
                load_next = false;

                if (!route.valid()) ROS_ERROR_STREAM("Loaded invalid Lua!");
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_STREAM("SOL: " << e.what());
        }

        rate.sleep();
    }

    ros::shutdown();
}
