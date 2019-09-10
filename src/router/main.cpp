// stdlib
#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <algorithm>

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
#include <sensor_msgs/NavSatFix.h>
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

const char *route_setup =
#include "setup.lua"
    ;

// Local
#include "subpub.h"

ros::NodeHandle *nh = 0;

// Pubs & Subs
ros::Publisher twist_out;
ros::Subscriber twist_in;
ros::Publisher hb;
ros::ServiceServer get_current_route;
ros::ServiceServer get_route_list;
ros::ServiceServer load_route;
ros::ServiceServer save_route;
ros::ServiceServer set_temporary_route;
ros::ServiceClient estop;
ros::Subscriber navsatfix;

// main timer
ros::Timer cmd_timer;
geometry_msgs::Twist cmd_msg;

// route state
sol::state lua;
std::unordered_map<std::string, std::shared_ptr<RouteSub_>> route_subs;
std::unordered_map<std::string, std::shared_ptr<RoutePub_>> route_pubs;

// random state
bool load_next = true;
std::string route_filename = "init";
std::string route_contents =
#include "default_route.lua"
    ;

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
    std::sort(ret.begin(), ret.end());
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

void
navsatfix_cb(const sensor_msgs::NavSatFix &msg)
{
    lua["_latitude"] = msg.latitude;
    lua["_longitude"] = msg.longitude;
    lua["_altitude"] = msg.altitude;
}

void
create_subs_n_pubs()
{
    std::string estop_service;
    if (!nh->getParam("estop_service", estop_service))
    {
        ROS_ERROR_STREAM("Must specify estop_service!");
        throw std::runtime_error("Must specify estop_service!");
    }

    std::string navsatfix_topic;
    if (!nh->getParam("navsatfix_topic", navsatfix_topic))
    {
        ROS_ERROR_STREAM("Must specify navsatfix_topic!");
        throw std::runtime_error("Must specify navsatfix_topic!");
    }

    twist_out = nh->advertise<geometry_msgs::Twist>("cmd", 0);
    hb = nh->advertise<std_msgs::Bool>("heartbeat", 0);
    get_current_route = nh->advertiseService("get_current_route", &get_current_route_cb);
    get_route_list = nh->advertiseService("get_route_list", &get_route_list_cb);
    load_route = nh->advertiseService("load_route", &load_route_cb);
    save_route = nh->advertiseService("save_route", &save_route_cb);
    set_temporary_route = nh->advertiseService("set_temporary_route", &set_temporary_route_cb);
    estop = nh->serviceClient<std_srvs::Empty>(estop_service);
    navsatfix = nh->subscribe(navsatfix_topic, 5, &navsatfix_cb);
}

void
heartbeat(void)
{
    static std_msgs::Bool hb_msg = {};
    hb_msg.data = !hb_msg.data;
    hb.publish(hb_msg);
}

void
emergency_stop(void)
{
    static std_srvs::Empty e;
    estop.call(e);
}

void
send_topic(std::string topic)
{
    // Stop anything caused by send()
    cmd_timer.stop();

    // Only screw with the subscriber if it is different
    if (twist_in.getTopic() != topic)
    {
        boost::function<void(const geometry_msgs::Twist &)> callback = [&](const geometry_msgs::Twist &msg) {
            twist_out.publish(msg);
        };
        twist_in = nh->subscribe<geometry_msgs::Twist>(topic, 1, callback);
    }
}

void
send_twist(float linear, float angular)
{
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
}

void
loop(const ros::TimerEvent &e)
{
    twist_out.publish(cmd_msg);
}

template <typename T, typename R>
void
add_sub(std::string name, std::function<R(const T)> get_data)
{
    lua.set_function(name, [&,get_data](std::string topic) -> R {
        typedef RouteSub<T> RS;
        auto it = route_subs.find(topic);
        if (it != route_subs.end())
        {
            std::shared_ptr<RS> ptr = std::dynamic_pointer_cast<RS>(it->second);
            if (ptr.get() != NULL) return get_data(ptr->last());
        }

        route_subs.emplace(std::make_pair(topic, std::make_shared<RS>(*nh, topic)));
        return {};
    });
}

template <typename T, typename R>
void
add_pub(std::string name, std::function<T(const R)> set_data)
{
    lua.set_function(name, [&,set_data](R tuple, std::string topic) -> void {
        typedef RoutePub<T> PS;
        auto it = route_pubs.find(topic);
        if (it == route_pubs.end())
            it = route_pubs.emplace(std::make_pair(topic, std::make_shared<PS>(*nh, topic))).first;
        std::shared_ptr<PS> ptr = std::dynamic_pointer_cast<PS>(it->second);
        T msg = set_data(tuple);
        if (ptr.get() != NULL) return ptr->publish(msg);
    });
}

void
reset_state()
{
    lua = {};

    // Might as well open a decent amount of stuff since we don't create many Lua states
    lua.open_libraries(sol::lib::base, sol::lib::string, sol::lib::math, sol::lib::table, sol::lib::os);

    // Used in wrappers that will exit from Lua on return true
    lua.set_function("__spin_once", []() -> bool {
        ros::spinOnce();
        return load_next || !ros::ok();
    });
    lua.set_function("__spin_for", [](int ms) -> bool {
        auto begin = std::chrono::high_resolution_clock::now();
        ros::Rate r(50);
        while (true)
        {
            auto elapsed = std::chrono::high_resolution_clock::now() - begin;
            if (elapsed >= std::chrono::milliseconds(ms)) break;
            ros::spinOnce();
            if (load_next || !ros::ok()) return true; // Return as early as possible if needed
            r.sleep();
        }
        return load_next;
    });

    lua.set_function("heartbeat", &heartbeat);
    lua.set_function("estop", &emergency_stop);
    lua.set_function("send_topic", &send_topic);
    lua.set_function("send", &send_twist);

    // clang-format off
    add_sub<std_msgs::Bool, bool>         ("last_bool",    [](auto x) { return x.data; });
    add_sub<std_msgs::Byte, unsigned char>("last_byte",    [](auto x) { return x.data; });
    add_sub<std_msgs::Char, char>         ("last_char",    [](auto x) { return x.data; });
    add_sub<std_msgs::Float32, float>     ("last_float32", [](auto x) { return x.data; });
    add_sub<std_msgs::Float64, double>    ("last_float64", [](auto x) { return x.data; });
    add_sub<std_msgs::Int16, int16_t>     ("last_int16",   [](auto x) { return x.data; });
    add_sub<std_msgs::Int32, int32_t>     ("last_int32",   [](auto x) { return x.data; });
    add_sub<std_msgs::Int8, int8_t>       ("last_int8",    [](auto x) { return x.data; });
    add_sub<std_msgs::String, std::string>("last_string",  [](auto x) { return x.data; });
    add_sub<std_msgs::UInt16, uint16_t>   ("last_uint16",  [](auto x) { return x.data; });
    add_sub<std_msgs::UInt32, uint32_t>   ("last_uint32",  [](auto x) { return x.data; });
    add_sub<std_msgs::UInt8, uint8_t>     ("last_uint8",   [](auto x) { return x.data; });

    add_pub<std_msgs::Bool, bool>         ("pub_bool",    [](auto x) {std_msgs::Bool msg; msg.data = x; return msg;} );
    add_pub<std_msgs::Byte, unsigned char>("pub_byte",    [](auto x) {std_msgs::Byte msg; msg.data = x; return msg;} );
    add_pub<std_msgs::Char, char>         ("pub_char",    [](auto x) {std_msgs::Char msg; msg.data = x; return msg;} );
    add_pub<std_msgs::Float32, float>     ("pub_float32", [](auto x) {std_msgs::Float32 msg; msg.data = x; return msg;} );
    add_pub<std_msgs::Float64, double>    ("pub_float64", [](auto x) {std_msgs::Float64 msg; msg.data = x; return msg;} );
    add_pub<std_msgs::Int16, int16_t>     ("pub_int16",   [](auto x) {std_msgs::Int16 msg; msg.data = x; return msg;} );
    add_pub<std_msgs::Int32, int32_t>     ("pub_int32",   [](auto x) {std_msgs::Int32 msg; msg.data = x; return msg;} );
    add_pub<std_msgs::Int8, int8_t>       ("pub_int8",    [](auto x) {std_msgs::Int8 msg; msg.data = x; return msg;} );
    add_pub<std_msgs::String, std::string>("pub_string",  [](auto x) {std_msgs::String msg; msg.data = x; return msg;} );
    add_pub<std_msgs::UInt16, uint16_t>   ("pub_uint16",  [](auto x) {std_msgs::UInt16 msg; msg.data = x; return msg;} );
    add_pub<std_msgs::UInt32, uint32_t>   ("pub_uint32",  [](auto x) {std_msgs::UInt32 msg; msg.data = x; return msg;} );
    add_pub<std_msgs::UInt8, uint8_t>     ("pub_uint8",   [](auto x) {std_msgs::UInt8 msg; msg.data = x; return msg;} );

    add_sub<geometry_msgs::Twist, std::tuple<double, double, double, double, double, double>>
        ("last_twist", [](auto x) { return std::make_tuple(x.linear.x, x.linear.y, x.linear.z, x.angular.x, x.angular.y, x.angular.z); });
    add_pub<geometry_msgs::Twist, std::tuple<double, double, double, double, double, double>>
        ("pub_twist", [](auto x) { geometry_msgs::Twist msg; msg.linear.x = std::get<0>(x); msg.linear.y = std::get<1>(x); msg.linear.z = std::get<2>(x); msg.angular.x = std::get<3>(x); msg.angular.y = std::get<4>(x); msg.angular.z = std::get<5>(x); return msg;});

    add_pub<sensor_msgs::NavSatFix, std::tuple<double, double>>
        ("pub_latlong", [](auto x) {sensor_msgs::NavSatFix msg; msg.latitude = std::get<0>(x); msg.longitude = std::get<1>(x); return msg;});

    // clang-format on

    lua.safe_script(route_setup);

    lua.open_libraries(sol::lib::base, sol::lib::package);
    const std::string package_path = lua["package"]["path"];
    lua["package"]["path"] = package_path + (!package_path.empty() ? ";" : "") + script_folder + "?.lua";
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "router");
    ros::start();
    nh = new ros::NodeHandle("~");

    nh->getParam("script_folder", script_folder);
    if (!ends_with(script_folder, "/")) script_folder += "/";

    create_subs_n_pubs();
    heartbeat();

    cmd_msg.linear.x = cmd_msg.linear.y = cmd_msg.linear.z = 0;
    cmd_msg.angular.x = cmd_msg.angular.y = cmd_msg.angular.z = 0;
    cmd_timer = nh->createTimer(ros::Duration(0.01), &loop);
    cmd_timer.start();

    bool valid = false;
    ros::Rate rate(30);
    while (ros::ok())
    {
        ros::spinOnce();

        try
        {
            if (load_next)
            {
                load_next = false;
                valid = true;

                route_subs.clear();
                route_pubs.clear();

                reset_state();
                lua.script(route_contents);

                if (!load_next)
                {
                    std_srvs::Empty e;
                    estop.call(e);
                }
            }
        }
        catch (const std::exception &err)
        {
            std_srvs::Empty e;
            estop.call(e);
            ros::spinOnce();
            valid = false;
            ROS_ERROR_STREAM("SOL: " << err.what());
        }

        rate.sleep();
    }

    ros::shutdown();
}
