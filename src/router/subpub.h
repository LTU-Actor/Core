#pragma once

#include <ros/ros.h>

class RouteSub_ {
public:
    virtual ~RouteSub_() = default; // to make "source type is not polymorphic" not an issue

protected:
    RouteSub_(){};
    ros::Subscriber sub;
    ros::Time last_time;
};

template <typename T> class RouteSub : public RouteSub_ {
public:
    RouteSub(ros::NodeHandle nh, const std::string topic)
    {
        boost::function<void(const T &)> callback = [&](const T &msg) {
            last_ = msg;
            last_time = ros::Time::now();
        };
        sub = nh.subscribe<T>(topic, 1, callback);
    }

    const T &last() { return last_; }

private:
    T last_;
};



#define lua_add_sub(lua, nh, sub_map, t, ret, field)                                                                   \
    lua.set_function("last_" #t, [&](std::string topic) -> ret {                                                       \
        typedef RouteSub<std_msgs::t> RS;                                                                              \
        auto it = sub_map.find(topic);                                                                                 \
        if (it != sub_map.end())                                                                                       \
        {                                                                                                              \
            std::shared_ptr<RS> ptr = std::dynamic_pointer_cast<RS>(it->second);                                       \
            if (ptr != NULL) return ptr->last().field;                                                                 \
        }                                                                                                              \
                                                                                                                       \
        sub_map.emplace(std::make_pair(topic, std::make_shared<RS>(nh, topic)));                                       \
        return {};                                                                                                     \
    })



class RoutePub_ {
public:
    virtual ~RoutePub_() = default; // to make "source type is not polymorphic" not an issue

protected:
    RoutePub_(){};
    ros::Publisher pub;
    ros::Time last_time;
};

template <typename T> class RoutePub : public RoutePub_ {
public:
    RoutePub(ros::NodeHandle nh, const std::string topic) { pub = nh.advertise<T>(topic, 1); }
    void publish(const T &msg) { pub.publish(msg); }
};



#define lua_add_pub(lua, nh, pub_map, t, ret, field)                                                                   \
    lua.set_function("pub_" #t, [&](std::string topic, ret r) -> void {                                                \
        typedef RoutePub<std_msgs::t> PS;                                                                              \
        auto it = pub_map.find(topic);                                                                                 \
        if (it == pub_map.end()) it = pub_map.emplace(std::make_pair(topic, std::make_shared<PS>(nh, topic))).first;   \
        std::shared_ptr<PS> ptr = std::dynamic_pointer_cast<PS>(it->second);                                           \
        std_msgs::t msg;                                                                                               \
        msg.field = r;                                                                                                 \
        if (ptr != NULL) return ptr->publish(msg);                                                                     \
    })
