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
