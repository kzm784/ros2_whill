#pragma once

#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class Odometry
{
private:
    long double confineRadian(long double rad);

    typedef struct
    {
        double x;
        double y;
        double theta;
    } Space2D;

    double wheel_radius;
    double wheel_tread;

    Space2D pose;
    Space2D velocity;

public:
    Odometry();
    void setParameters(double _wheel_radius, double _wheel_tread);
    void update(sensor_msgs::msg::JointState joint, double dt);
    void zeroVelocity(void);
    void set(Space2D pose);
    void reset();

    nav_msgs::msg::Odometry getROSOdometry();
    geometry_msgs::msg::TransformStamped getROSTransformStamped();
    Space2D getOdom();
};