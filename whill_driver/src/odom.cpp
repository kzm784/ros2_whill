#include <memory>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "whill_driver/odom.h"

const float base_link_height = 0.1325;

Odometry::Odometry()
{
    pose_.x = pose_.y = pose_.theta = 0.0;
    velocity_.x = velocity_.y = velocity_.theta = 0.0;
}

long double Odometry::confineRadian(long double rad)
{
    if (rad >= M_PI)
    {
        rad -= 2.0 * M_PI;
    }
    if (rad <= -M_PI)
    {
        rad += 2.0 * M_PI;
    }
    return rad;
}

void Odometry::setParameters(double _wheel_radius, double _wheel_tread)
{
    this->wheel_radius_ = _wheel_radius;
    this->wheel_tread_ = _wheel_tread;
}

void Odometry::update(sensor_msgs::msg::JointState joint_state, double dt)
{
    if (dt <= 0.00 || std::isnan(dt) || std::isinf(dt))
    {
        return;
    }

    double angle_vel_r = -joint_state.velocity[1];
    double angle_vel_l = joint_state.velocity[0];

    long double vr = angle_vel_r * wheel_radius_;
    long double vl = angle_vel_l * wheel_radius_;

    long double delta_L  = (vr + vl) / 2.0;
    long double delta_theta = (vr - vl) / (2.0 * wheel_tread_);

    pose_.x += delta_L * dt * cosl(pose_.theta + delta_theta * dt / 2.0);
    pose_.y += delta_L * dt * sinl(pose_.theta + delta_theta * dt / 2.0);

    velocity_.x = delta_L;
    velocity_.y = 0.0;
    velocity_.theta = delta_theta;

    double theta = pose_.theta + delta_theta * dt;
    pose_.theta = confineRadian(theta);

    return;
}

void Odometry::zeroVelocity()
{
    velocity_.x = 0;
    velocity_.y = 0;
    velocity_.theta = 0;
    return;
}

void Odometry::reset()
{
    Space2D poseZero = {0, 0, 0};
    set(poseZero);
    velocity_ = poseZero;
}

void Odometry::set(Space2D pose)
{
    this->pose_ = pose;
}

Odometry::Space2D Odometry::getOdom()
{
    return pose_;
}

nav_msgs::msg::Odometry Odometry::getROSOdometry()
{
    nav_msgs::msg::Odometry odom;

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, pose_.theta);

    odom.pose.pose.position.x = pose_.x;
    odom.pose.pose.position.y = pose_.y;
    odom.pose.pose.position.z = base_link_height;
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();

    odom.twist.twist.linear.x = velocity_.x;
    odom.twist.twist.linear.y = velocity_.y;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = velocity_.theta;

    return odom;
}

geometry_msgs::msg::TransformStamped Odometry::getROSTransformStamped()
{
    geometry_msgs::msg::TransformStamped odom_trans;

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, pose_.theta);

    odom_trans.transform.translation.x = pose_.x;
    odom_trans.transform.translation.y = pose_.y;
    odom_trans.transform.translation.z = base_link_height;
    odom_trans.transform.rotation.x = odom_quat.x();
    odom_trans.transform.rotation.y = odom_quat.y();
    odom_trans.transform.rotation.z = odom_quat.z();
    odom_trans.transform.rotation.w = odom_quat.w();

    return odom_trans;
}