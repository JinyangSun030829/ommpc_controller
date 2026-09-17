#ifndef OMMPC_CONTROLLER_STATE_PROVIDER_HPP
#define OMMPC_CONTROLLER_STATE_PROVIDER_HPP

#include "controller_config.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

#include <string>

struct VehicleState
{
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
    ros::Time stamp;
    bool valid = false;
};

class StateProvider
{
public:
    StateProvider(ros::NodeHandle &nh, const ControllerConfig &config)
        : config_(config)
    {
        if (config_.state_source == "odom")
        {
            odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
                config_.odom_topic, 10, &StateProvider::odomCallback, this);
        }
        else if (config_.state_source == "mocap")
        {
            mocap_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
                config_.mocap_pose_topic, 10,
                &StateProvider::mocapPoseCallback, this);
            mocap_velocity_sub_ = nh.subscribe<geometry_msgs::TwistStamped>(
                config_.mocap_velocity_topic, 10,
                &StateProvider::mocapVelocityCallback, this);
        }
        else
        {
            ROS_FATAL("state_source must be 'odom' or 'mocap', got '%s'.",
                      config_.state_source.c_str());
        }
    }

    bool configured() const
    {
        return config_.state_source == "odom" ||
               config_.state_source == "mocap";
    }

    bool ready() const
    {
        if (config_.state_source == "odom")
            return odom_received_;
        return mocap_pose_received_ && mocap_velocity_received_;
    }

    VehicleState state() const
    {
        VehicleState result;
        if (!ready())
            return result;

        result.position = position_;
        result.attitude = attitude_;
        result.velocity = raw_velocity_;

        const bool velocity_in_body =
            config_.state_source == "odom"
                ? config_.odom_velocity_in_body
                : config_.mocap_velocity_in_body;
        if (velocity_in_body)
            result.velocity = attitude_.toRotationMatrix() * raw_velocity_;

        result.stamp = state_stamp_;
        result.valid = !state_stamp_.isZero() && result.position.allFinite() &&
                       result.velocity.allFinite() && result.attitude.coeffs().allFinite();
        return result;
    }

    const std::string &sourceName() const { return config_.state_source; }

private:
    static bool normalizedQuaternion(double w, double x, double y, double z,
                                     Eigen::Quaterniond &q)
    {
        q = Eigen::Quaterniond(w, x, y, z);
        if (!q.coeffs().allFinite() || q.norm() < 1.0e-6)
            return false;
        q.normalize();
        return true;
    }

    static ros::Time messageStampOrNow(const ros::Time &stamp)
    {
        // Do not turn an unstamped/frozen measurement into fresh flight state.
        return stamp;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        Eigen::Quaterniond q;
        if (!normalizedQuaternion(
                msg->pose.pose.orientation.w,
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z, q))
        {
            ROS_WARN_THROTTLE(1.0, "Invalid quaternion in MAVROS odometry.");
            return;
        }

        position_ << msg->pose.pose.position.x,
                     msg->pose.pose.position.y,
                     msg->pose.pose.position.z;
        raw_velocity_ << msg->twist.twist.linear.x,
                         msg->twist.twist.linear.y,
                         msg->twist.twist.linear.z;
        attitude_ = q;
        state_stamp_ = messageStampOrNow(msg->header.stamp);
        odom_received_ = true;
    }

    void mocapPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        Eigen::Quaterniond q;
        if (!normalizedQuaternion(
                msg->pose.orientation.w,
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z, q))
        {
            ROS_WARN_THROTTLE(1.0,
                              "Invalid quaternion in motion-capture pose.");
            return;
        }

        position_ << msg->pose.position.x,
                     msg->pose.position.y,
                     msg->pose.position.z;
        attitude_ = q;
        pose_stamp_ = messageStampOrNow(msg->header.stamp);
        mocap_pose_received_ = true;
        updateMocapStamp();
    }

    void mocapVelocityCallback(
        const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        raw_velocity_ << msg->twist.linear.x,
                         msg->twist.linear.y,
                         msg->twist.linear.z;
        velocity_stamp_ = messageStampOrNow(msg->header.stamp);
        mocap_velocity_received_ = true;
        updateMocapStamp();
    }

    void updateMocapStamp()
    {
        if (!mocap_pose_received_ || !mocap_velocity_received_)
            return;
        state_stamp_ = pose_stamp_ < velocity_stamp_
                           ? pose_stamp_
                           : velocity_stamp_;
    }

    ControllerConfig config_;
    ros::Subscriber odom_sub_;
    ros::Subscriber mocap_pose_sub_;
    ros::Subscriber mocap_velocity_sub_;
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d raw_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond attitude_ = Eigen::Quaterniond::Identity();
    ros::Time state_stamp_;
    ros::Time pose_stamp_;
    ros::Time velocity_stamp_;
    bool odom_received_ = false;
    bool mocap_pose_received_ = false;
    bool mocap_velocity_received_ = false;
};

#endif
