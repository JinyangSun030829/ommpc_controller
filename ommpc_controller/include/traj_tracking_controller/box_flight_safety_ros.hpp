#ifndef OMMPC_BOX_FLIGHT_SAFETY_ROS_HPP
#define OMMPC_BOX_FLIGHT_SAFETY_ROS_HPP
#include "box_flight_safety.hpp"
#include <ros/ros.h>

namespace flight_safety
{
inline BoxOptions loadBoxOptions(const ros::NodeHandle &nh)
{
    BoxOptions box;
    XmlRpc::XmlRpcValue enabled;
    if (nh.getParam("flight_box/enabled", enabled))
    {
        if (enabled.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
            box.enabled = static_cast<bool>(enabled);
        else if (enabled.getType() == XmlRpc::XmlRpcValue::TypeInt &&
                 (static_cast<int>(enabled) == 0 || static_cast<int>(enabled) == 1))
            box.enabled = static_cast<int>(enabled) == 1;
        else
            throw std::invalid_argument("flight_box/enabled must be bool or 0/1");
    }
    auto vectorParam = [&](const std::string &name, Eigen::Vector3d &value) {
        std::vector<double> data;
        if (nh.getParam("flight_box/" + name, data))
        {
            if (data.size() != 3)
                throw std::invalid_argument("flight_box/" + name + " needs xyz");
            value = Eigen::Vector3d(data[0], data[1], data[2]);
        }
        else if (box.enabled)
            throw std::invalid_argument("missing flight_box/" + name + "; load flight_box.yaml");
    };
    vectorParam("minimum", box.minimum);
    vectorParam("maximum", box.maximum);
    XmlRpc::XmlRpcValue margin;
    if (nh.getParam("flight_box/margin", margin))
    {
        if (margin.getType() == XmlRpc::XmlRpcValue::TypeDouble)
            box.margin.setConstant(static_cast<double>(margin));
        else if (margin.getType() == XmlRpc::XmlRpcValue::TypeInt)
            box.margin.setConstant(static_cast<int>(margin));
        else if (margin.getType() == XmlRpc::XmlRpcValue::TypeArray &&
                 (margin.size() == 2 || margin.size() == 3))
        {
            for (int i = 0; i < margin.size(); ++i)
            {
                if (margin[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    box.margin(i) = static_cast<double>(margin[i]);
                else if (margin[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
                    box.margin(i) = static_cast<int>(margin[i]);
                else
                    throw std::invalid_argument("flight_box/margin must be numeric");
            }
        }
        else
            throw std::invalid_argument("flight_box/margin must be a number or xy/xyz array");
    }
    if (!box.margin.allFinite() || (box.margin.array() < 0).any())
        throw std::invalid_argument("flight_box/margin must be finite and nonnegative");
    if (margin.getType() == XmlRpc::XmlRpcValue::TypeArray && box.margin.z() != 0)
        ROS_WARN("[BOX] margin_z is ignored: only X/Y faces are contracted; Z bounds unchanged.");
    box.margin.z() = 0.0;
    nh.param("flight_box/check_interval", box.check_interval, box.check_interval);
    nh.param("flight_box/lookahead", box.lookahead, box.lookahead);
    nh.param("flight_box/reaction_time", box.reaction_time, box.reaction_time);
    nh.param("flight_box/max_stop_duration", box.max_stop_duration, box.max_stop_duration);
    nh.param("flight_box/lookahead_samples", box.lookahead_samples, box.lookahead_samples);
    nh.param("flight_box/duration_trials", box.duration_trials, box.duration_trials);
    nh.param("flight_box/subdivision_depth", box.subdivision_depth, box.subdivision_depth);
    nh.param("flight_box/path_stop_segments", box.path_stop_segments, box.path_stop_segments);
    box.validate();
    if (box.enabled)
        ROS_WARN("[BOX] Enabled WORLD cuboid: min=[%.2f %.2f %.2f], max=[%.2f %.2f %.2f], "
                 "margin=[%.2f %.2f %.2f]. Verify hardware limits and PX4 OFFBOARD-loss action.",
                 box.minimum.x(), box.minimum.y(), box.minimum.z(), box.maximum.x(),
                 box.maximum.y(), box.maximum.z(), box.margin.x(), box.margin.y(), box.margin.z());
    else
        ROS_WARN("[BOX] DISABLED: set measured experiment bounds and enable flight_box.yaml.");
    return box;
}
} // namespace flight_safety
#endif
