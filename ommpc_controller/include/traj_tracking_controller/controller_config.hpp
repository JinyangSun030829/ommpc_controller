#ifndef OMMPC_CONTROLLER_CONTROLLER_CONFIG_HPP
#define OMMPC_CONTROLLER_CONTROLLER_CONFIG_HPP

#include <ros/ros.h>
#include "stable_hover.h"

#include <string>

struct ControllerConfig
{
    std::string state_source = "odom";
    std::string mavros_namespace = "/uav1/mavros";
    std::string odom_topic = "/uav1/mavros/local_position/odom";
    std::string mocap_pose_topic = "/vrpn_client_node_1/uav1/pose";
    std::string mocap_velocity_topic =
        "/uav1/mavros/local_position/velocity_local";
    std::string vision_pose_topic = "/uav1/mavros/vision_pose/pose";
    std::string trajectory_topic = "/planning/poly_trajectory";
    std::string reference_frame = "world";
    bool odom_velocity_in_body = true;
    bool mocap_velocity_in_body = true;

    double control_rate = 100.0;
    double state_timeout = 0.10;
    double state_command_hold_time = 0.20;
    double print_interval = 1.0;
    std::string status_color = "always";
    double mass = 1.5;
    double gravity = 9.81;
    double hover_thrust = 0.20;
    double kp_x = 8.0;
    double kp_y = 8.0;
    double kp_z = 5.0;
    double kd_x = 5.0;
    double kd_y = 5.0;
    double kd_z = 6.0;
    double max_acceleration = 12.0;
    double max_tilt_deg = 65.0;
    double min_vertical_force_ratio = 0.5;
    double max_vertical_force_ratio = 2.0;
    double min_thrust_command = 0.04;
    double max_thrust_command = 0.90;

    bool ude_enabled = true;
    double ude_tau_x = 1000.0;
    double ude_tau_y = 1000.0;
    double ude_tau_z = 1.0;
    double ude_max_integral = 6.0;
    double ude_max_estimate = 12.0;
    double ude_ramp_time = 1.0;
    double ude_liftoff_height = 0.05;
    double ude_liftoff_vz = 0.05;
    double ude_landing_disable_height = 0.10;
    double ude_landing_reenable_height = 0.25;

    double takeoff_altitude = 2.0;
    double takeoff_speed = 0.33;
    double takeoff_tolerance = 0.10;
    double landing_speed = 0.25;
    bool allow_land_during_trajectory = true;
    double braking_max_acceleration = 3.0;
    double braking_max_jerk = 4.0;
    double braking_position_tolerance = 0.15;
    double landing_max_acceleration = 0.50;
    double landing_max_jerk = 1.0;
    double extended_state_timeout = 2.0;
    double landing_target_offset = -0.10;
    double touchdown_height = 0.08;
    double touchdown_max_vz = 0.10;
    double touchdown_max_vxy = 0.10;
    double touchdown_confirm_time = 0.50;
    double thrust_unload_time = 0.80;
    double touchdown_thrust = 0.05;
    double service_retry_interval = 1.0;

    bool use_trajectory_end_for_hover = true;
    minisnap::ActivationOptions activation;
    bool bag_enabled = true;
    std::string bag_directory = "/home/yundrone/uav_logs";

    void load(const ros::NodeHandle &nh)
    {
        nh.param("state_source", state_source, state_source);
        nh.param("topics/mavros_namespace", mavros_namespace, mavros_namespace);
        nh.param("topics/odom", odom_topic, odom_topic);
        nh.param("topics/mocap_pose", mocap_pose_topic, mocap_pose_topic);
        nh.param("topics/mocap_velocity", mocap_velocity_topic,
                 mocap_velocity_topic);
        nh.param("topics/vision_pose", vision_pose_topic,
                 vision_pose_topic);
        nh.param("topics/trajectory", trajectory_topic, trajectory_topic);
        nh.param("visualization/reference_frame",reference_frame,reference_frame);
        if(reference_frame.empty()) throw std::invalid_argument("empty reference point frame");
        nh.param("state/odom_velocity_in_body", odom_velocity_in_body,
                 odom_velocity_in_body);
        nh.param("state/mocap_velocity_in_body", mocap_velocity_in_body,
                 mocap_velocity_in_body);

        nh.param("control/rate", control_rate, control_rate);
        nh.param("control/state_timeout", state_timeout, state_timeout);
        nh.param("control/state_command_hold_time", state_command_hold_time, state_command_hold_time);
        nh.param("control/print_interval", print_interval, print_interval);
        nh.param("control/status_color", status_color, status_color);
        if(status_color!="auto" && status_color!="always" && status_color!="never")
            throw std::invalid_argument("control/status_color must be auto/always/never");
        nh.param("control/mass", mass, mass);
        nh.param("control/gravity", gravity, gravity);
        nh.param("control/hover_thrust", hover_thrust, hover_thrust);
        nh.param("control/kp_x", kp_x, kp_x);
        nh.param("control/kp_y", kp_y, kp_y);
        nh.param("control/kp_z", kp_z, kp_z);
        nh.param("control/kd_x", kd_x, kd_x);
        nh.param("control/kd_y", kd_y, kd_y);
        nh.param("control/kd_z", kd_z, kd_z);
        nh.param("control/max_acceleration", max_acceleration,
                 max_acceleration);
        nh.param("control/max_tilt_deg", max_tilt_deg, max_tilt_deg);
        nh.param("control/min_vertical_force_ratio",
                 min_vertical_force_ratio, min_vertical_force_ratio);
        nh.param("control/max_vertical_force_ratio",
                 max_vertical_force_ratio, max_vertical_force_ratio);
        nh.param("control/min_thrust_command", min_thrust_command,
                 min_thrust_command);
        nh.param("control/max_thrust_command", max_thrust_command,
                 max_thrust_command);

        nh.param("ude/enabled", ude_enabled, ude_enabled);
        nh.param("ude/tau_x", ude_tau_x, ude_tau_x);
        nh.param("ude/tau_y", ude_tau_y, ude_tau_y);
        nh.param("ude/tau_z", ude_tau_z, ude_tau_z);
        nh.param("ude/max_integral", ude_max_integral, ude_max_integral);
        nh.param("ude/max_estimate", ude_max_estimate, ude_max_estimate);
        nh.param("ude/ramp_time", ude_ramp_time, ude_ramp_time);
        nh.param("ude/liftoff_height", ude_liftoff_height,
                 ude_liftoff_height);
        nh.param("ude/liftoff_vz", ude_liftoff_vz, ude_liftoff_vz);
        nh.param("ude/landing_disable_height",
                 ude_landing_disable_height,
                 ude_landing_disable_height);
        nh.param("ude/landing_reenable_height", ude_landing_reenable_height, ude_landing_reenable_height);

        nh.param("takeoff/altitude", takeoff_altitude, takeoff_altitude);
        nh.param("takeoff/speed", takeoff_speed, takeoff_speed);
        nh.param("takeoff/tolerance", takeoff_tolerance,
                 takeoff_tolerance);
        nh.param("landing/speed", landing_speed, landing_speed);
        nh.param("landing/allow_during_trajectory", allow_land_during_trajectory,
                 allow_land_during_trajectory);
        nh.param("braking/max_acceleration", braking_max_acceleration, braking_max_acceleration);
        nh.param("braking/max_jerk", braking_max_jerk, braking_max_jerk);
        nh.param("braking/position_tolerance", braking_position_tolerance, braking_position_tolerance);
        nh.param("landing/max_acceleration", landing_max_acceleration, landing_max_acceleration);
        nh.param("landing/max_jerk", landing_max_jerk, landing_max_jerk);
        nh.param("landing/extended_state_timeout", extended_state_timeout, extended_state_timeout);
        nh.param("landing/target_offset", landing_target_offset,
                 landing_target_offset);
        nh.param("landing/touchdown_height", touchdown_height,
                 touchdown_height);
        nh.param("landing/touchdown_max_vz", touchdown_max_vz,
                 touchdown_max_vz);
        nh.param("landing/touchdown_max_vxy", touchdown_max_vxy,
                 touchdown_max_vxy);
        nh.param("landing/touchdown_confirm_time",
                 touchdown_confirm_time,
                 touchdown_confirm_time);
        nh.param("landing/thrust_unload_time", thrust_unload_time,
                 thrust_unload_time);
        nh.param("landing/touchdown_thrust", touchdown_thrust,
                 touchdown_thrust);
        nh.param("mavros/service_retry_interval", service_retry_interval,
                 service_retry_interval);

        nh.param("trajectory/use_end_for_hover",
                 use_trajectory_end_for_hover,
                 use_trajectory_end_for_hover);
        nh.param("trajectory/hover/duration", activation.hover.duration, activation.hover.duration);
        nh.param("trajectory/hover/max_speed", activation.hover.max_speed, activation.hover.max_speed);
        nh.param("trajectory/hover/max_position_span", activation.hover.max_position_span, activation.hover.max_position_span);
        nh.param("trajectory/hover/max_state_age", activation.hover.max_state_age, activation.hover.max_state_age);
        nh.param("trajectory/max_ready_age", activation.max_ready_age, activation.max_ready_age);
        nh.param("trajectory/max_start_position_error", activation.max_position_error, activation.max_position_error);
        nh.param("trajectory/start_boundary_tolerance", activation.boundary_tolerance, activation.boundary_tolerance);
        activation.validate();
        if (!std::isfinite(state_timeout) || state_timeout<=0 ||
            !std::isfinite(state_command_hold_time) || state_command_hold_time<0)
            throw std::invalid_argument("invalid state freshness/hold timeout");
        for (double x : {braking_max_acceleration, braking_max_jerk, braking_position_tolerance,
                         landing_speed, landing_max_acceleration, landing_max_jerk, extended_state_timeout,
                         ude_tau_x, ude_tau_y, ude_tau_z, ude_max_estimate, service_retry_interval,
                         touchdown_confirm_time, thrust_unload_time, touchdown_max_vz, touchdown_max_vxy})
            if (!std::isfinite(x) || x<=0) throw std::invalid_argument("invalid braking/landing parameter");
        if (!std::isfinite(ude_ramp_time) || ude_ramp_time<0)
            throw std::invalid_argument("invalid UDE ramp time");
        if (!std::isfinite(ude_landing_reenable_height) ||
            !std::isfinite(ude_landing_disable_height) || ude_landing_disable_height<0 ||
            ude_landing_reenable_height<=ude_landing_disable_height)
            throw std::invalid_argument("UDE landing hysteresis must have reenable > disable >= 0");
        nh.param("bag/enabled", bag_enabled, bag_enabled);
        nh.param("bag/directory", bag_directory, bag_directory);
    }
};

#endif
