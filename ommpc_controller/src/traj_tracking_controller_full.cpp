/*
    自定义位置与姿态/推力混合控制器 - 动态多项式轨迹实时追踪版
    程序功能：起飞后默认原点悬停，接收到 Minimum Snap 新多项式轨迹后解析时间并实时生成 P,V,A 前馈进行追踪
*/
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

#include "geometry_utils.h"
#include "sunray_logger.h"
using namespace std;
using namespace Eigen;
using sunray_logger::Logger;
// ======================== 可调参数集中区 ========================
const double Kp_x = 8, Kd_x =5;
const double Kp_y = 8, Kd_y =5;
const double Kp_z = 5, Kd_z =6;
const double GRAVITY = 9.81;
const double tau_ude[3] = {1000,1000,1};
const double dt = 0.01;
const double mass = 1.5;
const double max_ang = 75.0;
const double MAX_CURRENT_ANGLE_DEG = 80.0;
const double MAX_ACC = 12.0;
const double MAX_UDE_INTEGRAL = 6.0;
const double MAX_UDE_ESTIMATE = 12.0;
const double MIN_THRUST_COMMAND = 0.04;
const double MAX_THRUST_COMMAND = 0.90;
const double DEFAULT_HOVER_THRUST = 0.20;
const double DEFAULT_TAKEOFF_ALTITUDE = 2.0;     // 世界系绝对目标高度/m
const double DEFAULT_TAKEOFF_SPEED = 0.33;       // 匀速起飞速度/(m/s)
const double TAKEOFF_REACHED_TOLERANCE = 0.10;   // 起飞完成高度容差/m
const double UDE_LIFTOFF_HEIGHT = 0.05;           // 离地判定高度/m
const double UDE_LIFTOFF_VZ = 0.05;               // 离地判定上升速度/(m/s)
const double UDE_RAMP_TIME = 1.0;                 // UDE补偿从0渐入到1的时间/s
const bool UDE_ENABLE = true;
const double CONTROL_RATE_HZ = 100.0;
const double CONTROL_PRINT_INTERVAL = 1.0;       // 循环控制信息打印周期/s
const int OFFBOARD_WARMUP_CYCLES = 100;
const double MAVROS_REQUEST_INTERVAL = 1.0;
const int DEFAULT_UAV_ID = 1;
const std::string DEFAULT_UAV_NAME = "uav";

int uav_id = DEFAULT_UAV_ID;
std::string node_name;
std::string uav_name = DEFAULT_UAV_NAME;
double current_hover_thrust = DEFAULT_HOVER_THRUST;
double takeoff_altitude = DEFAULT_TAKEOFF_ALTITUDE;
double takeoff_speed = DEFAULT_TAKEOFF_SPEED;
// ================================================================

static double Intg_d[3] = {0.0, 0.0, 0.0};
mavros_msgs::State mavros_state;
nav_msgs::Odometry uav_odom;
Eigen::Vector3d uav_velocity_world = Eigen::Vector3d::Zero();
Eigen::Quaterniond uav_attitude_world = Eigen::Quaterniond::Identity();
bool mavros_state_received = false;
bool odom_received = false;
ros::Subscriber odom_sub;
ros::Publisher des_acc_d_pub;
ros::Publisher ude_omega_mixed_pub;
ros::Publisher ref_pos_pub;
ros::Publisher ref_vel_pub;
ros::Publisher ref_acc_pub;
ros::Subscriber traj_sub; // 轨迹订阅者
ros::Publisher rviz_ref_pos_pub;

rosbag::Bag bag;
bool start_recording = false;

double takeoff_ground_pos[3] = {0.0, 0.0, 0.0};
double takeoff_yaw = 0.0;
double takeoff_target_z = 0.0;
ros::Time takeoff_start_time;
bool takeoff_in_progress = false;
bool takeoff_complete = false;
bool is_airborne = false;
bool ude_active = false;
bool ude_just_started = false;
ros::Time ude_start_time;

// --- 共享多项式轨迹数据结构 ---
std::mutex traj_mutex;
bool has_trajectory = false;
ros::Time traj_start_time;
MatrixXd global_poly_coeff; // 每一行代表一段，包含X,Y,Z的所有系数
VectorXd global_segment_time;
int global_poly_coeff_num = 0; // 每个轴的系数个数

// 用于在没有轨迹或轨迹结束时保持悬停的目标点
double hover_target_pos[3] = {0.0, 0.0, 0.0};
bool need_update_hover_target = true;

// 信号拦截，确保Bag正常关闭
void mySigintHandler(int sig)
{
    if (start_recording) {
        bag.close();
        start_recording = false;
        std::cout << "\n[traj_tracking_controller] ROS bag saved and closed successfully." << std::endl;
    }
    std::cout << "[traj_tracking_controller] exit..." << std::endl;
    ros::shutdown();
    exit(EXIT_SUCCESS); 
}

void mavros_state_callback(const mavros_msgs::State::ConstPtr &msg)
{
    mavros_state = *msg;
    mavros_state_received = true;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    Eigen::Quaterniond attitude(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

    if (!attitude.coeffs().allFinite() || attitude.norm() < 1.0e-6)
    {
        ROS_WARN_THROTTLE(1.0, "Invalid quaternion in MAVROS odometry.");
        return;
    }

    attitude.normalize();

    const Eigen::Vector3d velocity_body(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z);

    uav_odom = *msg;
    uav_attitude_world = attitude;
    uav_velocity_world = attitude.toRotationMatrix() * velocity_body;
    odom_received = true;
}
/* 
   接收到 Minimum Snap 发出的多项式参数回调函数
   假设发布端 (Planner) 发送的数据排列格式如下：
   data[0] = num_segments (段数)
   data[1] = poly_coeff_num (单轴多项式系数个数，如 7阶为8)
   data[2] ... data[2+num_segments-1] = 各段时间
   data[2+num_segments] ... 结尾 = 多项式系数 (按段->X->Y->Z的顺序排列)
*/
void trajectory_callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(traj_mutex);

    if (!takeoff_complete)
    {
        ROS_WARN("Reject trajectory: takeoff has not completed yet.");
        return;
    }
    
    // 1. 安全检查，防止空消息导致数组越界崩溃
    if (msg->data.size() < 2) {
        Logger::print_color(int(LogColor::red), node_name, "收到的轨迹消息为空或格式错误！");
        return;
    }

    // 2. 解析基本维度信息
    int num_segments = static_cast<int>(msg->data[0]);
    global_poly_coeff_num = static_cast<int>(msg->data[1]);

    // 检查数组长度是否合法：2个基础信息 + num_segments个时间 + (num_segments * 3轴 * 每一轴的系数个数)
    int expected_size = 2 + num_segments + (num_segments * 3 * global_poly_coeff_num);
    if (msg->data.size() != expected_size) {
        std::string err = "轨迹长度不匹配！期望:" + std::to_string(expected_size) + " 实际:" + std::to_string(msg->data.size());
        Logger::print_color(int(LogColor::red), node_name, err);
        return;
    }

    // 3. 解析每一段的时间
    global_segment_time.resize(num_segments);
    for (int i = 0; i < num_segments; ++i) {
        global_segment_time[i] = msg->data[2 + i];
    }

    // 4. 解析多项式系数矩阵
    // global_poly_coeff 是一个 (num_segments 行) x (3 * global_poly_coeff_num 列) 的矩阵
    global_poly_coeff.resize(num_segments, 3 * global_poly_coeff_num);
    int coeff_start_idx = 2 + num_segments;
    
    for (int i = 0; i < num_segments; ++i) {
        for (int j = 0; j < 3 * global_poly_coeff_num; ++j) {
            global_poly_coeff(i, j) = msg->data[coeff_start_idx + i * (3 * global_poly_coeff_num) + j];
        }
    }

    // 5. 触发追踪状态，重置时间基准
    traj_start_time = ros::Time::now();
    has_trajectory = true;
    need_update_hover_target = true; 
    
    Logger::print_color(int(LogColor::green), node_name, 
        "======> 成功解析轨迹！段数: " + std::to_string(num_segments) + 
        "，重置时间并启动追踪...");
}

// 核心 UDE 控制器闭环核心
void calculate_and_pub_acceleration_control(ros::Publisher& cmd_pub, 
                                            const double ref_pos[3], 
                                            const double ref_vel[3], 
                                            const double ref_acc[3], 
                                            double ref_yaw) 
{
    double cur_pos[3] = {
        uav_odom.pose.pose.position.x,
        uav_odom.pose.pose.position.y,
        uav_odom.pose.pose.position.z};

    double cur_vel[3] = {
        uav_velocity_world.x(),
        uav_velocity_world.y(),
        uav_velocity_world.z()};

    const Eigen::Vector3d yaw_pitch_roll =
        uav_attitude_world.toRotationMatrix().eulerAngles(2, 1, 0);

    double cur_ang[3] = {
        yaw_pitch_roll(2),
        yaw_pitch_roll(1),
        yaw_pitch_roll(0)};

    double des_acc_d[3];
    double hat_f[3];
    des_acc_d[0] = Kp_x * (ref_pos[0] - cur_pos[0]) + Kd_x * (ref_vel[0] - cur_vel[0]) + ref_acc[0];
    des_acc_d[1] = Kp_y * (ref_pos[1] - cur_pos[1]) + Kd_y * (ref_vel[1] - cur_vel[1]) + ref_acc[1];
    des_acc_d[2] = Kp_z * (ref_pos[2] - cur_pos[2]) + Kd_z * (ref_vel[2] - cur_vel[2]) + ref_acc[2];
    des_acc_d[0] = std::max(std::min(des_acc_d[0], MAX_ACC), -MAX_ACC);
    des_acc_d[1] = std::max(std::min(des_acc_d[1], MAX_ACC), -MAX_ACC);
    des_acc_d[2] = std::max(std::min(des_acc_d[2], MAX_ACC), -MAX_ACC);

    double ude_blend = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        if (!ude_active)
        {
            hat_f[i] = 0.0;
            continue;
        }

        if (ude_just_started)
        {
            // 使UDE刚启动时的估计严格从0开始，避免补偿突跳。
            Intg_d[i] = cur_vel[i];
            hat_f[i] = 0.0;
        }
        else
        {
            Intg_d[i] += des_acc_d[i] * dt;
            Intg_d[i] = std::max(
                std::min(Intg_d[i], MAX_UDE_INTEGRAL),
                -MAX_UDE_INTEGRAL);

            hat_f[i] = (cur_vel[i] - Intg_d[i]) / tau_ude[i];
            hat_f[i] = std::max(
                std::min(hat_f[i], MAX_UDE_ESTIMATE),
                -MAX_UDE_ESTIMATE);
        }
    }

    if (ude_active)
    {
        ude_blend = std::min(
            1.0,
            std::max(0.0,
                     (ros::Time::now() - ude_start_time).toSec() /
                         UDE_RAMP_TIME));
    }
    ude_just_started = false;
    
    ros::Time current_time = ros::Time::now();

    // 发布调试话题
    geometry_msgs::Vector3Stamped des_acc_d_msg;
    des_acc_d_msg.header.stamp = current_time;
    des_acc_d_msg.header.frame_id = "world";
    des_acc_d_msg.vector.x = des_acc_d[0]; des_acc_d_msg.vector.y = des_acc_d[1]; des_acc_d_msg.vector.z = des_acc_d[2];
    des_acc_d_pub.publish(des_acc_d_msg);

    // 修复了之前的 std::msgs 编译错误，已替换为 std_msgs
    std_msgs::Float64MultiArray mixed_msg;
    mixed_msg.data.resize(4);
    mixed_msg.data[0] = hat_f[0]; mixed_msg.data[1] = hat_f[1]; mixed_msg.data[2] = hat_f[2];
    mixed_msg.data[3] = has_trajectory ? 1.0 : 0.0; // 用第4位表示轨迹激活状态
    ude_omega_mixed_pub.publish(mixed_msg);

    geometry_msgs::Vector3Stamped ref_pos_msg;
    ref_pos_msg.header.stamp = current_time; ref_pos_msg.header.frame_id = "world";
    ref_pos_msg.vector.x = ref_pos[0]; ref_pos_msg.vector.y = ref_pos[1]; ref_pos_msg.vector.z = ref_pos[2];
    ref_pos_pub.publish(ref_pos_msg);

    geometry_msgs::Vector3Stamped ref_vel_msg;
    ref_vel_msg.header.stamp = current_time; ref_vel_msg.header.frame_id = "world";
    ref_vel_msg.vector.x = ref_vel[0]; ref_vel_msg.vector.y = ref_vel[1]; ref_vel_msg.vector.z = ref_vel[2];
    ref_vel_pub.publish(ref_vel_msg);

    geometry_msgs::Vector3Stamped ref_acc_msg;
    ref_acc_msg.header.stamp = current_time; ref_acc_msg.header.frame_id = "world";
    ref_acc_msg.vector.x = ref_acc[0]; ref_acc_msg.vector.y = ref_acc[1]; ref_acc_msg.vector.z = ref_acc[2];
    ref_acc_pub.publish(ref_acc_msg);

    geometry_msgs::PointStamped rviz_point_msg;
    rviz_point_msg.header.stamp = current_time;
    rviz_point_msg.header.frame_id = "world"; // 必须与 RViz 的 Fixed Frame 保持一致
    rviz_point_msg.point.x = ref_pos[0];
    rviz_point_msg.point.y = ref_pos[1];
    rviz_point_msg.point.z = ref_pos[2];
    rviz_ref_pos_pub.publish(rviz_point_msg);

    // 姿态与推力分配
    double des_acc[3];
    for (int i = 0; i < 3; ++i)
        des_acc[i] = des_acc_d[i] - ude_blend * hat_f[i];

    Eigen::Map<const Eigen::Vector3d> acc_vec(des_acc);
    const double max_current_angle = uav_utils::toRad(MAX_CURRENT_ANGLE_DEG);
    cur_ang[0] = std::max(std::min(cur_ang[0], max_current_angle), -max_current_angle);
    cur_ang[1] = std::max(std::min(cur_ang[1], max_current_angle), -max_current_angle);

    double thrust=(des_acc[2]+GRAVITY)*mass/std::cos(cur_ang[0])/std::cos(cur_ang[1]);
    Eigen::Vector3d F_des = acc_vec * mass;
    F_des(2) += GRAVITY * mass;
   // ==================== 🛠️ 增益自适应映射版 ====================
    if (F_des(2) < 0.5 * mass * GRAVITY)
	{
		F_des = F_des / F_des(2) * (0.5 * mass * GRAVITY);
	}
	else if (F_des(2) > 2 * mass * GRAVITY)
	{
		F_des = F_des / F_des(2) * (2 * mass *GRAVITY);
	}
    if (std::fabs(F_des(0)/F_des(2)) > std::tan(uav_utils::toRad(max_ang)))
	{
		ROS_WARN("pitch too tilt");
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(uav_utils::toRad(max_ang));
	}

	// 角度限制幅度
	if (std::fabs(F_des(1)/F_des(2)) > std::tan(uav_utils::toRad(max_ang)))
	{
        ROS_WARN("roll too tilt");
		F_des(1) = F_des(1)/std::fabs(F_des(1)) * F_des(2) * std::tan(uav_utils::toRad(max_ang));	
	}
    // F_des是位于ENU坐标系的,F_c是FLU
    Eigen::Matrix3d wRc = uav_utils::rotz(ref_yaw);
    Eigen::Vector3d F_c = wRc.transpose() * F_des;
    double fx = F_c(0);
    double fy = F_c(1);
    double fz = F_c(2);

    // 期望roll, pitch
    double des_roll   = std::atan2(-fy, fz);
    double des_pitch  = std::atan2( fx, fz);
    // 悬停油门与电机参数有关系,也取决于质量
    double full_thrust = mass* GRAVITY /  current_hover_thrust;

    // 油门 = 期望推力/最大推力
    // 这里相当于认为油门是线性的,满足某种比例关系,即认为某个重量 = 悬停油门
    double des_thrust= thrust / full_thrust;

    // 6. 通过 MAVROS 姿态角 + 拉力接口发送控制指令。
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)，由 MAVROS 负责 ENU/FLU 到 NED/FRD 的转换。
    const Eigen::Quaterniond desired_attitude(
        Eigen::AngleAxisd(ref_yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(des_pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(des_roll, Eigen::Vector3d::UnitX()));

    mavros_msgs::AttitudeTarget attitude_cmd;
    attitude_cmd.header.stamp = ros::Time::now();
    attitude_cmd.orientation.w = desired_attitude.w();
    attitude_cmd.orientation.x = desired_attitude.x();
    attitude_cmd.orientation.y = desired_attitude.y();
    attitude_cmd.orientation.z = desired_attitude.z();
    attitude_cmd.body_rate.x = 0.0;
    attitude_cmd.body_rate.y = 0.0;
    attitude_cmd.body_rate.z = 0.0;
    attitude_cmd.thrust = std::max(
        MIN_THRUST_COMMAND,
        std::min(MAX_THRUST_COMMAND, des_thrust));
    attitude_cmd.type_mask =
        mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    cmd_pub.publish(attitude_cmd);

    // 录制 Bag 包数据
    if (start_recording) {
        try {
            bag.write("/uav1/mavros/setpoint_raw/attitude", current_time, attitude_cmd);
            bag.write("/uav1/mavros/state", current_time, mavros_state);
            bag.write("/uav1/mavros/local_position/odom", current_time, uav_odom);
            bag.write(uav_name + "/sunray/debug/ref_pos", current_time, ref_pos_msg);
            bag.write(uav_name + "/sunray/debug/ref_vel", current_time, ref_vel_msg);
            bag.write(uav_name + "/sunray/debug/ref_acc", current_time, ref_acc_msg);
            bag.write(uav_name + "/sunray/debug/des_acc_d", current_time, des_acc_d_msg);
            bag.write(uav_name + "/sunray/debug/ude_and_omega", current_time, mixed_msg);
            bag.write(uav_name + "/sunray/debug/rviz_ref_pos", current_time, rviz_point_msg);
        } catch (std::exception& e) {}
    }

    static ros::Time last_print_time = ros::Time(0);
    ros::Time now_time = ros::Time::now();
    if ((now_time - last_print_time).toSec() >= CONTROL_PRINT_INTERVAL)
    {
        printf("\033[1;34m[Ctrl] Ref:[%.2f, %.2f, %.2f] | Pos:[%.2f, %.2f, %.2f] | "
               "Err:[%.2f, %.2f, %.2f] | Thr:%.2f | Roll:%.1f, Pitch:%.1f\n\033[0m",
                 ref_pos[0], ref_pos[1], ref_pos[2],
                 cur_pos[0], cur_pos[1], cur_pos[2],
                 ref_pos[0]-cur_pos[0], ref_pos[1]-cur_pos[1], ref_pos[2]-cur_pos[2],
                 des_thrust, des_roll*180/M_PI, des_pitch*180/M_PI);
        last_print_time = now_time; 
    }
}

void publish_hold_attitude_setpoint(ros::Publisher &cmd_pub)
{
    const Eigen::Quaterniond hold_attitude(
        Eigen::AngleAxisd(takeoff_yaw, Eigen::Vector3d::UnitZ()));
    mavros_msgs::AttitudeTarget cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.orientation.w = hold_attitude.w();
    cmd.orientation.x = hold_attitude.x();
    cmd.orientation.y = hold_attitude.y();
    cmd.orientation.z = hold_attitude.z();
    cmd.thrust = current_hover_thrust;
    cmd.type_mask =
        mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    cmd_pub.publish(cmd);
}

bool enter_offboard_and_arm(ros::NodeHandle &nh,
                            ros::Publisher &cmd_pub,
                            ros::Rate &rate)
{
    ros::ServiceClient set_mode_client =
        nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");
    ros::ServiceClient arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");

    // PX4 requires a setpoint stream before accepting OFFBOARD mode.
    for (int i = 0; ros::ok() && i < OFFBOARD_WARMUP_CYCLES; ++i)
    {
        publish_hold_attitude_setpoint(cmd_pub);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request(0);
    while (ros::ok() &&
           (mavros_state.mode != "OFFBOARD" || !mavros_state.armed))
    {
        publish_hold_attitude_setpoint(cmd_pub);

        if ((ros::Time::now() - last_request).toSec() >= MAVROS_REQUEST_INTERVAL)
        {
            if (mavros_state.mode != "OFFBOARD")
            {
                mavros_msgs::SetMode mode_cmd;
                mode_cmd.request.custom_mode = "OFFBOARD";
                if (!set_mode_client.call(mode_cmd) ||
                    !mode_cmd.response.mode_sent)
                {
                    ROS_WARN("MAVROS rejected OFFBOARD mode request.");
                }
            }
            else if (!mavros_state.armed)
            {
                mavros_msgs::CommandBool arm_cmd;
                arm_cmd.request.value = true;
                if (!arming_client.call(arm_cmd) || !arm_cmd.response.success)
                {
                    ROS_WARN("MAVROS rejected arming request.");
                }
            }
            last_request = ros::Time::now();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return ros::ok();
}

double yaw_from_quaternion(const Eigen::Quaterniond &q)
{
    return std::atan2(
        2.0 * (q.w() * q.z() + q.x() * q.y()),
        1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

void reset_ude()
{
    ude_active = false;
    ude_just_started = false;
    Intg_d[0] = 0.0;
    Intg_d[1] = 0.0;
    Intg_d[2] = 0.0;
}

void update_liftoff_and_ude_state()
{
    const bool offboard_and_armed =
        mavros_state.mode == "OFFBOARD" && mavros_state.armed;

    if (takeoff_in_progress && offboard_and_armed && !is_airborne)
    {
        const double delta_z =
            uav_odom.pose.pose.position.z - takeoff_ground_pos[2];
        const double vertical_velocity = uav_velocity_world.z();

        if (delta_z > UDE_LIFTOFF_HEIGHT &&
            vertical_velocity > UDE_LIFTOFF_VZ)
        {
            is_airborne = true;
            ROS_WARN("LIFTOFF detected: dz=%.3f m, vz=%.3f m/s.",
                     delta_z, vertical_velocity);
        }
    }

    if (UDE_ENABLE && is_airborne && offboard_and_armed && !ude_active)
    {
        ude_active = true;
        ude_just_started = true;
        ude_start_time = ros::Time::now();
        ROS_WARN("UDE activated after liftoff detection.");
    }

    if (ude_active && (!UDE_ENABLE || !offboard_and_armed))
    {
        reset_ude();
        ROS_WARN("UDE reset because OFFBOARD or armed state was lost.");
    }
}

void generate_takeoff_reference(double ref_pos[3],
                                double ref_vel[3],
                                double ref_acc[3])
{
    const double elapsed =
        std::max(0.0, (ros::Time::now() - takeoff_start_time).toSec());

    // 起飞轨迹的X/Y在每个控制周期都强制使用起飞前锁存的位置。
    // 起飞阶段不允许hover_target_pos或外部轨迹覆盖这两个值。
    ref_pos[0] = takeoff_ground_pos[0];
    ref_pos[1] = takeoff_ground_pos[1];
    ref_pos[2] = std::min(
        takeoff_ground_pos[2] + takeoff_speed * elapsed,
        takeoff_target_z);

    ref_vel[0] = 0.0;
    ref_vel[1] = 0.0;
    ref_vel[2] = ref_pos[2] < takeoff_target_z ? takeoff_speed : 0.0;
    ref_acc[0] = 0.0;
    ref_acc[1] = 0.0;
    ref_acc[2] = 0.0;
}

int main(int argc, char **argv)
{
    Logger::init_default();
    ros::init(argc, argv, "traj_tracking_controller");
    ros::NodeHandle nh("~");
    ros::Rate rate(CONTROL_RATE_HZ); 
    signal(SIGINT, mySigintHandler);
    
    node_name = "[" + ros::this_node::getName() + "]:";

    nh.param<int>("uav_id", uav_id, uav_id);
    nh.param<string>("uav_name", uav_name, uav_name);
    nh.param<double>("hover_thrust", current_hover_thrust, current_hover_thrust);
    nh.param<double>("takeoff_altitude", takeoff_altitude, takeoff_altitude);
    nh.param<double>("takeoff_speed", takeoff_speed, takeoff_speed);
    uav_name = "/" + uav_name + std::to_string(uav_id);

    // 订阅与发布器定义
    ros::Subscriber mavros_state_sub =
        nh.subscribe<mavros_msgs::State>(
            "/uav1/mavros/state", 10, mavros_state_callback);
    ros::Publisher control_cmd_pub =
        nh.advertise<mavros_msgs::AttitudeTarget>(
            "/uav1/mavros/setpoint_raw/attitude", 10);
    
    // 动态多项式参考轨迹订阅
    traj_sub = nh.subscribe<std_msgs::Float64MultiArray>("/planning/poly_trajectory", 1, trajectory_callback);
    odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "/uav1/mavros/local_position/odom", 10, odom_callback);

    des_acc_d_pub = nh.advertise<geometry_msgs::Vector3Stamped>(uav_name + "/sunray/debug/des_acc_d", 1);
    ude_omega_mixed_pub = nh.advertise<std_msgs::Float64MultiArray>(uav_name + "/sunray/debug/ude_and_omega", 1);
    ref_pos_pub = nh.advertise<geometry_msgs::Vector3Stamped>(uav_name + "/sunray/debug/ref_pos", 1);
    ref_vel_pub = nh.advertise<geometry_msgs::Vector3Stamped>(uav_name + "/sunray/debug/ref_vel", 1);
    ref_acc_pub = nh.advertise<geometry_msgs::Vector3Stamped>(uav_name + "/sunray/debug/ref_acc", 1);
    rviz_ref_pos_pub = nh.advertise<geometry_msgs::PointStamped>(uav_name + "/sunray/debug/rviz_ref_pos", 1);
    
    int times = 0;
    while (ros::ok() &&
           (!mavros_state_received || !mavros_state.connected ||
            !odom_received)) {
        ros::spinOnce();
        ros::Duration(0.50).sleep();
        if (times++ > 5) Logger::print_color(int(LogColor::red), node_name, "Wait for MAVROS and odometry...");
    }

    // 每次启动都记录本次起飞前的世界系位置和偏航角。
    takeoff_ground_pos[0] = uav_odom.pose.pose.position.x;
    takeoff_ground_pos[1] = uav_odom.pose.pose.position.y;
    takeoff_ground_pos[2] = uav_odom.pose.pose.position.z;
    takeoff_yaw = yaw_from_quaternion(uav_attitude_world);
    takeoff_target_z = takeoff_altitude;

    if (takeoff_speed <= 0.0 ||
        takeoff_target_z <= takeoff_ground_pos[2] + TAKEOFF_REACHED_TOLERANCE)
    {
        ROS_ERROR("Invalid takeoff setup: initial_z=%.3f, target_z=%.3f, speed=%.3f.",
                  takeoff_ground_pos[2], takeoff_target_z, takeoff_speed);
        return 1;
    }

    hover_target_pos[0] = takeoff_ground_pos[0];
    hover_target_pos[1] = takeoff_ground_pos[1];
    hover_target_pos[2] = takeoff_target_z;
    need_update_hover_target = false;
    reset_ude();

    // 2. 初始化 ROS Bag 自动落盘
    std::string folder_path = std::string(getenv("HOME")) + "/uav_logs";
    std::time_t now = std::time(nullptr);
    std::tm* local_time = std::localtime(&now);
    std::stringstream ss;
    ss << std::put_time(local_time, "%Y%m%d_%H%M%S");
    std::string bag_file_path = folder_path + "/uav_traj_track_" + ss.str() + ".bag";
    std::system(("mkdir -p " + folder_path).c_str());

    try {
        bag.open(bag_file_path, rosbag::bagmode::Write);
        start_recording = true;
        Logger::print_color(int(LogColor::green), node_name, "======> ROS Bag 开始录制，路径: " + bag_file_path);
    } catch (rosbag::BagException& e) {
        Logger::print_color(int(LogColor::red), node_name, "错误: 无法创建 ROS Bag 文件: " + std::string(e.what()));
    }

    if (!enter_offboard_and_arm(nh, control_cmd_pub, rate))
    {
        return 1;
    }
    Logger::print_color(int(LogColor::green), node_name,
                        "MAVROS OFFBOARD enabled and UAV armed.");
    takeoff_start_time = ros::Time::now();
    takeoff_in_progress = true;
    Logger::print_color(
        int(LogColor::green), node_name,
        "Start constant-speed takeoff: locked_xy=[" +
            std::to_string(takeoff_ground_pos[0]) + ", " +
            std::to_string(takeoff_ground_pos[1]) +
            "], initial_z=" +
            std::to_string(takeoff_ground_pos[2]) +
            ", target_z=" + std::to_string(takeoff_target_z) +
            ", locked_yaw_deg=" +
            std::to_string(takeoff_yaw * 180.0 / M_PI));
    Logger::print_color(int(LogColor::green), node_name, "======> 控制运行中：等待外部多项式参考轨迹指令...");

    // 3. 运行闭环控制主循环（无限期运行，不设降落限制）
    while (ros::ok()) {
        double ref_p[3] = {0.0, 0.0, 0.0};
        double ref_v[3] = {0.0, 0.0, 0.0};
        double ref_acc_f[3] = {0.0, 0.0, 0.0};
        const double ref_yaw = takeoff_yaw;

        update_liftoff_and_ude_state();

        if (!takeoff_complete)
        {
            generate_takeoff_reference(ref_p, ref_v, ref_acc_f);

            if (uav_odom.pose.pose.position.z >=
                takeoff_target_z - TAKEOFF_REACHED_TOLERANCE)
            {
                takeoff_complete = true;
                takeoff_in_progress = false;
                ref_p[2] = takeoff_target_z;
                ref_v[2] = 0.0;
                Logger::print_color(
                    int(LogColor::green), node_name,
                    "Takeoff complete. Trajectory receiver enabled.");
            }
        }
        else
        {

        {
            std::lock_guard<std::mutex> lock(traj_mutex);
            if (has_trajectory) {
                // 计算自收到多项式轨迹以来经过的时间
                double t_track = (ros::Time::now() - traj_start_time).toSec();
                
                // 遍历时间段寻找当前时间对应的段
                int idx = -1;
                double accumulated_time = 0.0;
                double local_t = 0.0;
                for (int i = 0; i < global_segment_time.size(); ++i) {
                    if (t_track >= accumulated_time && t_track < accumulated_time + global_segment_time[i]) {
                        idx = i;
                        local_t = t_track - accumulated_time; // 获得当前段内的相对时间
                        break;
                    }
                    accumulated_time += global_segment_time[i];
                }

                // 判断轨迹是否已经全部执行完
                if (idx != -1) {
                    // --- 核心：提取对应段的多项式系数并解算 P, V, A 前馈 ---
                    for (int dim = 0; dim < 3; ++dim) {
    // 提取系数向量的起始位置
    int start_idx = dim * global_poly_coeff_num;
    
    double p_val = 0.0;
    double v_val = 0.0;
    double a_val = 0.0;

    // 多项式展开：注意生成端是从高次项到低次项排列的 (t^(n-1) -> t^0)
    for (int j = 0; j < global_poly_coeff_num; ++j) {
        double coeff = global_poly_coeff(idx, start_idx + j);
        
        // 关键修正：当前系数对应的 t 的指数 (power)
        int power = global_poly_coeff_num - 1 - j; 

        // 位置 P
        p_val += coeff * std::pow(local_t, power);
        
        // 速度 V (对 t 求一阶导)
        if (power >= 1) {
            v_val += power * coeff * std::pow(local_t, power - 1);
        }
        
        // 加速度 A 前馈 (对 t 求二阶导)
        if (power >= 2) {
            a_val += power * (power - 1) * coeff * std::pow(local_t, power - 2);
        }
    }
    
    ref_p[dim] = p_val;
    ref_v[dim] = v_val;
    ref_acc_f[dim] = a_val;
}
                } else {
                    // 轨迹执行完后，优雅切回原处悬停模式
                    has_trajectory = false;
                    int last_idx = global_segment_time.size() - 1;
    // 2. 获取最后一段的持续时间 (即该段的终点时刻)
    double end_t = global_segment_time[last_idx];

    // 3. 计算终点位置 (直接调用多项式解算)
    for (int dim = 0; dim < 3; ++dim) {
        int start_idx = dim * global_poly_coeff_num;
        double end_p = 0.0;
        for (int j = 0; j < global_poly_coeff_num; ++j) {
            double coeff = global_poly_coeff(last_idx, start_idx + j);
            int power = global_poly_coeff_num - 1 - j;
            end_p += coeff * std::pow(end_t, power);
        }
        hover_target_pos[dim] = end_p; // 将轨迹终点赋给悬停目标
    }
    
    need_update_hover_target = false; // 标记已更新，无需再捕获当前位置
    Logger::print_color(int(LogColor::yellow), node_name, "轨迹结束，已锁定终点为悬停目标。");
                }
            }
        }

        // 如果不在轨迹追踪状态下，保持在当前(或最新轨迹终点)点悬停
        if (!has_trajectory) {
            if (need_update_hover_target) {
                hover_target_pos[0] = uav_odom.pose.pose.position.x;
                hover_target_pos[1] = uav_odom.pose.pose.position.y;
                hover_target_pos[2] = uav_odom.pose.pose.position.z;
                need_update_hover_target = false;
            }
            ref_p[0] = hover_target_pos[0];
            ref_p[1] = hover_target_pos[1];
            ref_p[2] = hover_target_pos[2];
            // 速度与加速度设定为 0
            ref_v[0] = 0.0; ref_v[1] = 0.0; ref_v[2] = 0.0;
            ref_acc_f[0] = 0.0; ref_acc_f[1] = 0.0; ref_acc_f[2] = 0.0;
        }
        }

        // 执行无人机控制闭环
        calculate_and_pub_acceleration_control(control_cmd_pub, ref_p, ref_v, ref_acc_f, ref_yaw);
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
