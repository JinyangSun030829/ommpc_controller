#include "trajectory_planner.h"
#include "stable_hover.h"
#include "console_colors.h"

#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <unistd.h>

namespace {
Eigen::Vector3d vector(const geometry_msgs::Point& p) { return Eigen::Vector3d(p.x, p.y, p.z); }
geometry_msgs::Point point(const Eigen::Vector3d& p) {
    geometry_msgs::Point value;
    value.x = p.x(); value.y = p.y(); value.z = p.z();
    return value;
}

// Single ROS callback thread owns incoming state and publication. One worker
// owns optimization. Only immutable request/result snapshots cross the mutex.
class TrajectoryGeneratorNode {
public:
    TrajectoryGeneratorNode() : nh_("~") {
        nh_.param("planning/dev_order", options_.order, 4);
        nh_.param("planning/vel", options_.max_velocity, 3.0);
        nh_.param("planning/acc", options_.max_acceleration, 2.0);
        nh_.param("planning/max_jerk", options_.max_jerk, 4.0);
        nh_.param("planning/seed_safety_ratio", options_.seed_safety_ratio, 0.9);
        nh_.param("planning/limit_margin", options_.limit_margin, 0.98);
        nh_.param("planning/min_segment_time", options_.min_segment_time, 0.001);
        nh_.param("planning/duplicate_distance", options_.duplicate_distance, 1e-4);
        nh_.param("planning/max_total_time", options_.max_total_time, 3600.0);
        nh_.param("planning/max_segments", options_.max_segments, 2000);
        nh_.param("planning/max_iterations", options_.max_iterations, 30);
        nh_.param("planning/bound_tolerance", options_.bound_tolerance, 0.002);
        nh_.param("planning/bound_max_depth", options_.bound_max_depth, 12);
        nh_.param("planning/compress_time", options_.compress_time, true);
        nh_.param("planning/compression_max_trials", options_.compression_max_trials, 32);
        nh_.param("planning/compression_budget_ms", options_.compression_budget_ms, 50.0);
        nh_.param("planning/compression_step", options_.compression_step, 0.05);
        nh_.param("planning/compare_legacy_seed", options_.compare_legacy_seed, true);
        nh_.param("planning/hover/duration", hover_options_.duration, 1.0);
        nh_.param("planning/hover/max_speed", hover_options_.max_speed, 0.10);
        nh_.param("planning/hover/max_position_span", hover_options_.max_position_span, 0.05);
        nh_.param("planning/hover/max_state_age", hover_options_.max_state_age, 0.20);
        nh_.param("planning/status_interval",status_interval_,1.0);
        nh_.param("planning/require_controller_ready",require_controller_ready_,false);
        nh_.param("planning/controller_status_max_age",controller_status_max_age_,1.0);
        nh_.param<std::string>("planning/controller_status_topic",controller_status_topic_,
            "/traj_tracking_controller/trajectory_admission_status");
        hover_gate_.reset(new minisnap::StableHover(hover_options_));
        nh_.param<std::string>("planning/result_color", result_color_, "auto");
        if (options_.order != 4 || (result_color_ != "auto" && result_color_ != "always" && result_color_ != "never"))
            throw std::invalid_argument("hover-start ROS planner requires dev_order=4 and result_color=auto/always/never");
        options_.validate();
        nh_.param<std::string>("planning/frame_id", frame_, "world");
        nh_.param<std::string>("planning/odom_frame_alias", odom_alias_, "");
        nh_.param<std::string>("odom/topic", odom_topic_, "/uav1/mavros/local_position/odom");
        nh_.param<std::string>("odom/twist_frame", twist_frame_, "child");
        nh_.param("odom/max_age", max_odom_age_, 0.5);
        nh_.param("planning/max_result_age", max_result_age_, 0.5);
        nh_.param("planning/max_start_position_error", max_position_error_, 0.15);
        nh_.param("planning/max_start_velocity_error", max_velocity_error_, 0.3);
        nh_.param("vis/enabled", visualization_, true);
        nh_.param("vis/vis_traj_width", width_, 0.05);
        nh_.param("vis/sample_dt", sample_dt_, 0.05);
        nh_.param("vis/max_points", max_vis_points_, 2000);
        if (frame_.empty() || (twist_frame_ != "child" && twist_frame_ != "pose") ||
            !positive(max_odom_age_) || !positive(max_result_age_) ||
            !positive(max_position_error_) || !positive(max_velocity_error_) ||
            !positive(width_) || !positive(sample_dt_) || !positive(status_interval_) ||
            !positive(controller_status_max_age_) || controller_status_topic_.empty() ||
            max_vis_points_ < 2 || max_vis_points_ > 100000)
            throw std::invalid_argument("invalid ROS node frame, freshness, or visualization configuration");
        if (nh_.hasParam("planning/min_order"))
            ROS_WARN("planning/min_order is obsolete and ignored; use integer planning/dev_order=3 or 4.");
        if (!odom_alias_.empty() && odom_alias_ != frame_)
            ROS_WARN("Explicit identity frame alias: odometry '%s' == planning '%s'. "
                     "This preserves the legacy RViz convention; disable it if these frames are not aligned.",
                     odom_alias_.c_str(), frame_.c_str());
        ROS_INFO("Planner: minimum-%s, degree %d, limits V/A/J=%.2f/%.2f/%.2f. "
                 "Stable-hover start only (%.2f s, speed<=%.3f m/s); v/a/j boundaries are zero; execution is managed by FSM.",
                 options_.order == 4 ? "snap" : "jerk", 2 * options_.order - 1,
                 options_.max_velocity, options_.max_acceleration, options_.max_jerk,
                 hover_options_.duration, hover_options_.max_speed);
        odom_sub_ = nh_.subscribe(odom_topic_, 10, &TrajectoryGeneratorNode::odometry, this);
        waypoints_sub_ = nh_.subscribe("waypoints", 1, &TrajectoryGeneratorNode::waypoints, this);
        trajectory_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/planning/poly_trajectory", 1);
        trajectory_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
        path_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);
        admission_status_pub_=nh_.advertise<std_msgs::String>("waypoint_admission_status",1,true);
        controller_status_sub_=nh_.subscribe(controller_status_topic_,1,&TrajectoryGeneratorNode::controllerStatus,this);
        status_timer_=nh_.createWallTimer(ros::WallDuration(.1),&TrajectoryGeneratorNode::displayAdmission,this);
        result_timer_ = nh_.createWallTimer(ros::WallDuration(0.01),
                                           &TrajectoryGeneratorNode::publishResult, this);
        worker_ = std::thread(&TrajectoryGeneratorNode::work, this);
    }
    ~TrajectoryGeneratorNode() {
        stop_.store(true);
        cv_.notify_all();
        if (worker_.joinable()) worker_.join();
    }
private:
    struct State {
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        ros::Time stamp;
        ros::WallTime received;
        bool valid = false;
    };
    struct Request {
        std::vector<Eigen::Vector3d> targets;
        unsigned long long id = 0;
    };
    struct Completed {
        minisnap::PlanResult plan;
        State start;
        ros::WallTime started;
        unsigned long long id = 0;
    };
    static bool positive(double x) { return std::isfinite(x) && x > 0.0; }
    void controllerStatus(const std_msgs::String::ConstPtr& msg) {
        const size_t first=msg->data.find('|');
        const size_t split=first==std::string::npos?std::string::npos:msg->data.find('|',first+1);
        bool valid=false,allowed=false; double stamp=0; std::string reason="invalid controller status";
        try {
            const std::string tag=msg->data.substr(0,first);
            if(first!=std::string::npos && split!=std::string::npos && (tag=="ALLOW" || tag=="BLOCKED")) {
                const std::string number=msg->data.substr(first+1,split-first-1); size_t used=0;
                stamp=std::stod(number,&used); valid=used==number.size() && positive(stamp);
                allowed=tag=="ALLOW"; reason=msg->data.substr(split+1);
            }
        } catch(const std::exception&) {}
        std::lock_guard<std::mutex> lock(mutex_);
        controller_status_valid_=valid; controller_allowed_=allowed; controller_status_stamp_=stamp; controller_reason_=reason;
    }
    // Caller holds mutex_. This is the same gate used by admission and display.
    bool canAccept(std::string& reason) const {
        if(!state_.valid) { reason="等待有效里程计（检查话题、坐标系、时间戳及姿态）"; return false; }
        if(!fresh(state_) || (ros::Time::now()-state_.stamp).toSec()>hover_options_.max_state_age ||
            (ros::WallTime::now()-state_.received).toSec()>hover_options_.max_state_age) {
            reason="里程计已过期，暂不接收"; return false;
        }
        const double wall=ros::WallTime::now().toSec();
        if(require_controller_ready_) {
            const double age=wall-controller_status_stamp_;
            if(!controller_status_valid_ || age<-.1 || age>controller_status_max_age_ || controller_status_sub_.getNumPublishers()==0) {
                reason="等待控制器新鲜接收权限（确认控制器已启动并使用新版）"; return false;
            }
            if(!controller_allowed_) { reason="控制器暂不接收: "+controller_reason_; return false; }
        }
        if(!hover_gate_->stable(wall)) { reason="等待持续稳定悬停"; return false; }
        if(!require_controller_ready_) reason="规划器允许接收（独立模式，未保证控制器可执行）";
        else if(controller_reason_.find(", COMMAND=1")!=std::string::npos)
            reason="规划器与控制器均允许接收；COMMAND已启用，轨迹发送后由FSM验证并自动激活";
        else if(controller_reason_.find(", COMMAND=0")!=std::string::npos)
            reason="允许接收，但COMMAND未启用；建议先启用COMMAND，避免求解后READY等待超时";
        else reason="规划器与控制器均允许接收；执行仍需 COMMAND 已启用";
        return true;
    }
    std::string admissionLine(bool& allowed) const {
        std::string reason; allowed=canAccept(reason);
        const double age=state_.valid?(ros::Time::now()-state_.stamp).toSec():-1;
        std::ostringstream text; text<<std::fixed<<std::setprecision(3)
            <<"[航点接收: "<<(allowed?"允许":"禁止")<<"] "<<reason
            <<" | 速度="<<state_.velocity.norm()<<"/"<<hover_options_.max_speed<<" m/s"
            <<" | 稳定累计="<<std::min(hover_gate_->wallCoverage(),hover_gate_->stampCoverage())
            <<"/"<<hover_options_.duration<<" s | 位置跨度="<<hover_gate_->positionSpan()
            <<"/"<<hover_options_.max_position_span<<" m | 状态延迟="<<age<<" s";
        if(allowed) text<<" | 保持满足条件即可持续发送，无固定接收截止时间";
        return text.str();
    }
    void displayAdmission(const ros::WallTimerEvent&) {
        bool allowed; std::string line;
        { std::lock_guard<std::mutex> lock(mutex_); line=admissionLine(allowed); }
        const double wall=ros::WallTime::now().toSec();
        if(!status_displayed_ || allowed!=last_allowed_ || wall-last_status_wall_>=status_interval_) {
            // ROS/log4cxx under the C locale may replace UTF-8 with '?'.
            // Preserve readable Chinese on stdout, as with the solve summary.
            ROS_DEBUG_STREAM(line);
            printRoutineLine(line);
            std_msgs::String msg; msg.data=line; admission_status_pub_.publish(msg);
            last_status_wall_=wall; last_allowed_=allowed; status_displayed_=true;
        }
    }
    bool fresh(const State& s) const {
        if (!s.valid || (ros::WallTime::now() - s.received).toSec() > max_odom_age_) return false;
        const double age = (ros::Time::now() - s.stamp).toSec();
        return age >= -0.1 && age <= max_odom_age_;
    }
    void odometry(const nav_msgs::Odometry::ConstPtr& msg) {
        State value;
        const auto& orientation = msg->pose.pose.orientation;
        Eigen::Quaterniond rotation(orientation.w, orientation.x, orientation.y, orientation.z);
        const bool frame_ok = msg->header.frame_id == frame_ ||
            (!odom_alias_.empty() && msg->header.frame_id == odom_alias_);
        const auto& v = msg->twist.twist.linear;
        value.position = vector(msg->pose.pose.position);
        value.velocity = Eigen::Vector3d(v.x, v.y, v.z);
        if (!frame_ok || msg->header.stamp.isZero() || !value.position.allFinite() ||
            !value.velocity.allFinite() || !rotation.coeffs().allFinite() ||
            rotation.norm() < 1e-6 || std::fabs(rotation.norm() - 1.0) > 0.05 ||
            (twist_frame_ == "child" && msg->child_frame_id.empty())) {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.valid = false;
            hover_gate_->reset();
            ROS_WARN_THROTTLE(1.0, "Rejected odometry: invalid frame, stamp, quaternion, or state.");
            return;
        }
        // nav_msgs/Odometry twist is expressed in child_frame_id. The pose
        // quaternion rotates child-frame vectors into the pose/header frame.
        if (twist_frame_ == "child") value.velocity = rotation.normalized() * value.velocity;
        value.stamp = msg->header.stamp;
        value.received = ros::WallTime::now();
        value.valid = true;
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = value;
        const double age = (ros::Time::now() - value.stamp).toSec();
        hover_gate_->update(value.position, value.velocity, value.stamp.toSec(), value.received.toSec(),
            age >= -0.1 && age <= hover_options_.max_state_age);
    }
    void waypoints(const nav_msgs::Path::ConstPtr& msg) {
        // Supersede previous work even if the replacement request is invalid;
        // publishing an older request after a rejected new one is surprising.
        const auto id = latest_.fetch_add(1) + 1;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pending_ = false;
            completed_.reset();
        }
        if (msg->header.frame_id != frame_ || msg->poses.size() > static_cast<size_t>(options_.max_segments)) {
            ROS_ERROR("Rejected waypoints: expected frame '%s' and at most %d targets.",
                      frame_.c_str(), options_.max_segments);
            return;
        }
        Request request;
        request.id = id;
        for (const auto& pose : msg->poses) {
            if (!pose.header.frame_id.empty() && pose.header.frame_id != frame_) {
                ROS_ERROR("Rejected waypoint: inconsistent pose frame."); return;
            }
            const Eigen::Vector3d p = vector(pose.pose.position);
            if (!p.allFinite()) { ROS_ERROR("Rejected non-finite waypoint."); return; }
            if (p.z() < 0.0) break; // retain legacy end sentinel, do not append it
            request.targets.push_back(p);
        }
        if (request.targets.empty()) { ROS_ERROR("Rejected empty waypoint request."); return; }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            bool allowed; const std::string status=admissionLine(allowed);
            if (!allowed) {
                ROS_WARN("[WAYPOINT REJECTED] Admission gate closed; request not cached. See live admission status, then resend.");
                std::cout<<"[航点未接收] "<<status<<"；本次未缓存，请就绪后重发。"<<std::endl; return;
            }
            request_ = std::move(request);
            pending_ = true;
            std::ostringstream accepted;
            accepted<<"[航点已接收] request="<<id<<"，目标点="<<request_.targets.size()<<"；开始规划。";
            printRoutineLine(accepted.str());
        }
        cv_.notify_one();
    }
    void work() {
        const minisnap::TrajectoryPlanner planner(options_);
        while (!stop_.load()) {
            Request request;
            State start;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [&] { return stop_.load() || pending_; });
                if (stop_.load()) return;
                request = std::move(request_);
                pending_ = false;
                start = state_; // freshest complete state at actual planning start
                std::string reason;
                if (!canAccept(reason)) {
                    ROS_WARN_STREAM("Planning request blocked before worker start: "<<reason); continue;
                }
            }
            if (!fresh(start)) { ROS_WARN("Planning request expired before worker start."); continue; }
            const auto cancelled = [&] { return stop_.load() || latest_.load() != request.id; };
            try {
                std::unique_ptr<Completed> result(new Completed);
                result->id = request.id;
                result->start = start;
                result->started = ros::WallTime::now();
                Eigen::MatrixXd path(request.targets.size() + 1, 3);
                path.row(0) = start.position.transpose();
                for (int i = 0; i < static_cast<int>(request.targets.size()); ++i)
                    path.row(i + 1) = request.targets[i].transpose();
                result->plan = planner.plan(path, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), cancelled);
                if (!cancelled()) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!cancelled()) completed_ = std::move(result);
                }
            } catch (const std::exception& error) {
                if (!cancelled()) ROS_ERROR("Trajectory planning failed: %s", error.what());
            }
        }
    }
    void publishResult(const ros::WallTimerEvent&) {
        std::unique_ptr<Completed> result;
        State current;
        bool allowed = false; std::string admission_reason;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            result = std::move(completed_);
            current = state_;
            allowed = canAccept(admission_reason);
        }
        if (!result || result->id != latest_.load()) return;
        if (!fresh(current) || !allowed || (ros::WallTime::now() - result->started).toSec() > max_result_age_ ||
            (current.position - result->start.position).norm() > max_position_error_ ||
            (current.velocity - result->start.velocity).norm() > max_velocity_error_) {
            ROS_WARN("Discarded stale trajectory: odometry/latency/start-state mismatch. Resend waypoints from a stable state.");
            return;
        }
        try {
            std_msgs::Float64MultiArray msg;
            msg.data = result->plan.trajectory.serialize();
            trajectory_pub_.publish(msg); // publish BEFORE optional visualization
            printRoutineLine("[轨迹已发送] 已发给控制器；是否进入跟踪由 FSM 确认，求解成功不等于已开始执行。");
            printResult(result->plan);
            if (visualization_) visualize(result->plan);
        } catch (const std::exception& error) { ROS_ERROR("Trajectory publication failed: %s", error.what()); }
    }
    void printRoutineLine(const std::string& line) const {
        const bool color = result_color_ == "always" || (result_color_ == "auto" &&
            ::isatty(STDOUT_FILENO) && std::getenv("NO_COLOR") == nullptr);
        std::cout << (color ? minisnap_console::reset : "") << (color ? minisnap_console::cyan : "")
                  << line << (color ? minisnap_console::reset : "") << std::endl;
    }
    void printResult(const minisnap::PlanResult& plan) const {
        std::ostringstream text;
        text << std::fixed << std::setprecision(3)
             << "\n========== MINIMUM-SNAP 求解成功 ==========\n"
             << "轨迹段数: " << plan.trajectory.times.size() << "  总时长: " << plan.trajectory.times.sum() << " s\n"
             << "计算耗时: " << plan.planning_ms << " ms  可行性求解: " << plan.iterations << " 次\n"
             << "备选种子求解: " << plan.seed_comparison_solves << " 次  选中: " << (plan.used_legacy_seed ? "兼容种子" : "连续S形种子") << "\n"
             << "可行初解时长: " << plan.feasible_duration << " s  整体时间倍率: " << plan.uniform_time_factor << "\n"
             << "压缩尝试: " << plan.compression_trials << "  局部压缩接受: " << plan.local_compressions << "\n"
             << "V/A/J 保守上界: " << plan.peak_bounds.transpose() << " (m/s, m/s², m/s³)\n"
             << "起始条件: 稳定悬停，v/a/j = 0  重复点删除: " << plan.removed_duplicates << "\n"
             << "==========================================\n";
        // Terminal decoration never enters the ROS message or metrics files.
        ROS_DEBUG_STREAM(text.str());
        const bool color = result_color_ == "always" || (result_color_ == "auto" &&
            ::isatty(STDOUT_FILENO) && std::getenv("NO_COLOR") == nullptr);
        std::cout << (color ? minisnap_console::reset : "") << (color ? minisnap_console::green : "")
                  << text.str() << (color ? minisnap_console::reset : "") << std::flush;
    }
    visualization_msgs::Marker marker(const std::string& ns, int id, int type) const {
        visualization_msgs::Marker m;
        m.header.frame_id = frame_; m.header.stamp = ros::Time::now();
        m.ns = ns; m.id = id; m.type = type; m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = width_; m.scale.y = width_; m.scale.z = width_;
        m.color.a = 1.0; m.color.r = 1.0; m.color.g = 1.0;
        return m;
    }
    void visualize(const minisnap::PlanResult& plan) {
        auto line = marker("traj_node/trajectory_waypoints", 0, visualization_msgs::Marker::LINE_STRIP);
        // Fixed dark purple on white RViz backgrounds. Actual UAV traces use
        // speed-coloured blue/green/yellow/orange/red, not this purple hue.
        line.color.r = 0.45;
        line.color.g = 0.15;
        line.color.b = 0.65;
        line.color.a = 1.0;
        const auto& trajectory = plan.trajectory;
        const double total = trajectory.times.sum();
        const double step = std::max(sample_dt_, total / (max_vis_points_ - 1));
        int segment = 0;
        double segment_start = 0.0;
        for (int i = 0; i < max_vis_points_ - 1; ++i) {
            const double t = i * step;
            if (t >= total) break;
            while (segment + 1 < trajectory.times.size() &&
                   t >= segment_start + trajectory.times(segment)) {
                segment_start += trajectory.times(segment); ++segment;
            }
            line.points.push_back(point(trajectory.evaluate(segment, t - segment_start)));
        }
        line.points.push_back(point(trajectory.evaluate(trajectory.times.size() - 1,
                                  trajectory.times(trajectory.times.size() - 1))));
        trajectory_vis_pub_.publish(line);
        auto points = marker("wp_path", 0, visualization_msgs::Marker::SPHERE_LIST);
        auto path = marker("wp_path", 1, visualization_msgs::Marker::LINE_STRIP);
        points.color.r = 0.0; path.color.g = 0.0;
        for (int i = 0; i < plan.path.rows(); ++i) points.points.push_back(point(plan.path.row(i).transpose()));
        path.points = points.points;
        path_vis_pub_.publish(points);
        path_vis_pub_.publish(path);
    }

    ros::NodeHandle nh_;
    minisnap::PlannerOptions options_;
    minisnap::HoverOptions hover_options_;
    std::unique_ptr<minisnap::StableHover> hover_gate_;
    std::string result_color_;
    std::string controller_status_topic_,controller_reason_="waiting for controller";
    bool require_controller_ready_=false,controller_status_valid_=false,controller_allowed_=false;
    bool status_displayed_=false,last_allowed_=false;
    double status_interval_=1,controller_status_max_age_=1,controller_status_stamp_=0,last_status_wall_=0;
    std::string frame_, odom_alias_, odom_topic_, twist_frame_;
    double max_odom_age_, max_result_age_, max_position_error_, max_velocity_error_, width_, sample_dt_;
    bool visualization_;
    int max_vis_points_;
    ros::Subscriber odom_sub_, waypoints_sub_;
    ros::Subscriber controller_status_sub_;
    ros::Publisher admission_status_pub_;
    ros::Publisher trajectory_pub_, trajectory_vis_pub_, path_vis_pub_;
    ros::WallTimer result_timer_;
    ros::WallTimer status_timer_;
    std::thread worker_;
    std::mutex mutex_;
    std::condition_variable cv_;
    State state_;
    Request request_;
    bool pending_ = false;
    std::unique_ptr<Completed> completed_;
    std::atomic<bool> stop_{false};
    std::atomic<unsigned long long> latest_{0};
};
}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator_node");
    try {
        TrajectoryGeneratorNode node;
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("Trajectory generator startup failed: %s", error.what());
        return 1;
    }
    return 0;
}
