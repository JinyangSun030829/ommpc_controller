#ifndef OMMPC_CONTROLLER_HPP
#define OMMPC_CONTROLLER_HPP
#include <Eigen/Eigen>
#include <osqp/osqp.h>
#include <ros/ros.h>
#include <vector>
#include <memory>
#include <stdexcept>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "polynomial_trajectory.h" // for traj reading required by NMPC
#include "so3_math.h"
#include "disturbance_observer.hpp"
#include <queue>
#include <deque>
#include <algorithm>
#include <cmath>
#include <traj_utils/PolyTraj.h>  // for ego_planner_v2

static constexpr int nstep = 20;  // N steps 20
static constexpr int nx = 9;      // dimension of error state (δp, δv, δR)
static constexpr int nstate = 10; // dimension of state (pos quat vel)
static constexpr int nu = 4;      // dimension of control input (thrust omg)

struct Odom_Data_t
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Quaterniond q;
  Eigen::Vector3d w;

  nav_msgs::Odometry msg;
  ros::Time rcv_stamp;
  bool recv_new_msg;

  Odom_Data_t()
  {
    recv_new_msg = false;
  }

  void feed(nav_msgs::OdometryConstPtr pMsg, bool enu_frame, bool vel_in_body)
  {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    recv_new_msg = true;

    p(0) = msg.pose.pose.position.x;
    p(1) = msg.pose.pose.position.y;
    p(2) = msg.pose.pose.position.z;

    v(0) = msg.twist.twist.linear.x;
    v(1) = msg.twist.twist.linear.y;
    v(2) = msg.twist.twist.linear.z;

    q.w() = msg.pose.pose.orientation.w;
    q.x() = msg.pose.pose.orientation.x;
    q.y() = msg.pose.pose.orientation.y;
    q.z() = msg.pose.pose.orientation.z;

    w(0) = msg.twist.twist.angular.x;
    w(1) = msg.twist.twist.angular.y;
    w(2) = msg.twist.twist.angular.z;

    if (!enu_frame)
    {
      Eigen::Matrix3d R_mid;
      R_mid << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
      Eigen::Quaterniond q_mid(R_mid);

      p = q_mid.toRotationMatrix() * p;
      v = q_mid.toRotationMatrix() * v;
      q = q_mid * q * q_mid;
      q.normalize();
      w = q_mid.toRotationMatrix() * w;
    }

    if (vel_in_body)
      v = q.toRotationMatrix() * v;
  }
};

struct Controller_Output_t
{
  // Body rates in body frame
  Eigen::Vector3d bodyrates; // [rad/s]

  // Collective mass normalized thrust
  double thrust;
};

struct Imu_Data_t
{
  Eigen::Quaterniond q;
  Eigen::Vector3d w;
  Eigen::Vector3d a;

  sensor_msgs::Imu msg;
  ros::Time rcv_stamp;
  bool recv_new_msg;

  Imu_Data_t()
  {
    recv_new_msg = false;
  }
  void feed(sensor_msgs::ImuConstPtr pMsg, bool enu_frame)
  {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    recv_new_msg = true;

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;

    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    if (!enu_frame)
    {
      Eigen::Matrix3d R_mid;
      R_mid << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
      Eigen::Quaterniond q_mid(R_mid);

      a = q_mid.toRotationMatrix() * a;
      q = q_mid * q * q_mid;
      q.normalize();
      w = q_mid.toRotationMatrix() * w;
    }
  }
};

struct oneTraj_Data_t
{
public:
  ros::Time traj_start_time{0};
  ros::Time traj_end_time{0};
  Trajectory traj;
  Trajectory yaw_traj;
};

class Trajectory_Data_t
{
public:
  ros::Time total_traj_start_time{0};
  ros::Time total_traj_end_time{0};
  int exec_traj = 0; // use for aborting the trajectory, 0 means no trajectory is executing
                     // -1 means the trajectory is aborting, 1 means the trajectory is executing
  std::deque<oneTraj_Data_t> traj_queue;

  Trajectory_Data_t()
  {
    total_traj_start_time = ros::Time(0);
    total_traj_end_time = ros::Time(0);
    exec_traj = 0;
  }
  void adjust_end_time()
  {
    if (traj_queue.size() < 2)
    {
      return;
    }
    for (auto it = traj_queue.begin(); it != (traj_queue.end() - 1); it++)
    {
      it->traj_end_time = (it + 1)->traj_start_time;
    }
  }
  // adapted for ego_planner_v2 (with its traj server)
  void feed_from_traj_utils(traj_utils::PolyTrajConstPtr pMsg)
  {
    // #1. try to execuse the action
    const traj_utils::PolyTraj &traj = *pMsg;

    if (traj.order >= 3)
    {
      ROS_WARN("[MPCCtrl] Loading the trajectory.");
      if (traj.traj_id < 1)
      {
        ROS_ERROR("[MPCCtrl] The trajectory_id must start from 1");
        return;
      }
      oneTraj_Data_t traj_data;
      traj_data.traj_start_time = pMsg->start_time;
      double t_total = 0;

      for (int i = 0; i < int(traj.duration.size()); ++i)
      {
        int num_dim = 3;
        int num_order = traj.order;
        t_total += traj.duration[i];

        Eigen::MatrixXd piece_coef;
        piece_coef.resize(num_dim, num_order + 1);
        for (int j = 0; j <= num_order; j++)
        {
          piece_coef(0, j) = traj.coef_x[i * (num_order + 1) + j];
          piece_coef(1, j) = traj.coef_y[i * (num_order + 1) + j];
          piece_coef(2, j) = traj.coef_z[i * (num_order + 1) + j];
        }
        traj_data.traj.emplace_back(traj.duration[i], piece_coef);
      }

      traj_data.traj_end_time = traj_data.traj_start_time + ros::Duration(t_total);

      // If traj_start_time is after current time, push_back the traj; else push_front
      if (ros::Time::now() < traj_data.traj_start_time) // Future traj
      {
        // A future trajectory
        while ((!traj_queue.empty()) && traj_queue.back().traj_start_time > traj_data.traj_start_time)
        {
          traj_queue.pop_back();
        }
        traj_queue.push_back(traj_data);
        total_traj_end_time = traj_queue.back().traj_end_time;
        total_traj_start_time = traj_queue.front().traj_start_time;
      }
      else // older traj
      {
        while ((!traj_queue.empty()) && traj_queue.front().traj_start_time < traj_data.traj_start_time)
        {
          traj_queue.pop_front();
        }
        traj_queue.push_front(traj_data);
        total_traj_end_time = traj_queue.back().traj_end_time;
        total_traj_start_time = traj_queue.front().traj_start_time;
      }

      exec_traj = 1;
    }
    else
    {
      exec_traj = -1;
    }
  }
};

// MPC solver

struct Solution
{
  std::vector<Eigen::VectorXd> delta_u; // optimal control sequence
  std::vector<Eigen::VectorXd> delta_x; // optimal state sequence
  double optimal_cost;
};

class MpcWrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MpcWrapper()
  {
    // variables: [δx0, δu0, δx1, δu1, ..., δx{nstep-1}, δu{nstep-1}, δx{nstep}]
    total_vars_ = (nstep + 1) * nx + nstep * nu;

    // constraints: error dynamics (nstep) + init (1) + control bounds (nstep)
    total_constraints_ = nstep * nx + nx + nstep * nu;
  }

  ~MpcWrapper()
  {
    free(P_data_);
    free(P_indices_);
    free(P_indptr_);
  }
  // 设置当前初始误差
  void setInitValue(const Eigen::VectorXd &x0)
  {
    for (int i = 0; i < nx; ++i)
    {
      l_[i] = x0[i];
      u_[i] = x0[i];
    }
  }

  void setDesiredStart(const Eigen::VectorXd &xdes, const Eigen::VectorXd &udes)
  {
    x_des_start = xdes;
    u_des_start = udes;
  }
  void getDesiredStart(Eigen::VectorXd &xdes, Eigen::VectorXd &udes)
  {
    xdes = x_des_start;
    udes = u_des_start;
  }

  // solve QP problem 负责将所有组装好的数据投喂给 OSQP 求解器，调用 ADMM 算法求解二次规划问题，并将最优控制量提取保存
  bool solve(Solution &sol)
  {
    // create OSQP data  OSQP 底层 C 数据结构组装
    OSQPData osqp_data;
    OSQPSettings osqp_settings;
    OSQPWorkspace *work = nullptr;

    // fill in the data  $n = \text{total\_vars\_}$（决策变量总数），$m = \text{total\_constraints\_}$（约束条件总行数）
    osqp_data.n = total_vars_;
    osqp_data.m = total_constraints_;
    osqp_data.P = csc_matrix(total_vars_, total_vars_, P_nnz_, P_data_, P_indices_, P_indptr_);
    osqp_data.q = q_.data();
    osqp_data.A = csc_matrix(total_constraints_, total_vars_, A_nnz_, A_data_, A_indices_, A_indptr_);
    osqp_data.l = l_.data();
    osqp_data.u = u_.data();

    // set OSQP params
    osqp_set_default_settings(&osqp_settings);
    // 解的精细化
    osqp_settings.polish = true;
    // 关闭日志输出
    osqp_settings.verbose = false;

    // create workspace and solve 求解器工作区初始化与错误防护
    c_int exit_code = osqp_setup(&work, &osqp_data, &osqp_settings);
    if (exit_code != 0)
    {
      free(A_data_);
      free(A_indices_);
      free(A_indptr_);
      return false;
    }
    // 执行 ADMM 优化求解与结果提取
    osqp_solve(work);

    // check solution status
    bool success = false;
    if (work != nullptr && work->info != nullptr)
    {
      // Determine success based on OSQP solution status
      if (work->info->status_val == OSQP_SOLVED ||
          work->info->status_val == OSQP_SOLVED_INACCURATE)
      {
        success = true;

        // Extract solution // 提取优化求得的偏差序列 δx* 和 δu*
        extractSolution(work->solution->x, sol);
        sol.optimal_cost = work->info->obj_val;
      }
    }

    // Clean up 销毁 OSQP 求解器工作区 work，释放内部申请的 C 语言动态内存
    if (work != nullptr)
    {
      osqp_cleanup(work);
    }
    // 释放 buildConstraintMatrix 中使用 malloc 分配的稀疏矩阵 $A$ 数据内存，确保整个函数运行完毕后没有任何内存泄漏
    free(A_data_);
    free(A_indices_);
    free(A_indptr_);

    return success;
  }

  // Set weight matrix, only once为OSQP构造MPC二次规划目标函数中的Hessian矩阵 P
  void buildHessianMatrix(
      const Eigen::Matrix<double, nx, nx> &Q_diag,
      const Eigen::Matrix<double, nu, nu> &R_diag,
      const double state_cost_exponential,
      const double input_cost_exponential)
  {

    // Hessian matrix is diagonal, directly construct CSC format
    P_nnz_ = total_vars_;
    // 因为 P 是对角矩阵，每个变量对应一个对角元素，所以非零元素数量等于优化变量总数
    // P_data_	保存非零元素的数值
    // P_indices_	保存每个非零元素所在的行号
    // P_indptr_	保存每一列非零元素的起始位置
    P_data_ = (c_float *)malloc(P_nnz_ * sizeof(c_float));
    P_indices_ = (c_int *)malloc(P_nnz_ * sizeof(c_int));
    P_indptr_ = (c_int *)malloc((total_vars_ + 1) * sizeof(c_int));

    int var_idx = 0;

    // Set column pointers构造列指针
    for (int col = 0; col <= total_vars_; ++col)
    {
      P_indptr_[col] = col;
    }

    for (int k = 0; k < nstep; ++k)
    {
      // δxk part
      double state_decay = std::exp(-(double)k / (double)nstep * state_cost_exponential);
      for (int i = 0; i < nx; ++i)
      {
        P_indices_[var_idx] = var_idx;
        P_data_[var_idx] = Q_diag(i, i) * state_decay;
        var_idx++;
      }

      double input_decay = std::exp(-(double)k / (double)nstep * input_cost_exponential);
      // δuk part
      for (int i = 0; i < nu; ++i)
      {
        P_indices_[var_idx] = var_idx;
        P_data_[var_idx] = R_diag(i, i) * input_decay;
        var_idx++;
      }
    }

    const Eigen::Matrix<double, nx, nx> P_final_diag = Q_diag * std::exp(-state_cost_exponential);

    // δx{nstep} (final) part
    for (int i = 0; i < nx; ++i)
    {
      P_indices_[var_idx] = var_idx;
      P_data_[var_idx] = P_final_diag(i, i);
      var_idx++;
    }
  }

  // Set constraint matrix把控制理论中的状态方程和约束条件，组装并转换为 OSQP 求解器所要求的 CSC（Compressed Sparse Column，压缩稀疏列）格式全局约束矩阵 $A$ $ l \le A z \le u$
  void buildConstraintMatrix(
      const std::vector<Eigen::SparseMatrix<double>> Fx,
      const std::vector<Eigen::SparseMatrix<double>> Fu)
  {
    // 稀疏矩阵必须按列连续存储，且需要三个关键数组
    // Calculate the number of non-zero elements 阶段一：计算非零元素总量并分配内存
    A_nnz_ = nx; // Initial condition constraints

    // non-zero elements of dynamics constraints
    for (int k = 0; k < nstep; ++k)
    {
      A_nnz_ += Fx[k].nonZeros() + Fu[k].nonZeros() + nx;
    }

    // non-zero elements of control constraint (each constraint has 1 non-zero element)
    A_nnz_ += nstep * nu;

    // Allocate memory  malloc 预先一次性分配 OSQP 专用的 C 语言内存，避免动态扩容带来的性能损耗
    A_data_ = (c_float *)malloc(A_nnz_ * sizeof(c_float));
    A_indices_ = (c_int *)malloc(A_nnz_ * sizeof(c_int));
    A_indptr_ = (c_int *)malloc((total_vars_ + 1) * sizeof(c_int));

    // Initialize column pointers
    std::vector<int> col_nnz(total_vars_, 0);

    // First, count non-zero elements per column
    // Initial condition constraints: columns corresponding to δx0
    // 1. 统计初始条件对 δx0 列的贡献
    for (int i = 0; i < nx; ++i)
    {
      int col = i;
      col_nnz[col]++;
    }
    // 2. 统计动力学约束对 δxk, δuk, δx{k+1} 列的贡献
    // Dynamics constraints
    int constraint_idx = nx;
    for (int k = 0; k < nstep; ++k)
    {
      int xk_offset = k * (nx + nu);
      int uk_offset = xk_offset + nx;
      int xkp1_offset = (k + 1) * (nx + nu);

      // -Fx[k] part (corresponding to δxk)
      for (int j = 0; j < Fx[k].outerSize(); ++j)
      {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Fx[k], j); it; ++it)
        {
          int col = xk_offset + it.col();
          col_nnz[col]++;
        }
      }

      // -Fu[k] part (corresponding to δuk)
      for (int j = 0; j < Fu[k].outerSize(); ++j)
      {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Fu[k], j); it; ++it)
        {
          int col = uk_offset + it.col();
          col_nnz[col]++;
        }
      }

      // I part (corresponding to δx{k+1})
      for (int i = 0; i < nx; ++i)
      {
        int col = xkp1_offset + i;
        col_nnz[col]++;
      }

      constraint_idx += nx;
    }

    // Control constraints // 3. 统计控制约束对 δuk 列的贡献
    for (int k = 0; k < nstep; ++k)
    {
      int uk_offset = k * (nx + nu) + nx;

      for (int i = 0; i < nu; ++i)
      {
        int col = uk_offset + i;
        col_nnz[col]++;
      }
    }

    // Set column pointers // 4. 前缀和（Prefix Sum）计算列指针 A_indptr_
    A_indptr_[0] = 0;
    for (int col = 0; col < total_vars_; ++col)
    {
      A_indptr_[col + 1] = A_indptr_[col] + col_nnz[col];
    }

    // Then, fill in data 阶段三：第二趟扫描（Pass 2）—— 填入数值与行索引
    // 有了列指针 A_indptr_ 之后，每一列在内存中的起始位置就固定了。代码使用 col_pos[col] 作为列内相对偏移量，开始真正填充数据
    std::vector<int> col_pos(total_vars_, 0);
    std::vector<c_float> temp_data(A_nnz_);
    std::vector<c_int> temp_indices(A_nnz_);

    // Reset constraint index
    constraint_idx = 0;

    // Initial condition constraints
    for (int i = 0; i < nx; ++i)
    {
      int col = i;
      int pos = A_indptr_[col] + col_pos[col];
      temp_data[pos] = 1.0;
      temp_indices[pos] = constraint_idx++;
      col_pos[col]++;
    }

    // Dynamics constraints
    for (int k = 0; k < nstep; ++k)
    {
      int xk_offset = k * (nx + nu);
      int uk_offset = xk_offset + nx;
      int xkp1_offset = (k + 1) * (nx + nu);

      // -Fx[k]
      for (int j = 0; j < Fx[k].outerSize(); ++j)
      {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Fx[k], j); it; ++it)
        {
          int col = xk_offset + it.col();
          int pos = A_indptr_[col] + col_pos[col];
          temp_data[pos] = -it.value();
          temp_indices[pos] = constraint_idx + it.row();
          col_pos[col]++;
        }
      }

      // -Fu[k]
      for (int j = 0; j < Fu[k].outerSize(); ++j)
      {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Fu[k], j); it; ++it)
        {
          int col = uk_offset + it.col();
          int pos = A_indptr_[col] + col_pos[col];
          temp_data[pos] = -it.value();
          temp_indices[pos] = constraint_idx + it.row();
          col_pos[col]++;
        }
      }

      // I
      for (int i = 0; i < nx; ++i)
      {
        int col = xkp1_offset + i;
        int pos = A_indptr_[col] + col_pos[col];
        temp_data[pos] = 1.0;
        temp_indices[pos] = constraint_idx + i;
        col_pos[col]++;
      }

      constraint_idx += nx;
    }

    // Control constraints
    // int control_constraint_start = constraint_idx;
    for (int k = 0; k < nstep; ++k)
    {
      int uk_offset = k * (nx + nu) + nx;

      for (int i = 0; i < nu; ++i)
      {
        int col = uk_offset + i;
        int pos = A_indptr_[col] + col_pos[col];
        temp_data[pos] = 1.0;
        temp_indices[pos] = constraint_idx++;
        col_pos[col]++;
      }
    }

    // Copy to OSQP arrays 内存拷贝至 OSQP 结构体
    memcpy(A_data_, temp_data.data(), A_nnz_ * sizeof(c_float));
    memcpy(A_indices_, temp_indices.data(), A_nnz_ * sizeof(c_int));
  }

  // Build constraint right-hand side vectors 构建二次规划（QP）问题中的目标函数一次项向量 $q$ 以及约束的上下界向量 $l$（Lower Bound）和 $u$（Upper Bound）
  void buildConstraintVectors(
      const std::vector<Eigen::VectorXd> &u_min,
      const std::vector<Eigen::VectorXd> &u_max)
  {
    // 向量尺寸重置与一次项 $q$ 的初始化
    q_.resize(total_vars_, 0.0);
    l_.resize(total_constraints_, 0.0);
    u_.resize(total_constraints_, 0.0);

    // Dynamics constraints (equality constraints: l=0, u=0)
    int offset = nx;
    // 动力学等式约束填装
    std::fill(l_.begin() + offset, l_.begin() + offset + nstep * nx, 0.0);
    std::fill(u_.begin() + offset, u_.begin() + offset + nstep * nx, 0.0);

    // Control constraints
    offset += nstep * nx;
    for (int k = 0; k < nstep; ++k)
    {
      for (int i = 0; i < nu; ++i)
      {
        l_[offset] = u_min[k][i];
        u_[offset] = u_max[k][i];
        offset++;
      }
    }
  }

private:
  // dimension of the problem
  int total_vars_;
  int total_constraints_;

  // data in OSQP CSC format matrix
  c_float *P_data_ = nullptr;
  c_int *P_indices_ = nullptr;
  c_int *P_indptr_ = nullptr;
  int P_nnz_ = 0;

  c_float *A_data_ = nullptr;
  c_int *A_indices_ = nullptr;
  c_int *A_indptr_ = nullptr;
  int A_nnz_ = 0;

  // data in vectors
  std::vector<c_float> q_;
  std::vector<c_float> l_;
  std::vector<c_float> u_;

  // desired state, for init error setting
  Eigen::VectorXd x_des_start;
  Eigen::VectorXd u_des_start;

  // Extract solution
  void extractSolution(const c_float *solution, Solution &sol)
  {
    sol.delta_u.resize(nstep);
    sol.delta_x.resize(nstep + 1);

    for (int k = 0; k < nstep; ++k)
    {
      int x_offset = k * (nx + nu);
      int u_offset = x_offset + nx;

      sol.delta_x[k] = Eigen::VectorXd(nx);
      sol.delta_u[k] = Eigen::VectorXd(nu);

      for (int i = 0; i < nx; ++i)
      {
        sol.delta_x[k][i] = solution[x_offset + i];
      }

      for (int i = 0; i < nu; ++i)
      {
        sol.delta_u[k][i] = solution[u_offset + i];
      }
    }

    // Final state
    int final_offset = nstep * (nx + nu);
    sol.delta_x[nstep] = Eigen::VectorXd(nx);
    for (int i = 0; i < nx; ++i)
    {
      sol.delta_x[nstep][i] = solution[final_offset + i];
    }
  }
};
struct DobDebugData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ros::Time stamp;

  bool enabled = false;
  bool initialized = false;
  bool input_valid = false;

  // ESO 原始输出
  Eigen::Vector3d disturbance_raw =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d disturbance_dot_raw =
      Eigen::Vector3d::Zero();

  // 滤波结果
  Eigen::Vector3d disturbance_filtered =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d disturbance_dot_filtered =
      Eigen::Vector3d::Zero();

  // 真正进入微分平坦的补偿结果
  Eigen::Vector3d disturbance_comp =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d disturbance_comp_dot =
      Eigen::Vector3d::Zero();

  // 观测器输入与内部状态
  Eigen::Vector3d nominal_acceleration =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d measured_velocity =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d estimated_velocity =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d velocity_error =
      Eigen::Vector3d::Zero();

  double delayed_thrustacc = 0.0;
  double observer_dt = 0.0;
  double ramp = 0.0;
};
struct Parameter_t
{
  double step_T;
  double hover_percent;
  double Q_pos_xy, Q_pos_z, Q_velocity, Q_attitude_rp, Q_attitude_yaw;
  double R_thrust, R_pitchroll, R_yaw;
  double state_cost_exponential, input_cost_exponential;
  double max_bodyrate_xy, max_bodyrate_z, min_thrust, max_thrust;

  bool use_fix_yaw, use_trajectory_ending_pos;

  bool use_ref_txt; 
	std::string ref_filename;
	double ref_time_step;

  // Flight-state-machine parameters
  double state_timeout;
  double service_retry_interval;
  double takeoff_altitude;
  double takeoff_speed;
  double takeoff_tolerance;
  double landing_speed;
  double landing_target_offset;
  double touchdown_height;
  double touchdown_max_vz;
  double touchdown_max_vxy;
  double touchdown_confirm_time;
  double thrust_unload_time;
  double touchdown_thrust;
  // Disturbance observer parameters
  bool dob_enable;

  // Observer gains
  double dob_l1;
  double dob_l2;

  // Filtering and safety
  double dob_lpf_tau;
  double dob_derivative_lpf_tau;
  double dob_max_acc;
  double dob_max_jerk;
  double dob_ramp_time;
  double dob_landing_disable_height;
  // Delay between thrust command and actual thrust response
  double dob_input_delay;
  // Takeoff liftoff detection thresholds in the world frame.  These belong to
  // the flight-state machine; the DOB only consumes the resulting airborne
  // flag.
  double takeoff_liftoff_height;
  double takeoff_liftoff_vz;
};

class MpcController
{
private:
  const double gravity_ = 9.81;
  Eigen::Vector3d reference_acceleration_ = Eigen::Vector3d::Zero();

  // MPC param wrapper
	MpcWrapper mpc_wrapper_;

  // params
  Parameter_t param_;
	DisturbanceObserver3D disturbance_observer_;

  // 原始观测结果
  Eigen::Vector3d d_hat_raw_ =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d d_hat_dot_raw_ =
      Eigen::Vector3d::Zero();

  // 低通滤波结果
  Eigen::Vector3d d_hat_filtered_ =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d d_hat_dot_filtered_ =
      Eigen::Vector3d::Zero();

  // 真正用于微分平坦前馈补偿的量
  Eigen::Vector3d d_comp_ =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d d_comp_dot_ =
      Eigen::Vector3d::Zero();

  bool dob_initialized_ = false;

  ros::Time dob_last_update_stamp_;
  ros::Time dob_start_stamp_;

  struct AppliedThrustSample
  {
    ros::Time stamp;

    // 质量归一化推力加速度，单位 m/s^2
    double thrustacc;
  };

  std::deque<AppliedThrustSample>
      dob_thrust_history_;
   /******************************************************************
   * DOB debug variables
   ******************************************************************/

  ros::Time dob_debug_stamp_;

  bool dob_input_valid_ = false;

  Eigen::Vector3d dob_nominal_acceleration_ =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d dob_measured_velocity_ =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d dob_estimated_velocity_ =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d dob_velocity_error_ =
      Eigen::Vector3d::Zero();

  double dob_delayed_thrustacc_ = 0.0;
  double dob_observer_dt_ = 0.0;
  double dob_ramp_ = 0.0;
	// for MPC timing
	double timing_feedback_;
	
  // calculate yaw
	double last_yaw_, last_yaw_dot_;
	// Helper functions for calculating yaw and yawdot
  double angle_limit(double ang)
  {
    while (ang > M_PI)
    {
        ang -= 2.0 * M_PI;
    }
    while (ang <= -M_PI)
    {
        ang += 2.0 * M_PI;
    }
    return ang;
  }

  double angle_diff(double a, double b)
  {
      double d1, d2;
      d1 = a - b;
      d2 = 2 * M_PI - fabs(d1);
      if (d1 > 0)
          d2 *= -1.0;
      if (fabs(d1) < fabs(d2))
          return (d1);
      else
          return (d2);
  }

  void calculate_yaw(Eigen::Vector3d &vel, const double dt, double &yaw, double &yawdot)
  {
    const double YAW_DOT_MAX_PER_SEC = param_.max_bodyrate_z * 0.90;
    const double YAW_DOT_DOT_MAX_PER_SEC = param_.max_bodyrate_z * 4.0;

    // tangent line
    double yaw_temp = vel.norm() > 0.1
                          ? atan2(vel(1), vel(0))
                          : last_yaw_;
    
    double d_yaw = angle_diff(yaw_temp, last_yaw_);

    const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
    const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
    double d_yaw_max;
    if (fabs(last_yaw_dot_ + dt * YDDM) <= fabs(YDM))
    {
      d_yaw_max = last_yaw_dot_ * dt + 0.5 * YDDM * dt * dt;
    }
    else
    {
      // yawdot = YDM;
      double t1 = (YDM - last_yaw_dot_) / YDDM;
      d_yaw_max = ((dt - t1) + dt) * (YDM - last_yaw_dot_) / 2.0;
    }

    if (fabs(d_yaw) > fabs(d_yaw_max))
    {
      d_yaw = d_yaw_max;
    }
    yawdot = d_yaw / dt;

    yaw = last_yaw_ + d_yaw;
    if (yaw > M_PI)
      yaw -= 2 * M_PI;
    if (yaw < -M_PI)
      yaw += 2 * M_PI;

    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;
  }
  /**
   * @brief 对三维向量逐元素限幅
   */
  Eigen::Vector3d saturateVector(
      const Eigen::Vector3d &value,
      const double limit) const
  {
    Eigen::Vector3d result = value;

    for (int i = 0; i < 3; ++i)
    {
      result(i) = std::max(
          -limit,
          std::min(limit, result(i)));
    }

    return result;
  }

  /**
   * @brief 从历史控制输入中取得延迟后的推力加速度
   *
   * 例如 dob_input_delay = 0.04，表示使用约 40 ms 前发送的推力。
   */
  bool getDelayedThrustAcceleration(
      const ros::Time &current_stamp,
      double &thrustacc)
  {
    if (dob_thrust_history_.empty())
    {
      return false;
    }

    const ros::Time target_stamp =
        current_stamp -
        ros::Duration(param_.dob_input_delay);

    // 删除明显早于目标时刻的数据，但至少保留一个样本
    while (dob_thrust_history_.size() >= 2 &&
           dob_thrust_history_[1].stamp <= target_stamp)
    {
      dob_thrust_history_.pop_front();
    }

    if (dob_thrust_history_.front().stamp >
        target_stamp)
    {
      return false;
    }

    thrustacc =
        dob_thrust_history_.front().thrustacc;

    return true;
  }

  /**
   * @brief 构造扰动补偿后的微分平坦输入
   *
   * 动力学：
   *
   *   v_dot = aT * R * e3 - g * e3 + d
   *
   * 因此：
   *
   *   thrust_vector = a_des + g*e3 - d_hat
   *   thrust_jerk   = j_des - d_hat_dot
   */
  void buildCompensatedFlatnessInput(
      const Eigen::Vector3d &desired_acc,
      const Eigen::Vector3d &desired_jerk,
      Eigen::Vector3d &thrust_vector,
      Eigen::Vector3d &thrust_jerk) const
  {
    const Eigen::Vector3d gravity_vector(
        0.0, 0.0, gravity_);

    if (param_.dob_enable)
    {
      thrust_vector =desired_acc +gravity_vector -d_comp_;
      thrust_jerk =desired_jerk;
      // thrust_jerk =
      //     desired_jerk -
      //     d_comp_dot_;
    }
    else
    {
      thrust_vector =
          desired_acc +
          gravity_vector;

      thrust_jerk =
          desired_jerk;
    }
  }

public:
  void recordAppliedControl(
      const double normalized_thrust,
      const Eigen::Vector3d &bodyrates,
      const ros::Time &stamp)
  {
    (void)bodyrates;

    if (!param_.dob_enable)
    {
      return;
    }

    AppliedThrustSample sample;

    sample.stamp = stamp;

    // normalized thrust -> thrust acceleration
    sample.thrustacc =
        normalized_thrust * thr2acc;

    dob_thrust_history_.push_back(sample);

    while (dob_thrust_history_.size() > 200)
    {
      dob_thrust_history_.pop_front();
    }
  }

  /**
   * @brief 重置扰动观测器
   */
  void resetDisturbanceObserver()
  {
    dob_initialized_ = false;

    dob_last_update_stamp_ = ros::Time(0);
    dob_start_stamp_ = ros::Time(0);

    d_hat_raw_.setZero();
    d_hat_dot_raw_.setZero();

    d_hat_filtered_.setZero();
    d_hat_dot_filtered_.setZero();

    d_comp_.setZero();
    d_comp_dot_.setZero();

    dob_thrust_history_.clear();
     dob_debug_stamp_ = ros::Time(0);

    dob_input_valid_ = false;

    dob_nominal_acceleration_.setZero();
    dob_measured_velocity_.setZero();
    dob_estimated_velocity_.setZero();
    dob_velocity_error_.setZero();

    dob_delayed_thrustacc_ = 0.0;
    dob_observer_dt_ = 0.0;
    dob_ramp_ = 0.0;

  }

  /**
   * @brief 更新平移扰动观测器
   *
   * 输入状态：
   *   odom.v : 世界坐标系速度
   *   odom.q : 实际姿态
   *
   * 输入控制：
   *   历史记录中延迟后的实际发送推力
   */
  void startDisturbanceObserver(
    const Odom_Data_t &odom)
{
    /*
     * 先清除上一次飞行遗留状态。
     */
    resetDisturbanceObserver();

    if (!param_.dob_enable)
    {
        return;
    }

    if (!odom.recv_new_msg)
    {
        return;
    }

    /*
     * 直接使用离地瞬间的真实速度
     * 初始化ESO速度状态。
     *
     * 因此初始速度误差：
     *
     * v - v_hat = 0
     */
    disturbance_observer_.reset(
        odom.v);

    dob_initialized_ = true;

    /*
     * 记录DOB启动时刻。
     */
    dob_last_update_stamp_ =
        odom.rcv_stamp;

    dob_start_stamp_ =
        odom.rcv_stamp;

    /*
     * 调试数据初始化。
     */
    dob_debug_stamp_ =
        odom.rcv_stamp;

    dob_measured_velocity_ =
        odom.v;

    dob_estimated_velocity_ =
        odom.v;

    dob_velocity_error_.setZero();

    dob_nominal_acceleration_.setZero();

    dob_input_valid_ = false;

    dob_delayed_thrustacc_ = 0.0;

    dob_observer_dt_ = 0.0;

    dob_ramp_ = 0.0;

    ROS_WARN(
        "[DOB] Observer initialized at liftoff. "
        "v=[%.3f %.3f %.3f]",
        odom.v(0),
        odom.v(1),
        odom.v(2));
}
  void updateDisturbanceObserver(
      const Odom_Data_t &odom)
  {
    if (!param_.dob_enable ||
        !odom.recv_new_msg)
    {
      return;
    }

    const ros::Time current_stamp =
        odom.rcv_stamp;
    dob_debug_stamp_ = current_stamp;
    dob_measured_velocity_ = odom.v;
    dob_input_valid_ = false;
    // 第一次调用，只初始化速度估计
    if (!dob_initialized_)
    {
      disturbance_observer_.reset(odom.v);

      dob_last_update_stamp_ =
          current_stamp;

      dob_start_stamp_ =
          current_stamp;

      dob_initialized_ = true;

      return;
    }

    // 防止相同 odom 数据重复积分
    if (current_stamp <=
        dob_last_update_stamp_)
    {
      return;
    }

    double delayed_thrustacc = 0.0;

    if (!getDelayedThrustAcceleration(
            current_stamp,
            delayed_thrustacc))
    {
      return;
    }
    dob_input_valid_ = true;
    dob_delayed_thrustacc_ = delayed_thrustacc;

    const double dt =
        (current_stamp -
         dob_last_update_stamp_).toSec();
    dob_observer_dt_ = dt;      
    // 数据中断或时间异常时重新初始化
    if (dt <= 1.0e-4 || dt > 0.1)
    {
      disturbance_observer_.reset(odom.v);

      dob_last_update_stamp_ =
          current_stamp;

      d_hat_raw_.setZero();
      d_hat_dot_raw_.setZero();

      d_hat_filtered_.setZero();
      d_hat_dot_filtered_.setZero();

      d_comp_.setZero();
      d_comp_dot_.setZero();

      return;
    }

    Eigen::Quaterniond q = odom.q;
    q.normalize();

    const Eigen::Vector3d e3(
        0.0, 0.0, 1.0);

    /*
     * 已知名义加速度：
     *
     * a_nom = aT * R * e3 - g * e3
     */
    const Eigen::Vector3d a_nom =
        delayed_thrustacc *
        q.toRotationMatrix() * e3 -
        gravity_ * e3;
    dob_nominal_acceleration_ =
        a_nom;
    disturbance_observer_.update(
        odom.v,
        a_nom,
        dt);
    dob_estimated_velocity_ =
        disturbance_observer_.velocityEstimate();

    dob_velocity_error_ =
        odom.v -
        dob_estimated_velocity_;
    d_hat_raw_ =
        disturbance_observer_.disturbance();

    d_hat_dot_raw_ =
        disturbance_observer_
            .disturbanceDerivative();

    // 原始扰动限幅
    d_hat_raw_ =
        saturateVector(
            d_hat_raw_,
            param_.dob_max_acc);

    d_hat_dot_raw_ =
        saturateVector(
            d_hat_dot_raw_,
            param_.dob_max_jerk);

    // 扰动低通滤波
    const double alpha_d =
        dt /
        (param_.dob_lpf_tau + dt);

    d_hat_filtered_ +=
        alpha_d *
        (d_hat_raw_ -
         d_hat_filtered_);

    // 扰动导数低通滤波
    const double alpha_dd =
        dt /
        (param_.dob_derivative_lpf_tau + dt);

    d_hat_dot_filtered_ +=
        alpha_dd *
        (d_hat_dot_raw_ -
         d_hat_dot_filtered_);

    /*
     * 补偿渐入：
     * 防止观测器刚启用时，前馈姿态突然发生较大变化。
     */
    double ramp = 1.0;

    if (param_.dob_ramp_time > 1.0e-3)
    {
      ramp =
          (current_stamp -
           dob_start_stamp_).toSec() /
          param_.dob_ramp_time;

      ramp =
          std::max(0.0,
          std::min(1.0, ramp));
    }

    d_comp_ =
        ramp * d_hat_filtered_;

    d_comp_dot_ =
        ramp * d_hat_dot_filtered_;
    dob_ramp_ = ramp;
    dob_last_update_stamp_ =
        current_stamp;

    // ROS_INFO_THROTTLE(
    //     1.0,
    //     "Update:DOB d_hat=[%.3f %.3f %.3f], d_comp=[%.3f %.3f %.3f]",
    //     d_hat_raw_(0),
    //     d_hat_raw_(1),
    //     d_hat_raw_(2),
    //     d_comp_(0),
    //     d_comp_(1),
    //     d_comp_(2));
  }

  Eigen::Vector3d getDisturbanceEstimate() const
  {
    return d_comp_;
  }
  void getDobDebugData(
    DobDebugData &data) const
{
  data.stamp =
      dob_debug_stamp_;

  data.enabled =
      param_.dob_enable;

  data.initialized =
      dob_initialized_;

  data.input_valid =
      dob_input_valid_;

  data.disturbance_raw =
      d_hat_raw_;

  data.disturbance_dot_raw =
      d_hat_dot_raw_;

  data.disturbance_filtered =
      d_hat_filtered_;

  data.disturbance_dot_filtered =
      d_hat_dot_filtered_;

  data.disturbance_comp =
      d_comp_;

  data.disturbance_comp_dot =
      d_comp_dot_;

  data.nominal_acceleration =
      dob_nominal_acceleration_;

  data.measured_velocity =
      dob_measured_velocity_;

  data.estimated_velocity =
      dob_estimated_velocity_;

  data.velocity_error =
      dob_velocity_error_;

  data.delayed_thrustacc =
      dob_delayed_thrustacc_;

  data.observer_dt =
      dob_observer_dt_;

  data.ramp =
      dob_ramp_;
}
  std::queue<std::pair<ros::Time, double>> timed_thrust;

	// Thrust-accel mapping params
	const double rho2 = 0.998; // do not change
	double thr2acc;
	double P;

	// MPC reference state/input matrices
	std::vector<Eigen::SparseMatrix<double>> Fx;  // nstep，nx×nx
  std::vector<Eigen::SparseMatrix<double>> Fu;  // nstep，nx×nu

	// upper and lower bounds of control input
	std::vector<Eigen::VectorXd> u_lb;
	std::vector<Eigen::VectorXd> u_ub;
  bool getCurrentReferencePosition(Eigen::Vector3d &ref_position)
  {
      Eigen::VectorXd x_des;
      Eigen::VectorXd u_des;

      mpc_wrapper_.getDesiredStart(x_des, u_des);

      if (x_des.size() != nstate)
      {
          return false;
      }

      // x_des = [px, py, pz, qw, qx, qy, qz, vx, vy, vz]
      ref_position = x_des.head<3>();

      return true;
  }
  bool getCurrentReferenceState(Eigen::Vector3d &position,
                               Eigen::Vector3d &velocity,
                               Eigen::Vector3d &acceleration)
  {
      Eigen::VectorXd x_des, u_des;
      mpc_wrapper_.getDesiredStart(x_des, u_des);
      if (x_des.size() != nstate) return false;
      position = x_des.head<3>();
      velocity = x_des.tail<3>();
      acceleration = reference_acceleration_;
      return position.allFinite() && velocity.allFinite() && acceleration.allFinite();
  }

  void init(Parameter_t param)
  {
    param_ = param;
    thr2acc = gravity_ / param_.hover_percent;
    P = 1e6;
    disturbance_observer_.configure(param_.dob_l1,param_.dob_l2);
    resetDisturbanceObserver();
    // MPC Controller
    ROS_WARN("mpc time_step: %f", param_.step_T);

    // set gains
    Eigen::Matrix<double, nx, nx> Q = (Eigen::Matrix<double, nx, 1>() 
      << param_.Q_pos_xy, param_.Q_pos_xy, param_.Q_pos_z,
        param_.Q_velocity, param_.Q_velocity, param_.Q_velocity,
        param_.Q_attitude_rp, param_.Q_attitude_rp, param_.Q_attitude_yaw).finished().asDiagonal();
    Eigen::Matrix<double, nu, nu> R = (Eigen::Matrix<double, nu, 1>()  
      << param_.R_thrust, param_.R_pitchroll, param_.R_pitchroll, param_.R_yaw).finished().asDiagonal();
    // 构造Hessian矩阵
    mpc_wrapper_.buildHessianMatrix(Q, R, param_.state_cost_exponential, param_.input_cost_exponential);

    Fx.resize(nstep);
    Fu.resize(nstep);
    u_lb.resize(nstep);
    u_ub.resize(nstep);
    for (int i = 0; i < nstep; i++)
    {
      u_ub[i].resize(nu);
      u_lb[i].resize(nu);
    }

    // Timing
    timing_feedback_ = 0.0;
  }

  void computeFlatInputwithHopfFibration(const Eigen::Vector3d &thr_acc,
                                  const Eigen::Vector3d &jer,
                                  const double &yaw,
                                  const double &yawd,
                                  const Eigen::Quaterniond &att_est,
                                  Eigen::Quaterniond &att,
                                  Eigen::Vector3d &omg) const
  {
    static Eigen::Vector3d omg_old(0.0, 0.0, 0.0);
    //机身 $Z$ 轴在世界坐标系下的单位向量 
    Eigen::Vector3d abc = thr_acc.normalized();
    double a = abc(0), b = abc(1), c = abc(2);
    // 紧接着计算 $\mathbf{z}_b$ 对时间的导数
    Eigen::Vector3d abc_dot = (thr_acc.dot(thr_acc) * Eigen::MatrixXd::Identity(3, 3) - thr_acc * thr_acc.transpose()) / thr_acc.norm() / thr_acc.squaredNorm() * jer;
    double a_dot = abc_dot(0), b_dot = abc_dot(1), c_dot = abc_dot(2);
    // 奇异性保护条件
    // $c = abc(2)$ 是机身 $Z$ 轴在世界坐标系 Z 轴上的投影。当 $c \to -1$ 时，意味着无人机完全颠倒（机头垂直朝下）。在数学公式的分母中会出现 $1 + c$，因此这里防止除以零
    // thr_acc.norm() > 0.1：防止无人机处于自由落体（失重/零推力）状态时分母为零
    // if(1.0 + c > 1e-3 && thr_acc.norm() > 0.1){
    //   // 把基准向量 $[0, 0, 1]^T$ 旋转到目标的机身方向 $\mathbf{z}_b = [a, b, c]^T$，利用 Hopf 纤维化导出的无奇异四元数表示
    //   double norm = sqrt(2 * (1 + c));
    //   Eigen::Quaterniond q((1 + c) / norm, -b / norm, a / norm, 0);
    //   Eigen::Quaterniond q_yaw(cos(yaw / 2), 0, 0, sin(yaw / 2));
    //   att = q * q_yaw;
    //   // 代码直接利用 $\mathbf{z}_b$、$\dot{\mathbf{z}}_b$ 和 $\psi, \dot{\psi}$ 的解析关系，一步到位无滞后地算出机体三轴角速度 $\boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z]^T$
    //   double syaw = sin(yaw), cyaw = cos(yaw);
    //   omg(0) = syaw * a_dot - cyaw * b_dot - (a * syaw - b * cyaw) * c_dot / (c + 1);
    //   omg(1) = cyaw * a_dot + syaw * b_dot - (a * cyaw + b * syaw) * c_dot / (c + 1);
    //   omg(2) = (b * a_dot - a * b_dot) / (1 + c) + yawd;
    // }else{
    //   std::cout << "Near singularity!!!!!" << std::endl;
    //   omg = omg_old;
    //   att = att_est;
    // }
    if (thr_acc.norm() > 0.1) {
    if (c >= 0.0) {
        // ========== Chart 1: 适用于上半球 (正飞及一般倾角) ==========
        // 奇点位于 c = -1 (完全倒置)
        double norm = sqrt(2 * (1 + c));
        // 四元数顺序为 (w, x, y, z)
        Eigen::Quaterniond q((1 + c) / norm, -b / norm, a / norm, 0);
        Eigen::Quaterniond q_yaw(cos(yaw / 2), 0, 0, sin(yaw / 2));
        att = q * q_yaw;
        
        double syaw = sin(yaw), cyaw = cos(yaw);
        omg(0) = syaw * a_dot - cyaw * b_dot - (a * syaw - b * cyaw) * c_dot / (c + 1);
        omg(1) = cyaw * a_dot + syaw * b_dot - (a * cyaw + b * syaw) * c_dot / (c + 1);
        omg(2) = (b * a_dot - a * b_dot) / (1 + c) + yawd;
        
      } else {
        // ========== Chart 2: 适用于下半球 (大角度倒飞) ==========
        // 奇点位于 c = 1 (完全正飞)
        double norm = sqrt(2 * (1 - c));
        
        // 依据论文 Eq. 22 构造第二个图的四元数
        // 论文公式为 [-b, 1-c, 0, a]^T，映射到 Eigen::Quaterniond(w, x, y, z)
        Eigen::Quaterniond q_bar(-b / norm, (1 - c) / norm, 0, a / norm);
        
        // 依据论文 Eq. 24 计算 Chart 2 下的新偏航角 yaw_bar
        double yaw_bar = atan2(a, b) + yaw;
        Eigen::Quaterniond q_yaw_bar(cos(yaw_bar / 2), 0, 0, sin(yaw_bar / 2));
        att = q_bar * q_yaw_bar;
        
        // 计算 yaw_bar 的导数 (用于角速度 omg_z 的计算)
        // d(atan2(a,b))/dt = (b*a_dot - a*b_dot) / (a^2 + b^2)
        // 因为 a^2 + b^2 + c^2 = 1，所以 a^2 + b^2 = 1 - c^2
        double yawd_bar = yawd + (b * a_dot - a * b_dot) / (1.0 - c * c);
        
        // 依据论文 Eq. 25 计算 Chart 2 下的角速度
        double syaw = sin(yaw), cyaw = cos(yaw);
        omg(0) = syaw * a_dot + cyaw * b_dot - (a * syaw + b * cyaw) * c_dot / (c - 1);
        omg(1) = cyaw * a_dot - syaw * b_dot - (a * cyaw - b * syaw) * c_dot / (c - 1);
        omg(2) = (b * a_dot - a * b_dot) / (c - 1) + yawd_bar;
      }
    } else {
      // 处理推力极小的自由落体状态（内在动力学奇点）
      std::cout << "Free fall singularity!!!!!" << std::endl;
      omg = omg_old;
      att = att_est;
    }
    omg_old = omg;
    return;
  }

  bool execMPC(const Odom_Data_t &odom,
              Controller_Output_t &u)
  {
    const clock_t start = clock();

    // 1. set init error state of OMMPC
    auto rot_q = odom.q;
    rot_q.normalize();
    Eigen::VectorXd x_des_start(nstate), u_des_start(nu);
    mpc_wrapper_.getDesiredStart(x_des_start, u_des_start);
    const Eigen::Quaterniond est_q = rot_q;
    const Eigen::Quaterniond des_q = Eigen::Quaterniond(x_des_start(3), x_des_start(4), x_des_start(5), x_des_start(6));
    // 它不直接对绝对状态量做优化，而是对偏差量做优化
    Eigen::Vector3d err_q = SO3::log(est_q.toRotationMatrix().transpose() * des_q.toRotationMatrix());
    Eigen::Vector3d err_p = x_des_start.head(3) - odom.p;
    Eigen::Vector3d err_v = x_des_start.tail(3) - odom.v;
    Eigen::VectorXd delta_x_init(nx);
    delta_x_init << err_p(0), err_p(1), err_p(2), err_v(0), err_v(1), err_v(2), err_q(0), err_q(1), err_q(2);
    mpc_wrapper_.setInitValue(delta_x_init);
    // std::cout << x_des_start.transpose() << std::endl;
    // std::cout << delta_x_init.transpose() << std::endl;

    // 2. solve MPC optimization problem
    Solution solution;
    bool mpc_solved = mpc_wrapper_.solve(solution);

    // 3. get result from OMMPC
    for (int k = 0; k < 1; k++) {
        // std::cout << "k=" << k << ": error control:" << solution.delta_u[k].transpose() << std::endl;
        // std::cout << "k=" << k << ": error_state: " << solution.delta_x[k].transpose() << std::endl;
    }
    if (!mpc_solved){
    ROS_WARN_THROTTLE(1.0, "MPC Infeasible! Falling back to reference feedforward.");
    // 方案 A：直接输出微分平坦前馈指令 (u_des_start)，退化为纯前馈控制
    u.bodyrates = u_des_start.tail(3);
    u.thrust = u_des_start(0) / thr2acc;
    
    // 方案 B：切换至几何控制器 (PD on SE(3)) 运行 1 帧
    return true; 
    }
      // return false;这里将原本的直接false改成了控制退化，退化为纯前馈

    // 4. take first predicted state as input
    // 4.1 bodyrates
    // $$\text{实际控制指令 } u = \text{微分平坦前馈 } u_{\text{des\_start}} - \text{MPC 误差反馈 } \delta u_0$$
    u.bodyrates(0) = u_des_start(1) - solution.delta_u[0](1);
    u.bodyrates(1) = u_des_start(2) - solution.delta_u[0](2);
    u.bodyrates(2) = u_des_start(3) - solution.delta_u[0](3);
    // 4.2 thrustacc -> normalized thrust signal
    double thrustacc = u_des_start(0) - solution.delta_u[0](0);
    // std::cout << thrustacc << std::endl;
    double normalized_thrust;
    normalized_thrust = thrustacc / thr2acc;
    u.thrust = normalized_thrust;
    // std::cout << thrustacc << u.bodyrates.transpose() << std::endl;
    // std::cout << std::endl;

    // Used for thrust-accel mapping estimation
    // 后端的在线质量/推力系数自适应估计器（Thrust-Accel Mapping Estimation），用来在飞行过程中动态更新 thr2acc 参数
    timed_thrust.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
    while (timed_thrust.size() > 100)
      timed_thrust.pop();

    const clock_t end = clock();
    timing_feedback_ = 0.9 * timing_feedback_ + 0.1 * double(end - start) / CLOCKS_PER_SEC;

    // ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.3f ms", timing_feedback_ * 1000);
    return true;
  }
  // 沿参考轨迹建立动力学约束在预测时域的第 $i$ 个时间步，对四旋翼在 $SO(3)$ 流形上的非线性动力学进行局部线性化，
  // 离散化生成该步的状态转移矩阵 $F_x[i]$（即 $A_k$）、控制输入矩阵 $F_u[i]$（即 $B_k$），并将物理执行器的上下限约束映射为控制偏差量的动态约束界限 $u_{ub}[i]$ 和 $u_{lb}[i]$
  void setStateMatricesandBounds(
                const int i,
                const Eigen::Quaterniond &q,
                const Eigen::Vector3d &omg,
                const double t_step,
                const double thracc)
  {
    // 构建离散状态转移矩阵 $F_x[i]$ 
    Fx[i] = Eigen::SparseMatrix<double>(nx, nx);
    Fu[i] = Eigen::SparseMatrix<double>(nx, nu);

    // Fx  代码通过 tripletList（稀疏矩阵三元组）填充了 $F_x[i]$ 的四个关键子块
    std::vector<Eigen::Triplet<double>> tripletList;
    tripletList.reserve(27);

    // (0,0)位置预测
    for (int k = 0; k < 3; ++k) {
        tripletList.push_back(Eigen::Triplet<double>(k, k, 1.0));
    }

    // (3,3)  速度自适应
    for (int k = 0; k < 3; ++k) {
        tripletList.push_back(Eigen::Triplet<double>(3+k, 3+k, 1.0));
    }

    // (0,3): t_step * I 位置预测
    for (int k = 0; k < 3; ++k) {
        tripletList.push_back(Eigen::Triplet<double>(k, 3+k, t_step));
    }

    // (6,6)$SO(3)$ 流形上的姿态误差衰减/演化
    Eigen::Matrix3d exp_mat = SO3::exp(-omg * t_step);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            tripletList.push_back(Eigen::Triplet<double>(6+row, 6+col, exp_mat(row, col)));
        }
    }

    // (3,6)姿态偏差传导至加速度误差
    Eigen::Matrix3d mat_3_6 = t_step * q.toRotationMatrix() * SO3::hat(Eigen::Vector3d(0, 0, -thracc));
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            tripletList.push_back(Eigen::Triplet<double>(3+row, 6+col, mat_3_6(row, col)));
        }
    }

    Fx[i].setFromTriplets(tripletList.begin(), tripletList.end());
    Fx[i].makeCompressed();

    // Fu
    tripletList.clear();
    tripletList.reserve(12);

    // (3,0) 推力加速度偏差映射到世界坐标系速度：
    Eigen::Vector3d vec_3_0 = t_step * q.toRotationMatrix() * Eigen::Vector3d(0, 0, 1);
    for (int k = 0; k < 3; ++k) {
        tripletList.push_back(Eigen::Triplet<double>(3+k, 0, vec_3_0(k)));
    }

    // (6,1) 角速度偏差映射到姿态误差：利用 $SO(3)$ 的左雅可比矩阵（Left Jacobian），将机体角速度偏差 $\delta \boldsymbol{\omega}$ 精确投影到李代数姿态切空间 $\delta \boldsymbol{\theta}$ 上
    Eigen::Matrix3d mat_6_1 = SO3::leftJacobian(omg * t_step).transpose() * t_step;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            tripletList.push_back(Eigen::Triplet<double>(6+row, 1+col, mat_6_1(row, col)));
        }
    }

    Fu[i].setFromTriplets(tripletList.begin(), tripletList.end());
    Fu[i].makeCompressed();

    u_ub[i] << thracc - param_.min_thrust, 
                param_.max_bodyrate_xy + omg(0),
                param_.max_bodyrate_xy + omg(1),
                param_.max_bodyrate_z + omg(2); // lower
    u_lb[i] << -(param_.max_thrust - thracc), 
                omg(0) - param_.max_bodyrate_xy,
                omg(1) - param_.max_bodyrate_xy,
                omg(2) - param_.max_bodyrate_z; // upper 
    // std::cout << Fx[i] << std::endl;
    // std::cout << Fu[i] << std::endl;
    // std::cout << u_ub[i] << std::endl;
    // std::cout << u_lb[i] << std::endl;
    // std::cout << thracc << "  " << omg.transpose() << std::endl;
  }

  // Case 1: HOVER reference 为 MPC 控制器构建并设置“悬停模式（Hover Mode）”下的参考轨迹、预测时域动力学矩阵以及 OSQP 求解器的约束条件
  // Eigen::Vector4d(px, py, pz, yaw)
  void setHoverReference(const Eigen::Vector4d &quad_pose)
  {
    reference_acceleration_.setZero();
    double yaw = quad_pose(3);
    // std::cout << " yaw = " << yaw
    //           << " pos = [" << quad_pos(0) << "," << quad_pos(1) << "," << quad_pos(2) << std::endl;
    const Eigen::Vector3d desired_acc =Eigen::Vector3d::Zero();
    const Eigen::Vector3d desired_jerk =Eigen::Vector3d::Zero();
    Eigen::Vector3d des_thrust_in_world;
    Eigen::Vector3d des_thrust_jerk;
    buildCompensatedFlatnessInput(desired_acc,desired_jerk,des_thrust_in_world,des_thrust_jerk);
    double thracc = des_thrust_in_world.norm();
    // 防止 Hopf 反解中的归一化除零
    thracc = std::max(thracc, 0.1);
    Eigen::Quaterniond identity_q(1.0, 0.0, 0.0, 0.0);
    Eigen::Quaterniond q;
    Eigen::Vector3d omg;
    // des_acc, des_jerk, des_yaw, des_yawdot, des_q (when fail to calculate proper q), out_q, out_omg  微分平坦反解悬停姿态与角速度
    computeFlatInputwithHopfFibration(des_thrust_in_world,des_thrust_jerk, yaw, 0, identity_q, q, omg);
    q.normalize();
    double t_step = param_.step_T;
    for (int i = 0; i < nstep; ++i)
    {
      setStateMatricesandBounds(i, q, omg, t_step, thracc); 
      // 展开预测时域动力学与输入边界
    }
    // 组装 OSQP 求解器约束
    mpc_wrapper_.buildConstraintMatrix(Fx, Fu);
    mpc_wrapper_.buildConstraintVectors(u_lb, u_ub);
    // 置名义起始参考状态与前馈输入
    Eigen::VectorXd x_des_start(nstate);
    Eigen::VectorXd u_des_start(nu);
    x_des_start << quad_pose(0), quad_pose(1), quad_pose(2), 
                  q.w(), q.x(), q.y(), q.z(), 
                  0.0, 0.0, 0.0;
    u_des_start << gravity_, omg(0), omg(1), omg(2);
    mpc_wrapper_.setDesiredStart(x_des_start, u_des_start);
  }

  // Case 2: TXT reference 为 MPC 控制器构建并设置“动态轨迹跟踪模式（TXT/离散参考点模式）”下的预测时域参考状态、动力学线性化矩阵以及 OSQP 求解器的约束条件
  void setTextReference(const std::vector<Eigen::Vector3d> &quad_positions, 
                                  const std::vector<Eigen::Vector3d> &quad_velocities,
                                  const Odom_Data_t &odom, 
                                  const double start_yaw,
                                  const std::vector<double> &yaws,
                                  const std::vector<Eigen::Vector3d> *accelerations = nullptr,
                                  const std::vector<Eigen::Vector3d> *jerks = nullptr)
  {
    if ((accelerations == nullptr) != (jerks == nullptr) ||
        (accelerations && (accelerations->size() != nstep + 1 || jerks->size() != nstep + 1)))
      throw std::invalid_argument("invalid analytic MPC reference derivatives");
    if (quad_positions.size() != nstep + 1 || yaws.size() != nstep + 1 || quad_velocities.size() != nstep + 1)
      ROS_ERROR("Read reference Error!!");
    static Eigen::Vector3d last_quad_velocity = Eigen::Vector3d::Zero();
    static Eigen::Vector3d last_des_acc = Eigen::Vector3d::Zero(), last_acc;
    static Eigen::Vector3d last_des_jerk = Eigen::Vector3d::Zero();
    const double t_step = param_.step_T;
    Eigen::Quaterniond last_q = odom.q;
    last_q.normalize();

    for (int i = 0; i < nstep; ++i)
    {
      // Eigen::Vector3d quad_position = quad_positions.at(i);离散点的数值微分（提取加速度与 Jerk）
      Eigen::Vector3d quad_velocity = quad_velocities.at(i);
      Eigen::Vector3d quad_acc, quad_jerk;
      if (accelerations)
      {
        // BRAKE/LAND supply exact derivatives; do not reuse TXT finite-difference
        // history from a previous state for their first prediction point.
        quad_acc = accelerations->at(i);
        quad_jerk = jerks->at(i);
        last_acc = quad_acc;
        if (i == 1)
        {
          last_des_acc = quad_acc;
          last_des_jerk = quad_jerk;
        }
      }
      else if (i == 0)
      {
        quad_acc = last_des_acc;
        last_acc = quad_acc;
        quad_jerk = last_des_jerk;
      }
      else
      {
        quad_acc = (quad_velocities[i] - quad_velocities[i-1]) / t_step;
        quad_jerk = (quad_acc - last_acc) / t_step;
        last_acc = quad_acc;
        if (i == 1)
        {
          last_des_acc = quad_acc;
          last_des_jerk = quad_jerk;
        }
      }
      double yaw, yaw_dot;
      double thracc;
      // with acc  计算合力加速度与推力投影
      Eigen::Vector3d des_thrust_in_world;
      Eigen::Vector3d des_thrust_jerk;
      buildCompensatedFlatnessInput(quad_acc,quad_jerk,des_thrust_in_world,des_thrust_jerk);
      // 加入水平扰动后必须使用向量模长，
      // 不能继续对旧 body_z 做投影。
      thracc =des_thrust_in_world.norm();
      thracc =std::max(thracc, 0.1);
      Eigen::Quaterniond q;
      Eigen::Vector3d omg;
      // 动态偏航角（Yaw）生成
      if (param_.use_fix_yaw || accelerations)
      {
          yaw = start_yaw;
          yaw_dot = 0.0;
      }
      else  // directly compute from tangent line of traj
      {
        if (i == 0) // reset last_yaw_
        {
          last_yaw_ = start_yaw;
        }
        calculate_yaw(quad_velocity, t_step, yaw, yaw_dot);
        if (i == 0)
        {
          last_yaw_dot_ = yaw_dot;
        }
        // std::cout << yaw << " " << yaw_dot << std::endl;
      }
      // else  // use yaw from txt
      // {
      //   yaw = yaws.at(i);
      //   yaw_dot = 0.0;
      // }
      // des_acc, des_jerk, des_yaw, des_yawdot, des_q (assign to q when fail to calculate proper q), out_q, out_omg
      // if there's significant discontinuous in the txt traj (especially at the end of it), don't use jerk!
      // 微分平坦反解姿态与角速度
      computeFlatInputwithHopfFibration(des_thrust_in_world, des_thrust_jerk, yaw, yaw_dot, last_q, q, omg);
      // computeFlatInputwithHopfFibration(des_acc_in_world, quad_jerk, yaw, yaw_dot, last_q, q, omg);

      q.normalize();
      last_q = q;
      // 提取当前步前馈与构建预测时域动力学
      if (i == 0)
      {
        reference_acceleration_ = quad_acc;
        Eigen::VectorXd x_des_start(nstate);
        Eigen::VectorXd u_des_start(nu);
        x_des_start << quad_positions[i](0), quad_positions[i](1), quad_positions[i](2), 
                      q.w(), q.x(), q.y(), q.z(), 
                      quad_velocities[i](0), quad_velocities[i](1), quad_velocities[i](2);
        u_des_start << thracc, omg(0), omg(1), omg(2);
        mpc_wrapper_.setDesiredStart(x_des_start, u_des_start);
        // std::cout << x_des_start.transpose() << std::endl;
      }
      
      // body_z = q.toRotationMatrix() * Eigen::Vector3d(0, 0, 1);
      
      setStateMatricesandBounds(i, q, omg, t_step, thracc);
    }
    // 组装全局 QP 矩阵
    mpc_wrapper_.buildConstraintMatrix(Fx, Fu);
    mpc_wrapper_.buildConstraintVectors(u_lb, u_ub);
    last_quad_velocity = quad_velocities.at(0);
  }

  // Case 3: TRAJ reference为 MPC 控制器基于连续多项式轨迹（如 MINCO / Minimum-Jerk 轨迹，多用于 ego_planner 等前端规划器）
  // 构建并设置预测时域内的动态参考状态、前馈控制量以及 OSQP 约束矩阵
  // TODO: set proper yaw
  void setTrajectoryReference(
          const Trajectory &traj, 
          const double tstart, 
          const double start_yaw, 
          const Trajectory &yaw_traj, 
          const Odom_Data_t &odom)
  {
    // std::cout << "start_yaw " << start_yaw << std::endl;
    const double t_step = param_.step_T;
    double t_all = traj.getTotalDuration() - 1.0e-3;
    double t = tstart;
    double yaw, yaw_dot;
    Eigen::Vector3d pos_quad, vel_quad, acc_quad, jerk_quad; 
    Eigen::Quaterniond quat, last_quat;
    last_quat = odom.q;
    // Eigen::Vector3d body_z = last_quat.toRotationMatrix() * Eigen::Vector3d(0, 0, 1);
    last_quat.normalize();
    Eigen::Vector3d omg;

    for (int i = 0; i < nstep; i++)
    {
        Eigen::MatrixXd pvajs;
        if (t > t_all)
        { // if t is larger than the total time, use the last point  轨迹时序采样与尾部越界保护（Tail Holding）
            // TODO : Consider 2 trajectories
            t = t_all;
            pvajs = traj.getPVAJSC(t);
            pos_quad = pvajs.col(0);
            vel_quad = Eigen::Vector3d::Zero();
            acc_quad = Eigen::Vector3d::Zero();
            jerk_quad = Eigen::Vector3d::Zero();
        }
        else
        {
            pvajs = traj.getPVAJSC(t);
            pos_quad = pvajs.col(0);
            vel_quad = pvajs.col(1);
            acc_quad = pvajs.col(2);
            jerk_quad = pvajs.col(3);
        }
        // 动态航向角（Yaw）切线跟随
      if (param_.use_fix_yaw)
      {
            yaw = start_yaw;
            yaw_dot = 0.0;
      }
        else // directly compute from tangent line of traj
        {
          if (i == 0) // reset last_yaw_
          {
            last_yaw_ = start_yaw;
          }
          calculate_yaw(vel_quad, t_step, yaw, yaw_dot);
          if (i == 0)
          {
            last_yaw_dot_ = yaw_dot;
          }
            // std::cout << "yaw: " << yaw << std::endl;
        }
        // todo: support yaw traj
        // 利用解析 Jerk 进行高精度微分平坦反解
        Eigen::Vector3d des_thrust_in_world;
        Eigen::Vector3d des_thrust_jerk;

        buildCompensatedFlatnessInput(acc_quad,jerk_quad,des_thrust_in_world,des_thrust_jerk);
        /*
         * 扰动补偿后的质量归一化总推力。
         *
         * 不能继续使用旧 body_z 点乘，因为水平扰动会改变
         * 期望推力方向。
         */
        double thracc =des_thrust_in_world.norm();
        thracc =std::max(thracc, 0.1);
        // double thracc = des_acc_in_world.norm();
        computeFlatInputwithHopfFibration(des_thrust_in_world, des_thrust_jerk, yaw, yaw_dot, last_quat, quat, omg);
        quat.normalize();
        last_quat = quat;
        // 提取当前时刻（$i=0$）名义状态与前馈
        if (i == 0)
        {
            reference_acceleration_ = acc_quad;
            Eigen::VectorXd x_des_start(nstate);
          Eigen::VectorXd u_des_start(nu);
          x_des_start << pos_quad(0), pos_quad(1), pos_quad(2), 
                        last_quat.w(), last_quat.x(), last_quat.y(), last_quat.z(), 
                        vel_quad(0), vel_quad(1), vel_quad(2);
          u_des_start << thracc, omg(0), omg(1), omg(2);
          mpc_wrapper_.setDesiredStart(x_des_start, u_des_start);
        }
        // 步进预测时域与时变动力学构建
        // body_z = last_quat.toRotationMatrix() * Eigen::Vector3d(0, 0, 1);

        setStateMatricesandBounds(i, last_quat, omg, t_step, thracc);
        t += t_step;
    }
    mpc_wrapper_.buildConstraintMatrix(Fx, Fu);
    mpc_wrapper_.buildConstraintVectors(u_lb, u_ub);
  }
//  在线实时估计无人机的“推力-加速度映射系数” 通过带遗忘因子的递推最小二乘法（RLS），实时拟合 $a_z = \text{thr2acc} \cdot T_{cmd}$ 的标量线性模型，使 MPC 能够动态自适应电池掉电和负载变化
  void estimateThrustModel(const Eigen::Vector3d &est_a)
  {
    ros::Time t_now = ros::Time::now();
    while (timed_thrust.size() >= 1)
    {
      // Choose data before 35~45ms ago
      std::pair<ros::Time, double> t_t = timed_thrust.front();
      double time_passed = (t_now - t_t.first).toSec();
      if (time_passed > 0.045) // 45ms
      {
        timed_thrust.pop();
        continue;
      }
      if (time_passed < 0.035) // 35ms
      {
        return;
      }

      /***********************************************************/
      /* Recursive least squares algorithm with vanishing memory */
      /***********************************************************/
      double thr = t_t.second;
      timed_thrust.pop();
      
      /***********************************/
      // /* Model: est_a(2) = thr2acc * thr */带遗忘因子的递推最小二乘算法
      /***********************************/
      double gamma = 1 / (rho2 + thr * P * thr);
      double K = gamma * P * thr;
      thr2acc = thr2acc + K * (est_a(2) - thr * thr2acc);
      P = (1 - K * thr) * P / rho2;
      //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc, gamma, K, P);
      //fflush(stdout);
      // 悬停推力百分比的物理合法性保护
      const double hover_percentage = gravity_ / thr2acc;
      if ( hover_percentage > 0.8 || hover_percentage < 0.1 )
      {
        thr2acc = hover_percentage > 0.8 ? gravity_ / 0.8 : thr2acc;
        thr2acc = hover_percentage < 0.1 ? gravity_ / 0.1 : thr2acc;
      }
    }
  }
};

#endif
