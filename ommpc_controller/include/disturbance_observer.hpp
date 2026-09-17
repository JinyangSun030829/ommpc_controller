#ifndef DISTURBANCE_OBSERVER_HPP
#define DISTURBANCE_OBSERVER_HPP

#include <Eigen/Eigen>

/**
 * @brief 三维平移扰动观测器
 *
 * 被观测模型：
 *
 *   v_dot = a_nom + d
 *
 * 输入：
 *   v_meas : 世界坐标系速度
 *   a_nom  : 已知名义加速度 aT * R * e3 - g * e3
 *
 * 输出：
 *   d_hat      : 世界坐标系加速度型扰动估计
 *   d_hat_dot  : 扰动估计导数，可用于角速度前馈补偿
 *
 * 当前内部使用线性扩张状态观测器作为可运行模板。
 * 后续可以只替换 update() 内部的观测器方程为你的 FxTDO，
 * 其余 OMMPC、状态机和微分平坦代码不需要再改。
 */
class DisturbanceObserver3D
{
private:
  Eigen::Vector3d v_hat_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d d_hat_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d d_hat_dot_ = Eigen::Vector3d::Zero();

  double l1_ = 8.0;
  double l2_ = 16.0;

  bool initialized_ = false;

public:
  DisturbanceObserver3D() = default;

  void configure(const double l1, const double l2)
  {
    l1_ = l1;
    l2_ = l2;
  }

  void reset(const Eigen::Vector3d &v_meas)
  {
    v_hat_ = v_meas;
    d_hat_.setZero();
    d_hat_dot_.setZero();
    initialized_ = true;
  }

  void update(const Eigen::Vector3d &v_meas,
              const Eigen::Vector3d &a_nom,
              const double dt)
  {
    if (!initialized_)
    {
      reset(v_meas);
      return;
    }

    if (dt <= 0.0)
    {
      return;
    }

    // 速度观测误差
    const Eigen::Vector3d e_v = v_meas - v_hat_;

    /*
     * 当前可运行的线性 ESO：
     *
     * v_hat_dot = a_nom + d_hat + l1 * e_v
     * d_hat_dot = l2 * e_v
     *
     * 若使用你原来的 FxTDO，只需要替换下面三行。
     */
    const Eigen::Vector3d v_hat_dot =
        a_nom + d_hat_ + l1_ * e_v;

    d_hat_dot_ = l2_ * e_v;

    v_hat_ += dt * v_hat_dot;
    d_hat_ += dt * d_hat_dot_;
  }

  const Eigen::Vector3d &disturbance() const
  {
    return d_hat_;
  }

  const Eigen::Vector3d &disturbanceDerivative() const
  {
    return d_hat_dot_;
  }
  const Eigen::Vector3d &velocityEstimate() const
{
  return v_hat_;
}

Eigen::Vector3d velocityError(
    const Eigen::Vector3d &v_meas) const
{
  return v_meas - v_hat_;
}

bool initialized() const
{
  return initialized_;
}
};

#endif