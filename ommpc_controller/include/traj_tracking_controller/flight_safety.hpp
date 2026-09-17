#ifndef OMMPC_FLIGHT_SAFETY_HPP
#define OMMPC_FLIGHT_SAFETY_HPP
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flight_safety {
// First-order UDE driven by velocity increments and modeled, limited actuator
// acceleration. Absolute velocity integral clipping would introduce a bias.
class DisturbanceObserver {
public:
    void reset(const Eigen::Vector3d& v,double stamp) { previous_v_=v; previous_stamp_=stamp; estimate_.setZero(); }
    Eigen::Vector3d update(const Eigen::Vector3d& v,double stamp,const Eigen::Vector3d& applied,
                          const Eigen::Vector3d& tau,double limit,double max_gap) {
        const double dt=stamp-previous_stamp_;
        if(dt==0) return estimate_;
        if(dt<0 || dt>max_gap) { reset(v,stamp); return estimate_; }
        for(int i=0;i<3;++i) {
            const double gain=-std::expm1(-dt/std::max(1e-6,tau(i)));
            estimate_(i)=(1-gain)*estimate_(i)+gain*((v(i)-previous_v_(i))/dt-applied(i));
            estimate_(i)=std::max(-limit,std::min(limit,estimate_(i)));
        }
        previous_v_=v; previous_stamp_=stamp;
        return estimate_;
    }
private:
    Eigen::Vector3d previous_v_=Eigen::Vector3d::Zero(),estimate_=Eigen::Vector3d::Zero();
    double previous_stamp_=0;
};
struct Reference {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Vector3d a = Eigen::Vector3d::Zero();
};

// For Rz(yaw)*Ry(pitch)*Rx(roll), the third column must equal F/|F|.
inline Eigen::Quaterniond attitude(const Eigen::Vector3d& force, double yaw) {
    if (!force.allFinite() || force.z() <= 0 || !std::isfinite(yaw))
        throw std::invalid_argument("invalid attitude force");
    const Eigen::Vector3d f = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) * force;
    const double roll = std::atan2(-f.y(), std::hypot(f.x(), f.z()));
    const double pitch = std::atan2(f.x(), f.z());
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

// Velocity Bernstein control points [v0, v0+a0*T/4, 0, 0, 0].
// Their derivatives give rigorous norm bounds over the entire curve.
// p/v/a are continuous at entry; v/a/jerk vanish at exit. Initial jerk
// need not equal the incoming trajectory jerk, but remains bounded.
class SmoothStop {
public:
    void start(const Reference& r, double max_a, double max_j) {
        if (!r.p.allFinite() || !r.v.allFinite() || !r.a.allFinite() ||
            !std::isfinite(max_a) || !std::isfinite(max_j) || max_a<=0 || max_j<=0)
            throw std::invalid_argument("invalid braking reference/limits");
        initial_ = r;
        // A pre-existing acceleration cannot be removed instantaneously.
        acceleration_limit_ = std::max(max_a, 1.05*r.a.norm());
        duration_ = .5;
        for (int i=0;i<200;++i) {
            const Eigen::Vector3d ac = -4*r.v/duration_ - r.a;
            const Eigen::Vector3d j0 = -12*r.v/(duration_*duration_) - 6*r.a/duration_;
            const Eigen::Vector3d j1 = 12*r.v/(duration_*duration_) + 3*r.a/duration_;
            acceleration_bound_ = std::max(r.a.norm(), ac.norm());
            jerk_bound_ = std::max(j0.norm(), j1.norm());
            if (acceleration_bound_ <= acceleration_limit_ && jerk_bound_ <= max_j) return;
            duration_ *= 1.12;
        }
        throw std::runtime_error("cannot construct bounded braking curve");
    }
    Reference at(double t) const {
        const double s = std::max(0.0,std::min(1.0,t/duration_));
        const double s2=s*s,s3=s2*s,s4=s3*s,s5=s4*s;
        Reference r;
        r.p = initial_.p + duration_*initial_.v*(s-2*s3+2*s4-.6*s5)
              + duration_*duration_*initial_.a*(.5*s2-s3+.75*s4-.2*s5);
        r.v = initial_.v*(1-6*s2+8*s3-3*s4)
              + duration_*initial_.a*(s-3*s2+3*s3-s4);
        r.a = initial_.v/duration_*(-12*s+24*s2-12*s3)
              + initial_.a*(1-6*s+9*s2-4*s3);
        if (t>=duration_) { r.v.setZero(); r.a.setZero(); }
        return r;
    }
    Eigen::Vector3d jerk(double t) const {
        if (t>=duration_) return Eigen::Vector3d::Zero();
        const double s=std::max(0.0,t/duration_);
        return initial_.v/(duration_*duration_)*(-12+48*s-36*s*s)
            + initial_.a/duration_*(-6+18*s-12*s*s);
    }
    double duration() const { return duration_; }
    double accelerationBound() const { return acceleration_bound_; }
    double jerkBound() const { return jerk_bound_; }
    double accelerationLimit() const { return acceleration_limit_; }
private:
    Reference initial_;
    double duration_=1, acceleration_limit_=0, acceleration_bound_=0, jerk_bound_=0;
};

// Quintic speed ramps: zero acceleration and jerk at all joins.
class SmoothDescent {
public:
    void start(const Eigen::Vector3d& p, double target_z, double speed, double max_a, double max_j) {
        if (!p.allFinite() || !std::isfinite(target_z) || !std::isfinite(speed) ||
            !std::isfinite(max_a) || !std::isfinite(max_j) || speed<=0 || max_a<=0 || max_j<=0)
            throw std::invalid_argument("invalid descent configuration");
        p_=p; target_=std::min(p.z(),target_z);
        const double distance=p.z()-target_;
        auto ramp=[&](double v) { return std::max(.05,std::max(1.875*v/max_a,std::sqrt(6*v/max_j))); };
        peak_=speed;
        if (peak_*ramp(peak_)>distance) {
            double lo=0,hi=speed;
            for(int i=0;i<60;++i) { double mid=(lo+hi)/2; if(mid*ramp(mid)>distance)hi=mid;else lo=mid; }
            peak_=lo;
        }
        ramp_=ramp(peak_);
        cruise_=peak_>1e-12?std::max(0.0,distance/peak_-ramp_):0;
    }
    Reference at(double t) const {
        Reference r; r.p=p_; t=std::max(0.0,t);
        if (t>=duration() || peak_<=1e-12) { r.p.z()=target_; return r; }
        auto h=[](double s) { return s*s*s*(10+s*(-15+6*s)); };
        auto H=[](double s) { return s*s*s*s*(2.5+s*(-3+s)); };
        auto dh=[](double s) { return 30*s*s*(1-s)*(1-s); };
        if (t<ramp_) {
            double s=t/ramp_; r.p.z()-=peak_*ramp_*H(s);
            r.v.z()=-peak_*h(s); r.a.z()=-peak_/ramp_*dh(s);
        } else if(t<ramp_+cruise_) {
            r.p.z()-=peak_*(ramp_/2+t-ramp_); r.v.z()=-peak_;
        } else {
            double s=(t-ramp_-cruise_)/ramp_;
            r.p.z()-=peak_*(ramp_/2+cruise_+ramp_*(s-H(s)));
            r.v.z()=-peak_*(1-h(s)); r.a.z()=peak_/ramp_*dh(s);
        }
        return r;
    }
    Eigen::Vector3d jerk(double t) const {
        Eigen::Vector3d result=Eigen::Vector3d::Zero();
        if(t<0 || t>=duration() || peak_<=1e-12) return result;
        double s=0,sign=0;
        if(t<ramp_) { s=t/ramp_; sign=-1; }
        else if(t>=ramp_+cruise_) { s=(t-ramp_-cruise_)/ramp_; sign=1; }
        else return result;
        result.z()=sign*peak_/(ramp_*ramp_)*60*s*(1-s)*(1-2*s);
        return result;
    }
    double duration() const { return 2*ramp_+cruise_; }
private:
    Eigen::Vector3d p_=Eigen::Vector3d::Zero();
    double target_=0,peak_=0,ramp_=1,cruise_=0;
};
} // namespace flight_safety
#endif
