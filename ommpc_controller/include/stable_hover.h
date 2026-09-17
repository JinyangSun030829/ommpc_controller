#ifndef OMMPC_STABLE_HOVER_H
#define OMMPC_STABLE_HOVER_H

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <string>
#include <sstream>

namespace minisnap {
struct HoverOptions {
    double duration = 1.0;
    double max_speed = 0.10;
    double max_position_span = 0.05;
    double max_state_age = 0.20;
    void validate() const {
        for (double x : {duration, max_speed, max_position_span, max_state_age})
            if (!std::isfinite(x) || x <= 0.0) throw std::invalid_argument("invalid hover gate configuration");
        if (duration < max_state_age) throw std::invalid_argument("hover duration must cover at least one freshness interval");
    }
};

// No acceleration measurement is implied. This is a sustained low-speed /
// small-position-span proxy for the zero-v/a/j hover boundary model.
// Timestamps must progress; repeatedly reading one sample cannot create stability.
class StableHover {
public:
    explicit StableHover(const HoverOptions& options = HoverOptions()) : options_(options) { options_.validate(); }
    void reset() { samples_.clear(); last_reason_ = "window reset"; }
    void update(const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                double stamp, double wall, bool eligible = true) {
        if (!eligible || !p.allFinite() || !v.allFinite() || !std::isfinite(stamp) || !std::isfinite(wall)) {
            reset(); last_reason_="ineligible or invalid state"; return;
        }
        if (v.norm() > options_.max_speed) { reset(); last_reason_="speed exceeded"; return; }
        if (!samples_.empty()) {
            const auto& last = samples_.back();
            if (wall < last.wall || wall - last.wall > options_.max_state_age ||
                stamp < last.stamp || stamp - last.stamp > options_.max_state_age) {
                reset(); last_reason_="sample gap or clock reversal";
            }
            else if (stamp == last.stamp) return;
        }
        samples_.push_back({p, stamp, wall});
        // Keep an anchor covering BOTH clocks; otherwise scheduling jitter can
        // repeatedly discard the only sample covering a full measurement second.
        while (samples_.size() > 2 && wall - samples_[1].wall >= options_.duration &&
               stamp - samples_[1].stamp >= options_.duration)
            samples_.pop_front();
        // Bounding-box diagonal is a conservative upper bound on pairwise span.
        while (samples_.size() > 1 && span() > options_.max_position_span) {
            samples_.pop_front();
            last_reason_="position span exceeded";
        }
    }
    double wallCoverage() const { return samples_.empty()?0:samples_.back().wall-samples_.front().wall; }
    double stampCoverage() const { return samples_.empty()?0:samples_.back().stamp-samples_.front().stamp; }
    double positionSpan() const { return samples_.empty()?0:span(); }
    std::string diagnostic(double wall) const {
        std::ostringstream out;
        out << "stable_hover=" << stable(wall) << ", coverage_wall=" << wallCoverage()
            << "s, coverage_stamp=" << stampCoverage() << "s, required=" << options_.duration
            << "s, span=" << (samples_.empty()?0:span()) << "m/" << options_.max_position_span
            << "m, latest_window_event=" << last_reason_;
        return out.str();
    }
    bool stable(double wall) const {
        return std::isfinite(wall) && samples_.size() >= 3 && wall >= samples_.back().wall &&
            wall - samples_.back().wall <= options_.max_state_age &&
            samples_.back().wall - samples_.front().wall >= options_.duration - 1e-9 &&
            samples_.back().stamp - samples_.front().stamp >= options_.duration - 1e-9;
    }
private:
    struct Sample { Eigen::Vector3d p; double stamp, wall; };
    double span() const {
        Eigen::Vector3d low = samples_.front().p, high = low;
        for (const auto& s : samples_) { low = low.cwiseMin(s.p); high = high.cwiseMax(s.p); }
        return (high - low).norm();
    }
    HoverOptions options_;
    std::deque<Sample> samples_;
    std::string last_reason_="waiting for samples";
};

struct ActivationOptions {
    HoverOptions hover;
    double max_ready_age = 2.0;
    double max_position_error = 0.10;
    double boundary_tolerance = 1e-5;
    void validate() const {
        hover.validate();
        for (double x : {max_ready_age, max_position_error, boundary_tolerance})
            if (!std::isfinite(x) || x <= 0.0) throw std::invalid_argument("invalid activation gate configuration");
    }
};

inline std::string activationRejection(const ActivationOptions& o,
    const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& start_position, const Eigen::Vector3d& start_velocity,
    const Eigen::Vector3d& start_acceleration, const Eigen::Vector3d& start_jerk,
    double state_age, double ready_age, bool stable, bool permitted) {
    if (!std::isfinite(ready_age) || ready_age < 0.0 || ready_age > o.max_ready_age) return "READY trajectory expired; resend waypoints";
    if (!position.allFinite() || !velocity.allFinite() || !start_position.allFinite() ||
        !start_velocity.allFinite() || !start_acceleration.allFinite() || !start_jerk.allFinite()) return "non-finite activation state";
    if (!std::isfinite(state_age) || state_age < -0.1 || state_age > o.hover.max_state_age) return "state is stale";
    if (start_velocity.norm() > o.boundary_tolerance || start_acceleration.norm() > o.boundary_tolerance ||
        start_jerk.norm() > o.boundary_tolerance) return "trajectory is not a zero-v/a/j hover start";
    if ((position - start_position).norm() > o.max_position_error) return "start position mismatch; resend waypoints";
    if (!permitted) return "requires airborne HOVER, OFFBOARD and ARMED";
    if (!stable || velocity.norm() > o.hover.max_speed) return "waiting for stable hover";
    return "";
}
}  // namespace minisnap
#endif
