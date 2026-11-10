#pragma once
#include "shadow_localization/sampled_motion.hpp"
#include <set>

namespace shadow_localization {
// Forward planning uses a nonnegative boundary condition. Preserve the signed
// motion/odometry; only the existing <=0.05m/s standstill rollback allowance is
// normalized here. A real reverse motion still rejects planning.
inline std::optional<double> ForwardPlanningSpeed(double measured) {
  if(!std::isfinite(measured)||measured<-.05)return {};
  return std::max(0.,measured);
}
// Bounded source-time pairing. No pose/velocity integration in this consumer.
// Raw IMU supplies roll/pitch once for body OBB leveling; odometry supplies XY
// and relative yaw. The latter's yaw-only quaternion is not an attitude source.
class InputAlignment {
 public:
  unsigned epoch=0;
  double reorder_tolerance_s=0;
  std::string frame,reason="uninitialized";
  double pose_stamp=0,imu_stamp=0;
  void clear() { poses_.clear();attitudes_.clear();pose_stamp=imu_stamp=0;++epoch; }
  bool pose(const std::string& session,const local_planning::Motion& m) {
    if (session.rfind("local/",0)!=0 || retired_.count(session)) {
      reason="retired_or_invalid_motion_frame";return false;
    }
    if (!finite(m)) {reason="nonfinite_pose";return false;}
    if (frame!=session) {
      if (!frame.empty())retired_.insert(frame);
      clear();frame=session;
    }
    if(local_planning::BoundedSourceReorder(m.t,pose_stamp,reorder_tolerance_s)) {
      reason="pose_packet_reordered";return false;
    }
    if (pose_stamp && m.t<pose_stamp) {clear();reason="pose_clock_regression";return false;}
    if (!poses_.push(m)) {reason="duplicate_pose";return false;}
    pose_stamp=m.t;reason="pose_accepted";return true;
  }
  bool imu(const local_planning::Motion& m) {
    if (!finite(m) || std::abs(m.roll)>.5 || std::abs(m.pitch)>.5) {
      reason="invalid_attitude";return false;
    }
    if(local_planning::BoundedSourceReorder(m.t,imu_stamp,reorder_tolerance_s)) {
      reason="imu_packet_reordered";return false;
    }
    if (imu_stamp && m.t<imu_stamp) {clear();reason="imu_clock_regression";return false;}
    if (!attitudes_.push(m)) {reason="duplicate_imu";return false;}
    imu_stamp=m.t;reason="imu_accepted";return true;
  }
  std::optional<local_planning::Motion> pose_at(double t) const {
    return t<=pose_stamp?poses_.at(t):std::nullopt;
  }
  std::optional<local_planning::Motion> attitude_at(double t) const {
    return t<=imu_stamp?attitudes_.at(t):std::nullopt;
  }
  std::optional<local_planning::Motion> paired(double t) const {
    auto p=pose_at(t),a=attitude_at(t);
    if (!p || !a)return {};
    p->roll=a->roll;p->pitch=a->pitch;return p;
  }
 private:
  SampledMotion poses_,attitudes_;
  std::set<std::string> retired_;
  static bool finite(const local_planning::Motion& m) {
    return m.t>0 && std::isfinite(m.t+m.x+m.y+m.yaw+m.v+m.w+m.roll+m.pitch+m.a);
  }
};
}  // namespace shadow_localization
