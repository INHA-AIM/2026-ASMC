#include <asmc_lidar/bounded_pose.hpp>
#include <algorithm>
#include <stdexcept>

namespace asmc::lidar {
BoundedPoseHistory::BoundedPoseHistory(Ns age, unsigned recovery)
    : max_age_(age), recovery_samples_(recovery) {
  if (age <= 0 || age > 300000000 || !recovery)
    throw std::invalid_argument("bounded GPS age must be in (0,300ms], recovery nonzero");
}
void BoundedPoseHistory::clear() { samples_.clear(); }
std::string BoundedPoseHistory::append(const PoseSample& sample) {
  if (sample.stamp_ns <= 0 || sample.arrival_ns < sample.stamp_ns || sample.clock.empty())
    return "pose_time_invalid";
  if (!samples_.empty() && sample.arrival_ns < samples_.back().arrival_ns)
    return "arrival_regression";
  samples_.push_back(sample);
  // Keep barriers and recovery observations within the current five-second window.
  const Ns cutoff = sample.arrival_ns - 5000000000LL;
  samples_.erase(std::remove_if(samples_.begin(), samples_.end(),
      [cutoff](const PoseSample& s){return s.arrival_ns < cutoff;}), samples_.end());
  return "";
}
PoseSelection BoundedPoseHistory::at(const Observation& time, const Rot& rotation,
                                    const Vec& velocity) const {
  PoseSelection out;
  out.query_stamp_ns = time.stamp_ns; out.clock = time.clock;
  if (const auto e = validate_time(time); !e.empty()) {out.reason=e; return out;}
  if (!rotation_valid(rotation) || !velocity.allFinite()) {
    out.reason="local_motion_invalid"; return out;
  }
  const PoseSample* latest = nullptr;
  unsigned consecutive = 0;
  Ns last_source = 0;
  for (const auto& s : samples_) {
    if (s.clock != time.clock || s.arrival_ns > time.processing_ns ||
        s.stamp_ns > time.stamp_ns || s.stamp_ns <= last_source) continue;
    if (last_source && s.stamp_ns - last_source > max_age_) consecutive=0;
    last_source=s.stamp_ns; latest=&s;
    const bool good=s.valid && pose_valid(s.map_from_reference) && s.gps_in_base.allFinite();
    consecutive=good ? consecutive+1 : 0;
  }
  if (!latest) {out.reason="gps_missing"; return out;}
  out.sample_stamps={latest->stamp_ns}; out.sample_arrivals={latest->arrival_ns};
  if (!latest->valid || !pose_valid(latest->map_from_reference) || !latest->gps_in_base.allFinite()) {
    out.reason="gps_invalid"; return out;
  }
  if (time.stamp_ns-latest->stamp_ns > max_age_) {out.reason="gps_stale"; return out;}
  if (consecutive < recovery_samples_) {out.reason="gps_recovering"; return out;}
  const double dt=double(time.stamp_ns-latest->stamp_ns)*1e-9;
  out.map_from_base.linear()=rotation;
  out.map_from_base.translation()=latest->map_from_reference.translation()+rotation*velocity*dt;
  // The antenna position belongs to the GPS observation, so remove the
  // lever using that observation's attitude, not a later scan attitude.
  if (latest->reference_is_gps)
    out.map_from_base.translation()-=latest->map_from_reference.rotation()*latest->gps_in_base;
  out.valid=true; out.reason=dt==0. ? "gps_observed" : "gps_bounded_prediction";
  return out;
}
}  // namespace asmc::lidar
