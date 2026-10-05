#include <asmc_lidar/frame_time.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace asmc::lidar {
Rot rpy(double r, double p, double y) {
  return (Eigen::AngleAxisd(y, Vec::UnitZ()) * Eigen::AngleAxisd(p, Vec::UnitY()) *
          Eigen::AngleAxisd(r, Vec::UnitX())).toRotationMatrix();
}
bool rotation_valid(const Rot& r) {
  return r.allFinite() && (r.transpose()*r - Rot::Identity()).norm() < 1e-9 &&
         std::abs(r.determinant() - 1.) < 1e-9;
}
bool pose_valid(const Pose& p) {
  // Check stored coefficients, not rotation(): Eigen may extract an orthogonal
  // factor from a malformed/scaled transform and hide the invalid input.
  return p.matrix().allFinite() && rotation_valid(p.linear()) &&
         (p.matrix().row(3) - Eigen::RowVector4d(0,0,0,1)).norm() < 1e-12;
}
std::string validate_time(const Observation& t) {
  if (t.stamp_ns <= 0) return "zero_or_negative_stamp";
  if (t.clock.empty() || t.clock == "unknown") return "clock_unknown";
  if (t.stamp_provenance.empty()) return "stamp_provenance_missing";
  if (t.receipt_ns < t.stamp_ns) return "receipt_before_stamp";
  if (t.processing_ns < t.receipt_ns) return "processing_before_receipt";
  return "";
}
std::string StampGuard::accept(const Observation& t) {
  auto error = validate_time(t);
  if (!error.empty()) return error;
  if (!clock_.empty() && clock_ != t.clock) return "clock_changed";
  if (last_ && t.stamp_ns == last_) return "duplicate_stamp";
  if (last_ && t.stamp_ns < last_) return "regressed_stamp";
  last_ = t.stamp_ns; clock_ = t.clock;
  return "";
}
PoseHistory::PoseHistory(Ns gap): max_gap_(gap) {
  if (gap <= 0) throw std::invalid_argument("max_gap_ns must be positive");
}
std::string PoseHistory::append(const PoseSample& s) {
  if (s.arrival_ns < last_arrival_) return "arrival_regression";
  if (s.stamp_ns <= 0 || s.arrival_ns < s.stamp_ns) return "pose_time_invalid";
  // Invalid/NaN samples are retained as barriers; never interpolate across them.
  last_arrival_ = s.arrival_ns; samples_.push_back(s);
  return "";
}
void PoseHistory::trim_before(Ns stamp_ns) {
  samples_.erase(std::remove_if(samples_.begin(),samples_.end(),
      [stamp_ns](const PoseSample& s){return s.stamp_ns<stamp_ns;}),samples_.end());
}
void PoseHistory::clear() { samples_.clear(); last_arrival_=-1; }
PoseSelection PoseHistory::at(const Observation& t) const {
  PoseSelection out;
  out.query_stamp_ns=t.stamp_ns; out.clock=t.clock;
  if (const auto e = validate_time(t); !e.empty()) { out.reason = e; return out; }
  std::vector<const PoseSample*> available;
  for (const auto& s : samples_)
    if (s.arrival_ns <= t.processing_ns && s.clock == t.clock) available.push_back(&s);
  if (available.empty()) { out.reason = "pose_missing_or_clock_mismatch"; return out; }
  // Select by source stamp only AFTER the causal arrival cut. Sorting all input
  // by source time before replay would incorrectly admit late/future messages.
  std::sort(available.begin(), available.end(), [](auto a, auto b){return a->stamp_ns < b->stamp_ns;});
  const PoseSample *lo = nullptr, *hi = nullptr;
  for (auto s : available) {
    if (s->stamp_ns <= t.stamp_ns) lo = s;
    if (!hi && s->stamp_ns >= t.stamp_ns) hi = s;
  }
  if (!lo || !hi) { out.reason = "pose_unbracketed_no_extrapolation"; return out; }
  for (std::size_t i = 1; i < available.size(); ++i)
    if (available[i]->stamp_ns == available[i-1]->stamp_ns &&
        available[i]->stamp_ns >= lo->stamp_ns && available[i]->stamp_ns <= hi->stamp_ns) {
      out.reason = "duplicate_pose_stamp"; return out;
    }
  auto good = [](const PoseSample& s) {
    return s.valid && pose_valid(s.map_from_reference) && s.gps_in_base.allFinite();
  };
  if (!good(*lo) || !good(*hi)) { out.reason = "pose_invalid"; return out; }
  if (hi->stamp_ns - lo->stamp_ns > max_gap_) { out.reason = "pose_gap_exceeded"; return out; }
  auto base = [](const PoseSample& s) {
    Pose p = s.map_from_reference;
    if (s.reference_is_gps) p.translation() -= p.rotation() * s.gps_in_base;
    return p;
  };
  const Pose a = base(*lo), b = base(*hi);
  const double f = lo == hi ? 0. : double(t.stamp_ns-lo->stamp_ns)/double(hi->stamp_ns-lo->stamp_ns);
  out.map_from_base.linear() = Eigen::Quaterniond(a.rotation()).slerp(f, Eigen::Quaterniond(b.rotation())).toRotationMatrix();
  out.map_from_base.translation() = (1-f)*a.translation() + f*b.translation();
  out.sample_stamps.push_back(lo->stamp_ns); out.sample_arrivals.push_back(lo->arrival_ns);
  if (hi != lo) { out.sample_stamps.push_back(hi->stamp_ns); out.sample_arrivals.push_back(hi->arrival_ns); }
  out.valid = true; out.reason = "valid";
  return out;
}
std::optional<Pose> base_from_input(const Context& c) {
  if (c.frame == Frame::Base) return Pose::Identity();
  if (!pose_valid(c.base_from_sensor) || c.calibration_source.empty()) return {};
  Pose out = c.base_from_sensor;
  if (c.frame == Frame::LeveledSensor) {
    if (c.leveling_source.empty() || !rotation_valid(c.applied_leveling)) return {};
    out.linear() = out.rotation() * c.applied_leveling.transpose();
  }
  return out;
}
Geometry geometry(const Object& o, const Pose& t) {
  Geometry g;
  g.center = t * o.center;
  g.long_axis = t.rotation()*o.long_axis.normalized();
  for (std::size_t i=0; i<8; ++i) g.corners[i] = t*o.corners[i];
  if (g.long_axis.head<2>().norm() > 1e-9) g.yaw = std::atan2(g.long_axis.y(), g.long_axis.x());
  return g;
}
Result transform(const Object& o, const Context& c) {
  Result r; r.time=c.time; r.id=o.id; r.classification=o.classification;
  r.reason=validate_time(c.time);
  if (!r.reason.empty()) return r;
  const auto t=base_from_input(c);
  if (!t) { r.reason="input_transform_invalid_or_unproven"; return r; }
  if (!o.center.allFinite() || !o.long_axis.allFinite() || o.long_axis.norm()<1e-12 ||
      !std::all_of(o.corners.begin(),o.corners.end(),[](const Vec& p){return p.allFinite();})) {
    r.reason="nonfinite_or_invalid_geometry"; return r;
  }
  r.base=geometry(o,*t); r.valid=true; r.reason="valid";
  r.map_reason=c.map_pose.reason;
  if (c.map_pose.valid && c.map_pose.query_stamp_ns==c.time.stamp_ns &&
      c.map_pose.clock==c.time.clock && pose_valid(c.map_pose.map_from_base)) {
    r.map=geometry(o,c.map_pose.map_from_base * *t); r.map_reason="valid";
  } else if(c.map_pose.valid) r.map_reason="pose_invalid_or_time_mismatch";
  r.velocity_reason="legacy_meaning_unknown";
  if (o.velocity_meaning != VelocityMeaning::Unknown && !o.velocity.allFinite())
    r.velocity_reason="velocity_nonfinite";
  else if (o.velocity_meaning == VelocityMeaning::AbsoluteInputVector) {
    r.absolute_velocity_base=t->rotation()*o.velocity;
    r.velocity_reason="absolute_vector_rotated_only";
  } else if (o.velocity_meaning == VelocityMeaning::BasePositionDerivative) {
    // The derivative is explicitly d(position_in_base)/dt, not a vector merely
    // expressed in base axes. A derivative in a leveled/moving sensor frame has
    // different angular/origin terms and is not accepted under this declaration.
    if (c.frame != Frame::Base) r.velocity_reason="derivative_requires_base_input";
    else if (!c.twist_valid || c.twist_stamp_ns!=c.time.stamp_ns || c.twist_clock!=c.time.clock ||
             c.twist_arrival_ns<c.twist_stamp_ns || c.twist_arrival_ns>c.time.processing_ns ||
             !c.ego_linear_base.allFinite() || !c.ego_angular_base.allFinite())
      r.velocity_reason="twist_missing_or_time_mismatch";
    else {
      r.absolute_velocity_base=o.velocity+c.ego_linear_base+c.ego_angular_base.cross(r.base.center);
      r.velocity_reason="relative_derivative_with_ego_twist";
    }
  }
  if (r.absolute_velocity_base && r.map)
    r.absolute_velocity_map=c.map_pose.map_from_base.rotation()* *r.absolute_velocity_base;
  return r;
}
std::vector<Point> transform_points(const std::vector<Point>& points,const Pose& t) {
  if (!pose_valid(t)) throw std::invalid_argument("invalid point transform");
  std::vector<Point> out=points;
  for (auto& p:out) p.xyz=t*p.xyz;
  return out;
}
}  // namespace asmc::lidar
