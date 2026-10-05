#pragma once
// L1-B1 research core. No ROS, global state, sensor subscriptions or tracker.
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace asmc::lidar_eval {
using Ns = std::int64_t;
using Vec = Eigen::Vector3d;
using Rot = Eigen::Matrix3d;
using Pose = Eigen::Isometry3d;
Rot rpy(double roll, double pitch, double yaw);
bool rotation_valid(const Rot& r);
bool pose_valid(const Pose& p);

struct Observation {
  Ns stamp_ns = 0, receipt_ns = 0, processing_ns = 0;
  std::string clock, stamp_provenance;
};
// Receipt/processing timestamps below use the explicitly declared replay clock.
// CPU durations are measured separately with steady_clock, never subtracted here.
std::string validate_time(const Observation& t);
class StampGuard {
 public:
  std::string accept(const Observation& t);
 private:
  Ns last_ = 0;
  std::string clock_;
};

struct PoseSample {
  Ns stamp_ns = 0, arrival_ns = 0;
  std::string clock;
  bool valid = false;
  Pose map_from_reference = Pose::Identity();
  // map_from_reference is either rear axle or GPS antenna, NOT object bbox center.
  bool reference_is_gps = false;
  Vec gps_in_base = Vec::Zero();
};
struct PoseSelection {
  bool valid = false;
  Ns query_stamp_ns = 0;
  std::string clock;
  std::string reason = "pose_missing";
  Pose map_from_base = Pose::Identity();
  std::vector<Ns> sample_stamps, sample_arrivals;
};
class PoseHistory {
 public:
  explicit PoseHistory(Ns max_gap_ns = 100000000);
  std::string append(const PoseSample& sample);
  PoseSelection at(const Observation& observation) const;
  void trim_before(Ns stamp_ns);
  void clear();
 private:
  Ns max_gap_, last_arrival_ = -1;
  std::vector<PoseSample> samples_;
};

enum class Frame { Sensor, LeveledSensor, Base };
enum class VelocityMeaning { Unknown, AbsoluteInputVector, BasePositionDerivative };
struct Context {
  Observation time;
  Frame frame = Frame::Sensor;
  Pose base_from_sensor = Pose::Identity();
  std::string calibration_source;
  // Exact rotation used by preprocessing, not one inferred from nearby IMU.
  Rot applied_leveling = Rot::Identity();
  std::string leveling_source;
  PoseSelection map_pose;
  bool twist_valid = false;
  Ns twist_stamp_ns = 0;
  Ns twist_arrival_ns = 0;
  std::string twist_clock;
  Vec ego_linear_base = Vec::Zero(), ego_angular_base = Vec::Zero();
};
struct Object {
  std::int64_t id = -1;
  std::string classification;
  Vec center = Vec::Zero(), long_axis = Vec::UnitX();
  std::array<Vec, 8> corners{};
  VelocityMeaning velocity_meaning = VelocityMeaning::Unknown;
  Vec velocity = Vec::Zero();
};
struct Geometry {
  Vec center, long_axis;
  std::array<Vec, 8> corners;
  std::optional<double> yaw;
};
struct Result {
  bool valid = false;
  std::string reason, map_reason, velocity_reason;
  Observation time;
  std::int64_t id = -1;
  std::string classification;
  Geometry base;
  std::optional<Geometry> map;
  // Always absolute translational velocity, merely expressed in these axes.
  // Unknown is absent, NEVER zero. Input derivative is never silently relabelled.
  std::optional<Vec> absolute_velocity_base, absolute_velocity_map;
};
std::optional<Pose> base_from_input(const Context& c);
Result transform(const Object& object, const Context& context);
struct Point { Vec xyz; float intensity; };
std::vector<Point> transform_points(const std::vector<Point>& points, const Pose& transform);
}  // namespace asmc::lidar_eval
