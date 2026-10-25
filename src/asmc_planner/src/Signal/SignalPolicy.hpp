#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <string>
#include <vector>
#include "Signal/BrakingModel.hpp"

namespace asmc_planner::signal {
enum class Color { UNKNOWN, GREEN, YELLOW, RED };
struct Key {
  std::uint64_t route_generation = 0;
  std::string stop_line, movement;
  bool operator==(const Key& b) const {
    return route_generation==b.route_generation && stop_line==b.stop_line && movement==b.movement;
  }
  bool operator!=(const Key& b) const { return !(*this==b); }
  bool valid() const { return route_generation && !stop_line.empty() && !movement.empty(); }
};
struct Observation {
  std::uint64_t source_epoch = 0;
  std::uint64_t transport_epoch = 1;
  std::uint32_t sequence = 0;  // Diagnostic only, not original-frame identity.
  std::uint64_t original_stamp_ns=0;
  double stamp = 0, received = 0;
  std::string frame;
  std::string label;
  Color color = Color::UNKNOWN;
  bool valid = false, stale = true, clock_verified = false;
  bool association_valid = false;
  Key key;
};
struct Config {
  double freshness = .3, max_gap = .3, history_seconds = 10, interpolation_gap = .1;
  double front_extent = 3.845, rear_extent = .79, half_width = .95;
  double wheelbase = 3, half_track = std::numeric_limits<double>::quiet_NaN();
  double clearance = .5, position_error = .5, crossing_band = .5, buffer = .5;
  // Optional measured wheel-observation bound. Body/target uncertainty above
  // still governs stopping, conflict occupancy and complete exit clearance.
  double entry_observation_error = -1, entry_observation_speed_max = 0;
  double measured_max_delay = 0, standstill_speed = .15, standstill_time = .2;
  double recovery_speed=2.0;
  BrakeLimits brake;
};
struct Pose {
  double stamp = 0, x = 0, y = 0, yaw = 0, speed = 0, acceleration = 0;
  double route_s = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
};
class PoseHistory {
 public:
  explicit PoseHistory(Config c = {}) : config_(c) {}
  void Push(const Pose& p);
  bool At(double stamp, Pose& out) const;
  bool Covers(double begin,double end) const;
  bool MaxQualificationSpeed(double begin,double end,double acceleration_horizon,double& speed) const;
  void Clear() { poses_.clear(); }
 private:
  Config config_;
  std::deque<Pose> poses_;
};
struct Point { double x=0,y=0; };
struct Geometry {
  Key key;
  Point line, approach;  // Unit normal pointing in travel direction.
  bool reviewed = false;
  double route_s = std::numeric_limits<double>::quiet_NaN();
  std::vector<Point> conflict_polygon;
  Point exit, exit_direction;
  bool conflict_reviewed = false;
};
struct Distance { double min=0,max=0; bool valid=false; };
Distance FrontWheelDistance(const Pose&, const Geometry&, const Config&);
bool BodyBeforeLine(const Pose&, const Geometry&, const Config&);
bool BodyOutsideExit(const Pose&, const Geometry&, const Config&);
bool BodyOccupiesConflict(const Pose&, const Geometry&, const Config&);
bool BodyDefinitelyOccupiesConflict(const Pose&, const Geometry&, const Config&);
struct StopTarget { bool valid=false; double offset=0; Pose pose; };
StopTarget SelectLateTarget(const Geometry&,const Config&,
    const std::function<bool(double,Pose&)>& pose_at_s,double nominal_offset=5.5);

// Consumes every delivered observation, independently of the Behavior timer.
// Same-header invalid/stale messages are processed before duplicate filtering.
class GreenGate {
 public:
  explicit GreenGate(Config c = {}) : config_(c) {}
  void Bind(const Key& key);
  void Observe(const Observation&, double ros_now, double steady_now);
  void Tick(double ros_now, double steady_now);
  void Revoke();
  int count() const { return count_; }
  bool authorized() const { return count_>=3; }
  Color color() const { return color_; }
  double last_green() const { return last_green_; }
  double first_stop() const { return first_stop_; }
  std::uint64_t revocations() const { return revocations_; }
 private:
  Config config_;
  Key key_;
  std::uint64_t epoch_=0, revocations_=0;
  double stamp_=0, received_=0, last_green_=0, first_stop_=0, invalid_stamp_=0;
  double invalid_received_=0;
  std::string frame_;
  int count_=0;
  Color color_=Color::UNKNOWN;
};
enum class Cause { SIGNAL, COLLISION, PEDESTRIAN, INPUT_INVALID, EXECUTION_FAILURE, COUNT };
enum class Severity { NONE, STOP, EMERGENCY };
class StopCauses {
 public:
  void Set(Cause c, Severity s) { causes_[static_cast<unsigned>(c)]=s; }
  Severity Get(Cause c) const { return causes_[static_cast<unsigned>(c)]; }
  Severity Final() const;
  bool OthersClear() const;
 private:
  std::array<Severity, static_cast<unsigned>(Cause::COUNT)> causes_{};
};
enum class Motion { APPROACH, BRAKING, STATIONARY_HOLD, RELEASE_PENDING,
  NORMAL_CLEAR, RECOVERY_BRAKE, RECOVERY_CLEAR_PENDING, RECOVERY_CLEAR, EXITED,
  ENTRY_PENDING_CLEAR, GREEN_CROSSING_CLEAR };
enum class EventKind { NORMAL_ENTRY_CONFIRMED, TARGET_OVERRUN,
  STOP_LINE_CROSS_DURING_BRAKING, ENTRY_UNCERTAIN, RESPONSE_FAILURE, PATH_FAILURE };
struct Event {
  Key key;
  EventKind kind;
  double stamp=0, interval_begin=0, interval_end=0;
};
struct Evidence {
  bool position_valid=false, risks_fresh=false, inside_conflict=false;
  bool position_history_continuous=false;
  // Independent valid GPS/IMU positions can preserve the historical recovery
  // fact during missing speed assembly. This is never a motion/entry permit.
  bool localization_continuity_verified=false;
  // Original green capture time, interpolated only within verified pose
  // history. This is historical evidence, never a projected future pose.
  bool green_capture_pose_valid=false;
  Pose green_capture_pose;
  bool conflict_known=false, exit_path_valid=false, path_goal_satisfied=false;
  bool linked=false, exited=false;
  // These are two independent proofs; neither can be inferred from a timestamp.
  bool command_accepted=false, trajectory_belongs_to_command=false;
  bool planning_checked=false, stop_path_valid=false;
  bool target_geometry_valid=true;
  // A counterfactual shadow decision is NOT evidence of an issued brake.
  bool actuation_issued=false;
  // Fresh, matching operational watchdog proof, separate from the offline
  // monitor above. Expiration without a response arrives as execution failure.
  bool response_delay_known=false,response_observed=false;
  double response_delay_remaining=0;
  bool execution_recovery_acknowledged=false;  // Execution owner proof, never a green aspect.
  bool execution_failed=false;
  unsigned execution_failure_mask=0;
  double state_age=0;  // Verified vehicle ROS time -> current decision time.
  std::uint32_t command_id=0;
};
struct Output {
  std::array<Severity,static_cast<std::size_t>(Cause::COUNT)> causes{};
  bool release_barrier=false;
  bool execution_recovery_acknowledged=false;
  Motion motion=Motion::APPROACH;
  Severity severity=Severity::STOP;
  Severity signal_requirement=Severity::STOP;
  std::uint64_t target_revision=0, virtual_command_id=0;
  bool target_reached=false, stationary=false, model_feasible=false;
  bool entry_authorized=false, actual_control_allowed=false;
  bool entry_pending=false;
  double minimum_distance=std::numeric_limits<double>::infinity();
  double entry_observation_error=0;
};

class Policy {
 public:
  explicit Policy(Config config = {});
  void Bind(const Geometry&);
  void Observe(const Observation& o, double ros_now, double steady_now,
               const PoseHistory* history=nullptr);
  // Signed rear-axle offset before the physical line, not remaining distance.
  void SetTarget(double endpoint_offset, bool late);
  Output Step(const Pose&, double ros_now, double steady_now, const Evidence&,
              StopCauses causes);
  const std::vector<Event>& events() const { return events_; }
  const GreenGate& gate() const { return gate_; }
  Motion motion() const { return motion_; }
  bool RecoveryPositionEligible(const Pose&,bool continuous_history) const;
 private:
  void Record(EventKind, double stamp, double begin, double end);
  Config config_;
  Geometry geometry_;
  GreenGate gate_;
  Motion motion_=Motion::APPROACH;
  ResponseMonitor response_;
  std::vector<Event> events_;  // Append-only for this observer instance.
  std::uint64_t revision_=0, virtual_id_=0;
  double target_=std::numeric_limits<double>::quiet_NaN(), previous_stamp_=0;
  double previous_distance_=0, crossed_begin_=0, crossed_end_=0, stopped_since_=-1;
  double entry_error_=0,previous_entry_qualification_speed_=0;
  double last_before_stamp_=0;
  bool late_=false, brake_started_=false, crossed_=false, uncertain_=false;
  bool normal_=false, target_failed_=false, response_failed_=false;
  bool path_failed_=false;
  bool external_path_failed_=false;
  bool previous_authorized_=false;
  bool emergency_latched_=false;
  bool signal_brake_started_=false;
  bool braking_at_crossing_=false, braking_cross_recorded_=false;
  bool entry_pending_=false,crossing_ordered_=false;
  // A green-authorized crossing may lose camera coverage before a post-line
  // image proves normal entry. Keep its SIGNAL release separate from the
  // append-only uncertainty classification and from other stop causes.
  bool green_crossing_committed_=false;
  bool entry_window_started_=false;
  bool crossing_after_confirmed_=false;
  bool recovery_committed_=false;
  bool green_reentry_committed_=false;
  std::uint64_t crossing_revocations_=0;
  std::uint64_t previous_revocations_=0;
};
const char* MotionName(Motion);
}  // namespace asmc_planner::signal
