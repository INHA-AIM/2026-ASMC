#include "local/journal.hpp"
#include "local/motion.hpp"
#include "local/obstacle_memory.hpp"
#include "local/planner.hpp"
#include "local/road_memory.hpp"
#include "local/reference_prior.hpp"
#include "local/semantic_reference.hpp"
#include <asmc_msgs/CameraGroundProjection.h>
#include <asmc_msgs/LaneBoundaryFamily.h>
#include <asmc_msgs/LidarTrackingObservation.h>
#include <asmc_msgs/LocalTrajectory.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <fstream>
#include <iomanip>
#include <memory>
#include <morai_msgs/EgoVehicleStatus.h>
#include <mutex>
#include <nav_msgs/Path.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <set>
#include <sstream>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

namespace local_planning {
double yaw(const geometry_msgs::Quaternion &q) {
  return std::atan2(2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z));
}
geometry_msgs::Quaternion quaternion(double a) {
  geometry_msgs::Quaternion q;
  q.z = std::sin(a * .5);
  q.w = std::cos(a * .5);
  return q;
}
template <class T> void store(std::map<uint64_t, T> &m, const T &v) {
  m[v->header.stamp.toNSec()] = v;
  while (m.size() > 32)
    m.erase(m.begin());
}
class Node {
  ros::NodeHandle nh, p{"~"};
  Config config;
  MotionHistory motion;
  RoadMemory road_memory;
  ObstacleMemory obstacle_memory;
  bool retain_static_observations = false;
  ReferencePrior reference_prior;
  bool use_road_memory = true;
  bool semantic_reference_fallback = false;
  bool active_in_jamming_only = false, jamming_selected = false;
  uint64_t mode_generation = 0, scene_generation = 0, calculated_generation = ~uint64_t(0);
  bool preparing = false;
  double prepare_received = 0;
  std::string evaluation_frame, phase = "idle";
  uint64_t expected_scene_generation = 0, expected_mode_generation = 0;
  std::mutex diagnostic_mutex;
  double input_lock_wait_ms = 0, input_lock_hold_ms = 0, snapshot_ms = 0;
  double memory_ms = 0, debug_geometry_ms = 0, frame_budget_ms = 0;
  uint64_t deadline_discarded = 0;
  bool detail = false, scene_assembled = false;
  // Callbacks own the small input cache. Expensive Plan runs outside this lock;
  // a second callback thread keeps signed odometry and arrivals current.
  std::mutex input_mutex, calculation_mutex;
  uint64_t discarded_plans = 0;
  std::string session, run, frame, camera_epoch, speed_caller, config_id;
  ros::Subscriber speed_sub, imu_sub, prob_sub, cover_sub, context_sub,
      family_sub, objects_sub, mode_sub, prepare_sub;
  ros::Publisher odom_pub, path_pub, trajectory_pub, trace_pub, diagnostics_pub;
  ros::SteadyTimer planner_timer, diagnostic_timer;
  std::map<uint64_t, sensor_msgs::Image::ConstPtr> probs, covers;
  std::map<uint64_t, asmc_msgs::CameraGroundProjection::ConstPtr> contexts;
  std::map<uint64_t, asmc_msgs::LaneBoundaryFamily::ConstPtr> families;
  std::deque<asmc_msgs::LidarTrackingObservation::ConstPtr> objects;
  struct Arrival {
    double received, callback;
  };
  std::map<uint64_t, Arrival> family_arrivals, lidar_arrivals;
  template <class E>
  void arrival(std::map<uint64_t, Arrival> &history, uint64_t stamp,
               const E &event) {
    history[stamp] = {event.getReceiptTime().toSec(), ros::Time::now().toSec()};
    while (history.size() > 32)
      history.erase(history.begin());
  }
  void
  family(const ros::MessageEvent<asmc_msgs::LaneBoundaryFamily const> &event) {
    std::lock_guard<std::mutex> lock(input_mutex);
    auto message = event.getMessage();
    arrival(family_arrivals, message->header.stamp.toNSec(), event);
    store(families, message);
  }
  struct Speed {
    double stamp, value, received;
  };
  std::deque<Speed> speeds;
  double imu_received = 0, next_plan_due = 0;
  ros::SteadyTime last_invalid;
  uint64_t sequence = 0, last_scene = 0;
  uint64_t last_deferred_scene = 0, aged_bundle_deferred = 0;
  std::unique_ptr<Journal> journal;
  std::deque<std::string> recent;
  std::string reason = "initializing", last_json = "{}", cached_reason = "initializing";
  bool cached_valid = false;
  std::deque<std::array<double, 10>> timing_recent;
  std::vector<Pose> previous;
  Motion previous_origin;
  std::string input_ids = "{}";
  std::string scene_debug = "{}", join_debug = "{}";
  int home_role = 0;
  std::set<std::string> retired_camera_epochs;
  std::string lidar_epoch;
  std::set<std::string> retired_lidar_epochs;
  uint64_t lidar_sequence = 0;
  void clear_scene() {
    probs.clear();
    covers.clear();
    contexts.clear();
    families.clear();
    objects.clear();
    family_arrivals.clear();
    lidar_arrivals.clear();
    ++scene_generation; // Derived state is owned/reset by the calculation path.
  }
  void reset() {
    motion.reset();
    clear_scene();
  }
  void mode(const std_msgs::UInt8::ConstPtr &m) {
    std::lock_guard<std::mutex> lock(input_mutex);
    const bool next = m->data == 1;
    if (next != jamming_selected) {
      ++mode_generation;
      // Keep fresh preparation across entry; only in-flight mode results expire.
    }
    jamming_selected = next;
  }
  void prepare(const std_msgs::Bool::ConstPtr &m) {
    std::lock_guard<std::mutex> lock(input_mutex);
    preparing = m->data;
    prepare_received = ros::SteadyTime::now().toSec();
  }
  std::string current_frame() {
    return "local/" + session + "/" + std::to_string(motion.epoch);
  }
  void speed(const ros::MessageEvent<morai_msgs::EgoVehicleStatus const> &e) {
    std::lock_guard<std::mutex> lock(input_mutex);
    if (e.getPublisherName() != speed_caller)
      return;
    const auto m = e.getMessage();
    double t = m->header.stamp.toSec(), v = m->velocity.x;
    if (!std::isfinite(v) || t <= 0)
      return;
    if (!speeds.empty() && t <= speeds.back().stamp) {
      if (t < speeds.back().stamp) {
        reset();
        speeds.clear();
      } else
        return;
    }
    speeds.push_back({t, v, ros::SteadyTime::now().toSec()});
    while (speeds.size() > 64)
      speeds.pop_front();
  }
  void imu(const sensor_msgs::Imu::ConstPtr &m) {
    std::lock_guard<std::mutex> lock(input_mutex);
    const double t = m->header.stamp.toSec(),
                 now = ros::SteadyTime::now().toSec();
    auto it = std::find_if(speeds.rbegin(), speeds.rend(),
                           [&](auto &s) { return s.stamp <= t; });
    if (it == speeds.rend() || t - it->stamp > .10 || now - it->received > .30)
      return;
    const auto &q = m->orientation;
    double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(norm) || std::abs(norm - 1) > .05)
      return;
    double roll = std::atan2(2 * (q.w * q.x + q.y * q.z),
                             1 - 2 * (q.x * q.x + q.y * q.y));
    double pitch = std::asin(std::clamp(2 * (q.w * q.y - q.z * q.x), -1., 1.));
    if (std::abs(pitch) > .5)
      return;
    double w = (std::sin(roll) * m->angular_velocity.y +
                std::cos(roll) * m->angular_velocity.z) /
               std::cos(pitch);
    const auto epoch = motion.epoch;
    if (!motion.add(t, it->value, w, roll, pitch)) {
      if (epoch != motion.epoch)
        clear_scene();
      return;
    }
    if (epoch != motion.epoch)
      clear_scene();
    imu_received = now;
    auto state = motion.at(t);
    if (!state)
      return;
    odom_pub.publish(odometry(*state));
  }
  nav_msgs::Odometry odometry(const Motion &m, const std::string &frame_id = "") {
    nav_msgs::Odometry o;
    o.header.stamp.fromSec(m.t);
    o.header.frame_id = frame_id.empty() ? current_frame() : frame_id;
    o.child_frame_id = "base_link";
    o.pose.pose.position.x = m.x;
    o.pose.pose.position.y = m.y;
    const double cy = std::cos(m.yaw * .5), sy = std::sin(m.yaw * .5),
                 cr = std::cos(m.roll * .5), sr = std::sin(m.roll * .5),
                 cp = std::cos(m.pitch * .5), sp = std::sin(m.pitch * .5);
    o.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    o.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    o.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;
    o.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    o.twist.twist.linear.x = m.v;
    o.twist.twist.angular.z = m.w;
    return o;
  }
  void context(const asmc_msgs::CameraGroundProjection::ConstPtr &m) {
    std::lock_guard<std::mutex> lock(input_mutex);
    std::string epoch = m->producer_session_id + ":" +
                        std::to_string(m->generation) + ":" +
                        m->projection_session_id + ":" + m->config_id;
    if (retired_camera_epochs.count(epoch) ||
        retired_camera_epochs.size() >= 128)
      return;
    if (!camera_epoch.empty() && camera_epoch != epoch) {
      retired_camera_epochs.insert(camera_epoch);
      reset();
    }
    camera_epoch = epoch;
    store(contexts, m);
  }
  void lidar(const ros::MessageEvent<asmc_msgs::LidarTrackingObservation const>
                 &event) {
    std::lock_guard<std::mutex> lock(input_mutex);
    const auto m = event.getMessage();
    const auto &o = m->observation;
    std::string epoch = o.producer_session_id + ":" +
                        std::to_string(o.generation) + ":" + o.config_id + ":" +
                        o.calibration_id;
    if (o.producer_session_id.empty() || retired_lidar_epochs.count(epoch) ||
        retired_lidar_epochs.size() >= 128)
      return;
    if (!lidar_epoch.empty() && epoch != lidar_epoch) {
      retired_lidar_epochs.insert(lidar_epoch);
      reset();
      lidar_sequence = 0;
    }
    if (epoch == lidar_epoch && o.scan_sequence <= lidar_sequence)
      return;
    lidar_epoch = epoch;
    lidar_sequence = o.scan_sequence;
    arrival(lidar_arrivals, o.objects.header.stamp.toNSec(), event);
    objects.push_back(m);
    while (objects.size() > 32)
      objects.pop_front();
  }
  void report(const Result &r, double age, uint64_t stamp) {
    const auto serialization_start = ros::SteadyTime::now();
    std::ostringstream s;
    s << std::setprecision(17) << "{\"schema\":\"local1\",\"run_id\":\"" << run
      << "\",\"frame\":\"" << evaluation_frame
      << "\",\"plan_sequence\":" << sequence << ",\"inputs\":" << input_ids
      << ",\"scene\":" << scene_debug << ",\"input_join\":" << join_debug
      << ",\"scene_stamp_ns\":" << stamp
      << ",\"decision_stamp\":" << ros::Time::now().toSec()
      << ",\"scene_valid\":" << (scene_assembled ? "true" : "false")
      << ",\"valid\":" << (r.valid ? "true" : "false") << ",\"reason\":\""
      << r.reason << "\",\"age_s\":" << age
      << ",\"processing_ms\":" << r.elapsed_ms
      << ",\"reference_end_m\":" << r.reference_end
      << ",\"search_horizon_m\":" << r.search_horizon
      << ",\"selected_length_m\":" << (r.path.empty() ? 0 : r.path.back().s)
      << ",\"path_source\":" << std::quoted(r.path_source)
      << ",\"soft_semantic_margin_visits\":" << r.soft_margin_cells
      << ",\"search_ms\":" << r.search_ms << ",\"speed_ms\":" << r.speed_ms
      << ",\"alignment_ms\":" << r.alignment_ms
      << ",\"critical_path_ms\":" << r.alignment_ms + r.elapsed_ms + snapshot_ms
      << ",\"frame_budget_ms\":" << frame_budget_ms
      << ",\"deadline_discarded\":" << deadline_discarded
      << ",\"index_ms\":" << r.index_ms
      << ",\"input_lock_wait_ms\":" << input_lock_wait_ms
      << ",\"input_lock_hold_ms\":" << input_lock_hold_ms
      << ",\"snapshot_ms\":" << snapshot_ms
      << ",\"memory_ms\":" << memory_ms
      << ",\"debug_geometry_ms\":" << debug_geometry_ms
      << ",\"phase\":" << std::quoted(phase)
      << ",\"detail_enabled\":" << (detail ? "true" : "false")
      << ",\"requested_kph\":" << config.speed_cap * 3.6
      << ",\"curve_kph\":" << r.curve_cap * 3.6
      << ",\"space_kph\":" << r.space_cap * 3.6
      << ",\"actual_state_stopping_kph\":" << r.stopping_cap * 3.6
      << ",\"required_stopping_m\":" << r.required_stopping_m
      << ",\"available_stopping_m\":" << r.available_stopping_m
      << ",\"recovery_extension_m\":" << r.recovery_extension_m
      << ",\"selected_kph\":" << r.chosen_cap * 3.6 << ",\"speed_reason\":\""
      << r.speed_reason << "\",\"assisted\":" << (r.assisted ? "true" : "false")
      << ",\"recovery\":" << (r.recovery ? "true" : "false")
      << ",\"recovery_cost\":" << r.recovery_cost
      << ",\"recovery_obb_overlap\":" << r.recovery_obb_overlap
      << ",\"expanded\":" << r.expanded
      << ",\"budget_exhausted\":" << (r.budget_exhausted ? "true" : "false")
      << ",\"blocking_id\":" << r.blocking_id << ",\"blocked\":[" << r.blocked_x
      << "," << r.blocked_y
      << "],\"journal_dropped\":" << journal->dropped.load()
      << ",\"rejected\":{";
    bool first = true;
    for (auto &entry : r.rejected) {
      if (!first)
        s << ",";
      first = false;
      s << "\"" << entry.first << "\":" << entry.second;
    }
    s << "},\"path\":[";
    first = true;
    if (detail) for (auto &q : r.path) {
      if (!first)
        s << ",";
      first = false;
      s << "[" << q.x << "," << q.y << "," << q.yaw << "]";
    }
    s << "],\"goals\":[";
    first = true;
    if (detail) for (auto &q : r.goals) {
      if (!first)
        s << ",";
      first = false;
      s << "[" << q[0] << "," << q[1] << "]";
    }
    s << "],\"rejected_examples\":[";
    first = true;
    if (detail) for (auto &q : r.rejected_examples) {
      if (!first)
        s << ",";
      first = false;
      s << "{\"pose\":[" << q.pose.x << "," << q.pose.y << "," << q.pose.yaw
        << "],\"reason\":\"" << q.reason << "\",\"object_id\":" << q.object_id
        << "}";
    }
    s << "],\"candidates\":[";
    if (detail) for (size_t i = 0; i < r.candidates.size(); ++i) {
      if (i)
        s << ',';
      const auto &candidate = r.candidates[i];
      s << "{\"reason\":" << std::quoted(candidate.reason)
        << ",\"lateral_target_m\":" << candidate.lateral_target
        << ",\"transition_m\":" << candidate.transition << ",\"path\":[";
      for (size_t k = 0; k < candidate.path.size(); ++k) {
        if (k)
          s << ',';
        s << '[' << candidate.path[k].x << ',' << candidate.path[k].y << ','
          << candidate.path[k].yaw << ']';
      }
      s << "]}";
    }
    s << "],\"candidates_omitted\":" << r.candidates_omitted;
    s << ",\"discarded_generation_results\":" << discarded_plans;
    s << ",\"aged_bundle_deferred\":" << aged_bundle_deferred;
    s << ",\"vehicle\":{\"width\":" << config.width
      << ",\"front\":" << config.length - config.rear
      << ",\"rear\":" << config.rear << ",\"margin\":" << config.margin << "}}";
    std::string encoded = s.str();
    encoded.pop_back();
    encoded += ",\"serialization_ms\":" + std::to_string(
        (ros::SteadyTime::now()-serialization_start).toSec()*1000.) + "}";
    {
      std::lock_guard<std::mutex> cache(diagnostic_mutex);
      last_json = encoded;
      cached_reason = r.reason;
      cached_valid = r.valid;
      timing_recent.push_back({double(sequence), r.valid ? 1. : 0., input_lock_wait_ms,
          input_lock_hold_ms, r.alignment_ms + r.elapsed_ms + snapshot_ms,
          debug_geometry_ms, (ros::SteadyTime::now()-serialization_start).toSec()*1000.,
          r.index_ms, r.elapsed_ms, ros::Time::now().toSec()});
      if (timing_recent.size() > 100) timing_recent.pop_front();
    }
    journal->push(encoded);
    recent.push_back(encoded);
    if (recent.size() > 100)
      recent.pop_front();
    if (trace_pub.getNumSubscribers()) {
      std_msgs::String m;
      m.data = encoded;
      trace_pub.publish(m);
    }
    if (reason != r.reason) {
      ROS_WARN("LOCAL state %s -> %s (plan %lu)", reason.c_str(),
               r.reason.c_str(), sequence);
      reason = r.reason;
    }
    ROS_INFO_THROTTLE(5., "LOCAL %s plan=%lu cap=%.1fkm/h (%s) cost=%.1fms",
                      reason.c_str(), sequence, r.chosen_cap * 3.6,
                      r.speed_reason.c_str(), r.elapsed_ms);
  }
  void invalid(const std::string &why) {
    const auto now = ros::SteadyTime::now();
    if (reason == why && (now - last_invalid).toSec() < .1)
      return;
    last_invalid = now;
    input_ids = "{}";
    scene_debug = "{}";
    scene_assembled = false;
    previous.clear();
    Result r;
    r.reason = why;
    asmc_msgs::LocalTrajectory t;
    t.header.stamp = ros::Time::now();
    t.header.frame_id = evaluation_frame;
    t.reason = why;
    t.run_id = run;
    t.plan_sequence = ++sequence;
    {
      std::lock_guard<std::mutex> lock(input_mutex);
      if (scene_generation != expected_scene_generation || mode_generation != expected_mode_generation) {
        ++discarded_plans;
        return;
      }
      trajectory_pub.publish(t);
    }
    report(r, 0, 0);
  }
  void tick(const ros::SteadyTimerEvent &) {
    std::unique_lock<std::mutex> computation(calculation_mutex,
                                             std::try_to_lock);
    if (!computation.owns_lock())
      return;
    const auto wait_start = ros::SteadyTime::now();
    std::unique_lock<std::mutex> lock(input_mutex);
    const auto locked = ros::SteadyTime::now();
    // Bounded pointer/history snapshot. No raster copy or JSON under this lock.
    const auto motion = this->motion;
    const auto contexts = this->contexts;
    const auto probs = this->probs, covers = this->covers;
    const auto families = this->families;
    const auto objects = this->objects;
    const auto family_arrivals = this->family_arrivals;
    const auto lidar_arrivals = this->lidar_arrivals;
    const auto camera_epoch = this->camera_epoch, lidar_epoch = this->lidar_epoch;
    const auto imu_received = this->imu_received;
    const auto expected_epoch = motion.epoch;
    const auto expected_mode = mode_generation;
    const auto expected_camera = camera_epoch, expected_lidar = lidar_epoch;
    expected_scene_generation = scene_generation;
    expected_mode_generation = mode_generation;
    evaluation_frame = current_frame();
    const bool selected_mode = jamming_selected;
    const bool preparation = preparing && locked.toSec()-prepare_received <= .5;
    const bool compute = !active_in_jamming_only || selected_mode || preparation;
    lock.unlock();
    input_lock_wait_ms = (locked-wait_start).toSec()*1000.;
    input_lock_hold_ms = (ros::SteadyTime::now()-locked).toSec()*1000.;
    snapshot_ms = input_lock_hold_ms;
    memory_ms = debug_geometry_ms = 0;
    frame_budget_ms = config.budget_ms;
    detail = trace_pub.getNumSubscribers() || journal->enabled();
    scene_debug = join_debug = input_ids = "{}";
    scene_assembled = false;
    phase = selected_mode ? "active" : preparation ? "prepare" :
        active_in_jamming_only ? "idle" : "preview";
    if (calculated_generation != expected_scene_generation || !compute) {
      previous.clear(); road_memory.clear(); reference_prior.clear(); obstacle_memory.clear(); last_scene = 0; home_role = 0;
      calculated_generation = expected_scene_generation;
    }
    if (!compute) {
      invalid("inactive_normal_mode");
      return;
    }
    if (ros::SteadyTime::now().toSec() - imu_received > .15) {
      invalid("motion_unavailable");
      return;
    }
    uint64_t selected = 0;
    for (auto it = contexts.rbegin(); it != contexts.rend(); ++it) {
      if (!probs.count(it->first) || !covers.count(it->first) ||
          !families.count(it->first))
        continue;
      // Camera bundle identity stays exact; LiDAR selects its own latest scan.
      // A newer explicit invalid bundle is not hidden by an older valid one.
      selected = it->first;
      break;
    }
    if (detail) {
    std::ostringstream join;
    join << std::setprecision(17) << "{\"selected_stamp_ns\":" << selected
         << ",\"selection_policy\":\"independent_latest_to_plan\",\"newest_"
            "contexts\":[";
    unsigned joined = 0;
    for (auto it = contexts.rbegin(); it != contexts.rend() && joined < 3;
         ++it, ++joined) {
      if (joined)
        join << ',';
      double nearest = 1e9;
      for (const auto &object : objects)
        nearest = std::min(
            nearest, std::abs(object->observation.objects.header.stamp.toSec() -
                              it->first * 1e-9));
      join << "{\"stamp_ns\":" << it->first
           << ",\"age_s\":" << ros::Time::now().toSec() - it->first * 1e-9
           << ",\"probability\":" << (probs.count(it->first) ? "true" : "false")
           << ",\"coverage\":" << (covers.count(it->first) ? "true" : "false")
           << ",\"family\":" << (families.count(it->first) ? "true" : "false")
           << ",\"nearest_lidar_s\":" << nearest << '}';
    }
    join << "]}";
    join_debug = join.str();
    }
    if (!selected) {
      invalid("no_complete_scene");
      return;
    }
    double t = selected * 1e-9, age = ros::Time::now().toSec() - t;
    if (age < 0 || age > .35) {
      invalid("scene_stale");
      return;
    }
    if (selected == last_scene ||
        ros::SteadyTime::now().toSec() < next_plan_due)
      return;
    // Preserve350ms source freshness and reserve one20Hz control period.
    // Old fixed250ms preflight rejected usable270ms bundles and could wait
    // another full camera/plan cycle. Shorten THIS search budget instead of
    // extending freshness or treating an older observation as new.
    if (age > .295) {
      if (last_deferred_scene != selected) {
        last_deferred_scene = selected;
        ++aged_bundle_deferred;
      }
      ROS_WARN_THROTTLE(5., "LOCAL waiting for source deadline headroom: age=%.3fs", age);
      return;
    }
    const auto plan_stamp = ros::Time::now();
    auto camera_origin = motion.at(t);
    auto origin = motion.at(plan_stamp.toSec());
    if (!origin || !camera_origin) {
      const double state_age = plan_stamp.toSec() - motion.latest_stamp();
      if (!origin && camera_origin && state_age > .10 && state_age <= .15) {
        // Do not forecast past100ms or publish an invalid pulse during a
        // bounded sensor cadence gap.150ms receive/source expiry remains hard.
        ROS_WARN_THROTTLE(5., "LOCAL waiting for motion: age=%.3fs", state_age);
        return;
      }
      invalid(!camera_origin ? "camera_motion_history_unavailable"
                             : "plan_motion_extrapolation_limit");
      return;
    }
    const auto ctx = contexts.at(selected);
    const auto family = families.at(selected);
    if (!ctx->imu_corrected ||
        ctx->plane_coefficients.size() != 3 * ctx->views.size() ||
        ctx->views.empty() || (!family->valid && !semantic_reference_fallback)) {
      invalid("family_plane_invalid");
      return;
    }
    // deployment_allowed is a producer promotion flag, not geometric validity.
    // This explicitly selected experiment owns its separate control gate.
    if (family->header.frame_id != "base_link" ||
        family->operating_domain != "shadow_two_way_v1" ||
        (family->valid && family->boundaries.size() != 3) || family->config_sha256.empty() ||
        family->config_sha256 != ctx->config_id ||
        family->algorithm_id !=
            "twinlite_v6_shadow_boundary_family_candidate_v3" ||
        family->representation !=
            "shared_parametric_bspline_normal_offsets_v2") {
      invalid("family_contract");
      return;
    }
    auto probability = probs.at(selected), coverage = covers.at(selected);
    if (probability->encoding != "mono8" || coverage->encoding != "mono8" ||
        probability->width != 160 || probability->height != 250 ||
        coverage->width != probability->width ||
        coverage->height != probability->height ||
        probability->step != probability->width ||
        coverage->step != coverage->width ||
        probability->data.size() != 40000 || coverage->data.size() != 40000) {
      invalid("grid_contract");
      return;
    }
    if (objects.empty()) {
      invalid("local_objects_missing");
      return;
    }
    // Latest arrived scan, even if INVALID. Never revive an older valid scan
    // across a failure or choose it merely to match the camera's older stamp.
    auto chosen = std::prev(objects.end());
    const double object_age =
        plan_stamp.toSec() -
        (*chosen)->observation.objects.header.stamp.toSec();
    if (object_age < 0 || object_age > .35) {
      invalid("local_objects_stale");
      return;
    }
    const double delta =
        std::abs((*chosen)->observation.objects.header.stamp.toSec() - t);
    const auto &array = (*chosen)->observation.objects;
    if (array.header.frame_id != "base_link" ||
        (array.status != array.OBSERVED &&
         array.status != array.OBSERVED_EMPTY)) {
      invalid("local_objects_invalid");
      return;
    }
    auto om = motion.at(array.header.stamp.toSec());
    if (!om) {
      invalid("object_motion_unavailable");
      return;
    }
    const auto fi = family_arrivals.find(selected);
    const auto li = lidar_arrivals.find(array.header.stamp.toNSec());
    const double signed_skew = array.header.stamp.toSec() - t;
    if (detail) {
    std::ostringstream ids;
    ids << std::setprecision(17) << "{\"camera_session\":\""
        << ctx->producer_session_id
        << "\",\"camera_generation\":" << ctx->generation
        << ",\"projection_session\":\"" << ctx->projection_session_id
        << "\",\"projection_config\":\"" << ctx->config_id
        << "\",\"family_config\":\"" << family->config_sha256
        << "\",\"lidar_session\":\""
        << (*chosen)->observation.producer_session_id
        << "\",\"lidar_generation\":" << (*chosen)->observation.generation
        << ",\"lidar_scan\":" << (*chosen)->observation.scan_sequence
        << ",\"lidar_stamp_ns\":" << array.header.stamp.toNSec()
        << ",\"plan_stamp_ns\":" << plan_stamp.toNSec()
        << ",\"camera_age_s\":" << age << ",\"lidar_age_s\":" << object_age
        << ",\"alignment\":\"independent_source_to_plan_once\""
        << ",\"object_motion_compensated\":false"
        << ",\"unmodeled_object_motion_budget_mps\":5.0"
        << ",\"lidar_skew_s\":" << delta
        << ",\"lidar_minus_family_source_ms\":" << signed_skew * 1000;
    if (fi != family_arrivals.end() && li != lidar_arrivals.end()) {
      const auto &fa = fi->second, &la = li->second;
      ids << ",\"timing_available\":true"
          << ",\"family_source_to_receive_ms\":" << (fa.received - t) * 1000
          << ",\"lidar_source_to_receive_ms\":"
          << (la.received - array.header.stamp.toSec()) * 1000
          << ",\"lidar_minus_family_receive_ms\":"
          << (la.received - fa.received) * 1000
          << ",\"family_callback_queue_ms\":"
          << (fa.callback - fa.received) * 1000
          << ",\"lidar_callback_queue_ms\":"
          << (la.callback - la.received) * 1000;
    } else
      ids << ",\"timing_available\":false";
    ids << "}";
    input_ids = ids.str();
    }
    const auto alignment_start = ros::SteadyTime::now();
    Scene scene;
    scene.age = age;
    scene.aligned_to_plan = true;
    scene.speed = std::abs(origin->v) <= .05 ? 0. : origin->v;
    scene.yaw_rate = origin->w;
    scene.acceleration = origin->a;
    auto &g = scene.grid;
    g.rows = 250;
    g.cols = 160;
    g.probability = probability->data;
    g.coverage = coverage->data;
    auto front = std::find(ctx->views.begin(), ctx->views.end(), "front");
    size_t j = front == ctx->views.end() ? 0 : front - ctx->views.begin();
    double a = ctx->plane_coefficients[3 * j],
           b = ctx->plane_coefficients[3 * j + 1],
           c = ctx->plane_coefficients[3 * j + 2];
    if (!std::isfinite(a + b + c) || std::abs(a) > .5 || std::abs(b) > .5) {
      invalid("plane_out_of_range");
      return;
    }
    g.rotation = GroundRotation(a, b);
    g.ground_z = c / std::sqrt(1 + a * a + b * b);
    const double co = std::cos(origin->yaw), so = std::sin(origin->yaw);
    const double dx = camera_origin->x - origin->x,
                 dy = camera_origin->y - origin->y;
    g.source_in_plan = {co * dx + so * dy, -so * dx + co * dy,
                        camera_origin->yaw - origin->yaw};
    const double cy = std::cos(g.source_in_plan[2]),
                 sy = std::sin(g.source_in_plan[2]);
    bool semantic_reference = !family->valid;
    std::map<int, std::vector<std::array<double, 2>>> bounds;
    for (const auto &bound : family->boundaries) {
      // Only centre/right define LOCAL's home reference. A folded inferred
      // opposite outer line is not evidence that the semantic road is blocked.
      // Do not sort/repair a required boundary or bridge its support gaps.
      if (bound.role != 1 && bound.role != 2 && bound.role != 3) continue;
      if (bound.support_state.size() != bound.polyline.size())
        continue;
      bool optional_invalid = false;
      bool started = false;
      for (size_t k = 0; k < bound.polyline.size(); ++k) {
        if (bound.support_state[k] == 0) {
          if (started)
            break;
          continue;
        }
        auto v = bound.polyline[k];
        if (!std::isfinite(v.x + v.y + v.z)) {
          if (bound.role == 1) { optional_invalid = true; break; }
          invalid("family_nonfinite");
          return;
        }
        auto q = Rotate(g.rotation, v.x, v.y, v.z);
        const double qx = q[0], qy = q[1];
        q[0] = g.source_in_plan[0] + cy * qx - sy * qy;
        q[1] = g.source_in_plan[1] + sy * qx + cy * qy;
        auto &line = bounds[bound.role];
        if (!line.empty() && q[0] <= line.back()[0]) {
          if (bound.role == 1) { optional_invalid = true; break; }
          if (semantic_reference_fallback) { semantic_reference=true;break; }
          invalid("family_not_monotonic");
          return;
        }
        line.push_back({q[0], q[1]});
        started = true;
      }
      if (optional_invalid) bounds.erase(1);
    }
    if (bounds[2].size() < 2 || bounds[3].size() < 2) {
      if (semantic_reference_fallback) semantic_reference=true;
      else {
      invalid("family_support");
      return;
      }
    }
    // Rotated family members have different x samples. Interpolate in common
    // support, not by array index, and never bridge an invalid-support gap.
    auto at = [&](int role, double x) {
      const auto &line = bounds[role];
      for (size_t k = 1; k < line.size(); ++k)
        if (line[k][0] >= x) {
          auto l = line[k - 1], r = line[k];
          return l[1] + (r[1] - l[1]) * (x - l[0]) / (r[0] - l[0]);
        }
      return line.back()[1];
    };
    double begin = semantic_reference ? 0. : std::max(bounds[2].front()[0], bounds[3].front()[0]);
    double end = semantic_reference ? 0. : std::min(bounds[2].back()[0], bounds[3].back()[0]);
    if (begin > config.length - config.rear || end <= begin) {
      if (semantic_reference_fallback) semantic_reference=true;
      else {
      // Preserve the exact rejection evidence in input diagnostics, including
      // during body-motion cases where one role loses its near support.
      if (detail) join_debug.insert(join_debug.size() - 1,
                        ",\"family_support_begin_m\":" + std::to_string(begin) +
                            ",\"family_support_end_m\":" + std::to_string(end));
      invalid("family_disconnected_support");
      return;
      }
    }
    // LOCAL-1's explicit tunnel mission is right-strip travel (centre/right
    // outer). The producer's roles are geometric, not universal road policy.
    // Do not switch home strip just because the vehicle straddles the centre.
    // This is a navigation/return preference only: Plan checks semantic road
    // and local OBBs, never clips the footprint at these painted boundaries.
    // LOCAL-2B: left paint support no longer shortens right-home navigation.
    // Painted support never replaces the physical semantic/OBB sweep.
    home_role = 2;
    const auto reference_hint = semantic_reference_fallback ?
        reference_prior.aligned(Pose{origin->x,origin->y,origin->yaw},plan_stamp.toSec()) :
        std::vector<std::array<double,2>>{};
    if (semantic_reference) {
      scene.home=SemanticReference(g,config,reference_hint);
      if (scene.home.empty()) { invalid("semantic_reference_unavailable");return; }
    } else for (double x = begin; x <= end; x += .25) {
      scene.corridor.push_back(
          {x, bounds[1].empty() ? 0. : at(1, x), at(3, x)});
      scene.home.push_back({x, .5 * (at(home_role, x) + at(home_role + 1, x))});
    }
    if (!semantic_reference && semantic_reference_fallback)
      reference_prior.observe(scene.home,Pose{origin->x,origin->y,origin->yaw},t);
    if (detail) join_debug.insert(join_debug.size()-1,
        std::string(",\"reference_source\":\"")+(semantic_reference?"semantic_right_offset_trial":"family")+
        "\",\"family_input_valid\":"+(family->valid?"true":"false")+
        ",\"reference_prior_used\":"+(semantic_reference&&!reference_hint.empty()?"true":"false")+
        ",\"reference_prior_age_s\":"+std::to_string(reference_prior.age(plan_stamp.toSec())));
    for (const auto &o : array.objects) {
      if (o.observation_state != o.OBSERVED)
        continue;
      Obstacle ob;
      ob.id = o.track_id;
      ob.camera_class = o.camera_class;
      ob.classification = o.classification;
      ob.source_age = object_age;
      // Ego alignment does not move the object itself. Reserve an explicit
      // experimental motion envelope, including when velocity is UNKNOWN.
      // Never infer physical immobility from a semantic static group.
      double budget = 5.;
      if (o.absolute_velocity_valid) {
        const auto &v = o.absolute_velocity_mps;
        if (!std::isfinite(v.x + v.y + v.z)) {
          invalid("object_velocity_nonfinite");
          return;
        }
        budget = std::max(budget, std::hypot(v.x, v.y));
      }
      ob.uncertainty = object_age * budget;
      ob.motion_budget = budget;
      ob.moving = o.dynamic_valid && o.is_dynamic;
      for (const auto &corner : o.obb_corners) {
        if (!std::isfinite(corner.x + corner.y + corner.z)) {
          invalid("object_nonfinite");
          return;
        }
        auto q = Rotate(AttitudeRotation(om->roll, om->pitch), corner.x,
                        corner.y, corner.z);
        double wx = om->x + std::cos(om->yaw) * q[0] -
                    std::sin(om->yaw) * q[1] - origin->x,
               wy = om->y + std::sin(om->yaw) * q[0] +
                    std::cos(om->yaw) * q[1] - origin->y;
        ob.polygon.push_back({co * wx + so * wy, -so * wx + co * wy});
      }
      ob.polygon = geometry::ComputeConvexHull(ob.polygon);
      if (ob.polygon.size() < 3) {
        invalid("object_invalid_hull");
        return;
      }
      scene.obstacles.push_back(std::move(ob));
    }
    if(retain_static_observations)
      obstacle_memory.apply(scene.obstacles,Pose{origin->x,origin->y,origin->yaw},
                            array.header.stamp.toSec(),plan_stamp.toSec());
    for (auto q : previous) {
      double wx = previous_origin.x + std::cos(previous_origin.yaw) * q.x -
                  std::sin(previous_origin.yaw) * q.y - origin->x,
             wy = previous_origin.y + std::sin(previous_origin.yaw) * q.x +
                  std::cos(previous_origin.yaw) * q.y - origin->y;
      q.x = co * wx + so * wy;
      q.y = -so * wx + co * wy;
      q.yaw += previous_origin.yaw - origin->yaw;
      scene.previous.push_back(q);
    }
    const auto memory_start = ros::SteadyTime::now();
    if (use_road_memory) {
      road_memory.apply(g, Pose{origin->x, origin->y, origin->yaw},
                        plan_stamp.toSec());
      road_memory.observe(
          g, Pose{camera_origin->x, camera_origin->y, camera_origin->yaw}, t);
    } else {
      g.recent_road_age.assign(g.rows*g.cols, 0.f);
    }
    memory_ms = (ros::SteadyTime::now()-memory_start).toSec()*1000.;
    scene_assembled = true;
    const auto geometry_start = ros::SteadyTime::now();
    if (detail) {
    // Bounded debug geometry, never a second projection or image encoder.
    std::ostringstream debug;
    debug << std::setprecision(17) << "{\"ground_z\":" << g.ground_z
          << ",\"rotation\":[";
    for (size_t k = 0; k < 9; ++k) {
      if (k)
        debug << ',';
      debug << g.rotation[k];
    }
    debug << "],\"speed_mps\":" << scene.speed
          << ",\"yaw_rate_rad_s\":" << scene.yaw_rate
          << ",\"acceleration_mps2\":" << scene.acceleration
          << ",\"source_in_plan\":[" << g.source_in_plan[0] << ','
          << g.source_in_plan[1] << ',' << g.source_in_plan[2]
          << "],\"plan_stamp_ns\":" << plan_stamp.toNSec()
          << ",\"obstacles\":[";
    for (size_t k = 0; k < std::min(size_t(32), scene.obstacles.size()); ++k) {
      if (k)
        debug << ',';
      const auto &ob = scene.obstacles[k];
      debug << "{\"id\":" << ob.id
            << ",\"camera_class\":" << std::quoted(ob.camera_class)
            << ",\"classification\":" << std::quoted(ob.classification)
            << ",\"source_age_s\":" << ob.source_age
            << ",\"motion_uncertainty_m\":" << ob.uncertainty
            << ",\"retained_measurement\":" << (ob.retained?"true":"false")
            << ",\"motion_budget_mps\":" << ob.motion_budget
            << ",\"polygon\":[";
      for (size_t j = 0; j < ob.polygon.size(); ++j) {
        if (j)
          debug << ',';
        debug << '[' << ob.polygon[j].x << ',' << ob.polygon[j].y << ']';
      }
      debug << "]}";
    }
    debug << "],\"asset_retention_enabled\":" << (retain_static_observations?"true":"false")
          << ",\"obstacles_omitted\":"
          << (scene.obstacles.size() > 32 ? scene.obstacles.size() - 32 : 0)
          << ",\"home\":[";
    for (size_t k = 0; k < scene.home.size(); k += 4) {
      if (k)
        debug << ',';
      debug << '[' << scene.home[k][0] << ',' << scene.home[k][1] << ']';
    }
    debug << "],\"recent_road_runs\":[";
    bool first_run = true;
    unsigned memory_cells = 0;
    double memory_age = 0;
    for (int row = 0; row < g.rows; ++row) {
      int begin = -1;
      for (int col = 0; col <= g.cols; ++col) {
        double age = col < g.cols ? g.recent_road_age[row * g.cols + col] : 0;
        if (age > 0) {
          ++memory_cells;
          memory_age = std::max(memory_age, age);
          if (begin < 0)
            begin = col;
        } else if (begin >= 0) {
          if (!first_run)
            debug << ",";
          first_run = false;
          debug << "[" << row << "," << begin << "," << col - 1 << "]";
          begin = -1;
        }
      }
    }
    debug << "],\"recent_road_cells\":" << memory_cells
          << ",\"recent_road_max_age_s\":" << memory_age;
    debug << ",\"boundaries\":[";
    for (int role = 1; role <= 3; ++role) {
      if (role > 1)
        debug << ',';
      debug << "{\"role\":" << role << ",\"points\":[";
      for (size_t k = 0; k < bounds[role].size(); k += 2) {
        if (k)
          debug << ',';
        debug << '[' << bounds[role][k][0] << ',' << bounds[role][k][1] << ']';
      }
      debug << "]}";
    }
    debug << "]}";
    scene_debug = debug.str();
    }
    debug_geometry_ms = (ros::SteadyTime::now()-geometry_start).toSec()*1000.;
    double alignment_ms =
        (ros::SteadyTime::now() - alignment_start).toSec() * 1000 - debug_geometry_ms;
    const auto scheduled = ros::SteadyTime::now().toSec();
    next_plan_due = next_plan_due == 0
                        ? scheduled + .1
                        : std::max(next_plan_due + .1, scheduled);
    Config frame_config = config;
    if (semantic_reference) frame_config.speed_cap=std::min(frame_config.speed_cap,5./3.6);
    frame_budget_ms = std::min(config.budget_ms,
        (.30 - (ros::Time::now().toSec()-t))*1000.);
    if (frame_budget_ms < 5.) {
      Result skipped;
      skipped.reason = "planning_deadline_before_search";
      ++deadline_discarded;
      last_scene = selected;
      report(skipped, age, selected);
      return;
    }
    frame_config.budget_ms = frame_budget_ms;
    Result result;
    if (scene.speed < -.05)
      result.reason = "reverse_control_not_supported";
    else
      result = Plan(std::move(scene), frame_config);
      if (semantic_reference && result.speed_reason=="requested_cap")
        result.speed_reason="semantic_reference_trial_5kph";
    lock.lock();
    if (expected_mode != mode_generation || expected_epoch != this->motion.epoch || expected_camera != this->camera_epoch ||
        expected_lidar != this->lidar_epoch || expected_scene_generation != scene_generation) {
      ++discarded_plans;
      return; // Reset/restart cannot publish an already computing old scene.
    }
    if (result.valid && ros::Time::now().toSec()-t > .30) {
      ++deadline_discarded;
      last_scene = selected;
      lock.unlock();
      result.valid = false;
      result.reason = "planning_publish_deadline";
      result.alignment_ms = alignment_ms;
      report(result, age, selected);
      return; // Never publish a path lacking one control tick of headroom.
    }
    result.alignment_ms = alignment_ms;
    last_scene = selected;
    ++sequence;
    asmc_msgs::LocalTrajectory output;
    output.header = family->header;
    output.header.stamp = plan_stamp;
    output.header.frame_id = evaluation_frame;
    output.valid = result.valid;
    output.reason = result.reason;
    output.run_id = run;
    output.plan_sequence = sequence;
    output.scene_stamp = family->header.stamp;
    output.ego_at_plan = odometry(*origin, evaluation_frame);
    // Integration uses double seconds; the public identity keeps the exact
    // integer ROS stamp. Reconstructing it from double loses nanoseconds at
    // epoch-scale timestamps and falsely trips the controller contract.
    output.ego_at_plan.header = output.header;
    output.dt = .05;
    nav_msgs::Path path;
    path.header = output.header;
    for (auto &q : result.trajectory) {
      asmc_msgs::TrajectoryPoint p;
      p.x = origin->x + co * q[0] - so * q[1];
      p.y = origin->y + so * q[0] + co * q[1];
      p.yaw = origin->yaw + q[2];
      p.curvature = q[3];
      p.velocity = q[4];
      p.acceleration = q[5];
      output.points.push_back(p);
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p.x;
      pose.pose.position.y = p.y;
      pose.pose.orientation = quaternion(p.yaw);
      path.poses.push_back(pose);
    }
    trajectory_pub.publish(output);
    path_pub.publish(path);
    lock.unlock();
    report(result, age, selected);
    if (result.valid) {
      previous = result.path;
      previous_origin = *origin;
    } else
      previous.clear();
  }
  void diagnose(const ros::SteadyTimerEvent &) {
    std::lock_guard<std::mutex> lock(diagnostic_mutex);
    diagnostic_msgs::DiagnosticArray d;
    d.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus s;
    s.name = "planning/local";
    s.level = cached_valid ? s.OK : s.WARN;
    s.message = cached_reason;
    diagnostic_msgs::KeyValue v;
    v.key = "decision";
    v.value = last_json;
    s.values.push_back(v);
    std::ostringstream timings;
    timings << std::setprecision(17) << '[';
    for (size_t i=0; i<timing_recent.size(); ++i) {
      if (i) timings << ',';
      timings << '[';
      for (size_t j=0; j<timing_recent[i].size(); ++j) {
        if (j) timings << ',';
        timings << timing_recent[i][j];
      }
      timings << ']';
    }
    timings << ']';
    v.key = "timings"; v.value = timings.str(); s.values.push_back(v);
    d.status.push_back(s);
    diagnostics_pub.publish(d);
  }

public:
  Node() {
    std::ifstream uuid("/proc/sys/kernel/random/uuid");
    std::getline(uuid, session);
    if (session.empty())
      throw std::runtime_error("session UUID missing");
    p.param<std::string>("run_id", run, "local-preview");
    p.param("speed_cap_kph", config.speed_cap, 25.);
    config.speed_cap /= 3.6;
    p.param("near_blind_assist", config.blind_assist, false);
    p.param("causal_road_memory", use_road_memory, true);
    p.param("semantic_reference_fallback", semantic_reference_fallback, false);
    p.param("retain_static_observations", retain_static_observations, false);
    p.param("active_in_jamming_only", active_in_jamming_only, false);
    p.param("recovery", config.recovery, true);
    p.param("vehicle_width", config.width, 2.160835);
    p.param("vehicle_length", config.length, 4.635);
    p.param("rear_axle_to_bumper", config.rear, .790);
    p.param("clearance_margin", config.margin, .10);
    p.param("planning_budget_ms", config.budget_ms, 40.);
    p.param("acceleration_limit_mps2", config.accel, 1.);
    if (run.empty() || run.size() > 128 ||
        run.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUV"
                              "WXYZ0123456789-_.") != std::string::npos ||
        !std::isfinite(config.width + config.length + config.rear +
                       config.margin + config.budget_ms + config.speed_cap +
                       config.accel) ||
        config.width < 1.892 || config.length < 4.5 || config.rear <= 0 ||
        config.rear >= config.length || config.margin < 0 ||
        config.budget_ms <= 0 || config.budget_ms > 100 ||
        config.speed_cap <= 0 || config.speed_cap > 25. / 3.6 ||
        config.accel <= 0 || config.accel > 2.)
      throw std::runtime_error("invalid LOCAL-1 run/configuration");
    std::string log;
    p.param<std::string>("journal_path", log, "");
    journal = std::make_unique<Journal>(log);
    p.param<std::string>("speed_callerid", speed_caller, "/morai_udp_bridge");
    odom_pub = nh.advertise<nav_msgs::Odometry>("/planning/local/odometry", 1);
    path_pub = nh.advertise<nav_msgs::Path>("/planning/local/path", 1);
    trajectory_pub = nh.advertise<asmc_msgs::LocalTrajectory>(
        "/planning/local/trajectory", 1);
    trace_pub = nh.advertise<std_msgs::String>("/planning/local/trace", 1);
    diagnostics_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>(
        "/planning/local/diagnostics", 1);
    speed_sub = nh.subscribe("/Ego_topic", 64, &Node::speed, this);
    mode_sub = nh.subscribe("/planning/mode", 1, &Node::mode, this);
    if (active_in_jamming_only) {
      prepare_sub = nh.subscribe("/planning/mode/local_prepare", 1, &Node::prepare, this);
    }
    imu_sub = nh.subscribe("/imu", 64, &Node::imu, this);
    prob_sub = nh.subscribe<sensor_msgs::Image>(
        "/experimental/shadow/semantic_space_trial/drivable_probability", 1,
        [this](auto m) {
          std::lock_guard<std::mutex> lock(input_mutex);
          store(probs, m);
        });
    cover_sub = nh.subscribe<sensor_msgs::Image>(
        "/experimental/shadow/semantic_space_trial/coverage", 1,
        [this](auto m) {
          std::lock_guard<std::mutex> lock(input_mutex);
          store(covers, m);
        });
    context_sub =
        nh.subscribe("/experimental/shadow/semantic_space_trial/context", 1,
                     &Node::context, this);
    family_sub = nh.subscribe("/perception/lane/boundary_family", 4,
                              &Node::family, this);
    objects_sub =
        nh.subscribe("/perception/lidar/tracks_local", 4, &Node::lidar, this);
    planner_timer =
        // Poll latest complete input without tying compute supply to display.
        // Plan at most10Hz; camera processing can finish at its native rate.
        nh.createSteadyTimer(ros::WallDuration(.02), &Node::tick, this);
    diagnostic_timer =
        nh.createSteadyTimer(ros::WallDuration(1.), &Node::diagnose, this);
  }
};
} // namespace local_planning
int main(int argc, char **argv) {
  ros::init(argc, argv, "local_planner");
  try {
    local_planning::Node node;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception &e) {
    ROS_FATAL("LOCAL: %s", e.what());
    return 1;
  }
}
