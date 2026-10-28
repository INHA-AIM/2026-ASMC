#include "local/journal.hpp"
#include "local/motion.hpp"
#include "local/planner.hpp"
#include <asmc_msgs/CameraGroundProjection.h>
#include <asmc_msgs/LaneBoundaryFamily.h>
#include <asmc_msgs/LidarTrackingObservation.h>
#include <asmc_msgs/LocalTrajectory.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <fstream>
#include <iomanip>
#include <memory>
#include <morai_msgs/EgoVehicleStatus.h>
#include <nav_msgs/Path.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <set>
#include <sstream>
#include <std_msgs/String.h>

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
  std::string session, run, frame, camera_epoch, speed_caller, config_id;
  ros::Subscriber speed_sub, imu_sub, prob_sub, cover_sub, context_sub,
      family_sub, objects_sub;
  ros::Publisher odom_pub, path_pub, trajectory_pub, trace_pub, diagnostics_pub;
  ros::SteadyTimer planner_timer, diagnostic_timer;
  std::map<uint64_t, sensor_msgs::Image::ConstPtr> probs, covers;
  std::map<uint64_t, asmc_msgs::CameraGroundProjection::ConstPtr> contexts;
  std::map<uint64_t, asmc_msgs::LaneBoundaryFamily::ConstPtr> families;
  std::deque<asmc_msgs::LidarTrackingObservation::ConstPtr> objects;
  struct Speed {
    double stamp, value, received;
  };
  std::deque<Speed> speeds;
  double imu_received = 0, last_plan_receive = 0;
  ros::SteadyTime last_invalid;
  uint64_t sequence = 0, last_scene = 0;
  std::unique_ptr<Journal> journal;
  std::deque<std::string> recent;
  std::string reason = "initializing", last_json = "{}";
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
    previous.clear();
    last_scene = 0;
    home_role = 0;
  }
  void reset() {
    motion.reset();
    clear_scene();
  }
  std::string current_frame() {
    return "local/" + session + "/" + std::to_string(motion.epoch);
  }
  void speed(const ros::MessageEvent<morai_msgs::EgoVehicleStatus const> &e) {
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
  nav_msgs::Odometry odometry(const Motion &m) {
    nav_msgs::Odometry o;
    o.header.stamp.fromSec(m.t);
    o.header.frame_id = current_frame();
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
  void lidar(const asmc_msgs::LidarTrackingObservation::ConstPtr &m) {
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
    objects.push_back(m);
    while (objects.size() > 32)
      objects.pop_front();
  }
  void report(const Result &r, double age, uint64_t stamp) {
    std::ostringstream s;
    s << std::setprecision(17) << "{\"schema\":\"local1\",\"run_id\":\"" << run
      << "\",\"frame\":\"" << current_frame()
      << "\",\"plan_sequence\":" << sequence << ",\"inputs\":" << input_ids
      << ",\"scene\":" << scene_debug << ",\"input_join\":" << join_debug
      << ",\"scene_stamp_ns\":" << stamp
      << ",\"decision_stamp\":" << ros::Time::now().toSec()
      << ",\"valid\":" << (r.valid ? "true" : "false") << ",\"reason\":\""
      << r.reason << "\",\"age_s\":" << age
      << ",\"processing_ms\":" << r.elapsed_ms
      << ",\"search_ms\":" << r.search_ms << ",\"speed_ms\":" << r.speed_ms
      << ",\"alignment_ms\":" << r.alignment_ms
      << ",\"requested_kph\":" << config.speed_cap * 3.6
      << ",\"curve_kph\":" << r.curve_cap * 3.6
      << ",\"space_kph\":" << r.space_cap * 3.6
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
    for (auto &q : r.path) {
      if (!first)
        s << ",";
      first = false;
      s << "[" << q.x << "," << q.y << "," << q.yaw << "]";
    }
    s << "],\"goals\":[";
    first = true;
    for (auto &q : r.goals) {
      if (!first)
        s << ",";
      first = false;
      s << "[" << q[0] << "," << q[1] << "]";
    }
    s << "],\"rejected_examples\":[";
    first = true;
    for (auto &q : r.rejected_examples) {
      if (!first)
        s << ",";
      first = false;
      s << "{\"pose\":[" << q.pose.x << "," << q.pose.y << "," << q.pose.yaw
        << "],\"reason\":\"" << q.reason << "\",\"object_id\":" << q.object_id
        << "}";
    }
    s << "],\"candidates\":[";
    for (size_t i = 0; i < r.candidates.size(); ++i) {
      if (i)
        s << ',';
      const auto &candidate = r.candidates[i];
      s << "{\"reason\":" << std::quoted(candidate.reason) << ",\"path\":[";
      for (size_t k = 0; k < candidate.path.size(); ++k) {
        if (k)
          s << ',';
        s << '[' << candidate.path[k].x << ',' << candidate.path[k].y << ','
          << candidate.path[k].yaw << ']';
      }
      s << "]}";
    }
    s << "],\"candidates_omitted\":" << r.candidates_omitted;
    s << ",\"vehicle\":{\"width\":" << config.width
      << ",\"front\":" << config.length - config.rear
      << ",\"rear\":" << config.rear << ",\"margin\":" << config.margin << "}}";
    last_json = s.str();
    journal->push(last_json);
    recent.push_back(last_json);
    if (recent.size() > 100)
      recent.pop_front();
    if (trace_pub.getNumSubscribers()) {
      std_msgs::String m;
      m.data = last_json;
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
    previous.clear();
    Result r;
    r.reason = why;
    asmc_msgs::LocalTrajectory t;
    t.header.stamp = ros::Time::now();
    t.header.frame_id = current_frame();
    t.reason = why;
    t.run_id = run;
    t.plan_sequence = ++sequence;
    trajectory_pub.publish(t);
    report(r, 0, 0);
  }
  void tick(const ros::SteadyTimerEvent &) {
    if (ros::SteadyTime::now().toSec() - imu_received > .15) {
      invalid("motion_unavailable");
      return;
    }
    uint64_t selected = 0;
    for (auto it = contexts.rbegin(); it != contexts.rend(); ++it) {
      if (!probs.count(it->first) || !covers.count(it->first) ||
          !families.count(it->first))
        continue;
      // A fresh explicit invalid family cancels the plan. Mere asynchronous
      // scan arrival may use the newest complete older bundle within the same
      // 350ms.
      if (!families.at(it->first)->valid || !it->second->imu_corrected) {
        selected = it->first;
        break;
      }
      double stamp = it->first * 1e-9;
      bool have_scan =
          std::any_of(objects.begin(), objects.end(), [&](auto &m) {
            return std::abs(m->observation.objects.header.stamp.toSec() -
                            stamp) <= .050000001;
          });
      if (have_scan) {
        selected = it->first;
        break;
      }
    }
    std::ostringstream join;
    join << std::setprecision(17) << "{\"selected_stamp_ns\":" << selected
         << ",\"nearest_scan_limit_s\":0.05,\"newest_contexts\":[";
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
    if (!selected) {
      invalid("no_complete_scene");
      return;
    }
    double t = selected * 1e-9, age = ros::Time::now().toSec() - t;
    if (age < 0 || age > .35) {
      invalid("scene_stale");
      return;
    }
    if (selected == last_scene)
      return;
    auto origin = motion.at(t);
    if (!origin) {
      invalid("scene_motion_unavailable");
      return;
    }
    const auto ctx = contexts.at(selected);
    const auto family = families.at(selected);
    if (!ctx->imu_corrected ||
        ctx->plane_coefficients.size() != 3 * ctx->views.size() ||
        ctx->views.empty() || !family->valid) {
      invalid("family_plane_invalid");
      return;
    }
    // deployment_allowed is a producer promotion flag, not geometric validity.
    // This explicitly selected experiment owns its separate control gate.
    if (family->header.frame_id != "base_link" ||
        family->operating_domain != "shadow_two_way_v1" ||
        family->boundaries.size() != 3 || family->config_sha256.empty() ||
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
    auto chosen = objects.end();
    double delta = .050000001;
    for (auto it = objects.begin(); it != objects.end(); ++it) {
      double d = std::abs((*it)->observation.objects.header.stamp.toSec() - t);
      if (d < delta) {
        delta = d;
        chosen = it;
      }
    }
    if (chosen == objects.end()) {
      invalid("local_objects_skew");
      return;
    }
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
        << ",\"lidar_skew_s\":" << delta << "}";
    input_ids = ids.str();
    const auto alignment_start = ros::SteadyTime::now();
    Scene scene;
    scene.age = age;
    scene.speed = origin->v;
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
    std::map<int, std::vector<std::array<double, 2>>> bounds;
    for (const auto &bound : family->boundaries) {
      if (bound.support_state.size() != bound.polyline.size())
        continue;
      bool started = false;
      for (size_t k = 0; k < bound.polyline.size(); ++k) {
        if (bound.support_state[k] == 0) {
          if (started)
            break;
          continue;
        }
        auto v = bound.polyline[k];
        if (!std::isfinite(v.x + v.y + v.z)) {
          invalid("family_nonfinite");
          return;
        }
        auto q = Rotate(g.rotation, v.x, v.y, v.z);
        auto &line = bounds[bound.role];
        if (!line.empty() && q[0] <= line.back()[0]) {
          invalid("family_not_monotonic");
          return;
        }
        line.push_back({q[0], q[1]});
        started = true;
      }
    }
    if (bounds[1].size() < 2 || bounds[2].size() < 2 || bounds[3].size() < 2) {
      invalid("family_support");
      return;
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
    double begin = std::max(
        {bounds[1].front()[0], bounds[2].front()[0], bounds[3].front()[0]});
    double end = std::min(
        {bounds[1].back()[0], bounds[2].back()[0], bounds[3].back()[0]});
    if (begin > config.length - config.rear || end <= begin) {
      invalid("family_disconnected_support");
      return;
    }
    // LOCAL-1's explicit tunnel mission is right-strip travel (centre/right
    // outer). The producer's roles are geometric, not universal road policy.
    // Do not switch home strip just because the vehicle straddles the centre.
    home_role = 2;
    for (double x = begin; x <= end; x += .25) {
      scene.corridor.push_back({x, at(1, x), at(3, x)});
      scene.home.push_back({x, .5 * (at(home_role, x) + at(home_role + 1, x))});
    }
    const double co = std::cos(origin->yaw), so = std::sin(origin->yaw);
    for (const auto &o : array.objects) {
      if (o.observation_state != o.OBSERVED)
        continue;
      Obstacle ob;
      ob.id = o.track_id;
      ob.camera_class = o.camera_class;
      ob.classification = o.classification;
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
    // Bounded debug geometry, never a second projection or image encoder.
    std::ostringstream debug;
    debug << std::setprecision(17) << "{\"ground_z\":" << g.ground_z
          << ",\"rotation\":[";
    for (size_t k = 0; k < 9; ++k) {
      if (k)
        debug << ',';
      debug << g.rotation[k];
    }
    debug << "],\"obstacles\":[";
    for (size_t k = 0; k < std::min(size_t(32), scene.obstacles.size()); ++k) {
      if (k)
        debug << ',';
      const auto &ob = scene.obstacles[k];
      debug << "{\"id\":" << ob.id
            << ",\"camera_class\":" << std::quoted(ob.camera_class)
            << ",\"classification\":" << std::quoted(ob.classification)
            << ",\"polygon\":[";
      for (size_t j = 0; j < ob.polygon.size(); ++j) {
        if (j)
          debug << ',';
        debug << '[' << ob.polygon[j].x << ',' << ob.polygon[j].y << ']';
      }
      debug << "]}";
    }
    debug << "],\"obstacles_omitted\":"
          << (scene.obstacles.size() > 32 ? scene.obstacles.size() - 32 : 0)
          << ",\"home\":[";
    for (size_t k = 0; k < scene.home.size(); k += 4) {
      if (k)
        debug << ',';
      debug << '[' << scene.home[k][0] << ',' << scene.home[k][1] << ']';
    }
    debug << "],\"boundaries\":[";
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
    double alignment_ms =
        (ros::SteadyTime::now() - alignment_start).toSec() * 1000;
    Result result = Plan(scene, config);
    result.alignment_ms = alignment_ms;
    last_scene = selected;
    ++sequence;
    asmc_msgs::LocalTrajectory output;
    output.header = family->header;
    output.header.frame_id = current_frame();
    output.valid = result.valid;
    output.reason = result.reason;
    output.run_id = run;
    output.plan_sequence = sequence;
    output.scene_stamp = family->header.stamp;
    output.ego_at_plan = odometry(*origin);
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
    report(result, age, selected);
    if (result.valid) {
      previous = result.path;
      previous_origin = *origin;
    } else
      previous.clear();
  }
  void diagnose(const ros::SteadyTimerEvent &) {
    diagnostic_msgs::DiagnosticArray d;
    d.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus s;
    s.name = "planning/local";
    s.level = reason == "ok" ? s.OK : s.WARN;
    s.message = reason;
    diagnostic_msgs::KeyValue v;
    v.key = "decision";
    v.value = last_json;
    s.values.push_back(v);
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
    p.param("recovery", config.recovery, true);
    p.param("vehicle_width", config.width, 2.160835);
    p.param("vehicle_length", config.length, 4.635);
    p.param("rear_axle_to_bumper", config.rear, .790);
    p.param("clearance_margin", config.margin, .10);
    p.param("planning_budget_ms", config.budget_ms, 40.);
    if (run.empty() || run.size() > 128 ||
        run.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUV"
                              "WXYZ0123456789-_.") != std::string::npos ||
        !std::isfinite(config.width + config.length + config.rear +
                       config.margin + config.budget_ms + config.speed_cap) ||
        config.width < 1.892 || config.length < 4.5 || config.rear <= 0 ||
        config.rear >= config.length || config.margin < 0 ||
        config.budget_ms <= 0 || config.budget_ms > 100 ||
        config.speed_cap <= 0 || config.speed_cap > 25. / 3.6)
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
    imu_sub = nh.subscribe("/imu", 64, &Node::imu, this);
    prob_sub = nh.subscribe<sensor_msgs::Image>(
        "/experimental/shadow/semantic_space_trial/drivable_probability", 1,
        [this](auto m) { store(probs, m); });
    cover_sub = nh.subscribe<sensor_msgs::Image>(
        "/experimental/shadow/semantic_space_trial/coverage", 1,
        [this](auto m) { store(covers, m); });
    context_sub =
        nh.subscribe("/experimental/shadow/semantic_space_trial/context", 1,
                     &Node::context, this);
    family_sub = nh.subscribe<asmc_msgs::LaneBoundaryFamily>(
        "/perception/lane/boundary_family", 4,
        [this](auto m) { store(families, m); });
    objects_sub =
        nh.subscribe("/perception/lidar/tracks_local", 4, &Node::lidar, this);
    planner_timer =
        // Semantic producer is capped at10Hz; only a new source bundle runs
        // Plan(). Poll readiness more often to avoid a second100ms phase wait.
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
    ros::spin();
  } catch (const std::exception &e) {
    ROS_FATAL("LOCAL: %s", e.what());
    return 1;
  }
}
