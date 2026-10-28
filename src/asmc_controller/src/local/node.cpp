// Separate LOCAL-1 input boundary. Normal map converter/behavior stays intact.
#include "MPC/MPC.hpp"
#include "MPC/Reference/path_resampler.hpp"
#include "MPC/Reference/reference_validator.hpp"
#include "Util/MessageConverter.hpp"
#include "Util/ParameterLoader.hpp"
#include <asmc_msgs/LocalTrajectory.h>
#include <cmath>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <iomanip>
#include <morai_msgs/CtrlCmd.h>
#include <nav_msgs/Odometry.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

class LocalController {
  ros::NodeHandle nh, p{"~"};
  ros::Subscriber state_sub, path_sub, mode_sub;
  ros::Publisher command_pub, trace_pub, diagnostics_pub;
  std::string last_json = "{}";
  ros::SteadyTimer timer, diagnostic_timer;
  nav_msgs::Odometry::ConstPtr state;
  asmc_msgs::LocalTrajectory::ConstPtr plan, last_good, used_plan;
  ros::SteadyTime state_received, plan_received;
  ros::Time evaluation_time;
  ros::SteadyTime last_solved;
  bool command_published = false;
  std::string frame, reason = "initializing";
  bool enabled = false, selected = false, duplicate = false;
  double command_age = 0, last_steering = 0;
  double local_acceleration_limit = 1.;
  static double yaw(const geometry_msgs::Quaternion &q) {
    return std::atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z));
  }
  void mode(const std_msgs::UInt8::ConstPtr &m) {
    bool next = m->data == 1;
    if (next != selected) {
      ResetMPCOnFailSafe();
      plan.reset();
      last_good.reset();
      last_solved = ros::SteadyTime();
    }
    selected = next;
  }
  void receiveState(const nav_msgs::Odometry::ConstPtr &m) {
    if (!state || m->header.frame_id != state->header.frame_id) {
      ResetMPCOnFailSafe();
      plan.reset();
      last_good.reset();
      last_solved = ros::SteadyTime();
    } else if (m->header.stamp <= state->header.stamp) {
      if (m->header.stamp < state->header.stamp) {
        state.reset();
        plan.reset();
        last_good.reset();
        last_solved = ros::SteadyTime();
        ResetMPCOnFailSafe();
      }
      return;
    }
    state = m;
    state_received = ros::SteadyTime::now();
  }
  void receivePlan(const asmc_msgs::LocalTrajectory::ConstPtr &m) {
    if (plan && m->header.frame_id == plan->header.frame_id &&
        m->plan_sequence <= plan->plan_sequence)
      return;
    plan = m;

    plan_received = ros::SteadyTime::now();
  }
  void report(bool valid, const morai_msgs::CtrlCmd &cmd, double cost = 0,
              double reference = 0, double cross = 0, double heading = 0,
              const MPCOutput *solve = nullptr) {
    std::ostringstream s;
    s << std::setprecision(17) << "{\"valid\":" << (valid ? "true" : "false")
      << ",\"command_published\":" << (command_published ? "true" : "false")
      << ",\"reason\":" << std::quoted(reason) << ",\"frame\":"
      << std::quoted(state ? state->header.frame_id : std::string())
      << ",\"plan_sequence\":" << (used_plan ? used_plan->plan_sequence : 0)
      << ",\"scene_stamp_ns\":"
      << (used_plan ? used_plan->scene_stamp.toNSec() : 0)
      << ",\"decision_stamp\":" << evaluation_time.toSec() << ",\"plan_age_s\":"
      << (used_plan ? (evaluation_time - used_plan->header.stamp).toSec() : -1.)
      << ",\"scene_age_s\":"
      << (used_plan ? (evaluation_time - used_plan->scene_stamp).toSec() : -1.)
      << ",\"motion_age_s\":"
      << (state ? (evaluation_time - state->header.stamp).toSec() : -1.)
      << ",\"incoming_plan_sequence\":" << (plan ? plan->plan_sequence : 0)
      << ",\"incoming_reason\":"
      << std::quoted(plan ? plan->reason : std::string()) << ",\"state_age_s\":"
      << (state ? (evaluation_time - state->header.stamp).toSec() : -1.)
      << ",\"speed_mps\":" << (state ? state->twist.twist.linear.x : 0)
      << ",\"reference_speed_mps\":" << reference
      << ",\"reference_acceleration_mps2\":"
      << (solve ? solve->first_reference_acceleration : 0)
      << ",\"solver_acceleration_mps2\":" << (solve ? solve->control.accel : 0)
      << ",\"acceleration_residual_mps2\":"
      << (solve ? solve->first_acceleration_residual : 0)
      << ",\"steering\":" << cmd.steering << ",\"throttle\":" << cmd.accel
      << ",\"brake\":" << cmd.brake << ",\"processing_ms\":" << cost
      << ",\"tracking_error_m\":" << cross
      << ",\"heading_error_rad\":" << heading
      << ",\"control_enabled\":" << (enabled && selected ? "true" : "false")
      << "}";
    last_json = s.str();
    if (trace_pub.getNumSubscribers()) {
      std_msgs::String m;
      m.data = last_json;
      trace_pub.publish(m);
    }
  }
  void stop(const std::string &why) {
    if (reason != why) {
      ROS_WARN("LOCAL MPC: %s", why.c_str());
      reason = why;
    }
    auto cmd = mpc_controller_ros::MakeRosEmergencyStopCommand();
    cmd.steering = last_steering;
    ResetMPCOnFailSafe();
    last_solved = ros::SteadyTime();
    if (enabled && !duplicate && selected) {
      command_pub.publish(cmd);
      command_published = true;
    }
    report(false, cmd);
  }
  std::string controlConflict() {
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();
    if (!ros::master::execute("getSystemState", args, result, payload, false))
      return "ROS master ownership unavailable";
    for (int i = 0; i < payload[0].size(); ++i) {
      const std::string topic = payload[0][i][0];
      // /ctrl_cmd/jamming is the existing selector slot, not the old BBox
      // follower. Local control owns this slot exclusively. A NORMAL command
      // producer also means an old control stack was started during the trial;
      // refuse our own control instead of running two stacks or killing theirs.
      // The one selector's /ctrl_cmd publisher is expected and is not a
      // conflict.
      if (topic == command_pub.getTopic() || topic == "/ctrl_cmd/normal") {
        for (int j = 0; j < payload[0][i][1].size(); ++j)
          if (std::string(payload[0][i][1][j]) != ros::this_node::getName())
            return topic + " owned by " + std::string(payload[0][i][1][j]);
      }
    }
    return {};
  }
  void tick(const ros::SteadyTimerEvent &) {
    evaluation_time = ros::Time::now();
    command_published = false;
    static unsigned cycles = 0;
    if (enabled && cycles++ % 20 == 0) {
      const auto conflict = controlConflict();
      duplicate = !conflict.empty();
      if (duplicate) {
        ROS_FATAL("LOCAL control ownership conflict: %s; refusing control",
                  conflict.c_str());
        ros::shutdown();
        return;
      }
    }
    used_plan = plan;
    const auto now = evaluation_time;
    const auto steady = ros::SteadyTime::now();
    if (!state || !plan) {
      stop("missing_local_input");
      return;
    }
    if ((steady - state_received).toSec() > .15 ||
        (steady - plan_received).toSec() > .55) {
      stop("local_receive_stale");
      return;
    }
    auto candidate = plan;
    const double state_age = (now - state->header.stamp).toSec();
    const bool transient =
        !plan->valid &&
        (plan->reason == "scene_stale" || plan->reason == "no_complete_scene" ||
         plan->reason == "local_objects_skew");
    bool grace = false;
    // A brief camera/scan arrival gap does not demand full brake at crawl.
    // Reuse only the previous path in the SAME integrated frame for at most
    // 200ms beyond 350ms source freshness. This is explicitly degraded
    // evidence, not a re-stamped fresh observation, and never enabled above
    // 5km/h. Only a previously solved recovery path qualifies; its original
    // jerk-limited speed profile is retained, never re-stamped.
    if ((transient ||
         (plan->valid && (now - plan->scene_stamp).toSec() > .35)) &&
        last_good && last_good->reason.rfind("near_field_", 0) == 0 &&
        last_good->header.frame_id == state->header.frame_id &&
        state->twist.twist.linear.x <= 5. / 3.6 &&
        (now - last_good->scene_stamp).toSec() >= 0 &&
        (now - last_good->scene_stamp).toSec() <= .55) {
      candidate = last_good;
      grace = true;
    }
    used_plan = candidate;
    if (!candidate->valid || candidate->points.size() < 2) {
      stop(candidate->reason);
      return;
    }
    const double age = (now - candidate->header.stamp).toSec();
    // Header is now the planning basis, not the perception source. Re-anchoring
    // geometry must never renew its350ms source freshness (550ms crawl grace).
    const double source_age = (now - candidate->scene_stamp).toSec();
    // LOCAL-2C drive09: a 67us future motion stamp caused a full brake even
    // with a fresh path. Wait for the clock/sample handoff, NEVER solve with
    // future state or refresh a command. The already-solved command remains
    // bounded by150ms and the selector watchdog. A persistent or >5ms lead
    // still fails; this is not a timestamp correction or extrapolation.
    if (state_age < 0 && state_age >= -.005 && last_good &&
        candidate->header.frame_id == last_good->header.frame_id &&
        state->header.frame_id == last_good->header.frame_id &&
        state->child_frame_id == "base_link" && age >= 0 && age <= .25 &&
        source_age >= age && source_age <= .35 &&
        (steady - last_solved).toSec() < .15) {
      reason = "motion_clock_pending";
      report(false, morai_msgs::CtrlCmd());
      return;
    }
    if (age < 0 || age > (grace ? .55 : .35) || source_age < age ||
        source_age > (grace ? .55 : .50) || state_age < 0 || state_age > .15) {
      stop("local_source_stale");
      return;
    }
    if (candidate->header.frame_id.rfind("local/", 0) != 0 ||
        candidate->header.frame_id != state->header.frame_id ||
        candidate->ego_at_plan.header.frame_id != candidate->header.frame_id ||
        candidate->ego_at_plan.header.stamp != candidate->header.stamp ||
        !std::isfinite(candidate->dt) || candidate->dt < .01 ||
        candidate->dt > .2 || candidate->points.size() > 400 ||
        state->child_frame_id != "base_link") {
      stop("local_contract_mismatch");
      return;
    }
    if (frame != candidate->header.frame_id) {
      frame = candidate->header.frame_id;
      ResetMPCOnFailSafe();
    }
    PlannedPath path;
    path.dt = candidate->dt;
    for (const auto &q : candidate->points) {
      if (!std::isfinite(q.x + q.y + q.yaw + q.curvature + q.velocity +
                         q.acceleration) ||
          q.velocity < 0) {
        stop("nonfinite_trajectory");
        return;
      }
      path.x.push_back(q.x);
      path.y.push_back(q.y);
      path.yaw.push_back(q.yaw);
      path.kappa.push_back(q.curvature);
      path.v.push_back(q.velocity);
      path.a.push_back(q.acceleration);
    }
    MPCState ego{state->pose.pose.position.x, state->pose.pose.position.y,
                 yaw(state->pose.pose.orientation),
                 state->twist.twist.linear.x};
    const auto &orientation = state->pose.pose.orientation;
    const double norm =
        orientation.x * orientation.x + orientation.y * orientation.y +
        orientation.z * orientation.z + orientation.w * orientation.w;
    if (ego.vx < -.05) {
      stop("reverse_control_not_supported");
      return;
    }
    if (!std::isfinite(ego.x + ego.y + ego.yaw + ego.vx + norm) ||
        std::abs(norm - 1.) > .05) {
      stop("nonfinite_motion");
      return;
    }
    ego.vx = std::max(0., ego.vx);
    if (!grace && source_age > .35) {
      // No computation from expired perception. Hold the previously issued
      // command for at most150ms since its valid solve, without publishing or
      // refreshing it. This covers a measured17ms missed handoff; the external
      // 250ms watchdog remains authoritative. Sustained loss still fails.
      // LOCAL-2C: the planner is10Hz; a200.9ms-old plan can still have a
      // valid solve less than150ms ago. Do not turn that polling jitter into
      // full braking. Plan age is bounded250ms; last_solved still bounds the
      // command hold150ms and no expired source is recomputed/re-published.
      if (last_good &&
          candidate->header.frame_id == last_good->header.frame_id &&
          candidate->scene_stamp >= last_good->scene_stamp && age <= .25 &&
          state_age <= .10 && std::isfinite(state->twist.twist.angular.z) &&
          (steady - last_solved).toSec() < .15) {
        reason = "plan_replacement_pending";
        report(false, morai_msgs::CtrlCmd());
        return;
      }
      stop("local_source_stale");
      return;
    }

    // State and trajectory have independent source times. Propagate the most
    // recent odometry once to this control tick, within the same100ms bound as
    // the planner's MotionHistory. Camera latency was handled by the planner.
    if (state_age > .10 && state_age <= .15 &&
        std::isfinite(state->twist.twist.angular.z) &&
        (steady - last_solved).toSec() < .15) {
      // The same bounded handoff covers the measured114ms IMU gap at15km/h.
      // No new command and no forecast beyond100ms, at any experiment speed.
      reason = "motion_sample_pending";
      report(false, morai_msgs::CtrlCmd());
      return;
    }
    if (state_age > .10 || !std::isfinite(state->twist.twist.angular.z)) {
      stop("control_motion_extrapolation_limit");
      return;
    }
    const double turn = state->twist.twist.angular.z * state_age;
    ego.x += ego.vx * state_age * std::cos(ego.yaw + .5 * turn);
    ego.y += ego.vx * state_age * std::sin(ego.yaw + .5 * turn);
    ego.yaw += turn;
    double offset = (now - candidate->header.stamp).toSec();
    if (offset < 0) {
      stop("state_before_plan");
      return;
    }
    auto ref = ToReferencePath(path, GetMPCParams().dt, GetMPCParams().horizon,
                               offset);

    auto validation = ValidateReference(ref, GetMPCParams());
    if (!validation.ok()) {
      stop(ReferenceValidationStatusName(validation.status));
      return;
    }
    auto start = ros::SteadyTime::now();
    auto output = MPCProcess(ego, ref, local_acceleration_limit);
    if (!output.success) {
      stop(output.failure_reason ? output.failure_reason : "mpc_failed");
      return;
    }
    auto command =
        mpc_controller_ros::ToRosControlCommand(output.control, GetMPCParams());
    last_solved = ros::SteadyTime::now();
    last_steering = command.steering;
    if (enabled && selected) {
      command_pub.publish(command);
      command_published = true;
    }
    if (!grace)
      last_good = candidate;
    reason = grace ? "replan_grace_crawl" : "ok";
    const double dx = ego.x - ref.x_ref.front(), dy = ego.y - ref.y_ref.front();
    double error = -std::sin(ref.yaw_ref.front()) * dx +
                   std::cos(ref.yaw_ref.front()) * dy;
    report(true, command, (ros::SteadyTime::now() - start).toSec() * 1000,
           output.first_reference_speed, error,
           std::remainder(ego.yaw - ref.yaw_ref.front(), 2 * M_PI), &output);
    ROS_INFO_THROTTLE(5., "LOCAL MPC v=%.1f target=%.1fkm/h control=%d",
                      ego.vx * 3.6, output.first_reference_speed * 3.6,
                      enabled && selected);
  }

public:
  LocalController() {
    std::string yaml;
    p.getParam("controller_config", yaml);
    if (yaml.empty())
      throw std::runtime_error("controller config missing");
    MPCParams params;
    loadMPCParameters(yaml, params);
    // Local low-speed references are repeatedly re-anchored at the rear axle.
    // The NORMAL horizon/rate cost sees too little of their connection curve
    // at crawl. Explicit local-only tuning; no change to the common solver.
    p.param("prediction_horizon", params.horizon, params.horizon);
    p.param("steering_change_cost", params.weight_control_rate,
            params.weight_control_rate);
    p.param("heading_cost", params.weight_heading_error,
            params.weight_heading_error);
    // LOCAL-2B: cap the solve, not pedal normalization or NORMAL parameters.
    // Lowering params.accel_max would also change legacy pedal conversion.
    p.param("acceleration_limit_mps2", local_acceleration_limit, 1.);
    if (params.horizon < 20 || params.horizon > 61 ||
        !std::isfinite(params.weight_control_rate +
                       params.weight_heading_error +
                       local_acceleration_limit) ||
        params.weight_control_rate <= 0 || params.weight_heading_error <= 0 ||
        local_acceleration_limit <= 0 ||
        local_acceleration_limit > params.accel_max)
      throw std::runtime_error("invalid local MPC tuning");
    InitMPC(params);
    p.param("enable_control", enabled, false);
    // Reuse one MPC solver and the selector's historical JAMMING slot. This
    // never starts the old BBox follower or NORMAL controller. Preview has no
    // command publisher; control trials must exclude autonomy_no_bridges_v2.
    if (enabled)
      command_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd/jamming", 1);
    diagnostics_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>(
        "/planning/local/control_diagnostics", 1);
    trace_pub =
        nh.advertise<std_msgs::String>("/planning/local/control_trace", 1);
    state_sub = nh.subscribe("/planning/local/odometry", 1,
                             &LocalController::receiveState, this);
    path_sub = nh.subscribe("/planning/local/trajectory", 1,
                            &LocalController::receivePlan, this);
    mode_sub = nh.subscribe("/planning/mode", 1, &LocalController::mode, this);
    diagnostic_timer = nh.createSteadyTimer(
        ros::WallDuration(1.), [this](const ros::SteadyTimerEvent &) {
          diagnostic_msgs::DiagnosticArray d;
          d.header.stamp = ros::Time::now();
          diagnostic_msgs::DiagnosticStatus status;
          status.name = "control/local";
          status.level = reason == "ok" ? 0 : 1;
          status.message = reason;
          diagnostic_msgs::KeyValue value;
          value.key = "decision";
          value.value = last_json;
          status.values.push_back(value);
          d.status.push_back(status);
          diagnostics_pub.publish(d);
        });
    timer = nh.createSteadyTimer(ros::WallDuration(.05), &LocalController::tick,
                                 this);
  }
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "local_controller");
  try {
    LocalController n;
    ros::spin();
  } catch (const std::exception &e) {
    ROS_FATAL("LOCAL MPC %s", e.what());
    return 1;
  }
}
