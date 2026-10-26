#include <algorithm>
#include <memory>
#include <planning_inputs/ValidatedTrackedObjectArray.h>
#include <cmath>
#include <limits>
#include <string>
#include <stdexcept>
#include <vector>
#include <asmc_msgs/TrackedObjectArray.h>
#include <asmc_msgs/VehicleState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <planning_inputs/DetectedObjectArray.h>
#include <ros/ros.h>
#include "planning_inputs/converter.hpp"
#include "planning_inputs/MotionClassifier.hpp"
#include "planning_inputs/heading_history.hpp"
#include "planning_inputs/observation_sequence.hpp"
#include "planning_inputs/filters/PlanningObjectFilter.hpp"

namespace planning_inputs {
MotionClassifierConfig LoadMotionConfig(ros::NodeHandle& node) {
    MotionClassifierConfig config;
    int history_size = static_cast<int>(config.history_size);
    node.param("motion_history_size", history_size, history_size);
    node.param("motion_warmup_observations", config.warmup_observations,
               config.warmup_observations);
    node.param("motion_low_threat_confirmations",
               config.low_threat_confirmations,
               config.low_threat_confirmations);
    node.param("motion_dynamic_confirmations", config.dynamic_confirmations,
               config.dynamic_confirmations);
    node.param("motion_pending_static_confirmations",
               config.pending_static_confirmations,
               config.pending_static_confirmations);
    node.param("motion_dynamic_release_confirmations",
               config.dynamic_release_confirmations,
               config.dynamic_release_confirmations);
    node.param("motion_pending_max_frames", config.pending_max_frames,
               config.pending_max_frames);
    node.param("motion_change_confirmations", config.change_confirmations,
               config.change_confirmations);
    node.param("motion_short_miss_limit", config.short_miss_limit,
               config.short_miss_limit);
    node.param("motion_stale_track_frames", config.stale_track_frames,
               config.stale_track_frames);
    node.param("motion_lateral_threshold_kph",
               config.lateral_threshold_kph,
               config.lateral_threshold_kph);
    node.param("motion_longitudinal_threshold_kph",
               config.longitudinal_threshold_kph,
               config.longitudinal_threshold_kph);
    node.param("motion_fast_lateral_threshold_kph",
               config.fast_lateral_threshold_kph,
               config.fast_lateral_threshold_kph);
    node.param("motion_velocity_jump_kph", config.velocity_jump_kph,
               config.velocity_jump_kph);
    node.param("motion_direction_cosine_min",
               config.direction_cosine_min,
               config.direction_cosine_min);
    node.param("motion_longitudinal_displacement_kph",
               config.longitudinal_displacement_kph,
               config.longitudinal_displacement_kph);
    node.param("motion_lateral_displacement_kph",
               config.lateral_displacement_kph,
               config.lateral_displacement_kph);
    node.param("motion_exit_speed_kph", config.exit_speed_kph,
               config.exit_speed_kph);
    node.param("motion_edge_agreement_tolerance_m",
               config.edge_agreement_tolerance_m,
               config.edge_agreement_tolerance_m);

    config.history_size = static_cast<std::size_t>(std::max(3, history_size));
    config.warmup_observations = std::max(0, config.warmup_observations);
    config.low_threat_confirmations =
        std::max(1, config.low_threat_confirmations);
    config.dynamic_confirmations =
        std::max(1, config.dynamic_confirmations);
    config.pending_static_confirmations =
        std::max(1, config.pending_static_confirmations);
    config.dynamic_release_confirmations =
        std::max(1, config.dynamic_release_confirmations);
    config.pending_max_frames = std::max(1, config.pending_max_frames);
    config.change_confirmations =
        std::max(2, config.change_confirmations);
    config.short_miss_limit = std::max(0, config.short_miss_limit);
    config.stale_track_frames = std::max(1, config.stale_track_frames);
    config.lateral_threshold_kph =
        std::max(0.0, config.lateral_threshold_kph);
    config.longitudinal_threshold_kph =
        std::max(0.0, config.longitudinal_threshold_kph);
    config.fast_lateral_threshold_kph = std::max(
        config.lateral_threshold_kph,
        config.fast_lateral_threshold_kph);
    config.velocity_jump_kph = std::max(0.1, config.velocity_jump_kph);
    config.direction_cosine_min = std::max(
        -1.0, std::min(1.0, config.direction_cosine_min));
    config.longitudinal_displacement_kph =
        std::max(0.0, config.longitudinal_displacement_kph);
    config.lateral_displacement_kph =
        std::max(0.0, config.lateral_displacement_kph);
    config.exit_speed_kph = std::max(0.0, config.exit_speed_kph);
    config.edge_agreement_tolerance_m =
        std::max(0.0, config.edge_agreement_tolerance_m);
    return config;
}


class ObjectsNode {
 public:
  ObjectsNode() : private_("~"), classifier_(LoadMotionConfig(private_)) {
    std::string input,output,state;
    std::string policy;
    private_.param<std::string>("source_policy",policy,"asmc_lidar");
    private_.param<std::string>("input_topic",input,policy == "asmc_lidar" ?
        "/perception/lidar/tracks_map" : "/planning/input/tracked_objects");
    private_.param<std::string>("output_topic",output,"/planning/objects");
    private_.param<std::string>("state_topic",state,"/planning/vehicle/state");
    if(input==output) throw std::runtime_error("planning object topics must differ");
    publisher_=node_.advertise<DetectedObjectArray>(output,2);
    // Direct public contract, no JSON join or repeated perception filters.
    // TTC/follow/stop/avoid/sensor-failure actions remain planner-owned.
    if(policy=="asmc_lidar") {
      private_.param("source_max_age_s", source_max_age_s_, .35);
      private_.param("source_future_tolerance_s", source_future_s_, .05);
      if (!std::isfinite(source_max_age_s_) || source_max_age_s_ <= 0 ||
          !std::isfinite(source_future_s_) || source_future_s_ < 0)
        throw std::runtime_error("invalid LiDAR source time limits");
      input_=node_.subscribe<asmc_msgs::LidarTrackingObservation>(input,1,&ObjectsNode::LidarObjects,this,
                                                                 ros::TransportHints().tcpNoDelay());
      diagnostics_=private_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics",1);
      diagnostics_timer_=private_.createWallTimer(ros::WallDuration(1.),&ObjectsNode::Diagnostics,this);
    } else if(policy=="validated_lidar") {
      input_=node_.subscribe(input,2,&ObjectsNode::ValidatedObjects,this);
    } else if(policy=="main_raw") {
      filters_=std::make_unique<PlanningObjectFilter>(private_);
      input_=node_.subscribe(input,2,&ObjectsNode::Objects,this);
    } else throw std::runtime_error("unsupported planning LiDAR source_policy");
    state_=node_.subscribe(state,50,&ObjectsNode::State,this);
    ROS_INFO("Planning input (%s): %s -> %s",policy.c_str(),input.c_str(),output.c_str());
  }
 private:
  void LidarObjects(const asmc_msgs::LidarTrackingObservation::ConstPtr& source) {
    if (!source) return;
    ++received_;
    const double age = ros::Time::now().toSec() - source->observation.objects.header.stamp.toSec();
    if (!std::isfinite(age) || age > source_max_age_s_ || age < -source_future_s_) {
      ++rejected_; source_reason_ = "source_stale_or_future"; return;
    }
    std::string reason;
    if (!sequence_.Accept(source->observation, &reason)) {
      ++rejected_; source_reason_ = reason; return;
    }
    DetectedObjectArray out;
    if (ConvertLidarObservation(*source, &out, nullptr, &reason) != ConversionStatus::OK) {
      ++rejected_; source_reason_ = reason;
      ROS_WARN_THROTTLE(1., "Withheld common LiDAR input: %s", reason.c_str());
      return;  // invalid/pose-unavailable != an observed-empty free-space frame
    }
    publisher_.publish(out);
    source_reason_ = out.objects.empty() ? "observed_empty" : "observed";
    last_common_ = ros::SteadyTime::now();
    ++published_;
  }

  void Diagnostics(const ros::WallTimerEvent&) {
    diagnostic_msgs::DiagnosticArray message;
    message.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "planning_inputs/common_lidar";
    status.hardware_id = "asmc_lidar";
    const bool stale = last_common_.isZero() ||
        (ros::SteadyTime::now() - last_common_).toSec() > source_max_age_s_;
    status.level = stale || (source_reason_!="observed" && source_reason_!="observed_empty") ? status.WARN : status.OK;
    status.message = stale ? "no_fresh_representable_observation: " + source_reason_ : source_reason_;
    for (const auto& item : std::vector<std::pair<std::string, std::uint64_t>>{
        {"received",received_},{"published",published_},{"rejected",rejected_}}) {
      diagnostic_msgs::KeyValue value; value.key=item.first; value.value=std::to_string(item.second);
      status.values.push_back(std::move(value));
    }
    message.status.push_back(std::move(status));
    diagnostics_.publish(message);
  }
  void ValidatedObjects(const ValidatedTrackedObjectArray::ConstPtr& source) {
    if(!source) return;
    DetectedObjectArray out;
    ConversionStats stats; std::string detail;
    if(ConvertTrackedObjects(source->tracked,&out,&stats,&detail)!=ConversionStatus::OK ||
       source->motion_states.size()!=out.objects.size()) {
      ROS_ERROR_THROTTLE(1.0,"Rejected validated planning input: %s",detail.c_str()); return;
    }
    const double t=source->tracked.header.stamp.toSec();
    if(t<=last_source_) return;
    for(std::size_t i=0;i<out.objects.size();++i) {
      if(source->motion_states[i]>DetectedObject::MOTION_DYNAMIC) {
        ROS_ERROR_THROTTLE(1.0,"Rejected invalid validated motion state"); return;
      }
      out.objects[i].motion_state=source->motion_states[i];
    }
    last_source_=t;
    // Preserve Main's exact filtered geometry and applied velocity.
    // 이 규칙은 현재 legacy 계약이다. 새 asmc_lidar 경로는 원본 OBB/validity를
    // 보존하며, 기존 Main clipping이나 진단 JSON의 운동 상태를 재적용하지 않는다.
    publisher_.publish(out);
  }
  void State(const asmc_msgs::VehicleState::ConstPtr& s) {
    if(!s || !s->valid || s->header.frame_id!="map") return;
    headings_.Add(s->header.stamp.toSec(),s->yaw);
    state_received_=ros::SteadyTime::now();
  }
  void Objects(const asmc_msgs::TrackedObjectArray::ConstPtr& source) {
    if(!source) return;
    DetectedObjectArray out;
    ConversionStats stats; std::string detail;
    if(ConvertTrackedObjects(*source,&out,&stats,&detail)!=ConversionStatus::OK) {
      ROS_ERROR_THROTTLE(1.0,"Planning rejected canonical object frame: %s",detail.c_str());return;
    }
    const double t=source->header.stamp.toSec();
    if(t<=last_source_) return; // duplicates cannot vote for confirmation
    if(last_source_>0 && t-last_source_>.5) classifier_.Reset();
    last_source_=t;
    const auto heading=headings_.At(t);
    const bool fresh=!state_received_.isZero() &&
      (ros::SteadyTime::now()-state_received_).toSec()<=.2;
    if(!heading || !fresh) {
      classifier_.Reset();
      ROS_WARN_THROTTLE(1.0,"Planning motion state UNKNOWN: no fresh source-aligned heading");
      filters_->Apply(*source,&out);
      publisher_.publish(out);return;
    }
    std::vector<MotionObject> motion;motion.reserve(source->objects.size());
    const double c=std::cos(*heading),s=std::sin(*heading);
    for(const auto& o:source->objects) {
      MotionObject m;m.s32_Id=o.id;m.s32_MissCount=o.miss_count;
      m.f32_AbsVx=c*o.velocity_mps.x+s*o.velocity_mps.y;
      m.f32_AbsVy=-s*o.velocity_mps.x+c*o.velocity_mps.y;
      m.f32_CoreE=o.cluster_center.x;m.f32_CoreN=o.cluster_center.y;
      m.min_e=m.min_n=std::numeric_limits<double>::infinity();
      m.max_e=m.max_n=-std::numeric_limits<double>::infinity();
      for(const auto& p:o.obb_corners) {
        m.min_e=std::min(m.min_e,p.x);m.max_e=std::max(m.max_e,p.x);
        m.min_n=std::min(m.min_n,p.y);m.max_n=std::max(m.max_n,p.y);
      }
      motion.push_back(m);
    }
    classifier_.Update(motion,t);
    for(std::size_t i=0;i<out.objects.size();++i) {
      out.objects[i].motion_state=static_cast<std::uint8_t>(motion[i].s32_MotionState);
      if(out.objects[i].motion_state==DetectedObject::MOTION_LOW_THREAT) {
        out.objects[i].vx=0; out.objects[i].vy=0;
      }
    }
    // Only confirmed LOW_THREAT has zero private planning velocity. Main's
    // public message and UNKNOWN/PENDING/DYNAMIC velocities remain untouched.
    filters_->Apply(*source,&out);
    publisher_.publish(out);
  }
  ros::NodeHandle node_,private_;
  ros::Subscriber input_,state_;ros::Publisher publisher_;
  ros::SteadyTime state_received_;
  HeadingHistory headings_; MotionClassifier classifier_;
  std::unique_ptr<PlanningObjectFilter> filters_;
  double last_source_=0;
  ObservationSequence sequence_;
  ros::Publisher diagnostics_;
  ros::WallTimer diagnostics_timer_;
  ros::SteadyTime last_common_;
  double source_max_age_s_=.35, source_future_s_=.05;
  std::string source_reason_="waiting_for_input";
  std::uint64_t received_=0, published_=0, rejected_=0;
};
}
int main(int argc,char** argv) {
  ros::init(argc,argv,"planning_objects_adapter");
  try {planning_inputs::ObjectsNode node;ros::spin();}
  catch(const std::exception& e) {ROS_FATAL("%s",e.what());return 1;}
  return 0;
}
