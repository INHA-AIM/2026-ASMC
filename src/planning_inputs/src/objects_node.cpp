#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <stdexcept>
#include <vector>
#include <asmc_msgs/TrackedObjectArray.h>
#include <asmc_msgs/VehicleState.h>
#include <planning_inputs/DetectedObjectArray.h>
#include <ros/ros.h>
#include "planning_inputs/converter.hpp"
#include "planning_inputs/MotionClassifier.hpp"
#include "planning_inputs/heading_history.hpp"

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
    private_.param<std::string>("input_topic",input,"/planning/input/tracked_objects");
    private_.param<std::string>("output_topic",output,"/planning/objects");
    private_.param<std::string>("state_topic",state,"/planning/vehicle/state");
    if(input==output) throw std::runtime_error("planning object topics must differ");
    publisher_=node_.advertise<DetectedObjectArray>(output,2);
    input_=node_.subscribe(input,2,&ObjectsNode::Objects,this);
    state_=node_.subscribe(state,50,&ObjectsNode::State,this);
    ROS_INFO("Planning input: canonical main %s -> private %s; main sensor messages remain unchanged",input.c_str(),output.c_str());
  }
 private:
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
    publisher_.publish(out);
  }
  ros::NodeHandle node_,private_;
  ros::Subscriber input_,state_;ros::Publisher publisher_;
  ros::SteadyTime state_received_;
  HeadingHistory headings_; MotionClassifier classifier_;
  double last_source_=0;
};
}
int main(int argc,char** argv) {
  ros::init(argc,argv,"planning_objects_adapter");
  try {planning_inputs::ObjectsNode node;ros::spin();}
  catch(const std::exception& e) {ROS_FATAL("%s",e.what());return 1;}
  return 0;
}
