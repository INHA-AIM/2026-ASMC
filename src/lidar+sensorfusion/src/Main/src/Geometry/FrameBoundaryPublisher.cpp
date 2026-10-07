#include <Geometry/FrameBoundaryPublisher.hpp>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

namespace frame_boundary {
namespace {
using Array=asmc_msgs::LidarObjectArray;
void Point(const asmc::lidar::Vec& source,geometry_msgs::Point* target) {
  target->x=source.x(); target->y=source.y(); target->z=source.z();
}
asmc_msgs::LidarObject Message(const LiDARObjectInfo& original,const asmc::lidar::Geometry& geometry) {
  asmc_msgs::LidarObject out;
  out.track_id=original.s32_Id; out.detection_index=original.u16_Idx;
  out.observation_state=out.OBSERVED;
  out.age=std::max(0,original.s32_Age); out.miss_count=0;
  out.classification=original.str_ObjectClass; out.camera_class=original.str_YoloClass;
  Point(geometry.center,&out.center);
  for(unsigned i=0;i<8;++i) Point(geometry.corners[i],&out.obb_corners[i]);
  out.yaw=geometry.yaw.value_or(0.);
  out.length=original.f32_OBB_Width; out.width=original.f32_OBB_Height;
  out.height=original.f32_OBB_ZMax-original.f32_OBB_ZMin;
  out.source_point_count=original.pcl_ClusterPoints.size();
  // Legacy CA state is a derivative in moving/preprocessed sensor coordinates.
  // Its unproven 3D angular/origin terms must not become a valid zero/static.
  out.relative_velocity_valid=false; out.absolute_velocity_valid=false;
  out.dynamic_valid=false;
  return out;
}
Array Empty(const ros::Time& stamp,const char* frame,std::uint8_t status,const std::string& reason) {
  Array out; out.header.stamp=stamp; out.header.frame_id=frame;
  out.status=status; out.reason=reason; out.stamp_provenance="bridge_receive_ms";
  return out;
}
}
Publisher::Publisher(ros::NodeHandle& node) {
  if(!Enabled()) return;
  leveling_tf_=std::make_unique<tf2_ros::TransformBroadcaster>();
  local_=node.advertise<Array>("/perception/lidar/objects_local",5);
  map_=node.advertise<Array>("/perception/lidar/objects_map",5);
  diagnostics_=node.advertise<diagnostic_msgs::DiagnosticArray>("/perception/lidar/diagnostics",2);
  timer_=node.createWallTimer(ros::WallDuration(1.),&Publisher::Diagnostics,this);
}
void Publisher::SetLegacyReason(const std::string& reason) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);legacy_reason_=reason;
}
bool Publisher::Accept(const ros::Time& stamp) {
  if(stamp.isZero() || (!last_stamp_.isZero() && stamp<=last_stamp_)) {
    ++stamp_errors_; return false;
  }
  last_stamp_=stamp; last_received_=ros::WallTime::now(); return true;
}
void Publisher::Invalid(const ros::Time& stamp,const std::string& reason) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if(!Enabled() || !Accept(stamp)) return;
  input_reason_=reason; map_reason_="input_invalid"; ++invalid_;
  if(local_.getNumSubscribers()) local_.publish(Empty(stamp,"base_link",Array::INPUT_INVALID,reason));
  if(map_.getNumSubscribers()) map_.publish(Empty(stamp,"map",Array::INPUT_INVALID,reason));
}
void Publisher::Publish(const LiDAR& lidar) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if(!Enabled()) return;
  const auto& context=lidar.frame_context;
  const auto stamp=ros::Time().fromNSec(context.time.stamp_ns);
  const auto time_error=asmc::lidar::validate_time(context.time);
  if(!time_error.empty()) {Invalid(stamp,time_error); return;}
  const auto base_from_level=asmc::lidar::base_from_input(context);
  if(!base_from_level) {Invalid(stamp,"leveling_transform_invalid");return;}
  if(!Accept(stamp)) return;
  geometry_msgs::TransformStamped tf;tf.header.stamp=stamp;tf.header.frame_id="base_link";
  tf.child_frame_id="lidar_leveled";
  tf.transform.translation.x=base_from_level->translation().x();
  tf.transform.translation.y=base_from_level->translation().y();
  tf.transform.translation.z=base_from_level->translation().z();
  const Eigen::Quaterniond q(base_from_level->rotation());
  tf.transform.rotation.x=q.x();tf.transform.rotation.y=q.y();tf.transform.rotation.z=q.z();tf.transform.rotation.w=q.w();
  leveling_tf_->sendTransform(tf);
  last_context_=context;
  ++processed_; input_reason_="valid"; map_reason_=context.map_pose.reason;
  const bool local_needed=local_.getNumSubscribers(),map_needed=map_.getNumSubscribers();
  if(!local_needed && !map_needed) {last_serialization_ms_=0.; return;}
  const auto start=std::chrono::steady_clock::now();
  auto geometry_context=context;
  if(!map_needed) geometry_context.map_pose.valid=false;
  Array local=Empty(stamp,"base_link",Array::OBSERVED_EMPTY,"observed_empty");
  Array map=Empty(stamp,"map",context.map_pose.valid?Array::OBSERVED_EMPTY:Array::POSE_UNAVAILABLE,
      context.map_pose.valid?"observed_empty":context.map_pose.reason);
  for(const auto& original:lidar.vec_Objects) {
    if(original.s32_Id<0 || original.s32_MissCount!=0 || original.pcl_ClusterPoints.empty() ||
        original.str_ObjectClass=="noise" || original.str_ObjectClass=="지면노이즈") continue;
    asmc::lidar::Object object; object.id=original.s32_Id; object.classification=original.str_ObjectClass;
    object.center=asmc::lidar::Vec(original.f32_OBB_CenterX,original.f32_OBB_CenterY,original.f32_OBB_CenterZ);
    object.long_axis=asmc::lidar::Vec(std::cos(original.f32_OBB_YawRad),std::sin(original.f32_OBB_YawRad),0);
    for(unsigned i=0;i<8;++i) {
      const auto& p=original.OBB_Corners_Body[i]; object.corners[i]=asmc::lidar::Vec(p.x,p.y,p.z);
    }
    auto result=asmc::lidar::transform(object,geometry_context);
    const double dimensions[]={original.f32_OBB_Width,original.f32_OBB_Height,
        original.f32_OBB_ZMax-original.f32_OBB_ZMin};
    for(double dimension:dimensions) if(!std::isfinite(dimension) || dimension<0) {
      result.valid=false;result.reason="invalid_object_dimensions";
    }
    if(!result.valid) {
      local.status=Array::INPUT_INVALID; local.reason=result.reason; local.objects.clear();
      map.status=Array::INPUT_INVALID; map.reason=result.reason; map.objects.clear();
      input_reason_=result.reason; ++invalid_; break;
    }
    if(local_needed) {local.status=Array::OBSERVED; local.reason="observed"; local.objects.push_back(Message(original,result.base));}
    if(map_needed && result.map) {
      map.status=Array::OBSERVED; map.reason=context.map_pose.reason;
      map.objects.push_back(Message(original,*result.map));
    }
  }
  if(local_needed) local_.publish(local);
  if(map_needed) map_.publish(map);
  last_serialization_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
}
void Publisher::Diagnostics(const ros::WallTimerEvent&) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  diagnostic_msgs::DiagnosticArray array; array.header.stamp=ros::Time::now();
  diagnostic_msgs::DiagnosticStatus status; status.name="lidar_frame_boundary"; status.hardware_id="Main";
  const bool silent=last_received_.isZero() || (ros::WallTime::now()-last_received_).toSec()>1.;
  status.level=silent || input_reason_!="valid" ? status.WARN : status.OK;
  status.message=silent?"input_silence":input_reason_;
  auto value=[&](const std::string& key,const std::string& text){diagnostic_msgs::KeyValue v;v.key=key;v.value=text;status.values.push_back(v);};
  value("processed",std::to_string(processed_)); value("invalid",std::to_string(invalid_));
  value("stamp_errors",std::to_string(stamp_errors_)); value("map_reason",map_reason_);
  value("source_stamp_ns",std::to_string(last_stamp_.toNSec())); value("calibration_sha",CalibrationId());
  value("legacy_output",legacy_reason_);
  value("input_generation",std::to_string(last_context_.input_generation));
  value("input_coordinate","leveled_sensor");
  value("leveling_source",last_context_.leveling_source);
  if(!last_context_.map_pose.sample_stamps.empty()) {
    value("gps_sample_stamp_ns",std::to_string(last_context_.map_pose.sample_stamps.back()));
    value("gps_sample_arrival_ns",std::to_string(last_context_.map_pose.sample_arrivals.back()));
    value("gps_age_ms",std::to_string(double(last_context_.time.stamp_ns-last_context_.map_pose.sample_stamps.back())/1e6));
  }
  value("roll_leveling",UseRoll()?"true":"false");
  value("serialization_ms",std::to_string(last_serialization_ms_));
  value("local_subscribers",std::to_string(local_.getNumSubscribers()));
  value("map_subscribers",std::to_string(map_.getNumSubscribers()));
  value("velocity_status","unproven_moving_sensor_derivative");
  array.status.push_back(status); diagnostics_.publish(array);
}
}  // namespace frame_boundary
