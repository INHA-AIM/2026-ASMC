#include <Geometry/FrameBoundaryPublisher.hpp>
#include <Lidar/Tracking/MotionClassifier.hpp>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <iostream>

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
  out.status=status; out.reason=reason; out.stamp_provenance="bridge_receive_ns";
  return out;
}
using ArrayV2=asmc_msgs::LidarObjectArrayV2;
using ObjectV2=asmc_msgs::LidarObjectV2;
static_assert(static_cast<int>(ObjectV2::MOTION_UNKNOWN)==static_cast<int>(MotionState::Unknown),"V2 motion code");
static_assert(static_cast<int>(ObjectV2::MOTION_LOW_THREAT)==static_cast<int>(MotionState::LowThreat),"V2 motion code");
static_assert(static_cast<int>(ObjectV2::MOTION_DYNAMIC_PENDING)==static_cast<int>(MotionState::DynamicPending),"V2 motion code");
static_assert(static_cast<int>(ObjectV2::MOTION_DYNAMIC)==static_cast<int>(MotionState::Dynamic),"V2 motion code");
static_assert(static_cast<int>(ArrayV2::INPUT_INVALID)==static_cast<int>(Array::INPUT_INVALID),"V2 array status");
// Same predicate as V1 Publish() and LegacyMotionReady (FrameBoundary.cpp).
bool Publishable(const LiDARObjectInfo& o) {
  return o.s32_Id>=0 && o.s32_MissCount==0 && !o.pcl_ClusterPoints.empty() &&
      o.str_ObjectClass!="noise" && o.str_ObjectClass!="지면노이즈";
}
// Every LidarObject field comes from V1 Message() (single definition of their meaning);
// v2 adds only the motion fields and the compatibility-derived dynamic_valid/is_dynamic.
ObjectV2 MessageV2(const LiDARObjectInfo& original,const asmc::lidar::Geometry& geometry) {
  const auto v1=Message(original,geometry);
  ObjectV2 out;
  out.track_id=v1.track_id; out.detection_index=v1.detection_index;
  out.observation_state=v1.observation_state; out.age=v1.age; out.miss_count=v1.miss_count;
  out.classification=v1.classification; out.camera_class=v1.camera_class;
  out.center=v1.center; out.obb_corners=v1.obb_corners; out.yaw=v1.yaw;
  out.yaw_rate_valid=v1.yaw_rate_valid; out.yaw_rate_rad_s=v1.yaw_rate_rad_s;
  out.length=v1.length; out.width=v1.width; out.height=v1.height;
  out.relative_velocity_valid=v1.relative_velocity_valid; out.relative_velocity_mps=v1.relative_velocity_mps;
  out.absolute_velocity_valid=v1.absolute_velocity_valid; out.absolute_velocity_mps=v1.absolute_velocity_mps;
  out.dynamic_valid=v1.dynamic_valid; out.is_dynamic=v1.is_dynamic;
  out.source_point_count=v1.source_point_count;
  FillMotionV2(original,&out);
  return out;
}
}
output_gate::Inputs GateInputs(bool pose_ready,const std::string& pose_reason,bool lidar_processed,
                               const LiDAR& lidar,std::uint64_t generation) {
  output_gate::Inputs in;
  in.common_geometry=Enabled(); in.pose_ready=pose_ready; in.pose_reason=pose_reason;
  in.lidar_processed=lidar_processed;
  if(!in.common_geometry || !pose_ready) return in;
  const auto& context=lidar_processed?lidar.frame_context:Current();
  in.time_error=asmc::lidar::validate_time(context.time);
  in.map_pose_valid=context.map_pose.valid; in.map_reason=context.map_pose.reason;
  in.map_pose_matches_scan=context.map_pose.valid && context.map_pose.query_stamp_ns==context.time.stamp_ns &&
      context.map_pose.clock==context.time.clock && asmc::lidar::pose_valid(context.map_pose.map_from_base);
  if(lidar_processed) {
    in.context_current=context.input_generation==generation;
    in.leveling_valid=asmc::lidar::base_from_input(context).has_value();
  }
  return in;
}
void FillMotionV2(const LiDARObjectInfo& o,asmc_msgs::LidarObjectV2* out) {
  const int state=o.s32_MotionState;
  const bool valid=o.motion_valid && state>=static_cast<int>(MotionState::LowThreat) &&
      state<=static_cast<int>(MotionState::Dynamic);
  out->motion_valid=valid;
  out->motion_state=valid?static_cast<std::uint8_t>(state):static_cast<std::uint8_t>(ObjectV2::MOTION_UNKNOWN);
  out->dynamic_valid=valid;
  out->is_dynamic=static_cast<int>(out->motion_state)==static_cast<int>(ObjectV2::MOTION_DYNAMIC);
}
V2Frame BuildV2(const LiDAR& lidar,output_gate::Mode mode) {
  V2Frame out; out.mode=mode;
  if(mode==output_gate::Mode::kNone) {out.reason="v2_mode_none"; return out;}
  const auto& context=lidar.frame_context;
  if(const auto error=asmc::lidar::validate_time(context.time); !error.empty()) {
    out.reason="time_invalid:"+error; return out;
  }
  // Checked before the object loop so that an empty scene is OBSERVED_EMPTY only with valid inputs.
  if(!asmc::lidar::base_from_input(context)) {out.reason="leveling_transform_invalid"; return out;}
  const bool map_mode=mode==output_gate::Mode::kMap;
  if(map_mode && !context.map_pose.valid) {out.reason="map_pose_invalid:"+context.map_pose.reason; return out;}
  if(map_mode && !(context.map_pose.query_stamp_ns==context.time.stamp_ns &&
      context.map_pose.clock==context.time.clock && asmc::lidar::pose_valid(context.map_pose.map_from_base))) {
    out.reason="map_pose_time_mismatch"; return out;
  }
  auto geometry_context=context;
  if(!map_mode) geometry_context.map_pose.valid=false;     // local: map geometry is never computed
  ArrayV2 array; array.header.stamp=ros::Time().fromNSec(context.time.stamp_ns);
  array.header.frame_id=map_mode?"map":"base_link";
  array.status=ArrayV2::OBSERVED_EMPTY; array.reason="observed_empty";
  array.stamp_provenance=context.time.stamp_provenance;
  for(const auto& original:lidar.vec_Objects) {
    if(!Publishable(original)) continue;
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
    if(!result.valid) {out.reason=result.reason; return out;}
    if(map_mode && !result.map) {out.reason="map_transform_invalid:"+result.map_reason; return out;}
    array.objects.push_back(MessageV2(original,map_mode?*result.map:result.base));
  }
  if(!array.objects.empty()) {array.status=ArrayV2::OBSERVED; array.reason=map_mode?context.map_pose.reason:"observed";}
  out.array=std::move(array); out.valid=true; out.reason="valid"; return out;
}
Publisher::Publisher(ros::NodeHandle& node) {
  if(!Enabled()) return;
  leveling_tf_=std::make_unique<tf2_ros::TransformBroadcaster>();
  local_=node.advertise<Array>("/perception/lidar/objects_local",5);
  map_=node.advertise<Array>("/perception/lidar/objects_map",5);
  // Final LiDAR outputs (B0). /perception/lidar/objects_local|map above are compatibility
  // candidates; /lidar/object(_boxes), /lidar_tracking_markers and /lidar/debug_* are
  // visualization/debug topics, not final outputs.
  local_v2_=node.advertise<ArrayV2>("/lidar/local_object",5);
  map_v2_=node.advertise<ArrayV2>("/lidar/map_object",5);
  diagnostics_=node.advertise<diagnostic_msgs::DiagnosticArray>("/perception/lidar/diagnostics",2);
  timer_=node.createWallTimer(ros::WallDuration(1.),&Publisher::Diagnostics,this);
}
void Publisher::SetLegacyReason(const std::string& reason) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);legacy_reason_=reason;
}
void Publisher::ReportTotals() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if(!Enabled()) return;
  // Timer diagnostics can miss the last frames at bag end. This shutdown log
  // is accounting only: it publishes neither a new observation nor a stamp.
  // rosconsole has already shut down after worker joins; flush directly.
  std::cout << "[LIDAR_FRAME_TOTAL] processed=" << processed_ << " invalid=" << invalid_
            << " stamp_errors=" << stamp_errors_ << " source_stamp_ns=" << last_stamp_.toNSec()
            << std::endl;
  std::cout << "[LIDAR_V2_TOTAL] local=" << v2_local_ << " map=" << v2_map_ << " none=" << v2_none_
            << " stamp_errors=" << v2_stamp_errors_ << std::endl;
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
  local.stamp_provenance=map.stamp_provenance=context.time.stamp_provenance;
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
output_gate::Decision Publisher::PublishV2(const LiDAR& lidar,const ros::Time& scan_stamp,
                                           const output_gate::Decision& decision) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if(!Enabled()) return decision;
  last_leveling_store_count_=LevelingStoreCount();
  auto effective=decision;
  V2Frame local_frame, map_frame;
  if(effective.v2!=output_gate::Mode::kNone) {
    // Own stamp guard: V1 Accept() has already consumed this stamp.
    const auto context_ns=lidar.frame_context.time.stamp_ns;
    if(scan_stamp.isZero() || (!last_v2_stamp_.isZero() && scan_stamp<=last_v2_stamp_)) {
      ++v2_stamp_errors_; effective=output_gate::Downgrade(decision,"v2_stamp_rejected");
    } else if(context_ns<=0 || static_cast<std::uint64_t>(context_ns)!=scan_stamp.toNSec()) {
      effective=output_gate::Downgrade(decision,"context_stamp_mismatch");
    } else {
      const bool wants_local=effective.v2==output_gate::Mode::kLocal ||
          effective.v2==output_gate::Mode::kBoth;
      const bool wants_map=effective.v2==output_gate::Mode::kMap ||
          effective.v2==output_gate::Mode::kBoth;
      if(wants_local) {
        local_frame=BuildV2(lidar,output_gate::Mode::kLocal);
        if(!local_frame.valid)
          effective=output_gate::Downgrade(decision,local_frame.reason);
      }
      if(effective.v2!=output_gate::Mode::kNone && wants_map) {
        map_frame=BuildV2(lidar,output_gate::Mode::kMap);
        if(!map_frame.valid) {
          if(local_frame.valid) {
            effective.v2=output_gate::Mode::kLocal;
            effective.legacy_permitted=false;
            effective.suppress_legacy_markers=true;
            effective.reason="local_only:"+map_frame.reason;
          } else {
            effective=output_gate::Downgrade(decision,map_frame.reason);
          }
        }
      }
    }
  }
  // Local is emitted for every valid scan. Map is additionally emitted only
  // when the pose for this exact scan is valid. kNone emits neither.
  if(effective.v2==output_gate::Mode::kBoth) {
    local_v2_.publish(local_frame.array); map_v2_.publish(map_frame.array);
    last_v2_stamp_=scan_stamp; ++v2_local_; ++v2_map_;
  } else if(effective.v2==output_gate::Mode::kMap) {
    map_v2_.publish(map_frame.array); last_v2_stamp_=scan_stamp; ++v2_map_;
  } else if(effective.v2==output_gate::Mode::kLocal) {
    local_v2_.publish(local_frame.array); last_v2_stamp_=scan_stamp; ++v2_local_;
  } else ++v2_none_;
  v2_mode_=output_gate::ModeName(effective.v2); v2_reason_=effective.reason;
  gps_state_=output_gate::GpsStateName(effective.gps);
  return effective;
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
  value("source_clock",last_context_.time.clock);
  value("receipt_clock",last_context_.time.receipt_clock.empty()?last_context_.time.clock:last_context_.time.receipt_clock);
  value("receipt_ns",std::to_string(last_context_.time.receipt_ns));
  value("processing_ns",std::to_string(last_context_.time.processing_ns));
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
  value("v2_output",v2_mode_); value("v2_reason",v2_reason_); value("gps_state",gps_state_);
  value("v2_local_frames",std::to_string(v2_local_)); value("v2_map_frames",std::to_string(v2_map_));
  value("v2_none_frames",std::to_string(v2_none_)); value("v2_stamp_errors",std::to_string(v2_stamp_errors_));
  value("v2_local_subscribers",std::to_string(local_v2_.getNumSubscribers()));
  value("v2_map_subscribers",std::to_string(map_v2_.getNumSubscribers()));
  value("leveling_store_count",std::to_string(last_leveling_store_count_));
  array.status.push_back(status); diagnostics_.publish(array);
}
}  // namespace frame_boundary
