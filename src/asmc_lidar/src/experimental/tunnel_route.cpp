#include <asmc_lidar/experimental/tunnel_route.hpp>
#include <asmc_lidar/experimental/low_obstacle_evidence.hpp>
#include <asmc_lidar/detection/obb.hpp>
#include <asmc_lidar/geometry/scan_frame.hpp>
#include <asmc_lidar/geometry/projection.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <stdexcept>

namespace asmc::lidar::pipeline {
namespace {
// Same L-shape fitter as the tracker and the original MakeTrackingObbFunction.
void TrackingObbs(const CloudPtr& cloud, const std::vector<pcl::PointIndices>& clusters,
                  std::vector<tunnel_filter::Obb>* obbs) {
  obbs->assign(clusters.size(), tunnel_filter::Obb{});
  for (std::size_t c=0;c<clusters.size();++c) {
    Cloud points;
    for (int i:clusters[c].indices) points.push_back(cloud->points.at(i));
    obb_util::OBB2D xy;double zmin=0,zmax=0;std::array<obb_util::P3,8> corners{};
    obb_util::computeOBB3D_LShape(points,xy,zmin,zmax,corners);
    auto& b=obbs->at(c);b.cx=xy.cx;b.cy=xy.cy;b.cz=.5*(zmin+zmax);
    b.length=std::max(xy.w,xy.h);b.width=std::min(xy.w,xy.h);
    b.z_min=zmin;b.z_max=zmax;
    // Core Obb::length is always the long side; its yaw must follow that side.
    b.yaw=xy.yaw+(xy.h>xy.w?1.5707963267948966:0.);
    for (std::size_t k=0;k<8;++k)b.corners[k]=pcl::PointXYZ(corners[k].x,corners[k].y,corners[k].z);
  }
}
// Port of 8dde9b69 ComputeProtection: expanded 2D boxes and the closest 10%
// depth reference. Raw points use the validated sensor mount and camera K/T.
void ProtectionView(const Cloud& cloud, const VehicleSpec& vehicle, const TunnelCameraView* camera,
                const TunnelProtectionConfig& cfg, std::vector<std::uint8_t>& protect,
                std::vector<std::uint8_t>& ambiguous) {
  const auto n=cloud.size();protect.assign(n,tunnel_filter::kProtectNone);ambiguous.assign(n,0);
  std::vector<float> u(n,-1),v(n,-1),range(n,0);std::vector<bool> visible(n,false);
  Eigen::Matrix<double,3,4> P=Eigen::Matrix<double,3,4>::Zero();
  bool calibrated=camera && camera->calibrated;
  if(calibrated)P=camera->calibration.intrinsic *
      (camera->calibration.base_from_camera.inverse()*ScanFrame::SensorMount(vehicle)).matrix().topRows<3>();
  if(calibrated && (camera->common_projection || cfg.motion_compensated_fallback)) {
    Context sensor;sensor.frame=Frame::Sensor;sensor.base_from_sensor=ScanFrame::SensorMount(vehicle);
    sensor.calibration_source="sensor_mount";
    const auto canonical=camera_projection(sensor,camera->calibration.base_from_camera,camera->calibration.intrinsic);
    if(canonical)P=(*canonical)*camera->image_sensor_from_scan_sensor.matrix();else calibrated=false;
  }
  const double p00=P(0,0),p01=P(0,1),p02=P(0,2),p03=P(0,3);
  const double p10=P(1,0),p11=P(1,1),p12=P(1,2),p13=P(1,3);
  const double p20=P(2,0),p21=P(2,1),p22=P(2,2),p23=P(2,3);
  for(std::size_t i=0;i<n;++i) {
    const auto& p=cloud.points[i];if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
    range[i]=std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
    if(!calibrated) { // No usable projection: conservatively protect nearby wall candidates.
      ambiguous[i]=range[i]<=cfg.ambiguous_max_range_m;continue;
    }
    const double depth=p20*p.x+p21*p.y+p22*p.z+p23;if(depth<=.1)continue;
    u[i]=(p00*p.x+p01*p.y+p02*p.z+p03)/depth;
    v[i]=(p10*p.x+p11*p.y+p12*p.z+p13)/depth;
    visible[i]=u[i]>=0&&u[i]<camera->calibration.width&&v[i]>=0&&v[i]<camera->calibration.height;
    ambiguous[i]=visible[i]&&range[i]<=cfg.ambiguous_max_range_m;
  }
  if(!cfg.enabled || !calibrated || !camera->available || (camera->still_current&&!camera->still_current()))return;
  const float width=camera->calibration.width,height=camera->calibration.height;
  for(const auto& box:camera->boxes) {
    auto kind=tunnel_filter::ProtectClassFromLabel(box.source_class_name);
    if(kind==tunnel_filter::kProtectNone)kind=tunnel_filter::ProtectClassFromLabel(box.class_name);
    if(kind==tunnel_filter::kProtectNone || (!camera->common_projection && kind>=tunnel_filter::kProtectDog))continue;
    const float w=box.xmax-box.xmin,h=box.ymax-box.ymin;
    if(!std::isfinite(w)||!std::isfinite(h)||w<=0||h<=0)continue;
    const bool vehicle_class=kind==tunnel_filter::kProtectCar||kind==tunnel_filter::kProtectTruck||kind==tunnel_filter::kProtectBus;
    if(vehicle_class&&w>=width*.60f&&.5f*(box.ymin+box.ymax)>=height*.85f&&box.ymax>=height*.95f)continue;
    const float mx=cfg.box_margin_px+cfg.box_margin_ratio*w,my=cfg.box_margin_px+cfg.box_margin_ratio*h;
    std::vector<std::pair<float,std::size_t>> inside;
    for(std::size_t i=0;i<n;++i)if(visible[i]&&u[i]>=box.xmin-mx&&u[i]<=box.xmax+mx&&v[i]>=box.ymin-my&&v[i]<=box.ymax+my)inside.emplace_back(range[i],i);
    if(inside.size()<static_cast<std::size_t>(cfg.min_box_points))continue;
    const auto ref_index=static_cast<std::size_t>(std::floor(cfg.depth_ref_percentile*(inside.size()-1)));
    std::nth_element(inside.begin(),inside.begin()+ref_index,inside.end());
    const float ref=inside[ref_index].first;
    const float extent=tunnel_filter::IsPersonClass(kind)?cfg.depth_person_m:cfg.depth_vehicle_m;
    for(const auto& q:inside)if(q.first>=ref-cfg.depth_front_margin_m&&q.first<=ref+extent&&protect[q.second]==tunnel_filter::kProtectNone)protect[q.second]=kind;
  }
  if(camera->still_current&&!camera->still_current())
    std::fill(protect.begin(),protect.end(),tunnel_filter::kProtectNone);
}
// Union of per-view protections, without consuming post-tracking observations.
void Protection(const Cloud& cloud,const VehicleSpec& vehicle,const TunnelCameraInput* camera,
                const TunnelProtectionConfig& cfg,std::vector<std::uint8_t>& protect,
                std::vector<std::uint8_t>& ambiguous) {
  if(!camera||camera->views.empty()){ProtectionView(cloud,vehicle,camera,cfg,protect,ambiguous);return;}
  protect.assign(cloud.size(),tunnel_filter::kProtectNone);ambiguous.assign(cloud.size(),0);
  std::vector<std::uint8_t> covered(cloud.size(),0);
  for(const auto& view:camera->views) {
    std::vector<std::uint8_t> p,a;ProtectionView(cloud,vehicle,&view,cfg,p,a);
    const bool current=view.available&&view.calibrated&&(!view.still_current||view.still_current());
    for(std::size_t i=0;i<p.size();++i) {
      if(protect[i]==tunnel_filter::kProtectNone)protect[i]=p[i];
      // Availability is per view: a missing camera cannot turn the complete
      // union into unavailable, nor can another view certify unseen points.
      if(current)covered[i]|=a[i];
      else ambiguous[i]|=a[i];
    }
  }
  for(std::size_t i=0;i<ambiguous.size();++i)ambiguous[i]&=!covered[i];
}

}
void ValidateTunnelWall(const TunnelWallConfig& c) {
  if(!std::isfinite(c.fit_distance_m) || !std::isfinite(c.remove_distance_m) ||
     !std::isfinite(c.track_max_offset_m) || c.fit_distance_m<.01f || c.fit_distance_m>1.f ||
     c.remove_distance_m<.01f || c.remove_distance_m>2.f || c.track_max_offset_m<.01f || c.track_max_offset_m>2.f)
    throw std::invalid_argument("wall distances: fit [0.01,1], remove/track offset [0.01,2] meters");
}
void TunnelRoute::SetWallConfig(const TunnelWallConfig& c) {
  auto params=config_.params;ApplyTunnelWall(params,c);
  core_=tunnel_filter::TunnelPipeline(params,TrackingObbs);
  config_.params=params;anchor_valid_=false;
}
void ValidateTunnelClusterZ(const TunnelClusterZConfig& config) {
  if(!std::isfinite(config.min_sensor_z_m) || config.min_sensor_z_m < -5.f || config.min_sensor_z_m > 0.f)
    throw std::invalid_argument("tunnel cluster center Z must be finite and within [-5,0] sensor meters");
}
TunnelRoute::TunnelRoute(const TunnelRouteConfig& config):config_(config),core_(config.params,TrackingObbs) {
  ValidateTunnelClusterZ(config_.cluster_z);
  ValidateTunnelWall(TunnelWallFromParams(config_.params));
  const auto& p=config_.params;const auto& c=config_.protection;
  if(!std::isfinite(p.roi_min_x_m) ||
     !std::isfinite(p.roi_max_x_m) ||
     !std::isfinite(p.roi_min_y_m) ||
     !std::isfinite(p.roi_max_y_m) ||
     !std::isfinite(p.roi_min_z_m) ||
     !std::isfinite(p.roi_max_z_m) ||
     !std::isfinite(p.ceiling_max_z_lidar_m) ||
     !std::isfinite(p.pre_gps_structural_obb_min_length_m) ||
     !std::isfinite(p.pre_gps_structural_obb_min_width_m) ||
     !std::isfinite(p.pre_gps_structural_obb_min_height_m) ||
     !std::isfinite(p.expected_ground_z_lidar_m) ||
     !std::isfinite(p.ground_candidate_z_min_m) ||
     !std::isfinite(p.ground_candidate_z_max_m) ||
     !std::isfinite(p.ground_candidate_max_range_m) ||
     !std::isfinite(p.ground_prior_band_m) ||
     !std::isfinite(p.ground_mode_bin_m) ||
     !std::isfinite(p.ground_mode_max_range_m) ||
     !std::isfinite(p.ground_ransac_distance_threshold_m) ||
     !std::isfinite(p.ground_refit_band_m) ||
     !std::isfinite(p.ground_refit_distance_m) ||
     !std::isfinite(p.ground_refit_max_angle_deg) ||
     !std::isfinite(p.ground_refit_max_height_delta_m) ||
     !std::isfinite(p.ground_normal_max_angle_deg) ||
     !std::isfinite(p.ground_height_min_m) ||
     !std::isfinite(p.ground_height_max_m) ||
     !std::isfinite(p.ground_min_inlier_ratio) ||
     !std::isfinite(p.ground_max_height_jump_m) ||
     !std::isfinite(p.ground_max_normal_jump_deg) ||
     !std::isfinite(p.ground_remove_distance_m) ||
     !std::isfinite(p.ground_fallback_band_m) ||
     !std::isfinite(p.wall_search_min_abs_y_m) ||
     !std::isfinite(p.wall_search_x_min_m) ||
     !std::isfinite(p.wall_search_x_max_m) ||
     !std::isfinite(p.wall_candidate_min_height_m) ||
     !std::isfinite(p.wall_candidate_max_height_m) ||
     !std::isfinite(p.wall_fit_distance_m) ||
     !std::isfinite(p.wall_max_tilt_deg) ||
     !std::isfinite(p.wall_min_inlier_ratio) ||
     !std::isfinite(p.wall_min_span_m) ||
     !std::isfinite(p.wall_bin_size_m) ||
     !std::isfinite(p.wall_min_vertical_span_m) ||
     !std::isfinite(p.wall_min_lateral_distance_m) ||
     !std::isfinite(p.wall_beyond_margin_m) ||
     !std::isfinite(p.wall_max_beyond_fraction) ||
     !std::isfinite(p.wall_extend_min_run_m) ||
     !std::isfinite(p.wall_extend_max_gap_m) ||
     !std::isfinite(p.wall_step_max_angle_deg) ||
     !std::isfinite(p.wall_step_max_offset_m) ||
     !std::isfinite(p.wall_step_min_vertical_span_m) ||
     !std::isfinite(p.wall_continuation_max_angle_deg) ||
     !std::isfinite(p.wall_continuation_max_gap_m) ||
     !std::isfinite(p.wall_continuation_min_span_m) ||
     !std::isfinite(p.wall_continuation_search_m) ||
     !std::isfinite(p.wall_track_max_angle_deg) ||
     !std::isfinite(p.wall_track_max_offset_m) ||
     !std::isfinite(p.wall_track_motion_max_dt_s) ||
     !std::isfinite(p.wall_remove_distance_m) ||
     !std::isfinite(p.wall_remove_margin_m) ||
     !std::isfinite(p.wall_remove_min_height_m) ||
     !std::isfinite(p.cluster_tolerance_m) ||
     !std::isfinite(p.obb_ceiling_min_z_road_m) ||
     !std::isfinite(p.obb_wall_min_abs_y_m) ||
     !std::isfinite(p.obb_wall_min_length_m) ||
     !std::isfinite(p.obb_wall_min_aspect_ratio) ||
     !std::isfinite(p.obb_wall_attach_distance_m) ||
     !std::isfinite(p.obb_wall_attach_max_angle_deg) ||
     !std::isfinite(p.obb_wall_attached_min_length_m) ||
     !std::isfinite(p.obb_ground_residual_max_height_m) ||
     !std::isfinite(p.obb_ground_residual_max_top_road_m) ||
     !std::isfinite(p.protect_obb_min_fraction) ||
     !std::isfinite(c.max_time_offset_s) ||
     !std::isfinite(c.box_margin_ratio) ||
     !std::isfinite(c.box_margin_px) ||
     !std::isfinite(c.depth_vehicle_m) ||
     !std::isfinite(c.depth_person_m) ||
     !std::isfinite(c.depth_front_margin_m) ||
     !std::isfinite(c.depth_ref_percentile) ||
     !std::isfinite(c.ambiguous_max_range_m))throw std::invalid_argument("non-finite tunnel parameter");
  if(p.roi_min_x_m>=p.roi_max_x_m||p.roi_min_y_m>=p.roi_max_y_m||p.roi_min_z_m>=p.roi_max_z_m||
     p.cluster_tolerance_m<=0||p.cluster_min_points<1||p.cluster_max_points<p.cluster_min_points||
     p.wall_bin_size_m<=0||p.ground_mode_bin_m<=0||p.wall_min_persist_frames<1||
     c.min_box_points<1||!std::isfinite(c.max_time_offset_s)||c.max_time_offset_s<0||c.max_time_offset_s>.05||
     c.depth_ref_percentile<0||c.depth_ref_percentile>1||c.depth_vehicle_m<=0||c.depth_person_m<=0||
     c.box_margin_ratio<0||c.box_margin_px<0||c.ambiguous_max_range_m<0)
    throw std::invalid_argument("invalid original tunnel parameters");
}
bool TunnelRoute::Select(bool map_pose_valid) {
  const bool next=!map_pose_valid;
  if(next!=active_) {core_.Reset();anchor_valid_=false;active_=next;}
  return active_;
}
void TunnelRoute::RestrictRoi(const CloudPtr& cloud) const {
  const auto& p=config_.params;if(!active_||!p.dynamic_roi_enable)return;
  auto& v=cloud->points;v.erase(std::remove_if(v.begin(),v.end(),[&](const auto& q){return q.x<p.roi_min_x_m||q.x>p.roi_max_x_m||q.y<p.roi_min_y_m||q.y>p.roi_max_y_m||q.z<p.roi_min_z_m||q.z>p.roi_max_z_m;}),v.end());
  cloud->width=v.size();cloud->height=1;
}
void TunnelRoute::EntranceCeiling(const CloudPtr& input,const CloudPtr& output,std::vector<int>& indices) const {
  indices.clear();output->clear();output->header=input->header;
  for(std::size_t i=0;i<input->size();++i)if(!config_.params.pre_gps_ceiling_filter_enable||input->points[i].z<=config_.params.ceiling_max_z_lidar_m) {
    output->push_back(input->points[i]);indices.push_back(i);
  }
}
bool TunnelRoute::EntranceStructure(const TrackedObject& object) const {
  const auto& d=object.detection;
  return tunnel_filter::IsPreGpsStructuralObb(d.obb_length_m,d.obb_width_m,d.f32_OBB_ZMax-d.f32_OBB_ZMin,config_.params);
}
// A boundary object must not be hidden merely because GPS is still invalid.
// Require the ego AND every footprint corner to lie well inside the current
// independently verified support interval on BOTH sides of the corridor.
bool TunnelRoute::InsideCurrentWalls(const TrackedObject& object,float margin) const {
  for(const auto& side:core_.LastResult().walls) {
    if(!side.removal_allowed)return false;
    auto covered=[&](float x,float y) {
      for(const auto& segment:side.segments)
        if(segment.valid&&segment.InIntervals(segment.AlongWall(x,y),-margin))return true;
      return false;
    };
    if(!covered(0,0))return false;
    for(const auto& q:object.detection.obb_corners_level)if(!covered(q.x,q.y))return false;
  }
  return true;
}
TunnelRouteOutput TunnelRoute::Process(const CloudPtr& voxel,const LocalState& local,std::int64_t stamp,
                                       const VehicleSpec& vehicle,const TunnelCameraInput* camera,const tunnel_filter::EgoMotion* integrated_motion,
                                       const CloudPtr& measured_vehicle_support,float vehicle_match_m) {
  if(!active_||!voxel)throw std::invalid_argument("inactive/null tunnel route");
  tunnel_filter::EgoMotion motion;
  if(!config_.integrated_motion && anchor_valid_&&local.st_IMU.b_ImuReady) {
    const float dt=static_cast<float>((stamp-anchor_stamp_)*1e-9);
    const float dyaw=std::atan2(std::sin(local.st_IMU.f32_Yaw_rad-anchor_yaw_),std::cos(local.st_IMU.f32_Yaw_rad-anchor_yaw_));
    const float distance=.5f*(local.st_CAN.f32_CurrentVel_MPS+anchor_speed_)*dt;
    if(dt>0&&std::isfinite(dyaw)&&std::isfinite(distance)) {
      motion.valid=true;motion.dt_s=dt;motion.dyaw_rad=dyaw;motion.dx_m=distance*std::cos(.5f*dyaw);motion.dy_m=distance*std::sin(.5f*dyaw);
    }
  }
  if(config_.integrated_motion)motion=integrated_motion?*integrated_motion:tunnel_filter::EgoMotion{};
  anchor_valid_=local.st_IMU.b_ImuReady;anchor_stamp_=stamp;anchor_yaw_=local.st_IMU.f32_Yaw_rad;anchor_speed_=local.st_CAN.f32_CurrentVel_MPS;
  std::vector<std::uint8_t> protect,ambiguous;
  TunnelCameraInput compensated;
  const TunnelCameraInput* protection_camera=camera;
  if(config_.protection.motion_compensated_fallback&&camera&&camera->low_obstacle_camera) {
    const auto* causal=camera->low_obstacle_camera.get();
    // The original protection enum cannot represent every mission class. Do
    // not turn off ambiguity protection around steel/low obstacles it cannot
    // protect; their separate evidence path remains authoritative.
    const bool representable=std::all_of(causal->boxes.begin(),causal->boxes.end(),[](const camera::Box& b) {
      return tunnel_filter::ProtectClassFromLabel(b.source_class_name)!=tunnel_filter::kProtectNone||
        tunnel_filter::ProtectClassFromLabel(b.class_name)!=tunnel_filter::kProtectNone;
    });
    if(representable&&causal->calibrated&&causal->available&&causal->stamp_ns>0&&causal->stamp_ns<=stamp&&
       stamp-causal->stamp_ns<=150000000LL&&causal->motion_compensated&&
       pose_valid(causal->image_sensor_from_scan_sensor)&&!causal->identity.empty()&&
       causal->image_width==static_cast<int>(causal->calibration.width)&&
       causal->image_height==static_cast<int>(causal->calibration.height)&&
       causal->still_current&&causal->still_current()){
        if(camera->views.empty()&&!camera->available)protection_camera=causal;
        else if(!camera->views.empty()) {
          // A front lease cannot replace side-view evidence or ambiguity.
          compensated=*camera;
          for(auto& view:compensated.views)if(!view.available&&view.calibration.frame_id==causal->calibration.frame_id)
            view=static_cast<const TunnelCameraView&>(*causal);
          protection_camera=&compensated;
        }
      }
  }
  const auto protection_start=std::chrono::steady_clock::now();
  Protection(*voxel,vehicle,protection_camera,config_.protection,protect,ambiguous);
  if(measured_vehicle_support&&!measured_vehicle_support->empty()) {
    pcl::search::KdTree<pcl::PointXYZI> tree;tree.setInputCloud(measured_vehicle_support);
    std::vector<int> near(1);std::vector<float> distance(1);
    for(std::size_t i=0;i<voxel->size();++i)
      if(tree.nearestKSearch(voxel->at(i),1,near,distance)==1&&distance[0]<=vehicle_match_m*vehicle_match_m)
        protect[i]=tunnel_filter::kProtectCar;
  }
  const double protection_ms=std::chrono::duration<double,std::milli>(
    std::chrono::steady_clock::now()-protection_start).count();
  const bool available=config_.protection.enabled&&protection_camera&&
    (protection_camera->views.empty() ? protection_camera->calibrated&&protection_camera->available&&
      (!protection_camera->still_current||protection_camera->still_current()) :
     std::all_of(protection_camera->views.begin(),protection_camera->views.end(),[](const auto& v){
       return v.calibrated&&v.available&&(!v.still_current||v.still_current());}));
  const auto& f=core_.Process(*voxel,local.st_IMU.f32_Roll_rad,local.st_IMU.f32_Pitch_rad,&protect,&ambiguous,available,&motion);
  TunnelRouteOutput out;out.leveled=f.leveled;out.kept_indices=f.kept_indices;
  const auto low_evidence=LowObstacleEvidence(*voxel,f,vehicle,stamp,camera,config_);
  for(int i:f.kept_indices)out.filtered->push_back(f.leveled->points[i]);
  std::vector<int> compact(f.leveled->size(),-1);
  for(std::size_t k=0;k<f.kept_indices.size();++k)compact[f.kept_indices[k]]=k;
  for(std::size_t i=0;i<f.labels.size();++i)if(f.labels[i]!=tunnel_filter::kGround&&f.labels[i]!=tunnel_filter::kInvalid)out.pre_filter_indices.push_back(i);
  std::vector<bool> z_removed(f.leveled->size(),false);
  std::vector<bool> structure_removed(f.leveled->size(),false);
  bool has_structure_removed=false;
  int structure_wall_points=0,structure_ground_points=0,structure_ceiling_points=0,structure_ground_objects=0;
  for(std::size_t c=0;c<f.clusters.size();++c) {
    const auto& flags=f.objects.at(c).flags;
    const bool low_verified=!low_evidence[c].source_class.empty();
    if(low_verified)++out.report.low_obstacle_verified;
    double sensor_z=0;
    for(int i:f.clusters[c].indices)sensor_z+=voxel->points.at(i).z;
    const bool below_z=config_.cluster_z.enabled && !f.clusters[c].indices.empty() &&
        sensor_z/f.clusters[c].indices.size()<config_.cluster_z.min_sensor_z_m;
    const bool rescued=low_verified && (below_z || (flags.removed&&flags.ground_residual));
    if(rescued) {
      ++out.report.low_obstacle_rescued;
      out.report.low_obstacle_rescued_points+=low_evidence[c].support.size();
      for(int i:f.clusters[c].indices)
        if(std::find(low_evidence[c].support.begin(),low_evidence[c].support.end(),i)==low_evidence[c].support.end())z_removed.at(i)=true;
      const auto& p=f.leveled->at(f.clusters[c].indices.front());
      out.report.low_obstacle_decisions.push_back({{p.x,p.y,p.z,static_cast<float>(sensor_z/f.clusters[c].indices.size()),
        static_cast<float>(low_evidence[c].support.size()),below_z?1.f:0.f,flags.ground_residual?1.f:0.f},low_evidence[c].source_class});
    }
    // Camera evidence may skip only these two low-height decisions. Wall,
    // ceiling and point-level ground rejection remain authoritative.
    if(flags.removed && !(low_verified&&flags.ground_residual&&!flags.wall&&!flags.ceiling)) {
      // The original OBB stage rejected only the box. In the residual-surface
      // experiment remove its point support as well, so the real filtered
      // cloud cannot still contain a rejected wall/ground component.
      if(config_.params.residual_wall_surface_enable) {
        has_structure_removed=true;
        for(int i:f.clusters[c].indices)structure_removed.at(i)=true;
        const auto& flags=f.objects.at(c).flags;
        const int points=f.clusters[c].indices.size();
        if(flags.ceiling)structure_ceiling_points+=points;
        else if(flags.wall)structure_wall_points+=points;
        else if(flags.ground_residual){structure_ground_points+=points;++structure_ground_objects;}
      }
      continue;
    }
    // Use the ORIGINAL sensor coordinates, not base_link height or the OBB
    // center. The same indices survive leveling without reordering.
    if(below_z && !low_verified) {
      ++out.report.cluster_z_removed;
      for(int i:f.clusters[c].indices)z_removed.at(i)=true;
      continue;
    }
    pcl::PointIndices cluster;pcl::PointXYZI center{};
    const auto& support=rescued?low_evidence[c].support:f.clusters[c].indices;
    for(int i:support) {if(compact.at(i)<0)throw std::logic_error("tunnel cluster not in kept cloud");cluster.indices.push_back(compact[i]);const auto& q=f.leveled->points[i];center.x+=q.x;center.y+=q.y;center.z+=q.z;}
    if(cluster.indices.empty())continue;
    center.x/=cluster.indices.size();center.y/=cluster.indices.size();center.z/=cluster.indices.size();
    out.clusters.push_back(std::move(cluster));out.centroids->push_back(center);
    out.protected_classes.push_back(f.objects[c].is_protected?tunnel_filter::ProtectClassName(f.objects[c].protect_class):"unknown");
    out.low_obstacle_cues.push_back(rescued?LowObstacleCue{stamp,low_evidence[c].camera_epoch,low_evidence[c].source_class}:LowObstacleCue{});
  }
  // Remove rejected cluster support from the real non-ground cloud and remap
  // all surviving compact indices together; ROI/voxel views remain unchanged.
  if(out.report.cluster_z_removed || has_structure_removed || out.report.low_obstacle_rescued) {
    std::vector<int> remap(out.filtered->size(),-1),kept;
    CloudPtr filtered(new Cloud);filtered->header=out.filtered->header;
    for(std::size_t k=0;k<out.kept_indices.size();++k) {
      const int source_index=out.kept_indices[k];
      if(z_removed.at(source_index) || structure_removed.at(source_index)) {
        if(z_removed.at(source_index))++out.report.cluster_z_removed_points;
        continue;
      }
      remap[k]=filtered->size();filtered->push_back(out.filtered->at(k));kept.push_back(out.kept_indices[k]);
    }
    for(auto& cluster:out.clusters)for(auto& i:cluster.indices)i=remap.at(i);
    out.filtered=filtered;out.kept_indices=std::move(kept);
  }
  // Restore OBB eligibility for small elevated compact support at a wall
  // boundary. These points already survive the original filters: no wall,
  // ground, Z or protection decision is reversed, and no point is added.
  if(config_.boundary_cluster_rescue && config_.params.cluster_min_points>5 &&
     f.walls[0].removal_allowed && f.walls[1].removal_allowed &&
     f.ground.source!=tunnel_filter::PlaneSource::kUnavailable) {
    std::vector<bool> used(out.filtered->size(),false);
    for(const auto& c:out.clusters)for(int i:c.indices)used.at(i)=true;
    CloudPtr sparse(new Cloud);std::vector<int> original;
    for(std::size_t i=0;i<out.filtered->size();++i)if(!used[i]){sparse->push_back(out.filtered->at(i));original.push_back(i);}
    if(sparse->size()>=5) {
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
      tree->setInputCloud(sparse);pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      ec.setInputCloud(sparse);ec.setSearchMethod(tree);ec.setClusterTolerance(config_.params.cluster_tolerance_m);
      ec.setMinClusterSize(5);ec.setMaxClusterSize(std::min(9,config_.params.cluster_min_points-1));
      std::vector<pcl::PointIndices> candidates;ec.extract(candidates);
      std::vector<tunnel_filter::Obb> obbs;TrackingObbs(sparse,candidates,&obbs);
      for(std::size_t k=0;k<candidates.size();++k) {
        const auto& box=obbs[k];const float height=box.z_max-box.z_min;
        const float road=f.ground.plane.HeightAt(box.cx,box.cy);
        if(!std::isfinite(road)||box.length<.3f||box.length>3.f||box.width<.15f||box.width>2.f||
           height<0||height>2.f||box.cz-road<.35f||box.z_max-road<.5f||std::hypot(box.cx,box.cy)>30.f)continue;
        bool near_wall=false;
        for(const auto& side:f.walls)for(const auto& wall:side.segments)
          if(wall.valid&&std::fabs(wall.plane.SignedDistance(box.cx,box.cy,box.cz))<=2.5f)near_wall=true;
        if(!near_wall)continue;
        TrackedObject bounds;
        for(std::size_t j=0;j<8;++j)bounds.detection.obb_corners_level[j]={box.corners[j].x,box.corners[j].y,box.corners[j].z};
        if(InsideCurrentWalls(bounds))continue;
        pcl::PointIndices cluster;pcl::PointXYZI center{};double sensor_z=0;
        for(int i:candidates[k].indices) {
          const int compact_index=original.at(i);const auto& q=out.filtered->at(compact_index);
          cluster.indices.push_back(compact_index);center.x+=q.x;center.y+=q.y;center.z+=q.z;
          sensor_z+=voxel->at(out.kept_indices.at(compact_index)).z;
        }
        const float count=cluster.indices.size();
        if(config_.cluster_z.enabled&&sensor_z/count<config_.cluster_z.min_sensor_z_m)continue;
        center.x/=count;center.y/=count;center.z/=count;
        out.report.boundary_rescued_points+=cluster.indices.size();++out.report.boundary_rescued_clusters;
        out.clusters.push_back(std::move(cluster));out.centroids->push_back(center);out.protected_classes.push_back("unknown");
        out.low_obstacle_cues.emplace_back();
      }
    }
  }
  out.report.clusters_before_filters=f.clusters.size()+out.report.boundary_rescued_clusters;
  out.report.clusters_after_filters=out.clusters.size();
  out.report.unclustered_points=out.filtered->size();
  for(const auto& c:out.clusters)out.report.unclustered_points-=c.indices.size();
  out.report.wall_fit_distance_m=config_.params.wall_fit_distance_m;
  out.report.wall_remove_distance_m=config_.params.wall_remove_distance_m;
  out.report.wall_track_max_offset_m=config_.params.wall_track_max_offset_m;
  out.report.motion_integrated=config_.integrated_motion;
  out.report.motion_valid=f.motion.valid;out.report.motion_reliable=f.motion.valid&&f.motion.reliable;
  out.report.motion_dt_s=f.motion.dt_s;out.report.motion_dyaw_rad=f.motion.dyaw_rad;
  out.report.motion_dx_m=f.motion.dx_m;out.report.motion_dy_m=f.motion.dy_m;
  out.report.motion_steps=f.motion.integration_steps;
  out.report.wall_left_reanchored=f.walls[0].gap_reanchored;
  out.report.wall_right_reanchored=f.walls[1].gap_reanchored;
  auto state=[](const auto& w) {
    return w.gap_reanchored?std::string("gap_reanchored"):!w.fit_valid?w.first_attempt.reject_reason:
        w.removal_allowed?std::string("confirmed"):std::string("confirming");
  };
  out.report.wall_left_state=state(f.walls[0]);out.report.wall_right_state=state(f.walls[1]);
  out.report.cluster_z_enabled=config_.cluster_z.enabled;
  out.report.cluster_z_min_sensor_m=config_.cluster_z.min_sensor_z_m;
  out.report.ground_fallback_used=f.ground_fallback_used;
  out.report.wall_left_removal_allowed=f.walls[0].removal_allowed;
  out.report.wall_right_removal_allowed=f.walls[1].removal_allowed;
  out.report.wall_ambiguous_skipped=f.walls[0].ambiguous_skipped+f.walls[1].ambiguous_skipped;
  // The original core rounds Ry(pitch)*Rx(roll) into float coefficients for
  // point processing. Lifting those rounded coefficients to double is not a
  // proper SO(3) rotation and fails the common geometry contract. Represent the
  // SAME rotation from the SAME selected angles in double; keep the original
  // float point processing and strict transform validation unchanged.
  out.leveling=asmc::lidar::rpy(config_.params.level_roll?local.st_IMU.f32_Roll_rad:0.,
                                config_.params.level_pitch?local.st_IMU.f32_Pitch_rad:0.,0.);
  out.plane.valid=f.ground.source!=tunnel_filter::PlaneSource::kUnavailable;
  out.plane.coefficients={f.ground.plane.a,f.ground.plane.b,f.ground.plane.c,f.ground.plane.d};
  out.plane.near_threshold_m=out.plane.far_threshold_m=config_.params.ground_remove_distance_m;
  for(std::size_t side=0;side<f.walls.size();++side) {
    const auto& w=f.walls[side];
    auto add=[&](const tunnel_filter::WallSegment& seg,bool allowed) {
      TunnelWallEvidence e;e.side=side;e.allowed=allowed;e.valid=seg.valid;
      e.plane={seg.plane.a,seg.plane.b,seg.plane.c,seg.plane.d};e.span=seg.span;
      e.intervals=seg.intervals;e.reason=seg.reject_reason;out.report.wall_evidence.push_back(e);
    };
    for(const auto& seg:w.segments)add(seg,w.removal_allowed);
    if(w.segments.empty())add(w.first_attempt,false);
  }
  out.report.exterior_surface_removed=f.exterior_surface_removed;
  for(const auto& decision:f.residual_decisions) {
    const auto& b=decision.first;
    out.report.residual_decisions.push_back({{b.cx,b.cy,b.cz,b.length,b.width,b.z_max-b.z_min,b.yaw},decision.second});
  }
  out.report.original_pipeline=true;out.report.protection_available=f.protection_available;
  out.report.protected_points=f.protected_points;out.report.objects_removed=f.objects_removed_ceiling+f.objects_removed_wall+structure_ground_objects;
  out.report.cluster_tolerance_m=config_.params.cluster_tolerance_m;
  out.report.ran=true;out.report.input_points=f.input_points;out.report.ceiling_labeled=f.ceiling_removed+structure_ceiling_points;
  out.report.wall_labeled=f.wall_removed+structure_wall_points;out.report.ground_labeled=f.ground_removed+structure_ground_points;
  out.report.removed_points=f.input_points-out.filtered->size();out.report.core_ms=f.timings.total_ms;
  out.report.residual_clusters_reused=f.residual_clusters_reused;
  const auto& t=f.timings;
  out.report.timings_ms={{"protection",protection_ms},{"level",t.level_ms},
    {"ceiling",t.ceiling_ms},{"ground_fit",t.ground_fit_ms},{"ground_remove",t.ground_remove_ms},
    {"wall_fit",t.wall_fit_ms},{"wall_remove",t.wall_remove_ms},
    {"residual_cluster",t.residual_cluster_ms},{"residual_obb",t.residual_obb_ms},
    {"cluster",t.cluster_ms},{"obb",t.obb_ms},{"obb_filter",t.obb_filter_ms},{"core",t.total_ms}};
  return out;
}
}  // namespace asmc::lidar::pipeline
