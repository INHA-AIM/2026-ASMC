#include <asmc_lidar/core.hpp>
#include <asmc_lidar/general.hpp>
#include <asmc_lidar/ground_live.hpp>
#include <patchwork/patchworkpp.h>
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace asmc_lidar {
namespace {
using Clock = std::chrono::steady_clock;
double Ms(Clock::time_point t) {
  return std::chrono::duration<double,std::milli>(Clock::now()-t).count();
}
bool Finite(const pcl::PointXYZI& p) {
  return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&std::isfinite(p.intensity);
}
void Finish(Cloud* cloud) {
  cloud->width=static_cast<std::uint32_t>(cloud->size()); cloud->height=1;
  cloud->is_dense=false;
}
Cloud::Ptr BaseCloud(const Cloud& points,const Eigen::Isometry3d& t,
                     const Eigen::Matrix3d& leveling=Eigen::Matrix3d::Identity()) {
  Cloud::Ptr out(new Cloud); out->reserve(points.size());
  for(const auto& p:points.points) {
    const auto b=t*(leveling.transpose()*Eigen::Vector3d(p.x,p.y,p.z));
    pcl::PointXYZI q; q.x=float(b.x());q.y=float(b.y());q.z=float(b.z());q.intensity=p.intensity;
    out->push_back(q);
  }
  Finish(out.get()); return out;
}
Eigen::Matrix3d Rotation(const std::array<float,9>& a) {
  Eigen::Matrix3d r;
  for(int row=0;row<3;++row)for(int col=0;col<3;++col)r(row,col)=a[3*row+col];
  return r;
}
Cloud::Ptr Roi(const Cloud& raw,const Calibration& calibration,const tunnel_filter::Params& params,std::vector<int>* originals) {
  Cloud::Ptr out(new Cloud); out->reserve(raw.size()); originals->clear();
  for(std::size_t i=0;i<raw.size();++i) {
    const auto& p=raw[i];
    if(!Finite(p)) continue;
    const double range=std::sqrt(double(p.x)*p.x+double(p.y)*p.y+double(p.z)*p.z);
    if(range<.5||range>100||p.x< -30||p.x>50||p.y< -20||p.y>20||p.z< -2||p.z>3) continue;
    if(params.dynamic_roi_enable&&(p.x<params.roi_min_x_m||p.x>params.roi_max_x_m||p.y<params.roi_min_y_m||p.y>params.roi_max_y_m||p.z<params.roi_min_z_m||p.z>params.roi_max_z_m))continue;
    const auto b=calibration.base_from_sensor*Eigen::Vector3d(p.x,p.y,p.z);
    if(b.x()>=-.700&&b.x()<=3.845&&b.y()>=-.946&&b.y()<=.946) continue;
    originals->push_back(int(i));out->push_back(p);
  }
  Finish(out.get());return out;
}
std::vector<std::uint8_t> MissingCameraMask(const Cloud& cloud,const Calibration& c) {
  std::vector<std::uint8_t> mask(cloud.size(),0);
  Eigen::Isometry3d base_from_camera=Eigen::Isometry3d::Identity();
  base_from_camera.linear()=asmc::lidar_eval::rpy(0,2*M_PI/180,0);
  base_from_camera.translation()=Eigen::Vector3d(1.9,0,1.2);
  const auto camera_from_sensor=base_from_camera.inverse()*c.base_from_sensor;
  for(std::size_t i=0;i<cloud.size();++i) {
    const auto& p=cloud[i];
    const auto a=camera_from_sensor*Eigen::Vector3d(p.x,p.y,p.z);
    const double depth=a.x();if(depth<=.1)continue;
    const double u=640*(-a.y())/depth+640,v=640*(-a.z())/depth+360;
    const double range=std::sqrt(double(p.x)*p.x+double(p.y)*p.y+double(p.z)*p.z);
    if(u>=0&&u<1280&&v>=0&&v<720&&range<=30)mask[i]=1;
  }
  return mask;
}
}  // namespace

GroundPrepared PrepareGeneralGround(const Cloud& raw,const Calibration& calibration,float pitch) {
  return PrepareGeneralGround(raw,calibration,0.f,pitch,false);
}

GroundPrepared PrepareGeneralGround(const Cloud& raw,const Calibration& calibration,
                                    float roll,float pitch,bool level_roll) {
  if(!std::isfinite(roll)||!std::isfinite(pitch))throw std::invalid_argument("invalid roll/pitch");
  const auto started=Clock::now();GroundPrepared out;std::vector<int> originals;
  const auto roi=Roi(raw,calibration,tunnel_filter::Params{false},&originals);
  out.raw_to_voxel.assign(raw.size(),-1);
  if(!roi->empty()) {
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(roi);vg.setLeafSize(.1f,.1f,.1f);vg.setSaveLeafLayout(true);
    vg.filter(*out.voxel);
    for(std::size_t i=0;i<roi->size();++i)
      out.raw_to_voxel[originals[i]]=vg.getCentroidIndex(roi->at(i));
  }
  out.leveling=tunnel_filter::LevelingRotation(roll,pitch,level_roll,true);
  tunnel_filter::ApplyRotation(*out.voxel,out.leveling,out.leveled.get());
  out.prepare_ms=Ms(started);return out;
}

struct Processor::Impl {
  Mode mode;
  Calibration calibration;
  tunnel_filter::Params params;
  tunnel_filter::TunnelPipeline reference;
  patchwork::Params ground_params;
  std::unique_ptr<patchwork::PatchWorkpp> patchwork;
  asmc::lidar_eval::StampGuard stamps;
  std::int64_t last_stamp = 0;
  Impl(Mode m,Calibration c,tunnel_filter::Params p,double height)
      :mode(m),calibration(std::move(c)),params(p),reference(params,PinnedLShapeObb()) {
    if(!asmc::lidar_eval::pose_valid(calibration.base_from_sensor)||calibration.source.empty())
      throw std::invalid_argument("invalid sensor calibration");
    if(!(height>0)||!std::isfinite(height))throw std::invalid_argument("invalid sensor height");
    ground_params.sensor_height=height;ground_params.min_range=1;ground_params.max_range=60;
    ground_params.enable_RNR=false;
    if(mode==Mode::GroundReview)patchwork.reset(new patchwork::PatchWorkpp(ground_params));
  }
};

Processor::Processor(Mode m,Calibration c,tunnel_filter::Params p,double h)
    :impl_(new Impl(m,std::move(c),p,h)) {}
Processor::~Processor()=default;
void Processor::Reset() {
  impl_->reference.Reset();
  impl_->stamps=asmc::lidar_eval::StampGuard();
  impl_->last_stamp=0;
  if(impl_->patchwork)impl_->patchwork.reset(new patchwork::PatchWorkpp(impl_->ground_params));
}

FrameOutput Processor::Process(const FrameInput& input) {
  FrameOutput out;out.stamp_ns=input.stamp_ns;
  out.camera_state=input.camera_available?"valid_empty_or_positive_supplied":"missing";
  if(!input.cloud) {out.status="cloud_missing";return out;}
  out.raw_points=input.cloud->size();
  asmc::lidar_eval::Observation t;
  t.stamp_ns=input.stamp_ns;t.receipt_ns=input.receipt_ns;t.processing_ns=input.receipt_ns;
  t.clock=input.clock;t.stamp_provenance=input.stamp_provenance;
  if(!(input.allow_repeated_snapshot && input.stamp_ns==impl_->last_stamp)) {
    if(const auto e=impl_->stamps.accept(t);!e.empty()) {out.status=e;return out;}
    impl_->last_stamp=input.stamp_ns;
  }
  if(!input.imu_valid||!std::isfinite(input.roll_rad)||!std::isfinite(input.pitch_rad)) {
    out.status="imu_missing_or_invalid";return out;
  }
  if(input.cloud->empty()) {out.status="observed_empty";return out;}
  const auto start=Clock::now();
  std::vector<int> roi_original;
  Cloud::Ptr roi;
  if(input.already_preprocessed) {
    roi.reset(new Cloud(*input.cloud));
    for(std::size_t i=0;i<roi->size();++i)roi_original.push_back(int(i));
  } else roi=Roi(*input.cloud,impl_->calibration,input.profile==ReferenceProfile::Tunnel?impl_->params:tunnel_filter::Params{false},&roi_original);
  out.roi_points=roi->size();
  if(roi->empty()) {out.status="roi_empty";return out;}
  Cloud::Ptr voxel(new Cloud);
  out.raw_to_voxel.assign(input.cloud->size(),-1);
  if(input.already_preprocessed) {
    *voxel=*roi;
    for(std::size_t j=0;j<roi_original.size();++j)out.raw_to_voxel[roi_original[j]]=int(j);
  } else {
    pcl::VoxelGrid<pcl::PointXYZI> vg;vg.setInputCloud(roi);vg.setLeafSize(.1f,.1f,.1f);
    vg.setSaveLeafLayout(true);vg.filter(*voxel);
    for(std::size_t j=0;j<roi->size();++j)
      out.raw_to_voxel[roi_original[j]]=vg.getCentroidIndex((*roi)[j]);
  }
  out.voxel_points=voxel->size();out.preprocessing_ms=Ms(start);
  std::uint64_t digest=14695981039346656037ULL;
  for(const auto& p:voxel->points)for(float v:{p.x,p.y,p.z,p.intensity}) {
    const auto* bytes=reinterpret_cast<const unsigned char*>(&v);
    for(std::size_t k=0;k<sizeof(float);++k) {digest^=bytes[k];digest*=1099511628211ULL;}
  }
  std::ostringstream hash;hash<<std::hex<<std::setw(16)<<std::setfill('0')<<digest;
  out.voxel_fnv1a64=hash.str();
  if(voxel->empty()) {out.status="voxel_empty";return out;}
  out.input_base=BaseCloud(*input.cloud,impl_->calibration.base_from_sensor);
  std::vector<std::uint8_t> protect(voxel->size(),tunnel_filter::kProtectNone);
  if(!input.protected_classes.empty()) {
    if(input.protected_classes.size()!=voxel->size()) {out.status="protection_size_mismatch";return out;}
    protect=input.protected_classes;
  }
  std::vector<std::uint8_t> ambiguous;
  if(input.ambiguous.empty())ambiguous=MissingCameraMask(*voxel,impl_->calibration);
  else {
    if(input.ambiguous.size()!=voxel->size()) {out.status="ambiguity_size_mismatch";return out;}
    ambiguous=input.ambiguous;
  }
  const auto ref_start=Clock::now();
  tunnel_filter::FrameResult general_result;
  const tunnel_filter::FrameResult* result_ptr=nullptr;
  if(input.profile==ReferenceProfile::Tunnel)
    result_ptr=&impl_->reference.Process(*voxel,float(input.roll_rad),float(input.pitch_rad),
        &protect,&ambiguous,input.camera_available,&input.motion);
  else {
    general_result=ProcessGeneralReference(*voxel,float(input.pitch_rad),PinnedLShapeObb());
    result_ptr=&general_result;
  }
  const auto& r=*result_ptr;
  out.reference_ms=Ms(ref_start);out.reference_labels=r.labels;
  const auto level=Rotation(r.leveling);
  for(const auto& obj:r.objects)if(!obj.flags.removed) {
    auto copy=obj;
    const Eigen::Vector3d center(obj.obb.cx,obj.obb.cy,obj.obb.cz);
    const auto base=impl_->calibration.base_from_sensor*(level.transpose()*center);
    copy.obb.cx=float(base.x());copy.obb.cy=float(base.y());copy.obb.cz=float(base.z());
    for(auto& corner:copy.obb.corners) {
      const auto p=impl_->calibration.base_from_sensor*(level.transpose()*Eigen::Vector3d(corner.x,corner.y,corner.z));
      corner=pcl::PointXYZ(float(p.x()),float(p.y()),float(p.z()));
    }
    out.reference_objects_base.push_back(copy);
  }
  if(!impl_->patchwork) {
    Cloud ground,nonground,other;
    for(std::size_t i=0;i<r.labels.size();++i) {
      if(r.labels[i]==tunnel_filter::kGround)ground.push_back(r.leveled->at(i));
      else if(r.labels[i]==tunnel_filter::kKept)nonground.push_back(r.leveled->at(i));
      else other.push_back(r.leveled->at(i));
    }
    out.ground_base=BaseCloud(ground,impl_->calibration.base_from_sensor,level);
    out.nonground_base=BaseCloud(nonground,impl_->calibration.base_from_sensor,level);
    out.uncertain_base=BaseCloud(other,impl_->calibration.base_from_sensor,level);
  }
  if(impl_->patchwork) {
    auto& g=out.patchwork;
    g.by_original.assign(input.cloud->size(),PointClass::Unknown);
    g.roi_to_original=roi_original;
    Cloud leveled;
    const auto level_matrix=tunnel_filter::LevelingRotation(float(input.roll_rad),float(input.pitch_rad),true,true);
    tunnel_filter::ApplyRotation(*roi,level_matrix,&leveled);
    const auto copy_start=Clock::now();
    Eigen::MatrixXf matrix(leveled.size(),4);
    for(std::size_t i=0;i<leveled.size();++i)
      matrix.row(Eigen::Index(i))<<leveled[i].x,leveled[i].y,leveled[i].z,leveled[i].intensity;
    g.input_copy_ms=Ms(copy_start);
    const auto segment_start=Clock::now();
    impl_->patchwork->estimateGround(matrix);
    g.segmentation_ms=Ms(segment_start);
    const auto restore_start=Clock::now();
    std::vector<std::uint8_t> seen(roi->size(),0);
    auto restore=[&](const Eigen::VectorXi& indices,PointClass label,Cloud* cloud) {
      for(Eigen::Index j=0;j<indices.size();++j) {
        const int k=indices(j);
        if(k<0||std::size_t(k)>=roi->size()||seen[k]++)throw std::runtime_error("Patchwork++ index out of bounds or duplicated");
        const int original=roi_original[std::size_t(k)];
        g.by_original[std::size_t(original)]=label;
        cloud->push_back((*roi)[std::size_t(k)]);
      }
    };
    restore(impl_->patchwork->getGroundIndices(),PointClass::Ground,g.ground.get());
    restore(impl_->patchwork->getNongroundIndices(),PointClass::Nonground,g.nonground.get());
    for(std::size_t i=0;i<roi->size();++i)if(!seen[i])g.uncertain->push_back((*roi)[i]);
    for(auto* cloud:{g.ground.get(),g.nonground.get(),g.uncertain.get()})Finish(cloud);
    g.ground=BaseCloud(*g.ground,impl_->calibration.base_from_sensor);
    g.nonground=BaseCloud(*g.nonground,impl_->calibration.base_from_sensor);
    g.uncertain=BaseCloud(*g.uncertain,impl_->calibration.base_from_sensor);
    out.ground_base=g.ground;out.nonground_base=g.nonground;out.uncertain_base=g.uncertain;
    g.restore_ms=Ms(restore_start);
    g.reason=g.uncertain->empty()?"indexed_complete":"unassigned_indices";
  }
  out.status="valid";return out;
}

Calibration LoadCalibration(const std::string& path) {
  const auto root=YAML::LoadFile(path);Calibration c;
  const auto p=root["sensor_pose_base_link"];
  if(!p)throw std::runtime_error("missing sensor_pose_base_link");
  c.base_from_sensor.linear()=asmc::lidar_eval::rpy(p["roll_deg"].as<double>()*M_PI/180,
      p["pitch_deg"].as<double>()*M_PI/180,p["yaw_deg"].as<double>()*M_PI/180);
  c.base_from_sensor.translation()=Eigen::Vector3d(p["x_m"].as<double>(),p["y_m"].as<double>(),p["z_m"].as<double>());
  c.source=root["source"].as<std::string>();return c;
}

tunnel_filter::Params LoadReferenceParams(const std::string& path) {
  const auto n=YAML::LoadFile(path);tunnel_filter::Params p;
#define LOAD(name,key) if(n[#key])p.name=n[#key].as<decltype(p.name)>()
  LOAD(dynamic_roi_enable,tunnel_dynamic_roi_enable);
  LOAD(roi_min_x_m,tunnel_roi_min_x_m);LOAD(roi_max_x_m,tunnel_roi_max_x_m);
  LOAD(roi_min_y_m,tunnel_roi_min_y_m);LOAD(roi_max_y_m,tunnel_roi_max_y_m);
  LOAD(roi_min_z_m,tunnel_roi_min_z_m);LOAD(roi_max_z_m,tunnel_roi_max_z_m);
  LOAD(level_roll,tunnel_level_roll);LOAD(level_pitch,tunnel_level_pitch);
  LOAD(ceiling_max_z_lidar_m,tunnel_ceiling_max_z_lidar_m);
  LOAD(ground_refit_enable,tunnel_ground_refit_enable);
  LOAD(ground_remove_distance_m,tunnel_ground_remove_distance_m);
  LOAD(wall_search_min_abs_y_m,tunnel_wall_search_min_abs_y_m);
  LOAD(wall_remove_distance_m,tunnel_wall_remove_distance_m);
  LOAD(cluster_tolerance_m,tunnel_cluster_tolerance_m);
  LOAD(cluster_min_points,tunnel_cluster_min_points);
  LOAD(cluster_max_points,tunnel_cluster_max_points);
  LOAD(obb_filter_enable,tunnel_obb_filter_enable);
  LOAD(obb_remove_ground_residual,tunnel_obb_remove_ground_residual);
#undef LOAD
  return p;
}
}  // namespace asmc_lidar
