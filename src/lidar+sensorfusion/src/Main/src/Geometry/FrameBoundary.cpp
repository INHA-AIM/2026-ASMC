#include <Geometry/FrameBoundary.hpp>
#include <Lidar/Tracking/TrackTrace.hpp>
#include <SensorFusion/FusionTrace.hpp>
#include <ros/package.h>
#include <openssl/sha.h>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace frame_boundary {
namespace {
bool enabled=false, use_roll=false;
double altitude_origin=0.;
asmc::lidar::Vec map_origin=asmc::lidar::Vec::Zero();
std::string calibration_id;
asmc::lidar::Context current;
asmc::lidar::Rot scan_attitude=asmc::lidar::Rot::Identity();
double scan_yaw=0.;
unsigned leveling_stores=0;
std::string Sha256(const std::string& path) {
  std::ifstream stream(path,std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot read calibration source: "+path);
  SHA256_CTX state; SHA256_Init(&state);
  char buffer[8192];
  while (stream) {stream.read(buffer,sizeof(buffer)); SHA256_Update(&state,buffer,stream.gcount());}
  unsigned char digest[SHA256_DIGEST_LENGTH]; SHA256_Final(digest,&state);
  std::ostringstream out;
  for(auto value:digest) out<<std::hex<<std::setfill('0')<<std::setw(2)<<unsigned(value);
  return out.str();
}
void Near(double a,double b,const char* name) {
  if(!std::isfinite(a)||!std::isfinite(b)||std::abs(a-b)>1e-6)
    throw std::runtime_error(std::string("Calibration mismatch: ")+name);
}
}
bool Enabled(){return enabled;}
bool UseRoll(){return enabled && use_roll;}
double MapAltitudeOrigin(){return altitude_origin;}
bool MapOffsetsValid(double east,double north) {
  return std::isfinite(east) && std::isfinite(north) &&
      std::abs(east-map_origin.x())<=1e-6 && std::abs(north-map_origin.y())<=1e-6;
}
const std::string& CalibrationId(){return calibration_id;}
const asmc::lidar::Context& Current(){return current;}
void SetCurrent(const asmc::lidar::Context& context){current=context;}
asmc::lidar::Pose SensorMount(const VehicleSpec& vehicle) {
  asmc::lidar::Pose mount=asmc::lidar::Pose::Identity();
  mount.translation()=asmc::lidar::Vec(vehicle.st_LiDARPos.f32_X,vehicle.st_LiDARPos.f32_Y,vehicle.st_LiDARPos.f32_Z);
  mount.linear()=asmc::lidar::rpy(Deg2Rad(vehicle.st_LiDARRotationDeg.f32_X),
      Deg2Rad(vehicle.st_LiDARRotationDeg.f32_Y),Deg2Rad(vehicle.st_LiDARRotationDeg.f32_Z));
  return mount;
}
asmc::lidar::Rot MapRotation(const GPS_IMU& pose) {
  return asmc::lidar::rpy(pose.st_IMU.f32_Roll_rad,pose.st_IMU.f32_Pitch_rad,pose.st_IMU.f32_Yaw_rad);
}
void Configure(ros::NodeHandle& node,const VehicleSpec& vehicle,const CAMERA& camera) {
  node.param("enable_common_geometry",enabled,false);
  node.param("lidar_level_roll",use_roll,false);
  node.param("map_altitude_origin_m",altitude_origin,0.);
  if(!enabled) return;
  if(!std::isfinite(altitude_origin)) throw std::runtime_error("map altitude origin is not finite");
  std::string lidar_path=ros::package::getPath("asmc_lidar")+"/config/autonav_v3.yaml";
  std::string camera_path=ros::package::getPath("asmc_camera")+"/config/cameras_v3.yaml";
  node.param("lidar_mount_config",lidar_path,lidar_path);
  node.param("fusion_camera_config",camera_path,camera_path);
  const auto lidar=YAML::LoadFile(lidar_path), cameras=YAML::LoadFile(camera_path);
  const auto source=lidar["source"].as<std::string>();
  const auto at=source.find('@');
  if(at==std::string::npos || source.substr(0,at)!=cameras["source_sensor_set"].as<std::string>() ||
      source.substr(at+1)!=cameras["source_sha256"].as<std::string>())
    throw std::runtime_error("Camera and LiDAR sensor provenance mismatch");
  const std::string workspace=ros::package::getPath("asmc_lidar")+"/../..";
  if(Sha256(workspace+"/"+source.substr(0,at))!=source.substr(at+1))
    throw std::runtime_error("Sensor source SHA mismatch");
  const auto reference=YAML::LoadFile(workspace+"/"+source.substr(0,at));
  // Mount snapshots must agree with their actual hashed sensor JSON, not just labels.
  const auto& source_camera=reference["cameraList"][0];
  const auto source_pos=source_camera["pos"],source_rot=source_camera["rot"];
  const auto yaml_camera=cameras["cameras"]["front"];
  const auto camera_xyz=yaml_camera["translation_m"].as<std::vector<double>>();
  const auto camera_rpy=yaml_camera["rotation_deg"].as<std::vector<double>>();
  if(camera_xyz.size()!=3 || camera_rpy.size()!=3) throw std::runtime_error("Invalid camera snapshot");
  for(unsigned i=0;i<3;++i) {
    const char* axes[]={"x","y","z"}; const char* angles[]={"roll","pitch","yaw"};
    Near(camera_xyz[i],source_pos[axes[i]].as<double>(),"Camera source mount");
    Near(camera_rpy[i],source_rot[angles[i]].as<double>(),"Camera source rotation");
  }
  double gps_age=.30;
  node.param("lidar_pose_gps_max_age_s",gps_age,.30);
  if(gps_age<=0. || gps_age>.30) throw std::runtime_error("Common GPS age must be in (0,300ms]");
  const std::string map_contract=workspace+"/config/contracts/hd_map_topic_contract_v1.yaml";
  const auto origin=YAML::LoadFile(map_contract)["local_crop_policy"]["gps_imu_policy"]["local_origin_in_global"].as<std::vector<double>>();
  if(origin.size()!=3) throw std::runtime_error("Invalid HD map origin");
  map_origin=asmc::lidar::Vec(origin[0],origin[1],origin[2]);
  Near(altitude_origin,map_origin.z(),"Map altitude origin");
  const auto mount=lidar["sensor_pose_base_link"];
  Near(vehicle.st_LiDARPos.f32_X,mount["x_m"].as<double>(),"LiDAR x");
  Near(vehicle.st_LiDARPos.f32_Y,mount["y_m"].as<double>(),"LiDAR y");
  Near(vehicle.st_LiDARPos.f32_Z,mount["z_m"].as<double>(),"LiDAR z");
  Near(vehicle.st_LiDARRotationDeg.f32_X,mount["roll_deg"].as<double>(),"LiDAR roll");
  Near(vehicle.st_LiDARRotationDeg.f32_Y,mount["pitch_deg"].as<double>(),"LiDAR pitch");
  Near(vehicle.st_LiDARRotationDeg.f32_Z,mount["yaw_deg"].as<double>(),"LiDAR yaw");
  const auto source_lidar=reference["Lidar3DList"][0],source_gps=reference["GPSList"][0];
  const char* axes[]={"x","y","z"}; const char* angles[]={"roll","pitch","yaw"};
  const char* mount_axes[]={"x_m","y_m","z_m"}; const char* mount_angles[]={"roll_deg","pitch_deg","yaw_deg"};
  const double gps_xyz[]={vehicle.st_GPSPos.f32_X,vehicle.st_GPSPos.f32_Y,vehicle.st_GPSPos.f32_Z};
  for(unsigned i=0;i<3;++i) {
    Near(mount[mount_axes[i]].as<double>(),source_lidar["pos"][axes[i]].as<double>(),"LiDAR source mount");
    Near(mount[mount_angles[i]].as<double>(),source_lidar["rot"][angles[i]].as<double>(),"LiDAR source rotation");
    Near(gps_xyz[i],source_gps["pos"][axes[i]].as<double>(),"GPS source mount");
  }
  Near(yaml_camera["width"].as<double>(),source_camera["cc"]["cameraResWidth"].as<double>(),"Camera source width");
  Near(yaml_camera["height"].as<double>(),source_camera["cc"]["cameraResHeight"].as<double>(),"Camera source height");
  Near(yaml_camera["horizontal_fov_deg"].as<double>(),source_camera["cc"]["cameraFOV"].as<double>(),"Camera source HFOV");
  const auto front=cameras["cameras"]["front"];
  const auto t=front["translation_m"].as<std::vector<double>>();
  const auto r=front["rotation_deg"].as<std::vector<double>>();
  if(t.size()!=3 || r.size()!=3) throw std::runtime_error("Invalid camera pose dimensions");
  asmc::lidar::Pose expected=asmc::lidar::Pose::Identity();
  expected.translation()=asmc::lidar::Vec(t[0],t[1],t[2]);
  expected.linear()=asmc::lidar::rpy(r[0]*M_PI/180,r[1]*M_PI/180,r[2]*M_PI/180);
  const auto actual=camera.params.bird_eye_view_transform.inverse();
  if((actual-expected.matrix()).norm()>1e-6) throw std::runtime_error("Fusion camera extrinsics mismatch");
  Near(camera.params.intrinsic_matrix_90_fov(0,2),front["width"].as<double>()/2,"Camera cx");
  Near(camera.params.intrinsic_matrix_90_fov(1,2),front["height"].as<double>()/2,"Camera cy");
  const double f=front["width"].as<double>()/(2*std::tan(front["horizontal_fov_deg"].as<double>()*M_PI/360));
  Near(camera.params.intrinsic_matrix_90_fov(0,0),f,"Camera fx");
  Near(camera.params.intrinsic_matrix_90_fov(1,1),f,"Camera fy");
  calibration_id=Sha256(lidar_path)+":"+Sha256(camera_path)+":"+Sha256(map_contract);
  ROS_INFO("Common geometry enabled; roll=%s calibration=%s altitude_origin=%.3f",
      use_roll?"on":"off",calibration_id.c_str(),altitude_origin);
}
void PrepareScan(const ros::Time& stamp,const GPS_IMU& local,const VehicleSpec& vehicle,
                 const asmc::lidar::PoseSelection& map_pose,std::uint64_t input_generation) {
  asmc::lidar::Observation time{static_cast<asmc::lidar::Ns>(stamp.toNSec()),
      static_cast<asmc::lidar::Ns>(stamp.toNSec()),
      static_cast<asmc::lidar::Ns>(ros::Time::now().toNSec()),"ros","bridge_receive_ms"};
  PrepareScan(time,local,vehicle,map_pose,input_generation);
}
ros::Time SourceStamp(const RAW_LIDAR_DATA& raw) {
  return ros::Time().fromNSec(Enabled() && raw.source_stamp_ns?raw.source_stamp_ns:raw.u64_Timestamp*1000000ULL);
}
asmc::lidar::Observation ScanTime(const RAW_LIDAR_DATA& raw) {
  const auto ns=SourceStamp(raw).toNSec();
  if(raw.receipt_steady_ns) return {static_cast<asmc::lidar::Ns>(ns),
      static_cast<asmc::lidar::Ns>(raw.receipt_steady_ns),
      static_cast<asmc::lidar::Ns>(fusion_trace::SteadyNowNs()),"ros","bridge_receive_ns","steady"};
  return {static_cast<asmc::lidar::Ns>(ns),static_cast<asmc::lidar::Ns>(ns),
      static_cast<asmc::lidar::Ns>(ros::Time::now().toNSec()),"ros","bridge_receive_ms"};
}
void PrepareScan(const asmc::lidar::Observation& time,const GPS_IMU& local,
                 const VehicleSpec& vehicle,const asmc::lidar::PoseSelection& map_pose,
                 std::uint64_t input_generation) {
  current=asmc::lidar::Context{};
  scan_attitude=MapRotation(local); scan_yaw=local.st_IMU.f32_Yaw_rad;
  current.time=time;
  current.input_generation=input_generation;
  current.frame=asmc::lidar::Frame::LeveledSensor;
  current.base_from_sensor=SensorMount(vehicle);
  current.calibration_source=calibration_id;
  // leveling_source is written only by StoreLeveling. A scan whose points were never leveled
  // (e.g. LidarProcess early return) stays unproven and base_from_input fails closed.
  current.leveling_source.clear(); leveling_stores=0;
  current.map_pose=map_pose;
  // KF derivatives are not promoted to an absolute 3D velocity by this adapter.
  current.ego_linear_base=asmc::lidar::Vec(local.st_CAN.f32_CurrentVel_MPS,0,0);
}
namespace {
asmc::lidar::Rot YawFromInput() {
  return asmc::lidar::rpy(0,0,scan_yaw).transpose()*scan_attitude*
      current.base_from_sensor.linear()*current.applied_leveling.transpose();
}
asmc::lidar::Vec OriginInYaw() {
  return asmc::lidar::rpy(0,0,scan_yaw).transpose()*scan_attitude*
      current.base_from_sensor.translation();
}
}
bool LegacyMotionReady(const LiDAR& lidar) {
  for(const auto& object:lidar.vec_Objects)
    if(object.s32_Id>=0 && object.s32_MissCount==0 && !object.pcl_ClusterPoints.empty() &&
        object.str_ObjectClass!="noise" && object.str_ObjectClass!="지면노이즈" && !object.motion_valid)
      return false;
  return true;
}
void RecordTrackingFrame(LiDARObjectInfo& object) {
  if(!Enabled()) return;
  object.tracking_frame_valid=true; object.tracking_height_input=object.f32_CoreZ;
  object.tracking_yaw_from_input=YawFromInput();
  object.tracking_origin_in_yaw=OriginInYaw();
  object.tracking_stamp_ns=current.time.stamp_ns;
  object.tracking_ego_speed_mps=static_cast<float>(current.ego_linear_base.x());
}
// base_link displacement from the recorded scan to the current scan, expressed in the recorded
// (leveled, heading-aligned) yaw frame: constant-curvature arc with the mean of the two CAN
// (vel_x) samples and the IMU yaw change. Zero when the record is missing or implausible.
asmc::lidar::Vec EgoShiftSinceRecord(const LiDARObjectInfo& object,double yaw_now_rad) {
  if(object.tracking_stamp_ns<=0 || current.time.stamp_ns<=object.tracking_stamp_ns) return asmc::lidar::Vec::Zero();
  const double dt=1e-9*static_cast<double>(current.time.stamp_ns-object.tracking_stamp_ns);
  const double v=0.5*(static_cast<double>(object.tracking_ego_speed_mps)+current.ego_linear_base.x());
  if(dt>1.0 || !std::isfinite(v)) return asmc::lidar::Vec::Zero();
  const double dpsi=std::remainder(yaw_now_rad-object.f32_PrevYaw,2.0*M_PI), s=v*dt;
  if(std::abs(dpsi)<1e-6) return asmc::lidar::Vec(s,0.5*s*dpsi,0);
  const double rho=s/dpsi;
  return asmc::lidar::Vec(rho*std::sin(dpsi),rho*(1.0-std::cos(dpsi)),0);
}
void AlignTrackingFrame(LiDARObjectInfo& object,double yaw_now_rad,bool rotate_obb_yaw) {
  if(!Enabled() || !object.tracking_frame_valid) return;
  const auto inverse=YawFromInput().transpose().eval();
  const auto yaw_change=asmc::lidar::rpy(0,0,object.f32_PrevYaw-yaw_now_rad);
  const auto rotation=(inverse*yaw_change*object.tracking_yaw_from_input).eval();
  if(tracking_options::Get().two_point_init_cov) {
    const Eigen::Matrix2f r=rotation.topLeftCorner<2,2>().cast<float>();
    object.first_observation_R=(r*object.first_observation_R*r.transpose()).eval();
  }
  // P0: relative CA, the ego translation stays in the state. Option ego_translation_comp:
  // remove the base_link arc so the state (and its velocity) is ground-referenced.
  const asmc::lidar::Vec ego_shift=tracking_options::Get().ego_translation_comp
      ? EgoShiftSinceRecord(object,yaw_now_rad) : asmc::lidar::Vec::Zero();
  const auto translation=(inverse*(yaw_change*(object.tracking_origin_in_yaw-ego_shift)-OriginInYaw())).eval();
  // Re-express the previous state before the unchanged relative CA prediction.
  // Do not subtract CAN displacement: relative CA already models ego motion.
  const asmc::lidar::Vec position(object.Eigen_KfState_CA(0),object.Eigen_KfState_CA(1),object.tracking_height_input);
  const auto next=(rotation*position+translation).eval();
  Eigen::Matrix<float,6,6> jacobian=Eigen::Matrix<float,6,6>::Zero();
  for(unsigned block=0;block<3;++block)
    jacobian.block<2,2>(2*block,2*block)=rotation.topLeftCorner<2,2>().cast<float>();
  object.Eigen_KfState_CA=jacobian*object.Eigen_KfState_CA;
  object.Eigen_KfState_CA(0)=next.x();object.Eigen_KfState_CA(1)=next.y();
  object.Eigen_KfP_CA=jacobian*object.Eigen_KfP_CA*jacobian.transpose();
  if(rotate_obb_yaw) {
    // Only a previous-scan OBB (aligned copy, missed track) is re-expressed; a current
    // detection's OBB yaw is already measured in this scan's frame.
    const auto axis=(rotation*asmc::lidar::Vec(std::cos(object.f32_OBB_YawRad),std::sin(object.f32_OBB_YawRad),0)).eval();
    object.f32_OBB_YawRad=std::atan2(axis.y(),axis.x());
  }
  object.f32_PrevYaw=yaw_now_rad;
  RecordTrackingFrame(object); object.tracking_height_input=next.z();
}
bool StoreLeveling(const asmc::lidar::Rot& rotation) {
  // Exactly one valid store per PrepareScan. A second store or an invalid rotation leaves the
  // scan unproven (empty leveling_source) instead of silently replacing the applied rotation.
  if(++leveling_stores!=1 || !asmc::lidar::rotation_valid(rotation)) {
    current.leveling_source.clear();
    return false;
  }
  current.applied_leveling=rotation;
  current.leveling_source="Main::CorrectingPoints";
  return true;
}
unsigned LevelingStoreCount(){return leveling_stores;}
void ConfigureForTest(bool enable,bool roll,const std::string& calibration) {
  enabled=enable; use_roll=roll; calibration_id=calibration;
  current=asmc::lidar::Context{}; scan_attitude=asmc::lidar::Rot::Identity(); scan_yaw=0.;
  leveling_stores=0;
}
}  // namespace frame_boundary
