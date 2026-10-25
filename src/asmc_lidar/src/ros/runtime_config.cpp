// Shadow runtime configuration loader: config/lidar_pipeline.yaml (schema 1).
// Values come from Main setting.yaml / main.launch / the HD-map contract (see
// the provenance block of the file); the checks are Main's start-up checks
// (main.cpp:608-613, FrameBoundary.cpp:54-130, Parser.cpp:21-48, FusionCamera.cpp
// parameter clamps) for the part of the configuration this port uses.
#include <asmc_lidar/ros/runtime.hpp>

#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace asmc {
namespace lidar {
namespace runtime {
namespace {

[[noreturn]] void Fail(const std::string& file, const std::string& what) {
  throw std::runtime_error("shadow config " + file + ": " + what);
}

// Ported from Main src/Geometry/FrameBoundary.cpp:18-28 (8f93c22e).
std::string Sha256(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot read calibration source: " + path);
  SHA256_CTX state; SHA256_Init(&state);
  char buffer[8192];
  while (stream) {stream.read(buffer, sizeof(buffer)); SHA256_Update(&state, buffer, stream.gcount());}
  unsigned char digest[SHA256_DIGEST_LENGTH]; SHA256_Final(digest, &state);
  std::ostringstream out;
  for (auto value : digest) out << std::hex << std::setfill('0') << std::setw(2) << unsigned(value);
  return out.str();
}

// Ported from Main src/Geometry/FrameBoundary.cpp:29-32 (8f93c22e).
void Near(double a, double b, const std::string& file, const std::string& name) {
  if (!std::isfinite(a) || !std::isfinite(b) || std::abs(a - b) > 1e-6)
    Fail(file, "Calibration mismatch: " + name);
}

std::vector<std::string> Split(const std::string& dotted) {
  std::vector<std::string> parts;
  std::string part;
  std::istringstream in(dotted);
  while (std::getline(in, part, '.')) parts.push_back(part);
  return parts;
}

// Strict reader: every key must exist. Lookups never assign YAML::Node (its
// operator= rebinds the referenced tree node).
class Reader {
 public:
  Reader(const YAML::Node& root, std::string file) : root_(root), file_(std::move(file)) {}
  const std::string& file() const { return file_; }

  YAML::Node Node(const std::string& dotted) const {
    const auto parts = Split(dotted);
    return Child(root_, parts, 0, dotted);
  }
  template <class T>
  T Get(const std::string& dotted) const {
    const YAML::Node node = Node(dotted);
    try {
      return node.as<T>();
    } catch (const YAML::Exception& error) {
      Fail(file_, "invalid value for " + dotted + " (" + error.what() + ")");
    }
  }

 private:
  YAML::Node Child(const YAML::Node& node, const std::vector<std::string>& parts, std::size_t i,
                   const std::string& dotted) const {
    if (i == parts.size()) return node;
    if (!node.IsMap()) Fail(file_, "missing key " + dotted);
    const YAML::Node next = node[parts[i]];
    if (!next.IsDefined() || next.IsNull()) Fail(file_, "missing key " + dotted);
    return Child(next, parts, i + 1, dotted);
  }
  const YAML::Node root_;
  std::string file_;
};

YAML::Node LoadYaml(const std::string& path) {
  try {
    return YAML::LoadFile(path);
  } catch (const YAML::Exception& error) {
    Fail(path, std::string("cannot load (") + error.what() + ")");
  }
}

std::string Resolve(const std::string& path, const std::string& package_share_dir) {
  if (path.empty() || path.front() == '/' || package_share_dir.empty()) return path;
  return package_share_dir + "/" + path;
}

}  // namespace

RuntimeConfig LoadRuntimeConfig(const std::string& yaml_path, const std::string& package_share_dir) {
  const auto root=LoadYaml(yaml_path);
  const Reader r(root, yaml_path);
  const auto& file = r.file();
  if (r.Get<int>("schema_version") != 1) Fail(file, "unsupported schema_version (expected 1)");

  RuntimeConfig config;
  auto& p = config.pipeline;

  // Ported from Main src/main.cpp:175-191 (InitVehicleSpec) (8f93c22e): float reads.
  auto& v = p.vehicle;
  v.f32_Length = r.Get<float>("vehicle.Length");
  v.f32_Width = r.Get<float>("vehicle.Width");
  v.f32_WheelBase = r.Get<float>("vehicle.WheelBase");
  v.f32_FrontOverhang = r.Get<float>("vehicle.FrontOverhang");
  v.f32_RearOverhang = r.Get<float>("vehicle.RearOverhang");
  v.st_LiDARPos.f32_X = r.Get<float>("vehicle.LiDAR.x");
  v.st_LiDARPos.f32_Y = r.Get<float>("vehicle.LiDAR.y");
  v.st_LiDARPos.f32_Z = r.Get<float>("vehicle.LiDAR.z");
  v.st_LiDARRotationDeg.f32_X = r.Get<float>("vehicle.LiDAR.roll");
  v.st_LiDARRotationDeg.f32_Y = r.Get<float>("vehicle.LiDAR.pitch");
  v.st_LiDARRotationDeg.f32_Z = r.Get<float>("vehicle.LiDAR.yaw");
  v.st_GPSPos.f32_X = r.Get<float>("vehicle.GPS.x");
  v.st_GPSPos.f32_Y = r.Get<float>("vehicle.GPS.y");
  v.st_GPSPos.f32_Z = r.Get<float>("vehicle.GPS.z");

  // Ported from Main src/main.cpp:317-368 (InitLidarSettings) (8f93c22e). The
  // roi_* ROS overrides (:370-381) carry the same launch values. Main's
  // `.as<T>(fallback)` reads are strict here.
  p.range.f32_Min = r.Get<float>("lidar.Range.Min");
  p.range.f32_Max = r.Get<float>("lidar.Range.Max");
  p.roi.f32_MinX = r.Get<float>("lidar.ROI.MinX");
  p.roi.f32_MaxX = r.Get<float>("lidar.ROI.MaxX");
  p.roi.f32_MinY = r.Get<float>("lidar.ROI.MinY");
  p.roi.f32_MaxY = r.Get<float>("lidar.ROI.MaxY");
  p.roi.f32_MinZ = r.Get<float>("lidar.ROI.MinZ");
  p.roi.f32_MaxZ = r.Get<float>("lidar.ROI.MaxZ");
  if(root["lidar"]["height_roi"]) {
    const auto h=root["lidar"]["height_roi"];
    p.height_roi.enabled=h["enabled"].as<bool>();
    p.height_roi.max_height_above_ground_m=h["max_height_above_ground_m"].as<double>();
    p.height_roi.nominal_base_height_m=h["nominal_base_height_m"].as<double>();
    if(!std::isfinite(p.height_roi.max_height_above_ground_m)||p.height_roi.max_height_above_ground_m<2.0||
       p.height_roi.max_height_above_ground_m>5.0||!std::isfinite(p.height_roi.nominal_base_height_m)||
       p.height_roi.nominal_base_height_m<0.1||p.height_roi.nominal_base_height_m>1.0)
      Fail(file,"invalid gravity-aligned height ROI");
  }
  auto& cl = p.clustering;
  cl.b_Enabled = r.Get<bool>("lidar.Clustering.Enabled");
  cl.f32_Eps = r.Get<float>("lidar.Clustering.Eps");
  cl.s32_MinPoints = r.Get<pipeline::int32>("lidar.Clustering.MinPoints");
  cl.b_Adaptive = r.Get<bool>("lidar.Clustering.Adaptive.Enabled");
  cl.f32_AdaptiveNearRange = r.Get<float>("lidar.Clustering.Adaptive.NearRange");
  cl.f32_AdaptiveFarRange = r.Get<float>("lidar.Clustering.Adaptive.FarRange");
  cl.f32_AdaptiveNearEps = r.Get<float>("lidar.Clustering.Adaptive.NearEps");
  cl.f32_AdaptiveFarEps = r.Get<float>("lidar.Clustering.Adaptive.FarEps");
  auto& tr = p.tracking;
  tr.b_UseMapAssociation = r.Get<bool>("lidar.Tracking.UseMapAssociation");
  tr.f32_COST_W_DIST = r.Get<float>("lidar.Tracking.DistanceWeight");
  tr.f32_COST_W_IOU = r.Get<float>("lidar.Tracking.ShapeWeight");
  tr.f32_COST_THRESHOLD = r.Get<float>("lidar.Tracking.CostThreshold");
  tr.f32_ContinuityBonus = r.Get<float>("lidar.Tracking.ContinuityBonus");
  tr.f32_DuplicateCenterDistance = r.Get<float>("lidar.Tracking.DuplicateCenterDistance");
  tr.f32_DuplicateIouThreshold = r.Get<float>("lidar.Tracking.DuplicateIouThreshold");
  tr.f32_MergedTrackRetireDistance = r.Get<float>("lidar.Tracking.MergedTrackRetireDistance");
  // Main ~lidar_level_roll (FrameBoundary.cpp:56); the shadow file sets true.
  p.use_roll = r.Get<bool>("lidar.level_roll");

  // Main ~ground_* (Ransac.cpp ConfigureOnce); the clamps are GroundFilter's.
  auto& g = p.ground;
  g.near_threshold_m = r.Get<double>("ground.ground_distance_threshold_near_m");
  g.far_threshold_m = r.Get<double>("ground.ground_distance_threshold_far_m");
  g.far_range_m = r.Get<double>("ground.ground_threshold_far_range_m");
  g.maximum_tilt_deg = r.Get<double>("ground.ground_max_tilt_deg");
  g.seed_min_z_m = r.Get<double>("ground.ground_seed_min_z_m");
  g.seed_max_z_m = r.Get<double>("ground.ground_seed_max_z_m");
  g.maximum_iterations = r.Get<int>("ground.ground_ransac_maximum_iterations");
  g.minimum_seed_points = r.Get<int>("ground.ground_minimum_seed_points");

  p.tunnel.enabled = r.Get<bool>("experimental.tunnel_filter.enabled");
  p.tunnel.suppress_ceiling = r.Get<bool>("experimental.tunnel_filter.suppress_ceiling");
  p.tunnel.suppress_wall = r.Get<bool>("experimental.tunnel_filter.suppress_wall");

  // Ported from Main src/Parser/Parser.cpp:21-41 (ConfigureSensorPoseHistory) (8f93c22e).
  auto& pose = config.pose;
  pose.imu_yaw_is_true_enu = r.Get<bool>("pose.imu_yaw_is_true_enu");
  pose.gps_max_age_s = r.Get<double>("pose.lidar_pose_gps_max_age_s");
  pose.imu_max_age_s = r.Get<double>("pose.lidar_pose_imu_max_age_s");
  pose.can_max_age_s = r.Get<double>("pose.lidar_pose_can_max_age_s");
  pose.maximum_extrapolation_speed_mps = r.Get<double>("pose.lidar_pose_maximum_speed_mps");
  pose.history_duration_s = r.Get<double>("pose.lidar_pose_history_duration_s");
  pose.gps_in_base = v.st_GPSPos;  // Main SensorPoseHistory reads st_VehicleSpec.st_GPSPos
  config.local_wait_timeout_s = std::max(0.0, r.Get<double>("pose.lidar_pose_wait_timeout_s"));
  // Ported from Main src/Geometry/FrameBoundary.cpp:86-88 (8f93c22e).
  if (pose.gps_max_age_s <= 0. || pose.gps_max_age_s > .30)
    Fail(file, "Common GPS age must be in (0,300ms]: pose.lidar_pose_gps_max_age_s");

  // Ported from Main src/Parser/Parser.cpp:361-378 (ParseGPS sanity params) (8f93c22e).
  auto& s = config.gps.sanity;
  s.min_x = r.Get<double>("gps_sanity.gps_sanity_min_x");
  s.min_y = r.Get<double>("gps_sanity.gps_sanity_min_y");
  s.max_x = r.Get<double>("gps_sanity.gps_sanity_max_x");
  s.max_y = r.Get<double>("gps_sanity.gps_sanity_max_y");
  s.base_jump_allowance_m = r.Get<double>("gps_sanity.gps_sanity_base_jump_allowance_m");
  s.maximum_speed_mps = r.Get<double>("gps_sanity.gps_sanity_maximum_speed_mps");
  s.reanchor_after_s = r.Get<double>("gps_sanity.gps_sanity_reanchor_after_s");

  // Ported from Main src/Geometry/FrameBoundary.cpp:57-59, 89-93 (8f93c22e). The
  // origin is a copy of the HD-map contract value (provenance in the file).
  const auto origin = r.Get<std::vector<double>>("map.origin");
  if (origin.size() != 3) Fail(file, "Invalid HD map origin: map.origin needs 3 values");
  for (double value : origin) if (!std::isfinite(value)) Fail(file, "Invalid HD map origin: map.origin not finite");
  config.gps.map_origin = asmc::lidar::Vec(origin[0], origin[1], origin[2]);
  config.gps.map_altitude_origin_m = r.Get<double>("map.map_altitude_origin_m");
  if (!std::isfinite(config.gps.map_altitude_origin_m)) Fail(file, "map altitude origin is not finite");
  Near(config.gps.map_altitude_origin_m, config.gps.map_origin.z(), file, "Map altitude origin");

  // Ported from Main src/main.cpp:608-613, 622-623 and src/SensorFusion/
  // FusionCamera.cpp:366-384, 784-799 (parameter reads and clamps) (8f93c22e).
  config.enable_camera_fusion = r.Get<bool>("fusion.enable_sensor_fusion");
  const auto profile = r.Get<std::string>("fusion.camera_class_profile");
  if (profile != "legacy" && profile != "mission16_v1") Fail(file, "Unsupported camera_class_profile");
  auto& f = config.fusion;
  f.mission_profile = profile == "mission16_v1";
  f.yolo_policy = r.Get<std::string>("fusion.fusion_yolo_policy");
  if (f.yolo_policy != "unique" && f.yolo_policy != "bounded_reuse") Fail(file, "invalid fusion_yolo_policy");
  f.max_time_offset_s = std::max(0.001, r.Get<double>("fusion.fusion_max_time_offset_sec"));
  f.sync_wait_timeout_s = std::max(f.max_time_offset_s, r.Get<double>("fusion.fusion_sync_wait_timeout_sec"));
  f.initial_confirmations = std::max(1, r.Get<int>("fusion.fusion_initial_confirmations"));
  f.change_confirmations = std::max(1, r.Get<int>("fusion.fusion_change_confirmations"));
  f.class_miss_grace = std::max(0, r.Get<int>("fusion.fusion_class_miss_grace"));
  f.iou_threshold = r.Get<double>("fusion.fusion_iou_threshold");  // Main static double
  f.exclude_hood_y_band = r.Get<bool>("fusion.fusion_exclude_hood_y_band");
  f.exclude_ymin_px = r.Get<int>("fusion.exclude_ymin_px");
  f.exclude_ymax_px = r.Get<int>("fusion.exclude_ymax_px");
  f.confidence_threshold = r.Get<double>("fusion.camera_objects_conf_thres");

  // Main RAW_LIDAR_DATA capacity: 2,000,000 B / sizeof(pcl::PointXYZI).
  const auto max_points = r.Get<long long>("runtime.max_points");
  if (max_points <= 0) Fail(file, "runtime.max_points must be positive");
  config.max_points = static_cast<std::size_t>(max_points);

  // Ported from Main src/Geometry/FrameBoundary.cpp:94-100 (8f93c22e): the
  // mount file must equal the vehicle LiDAR block (Main's float values).
  config.mount_config_path = Resolve(r.Get<std::string>("mount_config"), package_share_dir);
  const YAML::Node mount_root = LoadYaml(config.mount_config_path);
  const Reader m(mount_root, config.mount_config_path);
  Near(v.st_LiDARPos.f32_X, m.Get<double>("sensor_pose_base_link.x_m"), file, "LiDAR x");
  Near(v.st_LiDARPos.f32_Y, m.Get<double>("sensor_pose_base_link.y_m"), file, "LiDAR y");
  Near(v.st_LiDARPos.f32_Z, m.Get<double>("sensor_pose_base_link.z_m"), file, "LiDAR z");
  Near(v.st_LiDARRotationDeg.f32_X, m.Get<double>("sensor_pose_base_link.roll_deg"), file, "LiDAR roll");
  Near(v.st_LiDARRotationDeg.f32_Y, m.Get<double>("sensor_pose_base_link.pitch_deg"), file, "LiDAR pitch");
  Near(v.st_LiDARRotationDeg.f32_Z, m.Get<double>("sensor_pose_base_link.yaw_deg"), file, "LiDAR yaw");

  // New optional block keeps historical schema-1 replay profiles readable.
  if (root["hd_map_audit"]) {
    const auto preview=root["hd_map_audit"]["delivery_preview"];
    if(preview) {
      auto& p=config.delivery_preview;
      p.enabled=preview["enabled"].as<bool>();
      p.roof_base_z_m=preview["roof_base_z_m"].as<double>();
      p.roof_margin_m=preview["roof_margin_m"].as<double>();
      p.minimum_points=preview["minimum_points"].as<unsigned>();
      p.minimum_road_points=preview["minimum_road_points"].as<unsigned>();
      p.minimum_road_ratio=preview["minimum_road_ratio"].as<double>();
      p.camera_age_ms=preview["camera_age_ms"].as<double>();
      if(!std::isfinite(p.roof_base_z_m)||p.roof_base_z_m<1||p.roof_base_z_m>3||
         !std::isfinite(p.roof_margin_m)||p.roof_margin_m<0.3||p.roof_margin_m>2||
         p.minimum_points<3||p.minimum_road_points<1||!std::isfinite(p.minimum_road_ratio)||
         p.minimum_road_ratio<=0||p.minimum_road_ratio>1||!std::isfinite(p.camera_age_ms)||p.camera_age_ms<=0||p.camera_age_ms>500)
        Fail(file,"invalid delivery preview safety limits");
    }
    config.hd_map_audit_mode = root["hd_map_audit"]["mode"].as<std::string>();
    if (config.hd_map_audit_mode != "off" && config.hd_map_audit_mode != "audit")
      Fail(file, "hd_map_audit.mode must be off or audit (no exclusion mode)");
    if (config.hd_map_audit_mode == "audit") {
      // Audit setup failure must never disable the original LiDAR outputs.
      try {
        config.hd_map_manifest = Resolve(root["hd_map_audit"]["manifest"].as<std::string>(), package_share_dir);
        config.hd_map_manifest_sha256 = root["hd_map_audit"]["manifest_sha256"].as<std::string>();
        config.hd_map_audit = std::make_shared<const filtering::HdMapAudit>(
            config.hd_map_manifest, config.hd_map_manifest_sha256);
        const auto product=root["hd_map_audit"]["source_product"];
        if(product&&filtering::FileSha256(Resolve(product.as<std::string>(),package_share_dir))!=config.hd_map_audit->map_sha256()) {
          config.hd_map_audit.reset();throw std::runtime_error("audit_current_product_sha_mismatch");
        }
      } catch (const std::exception& e) { config.hd_map_audit.reset();config.hd_map_audit_error = e.what(); }
    }
  }

  config.config_path = yaml_path;
  config.config_sha256 = Sha256(yaml_path);
  config.mount_config_sha256 = Sha256(config.mount_config_path);
  // Historical PR #103 comparison identity (not observation.calibration_id; Main: sha(lidar yaml):sha(camera yaml):sha(map contract)).
  p.calibration_id = "shadow:" + config.config_sha256 + ":" + config.mount_config_sha256;
  return config;
}

void FinalizeRuntimeIdentity(RuntimeConfig& config, const std::string& camera_calibration_id) {
  auto hash = [](const std::string& bytes) {
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size(), digest);
    std::ostringstream out;
    for (auto byte : digest) out << std::hex << std::setfill('0') << std::setw(2) << unsigned(byte);
    return out.str();
  };
  config.calibration_id = "sha256:" + hash(config.mount_config_sha256 + "\n" + camera_calibration_id);
  // File bytes cover every strict YAML setting; explicit effective choices cover
  // all launch/replay overrides. No host path, UUID or generation enters the hash.
  std::ostringstream effective;
  effective << "lidar-runtime-v1\n" << config.config_sha256 << '\n' << config.calibration_id << '\n'
            << config.pipeline.use_roll << ',' << config.enable_camera_fusion << ','
            << config.pipeline.tunnel.enabled << ',' << config.pipeline.capture_debug << '\n';
  effective << config.hd_map_audit_mode << '\n' << config.hd_map_manifest_sha256 << '\n';
  effective << config.pipeline.height_roi.enabled << ',' << config.pipeline.height_roi.max_height_above_ground_m
            << ',' << config.pipeline.height_roi.nominal_base_height_m << '\n';
  if (config.hd_map_audit) effective << config.hd_map_audit->bundle_sha256() << '\n'
      << config.hd_map_audit->policy_sha256() << '\n';
  const auto& p=config.delivery_preview;
  effective << p.enabled << ',' << p.roof_base_z_m << ',' << p.roof_margin_m << ','
    << p.minimum_points << ',' << p.minimum_road_points << ',' << p.minimum_road_ratio << ',' << p.camera_age_ms;
  if(p.enabled)effective << ',' << filtering::kDeliveryProtectionPolicy;
  config.config_id = hash(effective.str());
}

}  // namespace runtime
}  // namespace lidar
}  // namespace asmc
