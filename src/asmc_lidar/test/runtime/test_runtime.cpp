// LidarRuntime (Main TemporalThread order), LoadRuntimeConfig and the ROS
// input converters, with synthetic inputs and a deterministic logical clock.
#include <asmc_lidar/ros/ros_inputs.hpp>
#include <asmc_lidar/ros/runtime.hpp>
#include <asmc_lidar/preprocessing/preprocessing.hpp>
#include <asmc_lidar/ros/delivery_display_history.hpp>

#include <GeographicLib/UTMUPS.hpp>
#include <gtest/gtest.h>
#include <openssl/sha.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <ros/serialization.h>
#include <asmc_lidar/ros/scan_trace.hpp>
#include <yaml-cpp/yaml.h>

#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace pl = asmc::lidar::pipeline;
namespace sh = asmc::lidar::runtime;
using Array = asmc_msgs::LidarObjectArray;

// ---------------------------------------------------------------------------
// Paths and files
// ---------------------------------------------------------------------------

std::string PackageDir() {
  std::string file = __FILE__;  // <pkg>/test/runtime/test_runtime.cpp (absolute)
  for (int i = 0; i < 3; ++i) file = file.substr(0, file.find_last_of('/'));
  return file;
}
std::string ShippedConfig() { return PackageDir() + "/config/lidar_pipeline.yaml"; }

std::string FileSha256(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream buffer; buffer << in.rdbuf();
  const std::string bytes = buffer.str();
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size(), digest);
  std::ostringstream out;
  for (auto b : digest) out << std::hex << std::setfill('0') << std::setw(2) << unsigned(b);
  return out.str();
}

// Scratch directory under the working directory (removed on destruction).
class TempDir {
 public:
  TempDir() {
    char pattern[] = "./shadow_runtime_test_XXXXXX";
    const char* made = mkdtemp(pattern);
    if (!made) throw std::runtime_error("mkdtemp failed");
    path_ = made;
  }
  ~TempDir() {
    for (const auto& f : files_) std::remove(f.c_str());
    rmdir(path_.c_str());
  }
  std::string Bytes(const std::string& name, const std::string& bytes) {
    const std::string file = path_ + "/" + name;
    std::ofstream out(file, std::ios::binary); out.write(bytes.data(), bytes.size());
    files_.push_back(file); return file;
  }
  std::string Write(const std::string& name, const YAML::Node& node) {
    const std::string file = path_ + "/" + name;
    std::ofstream out(file);
    out << node << "\n";
    files_.push_back(file);
    return file;
  }

 private:
  std::string path_;
  std::vector<std::string> files_;
};

sh::RuntimeConfig ShippedRuntimeConfig() {
  auto cfg=sh::LoadRuntimeConfig(ShippedConfig(), PackageDir());
  cfg.fusion.views = "front"; // historical front-only fixtures, not deployment default
  cfg.pipeline.tunnel.enabled = false;  // isolate common-path fixtures; GPS mode tests cover shipped defaults
  // Existing Main-equivalence fixtures use the pre-height/audit path. Production
  // defaults are checked independently below and in the ROS integration tests.
  cfg.pipeline.height_roi.enabled=false;cfg.hd_map_audit_mode="off";
  // These historical fixtures isolate classification-only camera fusion.
  // Point-based splitting has its own tests and is enabled in the release YAML.
  cfg.pipeline.camera_point_refinement.enabled=false;
  cfg.delivery_filters.reset();  // pre-final-publication reference, tested separately below
  cfg.hd_map_audit.reset();return cfg;
}

TEST(RuntimeDeployment, DefaultMapIsPinnedAndHeightCapIsLocal) {
  const auto cfg=sh::LoadRuntimeConfig(ShippedConfig(),PackageDir());
  EXPECT_EQ(cfg.fusion.views,"all");
  ASSERT_TRUE(cfg.delivery_filters);
  EXPECT_TRUE(cfg.delivery_filters->enabled());
  EXPECT_EQ(cfg.delivery_filters->status(),"observation_quality_only");
  EXPECT_DOUBLE_EQ(cfg.delivery_preview.minimum_road_ratio,.10);
  EXPECT_DOUBLE_EQ(cfg.delivery_preview.minimum_obb_road_ratio,.10);
  EXPECT_DOUBLE_EQ(cfg.delivery_preview.road_inset_m,.10);
  if(!std::filesystem::exists(PackageDir()+"/../asmc_hd_map/config/kcity2025_compact_drivable_map.json")) {
    EXPECT_FALSE(cfg.hd_map_audit);EXPECT_FALSE(cfg.hd_map_audit_error.empty());return;
  }
  ASSERT_TRUE(cfg.hd_map_audit) << cfg.hd_map_audit_error;
  EXPECT_EQ(cfg.hd_map_audit_mode,"filter");EXPECT_TRUE(cfg.pipeline.height_roi.enabled);
  EXPECT_DOUBLE_EQ(cfg.pipeline.height_roi.max_height_above_ground_m,2.5);
  EXPECT_EQ(cfg.hd_map_audit->map_sha256(),FileSha256(PackageDir()+"/../asmc_hd_map/config/kcity2025_compact_drivable_map.json"));
}

TEST(RuntimeDeployment, MissingCurrentProductCannotKeepLoadedOldAudit) {
  TempDir dir;auto yaml=YAML::LoadFile(ShippedConfig());
  yaml["hd_map_audit"]["source_product"]="/no/current-map-product";
  const auto cfg=sh::LoadRuntimeConfig(dir.Write("stale-product.yaml",yaml),PackageDir());
  EXPECT_FALSE(cfg.hd_map_audit);EXPECT_FALSE(cfg.hd_map_audit_error.empty());
}

// ---------------------------------------------------------------------------
// Synthetic scene and sensor streams
// ---------------------------------------------------------------------------

constexpr std::int64_t kT0 = 1700000000LL * 1000000000LL;  // ROS epoch (source clock)
constexpr std::int64_t kSteady0 = 5000000000LL;            // logical steady clock
constexpr std::int64_t kMs = 1000000LL;
std::int64_t Ros(int ms) { return kT0 + ms * kMs; }
std::int64_t Steady(int ms) { return kSteady0 + ms * kMs; }

// Sensor frame (velodyne): flat ground at z=-1.7, a parked car (4.0 x 1.8 m)
// centred at (12, 3) and a pole/pedestrian column (0.5 x 0.5 m) at (6, -4).
pl::CloudPtr MakeScene() {
  pl::CloudPtr cloud(new pl::Cloud());
  auto add = [&](float x, float y, float z) {
    pcl::PointXYZI p; p.x = x; p.y = y; p.z = z; p.intensity = 10.f;
    cloud->push_back(p);
  };
  for (int ix = 0; ix <= 120; ++ix)
    for (int iy = 0; iy <= 60; ++iy) add(-20.f + 0.5f * ix, -15.f + 0.5f * iy, -1.7f);
  // car: faces and roof, 0.1 m spacing, z in [-1.2, 0.0]
  for (int iz = 0; iz <= 12; ++iz) {
    const float z = -1.2f + 0.1f * iz;
    for (int i = 0; i <= 18; ++i) { add(10.f, 2.1f + 0.1f * i, z); add(14.f, 2.1f + 0.1f * i, z); }
    for (int i = 0; i <= 40; ++i) { add(10.f + 0.1f * i, 2.1f, z); add(10.f + 0.1f * i, 3.9f, z); }
  }
  for (int i = 0; i <= 40; ++i)
    for (int j = 0; j <= 18; ++j) add(10.f + 0.1f * i, 2.1f + 0.1f * j, 0.f);
  // column: z in [-1.2, 0.3]
  for (int iz = 0; iz <= 15; ++iz) {
    const float z = -1.2f + 0.1f * iz;
    for (int i = 0; i <= 5; ++i) {
      add(5.75f + 0.1f * i, -4.25f, z); add(5.75f + 0.1f * i, -3.75f, z);
      add(5.75f, -4.25f + 0.1f * i, z); add(6.25f, -4.25f + 0.1f * i, z);
    }
  }
  return cloud;
}

asmc::lidar::camera::Calibration FrontCalibration() {
  asmc::lidar::camera::Calibration k;
  k.frame_id = "camera_front"; k.id = "fixture"; k.width = 1280; k.height = 720;
  k.base_from_camera.translation() = asmc::lidar::Vec(1.9, 0.0, 1.2);
  k.base_from_camera.linear() = asmc::lidar::rpy(0.0, 2.0 * M_PI / 180.0, 0.0);
  k.intrinsic << 640, 0, 640, 0, 640, 360, 0, 0, 1;
  return k;
}

class FakeCamera : public pl::fusion::CameraSource {
 public:
  pl::fusion::CameraSnapshot Read(std::int64_t, std::int64_t, std::int64_t, std::int64_t) override {
    ++reads;
    pl::fusion::CameraSnapshot snapshot;
    snapshot.active = false;
    snapshot.reason = "fake_inactive";
    return snapshot;
  }
  int reads = 0;
};

enum class Mode { kValid, kInvalid, kSilent };

// Feeds IMU/CAN at 50 Hz, GPS at 10 Hz (t % 100 == 50) and scans at 10 Hz
// (t % 100 == 0, from 300 ms) in stamp order; samples at a scan stamp first.
struct Sim {
  explicit Sim(sh::LidarRuntime& runtime) : rt(runtime), scene(MakeScene()) {}
  sh::LidarRuntime& rt;
  pl::CloudPtr scene;
  std::function<Mode(int)> imu = [](int) { return Mode::kValid; };
  std::function<Mode(int)> can = [](int) { return Mode::kValid; };
  std::function<Mode(int)> gps = [](int) { return Mode::kValid; };
  int now = -1;
  std::uint64_t generation = 0;
  std::map<int, sh::ScanOutput> scans;

  void Imu(int t) {
    const Mode m = imu(t);
    if (m == Mode::kSilent) return;
    pl::ImuInput in;
    in.stamp_ns = static_cast<std::uint64_t>(Ros(t));
    in.qw = m == Mode::kValid ? 1.0 : 2.0;  // |q|^2 = 4 -> not ready
    in.az = 9.81;
    rt.OnImu(in, Steady(t) + kMs, Ros(t) + kMs);
  }
  void Can(int t) {
    const Mode m = can(t);
    if (m == Mode::kSilent) return;
    pl::CanInput in;
    in.stamp_ns = static_cast<std::uint64_t>(Ros(t));
    in.vx = m == Mode::kValid ? 0.0 : NAN;
    rt.OnCan(in, Steady(t) + kMs, Ros(t) + kMs);
  }
  void Gps(int t) {
    const Mode m = gps(t);
    if (m == Mode::kSilent) return;
    pl::GpsInput in;
    in.stamp_ns = static_cast<std::uint64_t>(Ros(t));
    GeographicLib::UTMUPS::Reverse(52, true, 302595.0 + 10.0, 4124145.0 + 20.0, in.latitude, in.longitude);
    in.altitude = 30.0;
    in.east_offset = 302595.0; in.north_offset = 4124145.0;
    in.status = m == Mode::kValid ? 1 : 0;
    rt.OnGps(in, Steady(t) + kMs, Ros(t) + kMs);
  }
  sh::ScanOutput Scan(int t) {
    pl::ScanInput scan;
    scan.time.stamp_ns = Ros(t);
    scan.points.reset(new pl::Cloud(*scene));
    scan.scan_sequence = ++generation;
    sh::ScanTimes times;
    times.receipt_ns = Steady(t) + 2 * kMs;
    times.processing_ns = times.receipt_ns + kMs;
    times.decision_ns = times.processing_ns;
    times.ros_now_ns = Ros(t) + 3 * kMs;
    times.steady_now_ns = times.processing_ns;
    if (times_hook) times_hook(times);
    return rt.ProcessScan(scan, 0.0, times);
  }
  std::function<void(sh::ScanTimes&)> times_hook;
  void RunTo(int to) {
    for (int t = now + 1; t <= to; ++t) {
      if (t % 20 == 0) { Imu(t); Can(t); }
      if (t % 100 == 50) Gps(t);
      if (t >= 300 && t % 100 == 0) scans[t] = Scan(t);
    }
    now = to;
  }
};

std::set<int> Ids(const Array& a) {
  std::set<int> ids;
  for (const auto& o : a.objects) ids.insert(o.track_id);
  return ids;
}

asmc::lidar::Vec ToVec(const geometry_msgs::Point& p) { return asmc::lidar::Vec(p.x, p.y, p.z); }

// The scene yields exactly the two static objects; returns their track ids.
std::set<int> RunNormal(Sim& sim, int to) {
  sim.RunTo(to);
  const auto& first = sim.scans.at(300);
  EXPECT_TRUE(first.processed);
  const auto ids = Ids(first.arrays.local);
  EXPECT_EQ(ids.size(), 2u);
  for (const auto& entry : sim.scans) {
    EXPECT_TRUE(entry.second.processed) << entry.first;
    EXPECT_EQ(Ids(entry.second.arrays.local), ids) << entry.first;
  }
  return ids;
}


std::vector<std::uint8_t> Wire(const Array& array) {
  std::vector<std::uint8_t> bytes(ros::serialization::serializationLength(array));
  ros::serialization::OStream stream(bytes.data(), bytes.size());
  ros::serialization::serialize(stream, array); return bytes;
}

// Tiny map fixture: local/map identity is the shipped origin; no external data.
void WithAudit(TempDir& dir, sh::RuntimeConfig& cfg,
               const std::string& cells=std::string(160000,char(100))) {
  const auto occupancy=dir.Bytes("occupancy.bin",cells);
  const auto exceptions=dir.Bytes("exceptions.bin",std::string(160000, char(0)));
  const std::string policy=R"({"expected_map_id":"fixture","map_origin":[302595,4124145,0],"pose_age_limit_ms":300,"candidate_guard_m":1,"exception_protection_m":10})";
  const auto policy_file=dir.Bytes("policy.json",policy);
  YAML::Node m;
  m["schema_version"]="asmc.lidar.hd_map_audit_bundle/v1";m["frame_id"]="map";m["map_id"]="fixture";
  m["map_origin"].push_back(302595);m["map_origin"].push_back(4124145);m["map_origin"].push_back(0);
  m["origin_xy"].push_back(-100);m["origin_xy"].push_back(-100);
  m["height"]=400;m["width"]=400;m["resolution_m"]=1;
  m["policy_canonical"]=policy;m["policy_sha256"]=FileSha256(policy_file);
  m["source_sha256"]["product"]=std::string(64,'a');
  for(const auto& pair:std::vector<std::pair<std::string,std::string>>{{"occupancy",occupancy},{"exceptions",exceptions}}){
    auto n=m["arrays"][pair.first];n["file"]=pair.first+".bin";n["bytes"]=160000;n["sha256"]=FileSha256(pair.second);
  }
  cfg.hd_map_audit_mode="audit";cfg.delivery_preview.enabled=false; // legacy A1 geometry fixture
  cfg.hd_map_manifest=dir.Write("manifest.yaml",m);
  cfg.hd_map_manifest_sha256=FileSha256(cfg.hd_map_manifest);
  cfg.hd_map_audit=std::make_shared<const asmc::lidar::filtering::HdMapAudit>(cfg.hd_map_manifest,cfg.hd_map_manifest_sha256);
}

TEST(HdMapAuditRuntime, InsetShrinksOnlyUnionBoundaryAndPreservesRawAndUnknownCounts) {
  namespace f=asmc::lidar::filtering;
  TempDir dir;auto cfg=ShippedRuntimeConfig();std::string cells(160000,char(100));
  for(int y=100;y<120;++y)for(int x=100;x<120;++x)cells[y*400+x]=0;
  cells[110*400+99]=char(-1);
  WithAudit(dir,cfg,cells);
  auto manifest=YAML::LoadFile(cfg.hd_map_manifest);manifest["resolution_m"]=.2;
  const auto path=dir.Write("inset.yaml",manifest),sha=FileSha256(path);
  f::HdMapAudit plain(path,sha),inset(path,sha,.2);
  EXPECT_DOUBLE_EQ(inset.road_inset_m(),.2);
  f::PointEvidence raw,inside;
  // First road cell, inner road cell, union middle (no per-lane erosion), unknown.
  for(const auto& p:std::vector<f::Point>{{-79.9,-77.9,0},{-79.7,-77.9,0},
                                          {-78.1,-77.9,0},{-80.1,-77.9,0}}) {
    plain.Accumulate(raw,p,.1);inset.Accumulate(inside,p,.1);
  }
  EXPECT_EQ(raw.road,3u);EXPECT_EQ(raw.interior_road,3u);
  EXPECT_EQ(inside.road,raw.road);EXPECT_EQ(inside.interior_road,2u);
  EXPECT_EQ(inside.unknown,1u);EXPECT_EQ(inside.total,4u);
  f::AuditFrame frame;frame.frame_id="map";frame.status=1;frame.map_id="fixture";
  frame.map_origin={302595,4124145,0};frame.pose_valid=true;frame.stamp_ns=frame.pose_stamp_ns=1;
  f::AuditObject box;
  for(double z:{0.,1.})for(const auto& xy:std::vector<std::array<double,2>>{
      {-79.95,-77.95},{-79.91,-77.95},{-79.91,-77.55},{-79.95,-77.55}})
    box.corners.push_back({xy[0],xy[1],z});
  const auto before=plain.Evaluate(frame,box),after=inset.Evaluate(frame,box);
  ASSERT_TRUE(before.spatial.available);ASSERT_TRUE(after.spatial.available);
  EXPECT_DOUBLE_EQ(before.spatial.road_ratio_grid,after.spatial.road_ratio_grid);
  EXPECT_DOUBLE_EQ(after.spatial.road_ratio_grid,1.);
  EXPECT_DOUBLE_EQ(after.spatial.interior_road_ratio_grid,0.);
  f::AuditDecision missing;missing.preview_enabled=true;
  std::ostringstream unknown_json;f::WriteAuditDecision(unknown_json,missing);
  EXPECT_TRUE(YAML::Load(unknown_json.str())["preview"]["obb_interior_road_ratio"].IsNull());
  auto measured=after;measured.preview_enabled=true;measured.obb_interior_road_ratio=0.;
  std::ostringstream zero_json;f::WriteAuditDecision(zero_json,measured);
  EXPECT_DOUBLE_EQ(YAML::Load(zero_json.str())["preview"]["obb_interior_road_ratio"].as<double>(),0.);
  EXPECT_THROW(f::HdMapAudit(path,sha,-.1),std::runtime_error);
  EXPECT_THROW(f::HdMapAudit(path,sha,.7),std::runtime_error);
}

TEST(HdMapAuditRuntime, SubcellMarginsAreDistinctAndObbUsesFractionalBoundaryArea) {
  namespace f=asmc::lidar::filtering;
  TempDir dir;auto cfg=ShippedRuntimeConfig();std::string cells(160000,char(100));
  for(int y=100;y<120;++y)for(int x=100;x<120;++x)cells[y*400+x]=0;
  WithAudit(dir,cfg,cells);
  auto manifest=YAML::LoadFile(cfg.hd_map_manifest);manifest["resolution_m"]=.2;
  const auto path=dir.Write("subcell.yaml",manifest),sha=FileSha256(path);
  for(double margin:{.10,.15,.20}) {
    f::HdMapAudit map(path,sha,margin);EXPECT_DOUBLE_EQ(map.road_inset_m(),margin);
    f::PointEvidence evidence;
    // 5, 12.5 and 17.5cm inside a straight road boundary.
    for(double offset:{.05,.125,.175})map.Accumulate(evidence,{-80+offset,-78,0},0);
    EXPECT_EQ(evidence.interior_road,margin==.10?2u:margin==.15?1u:0u);
    f::AuditFrame frame;frame.frame_id="map";frame.status=1;frame.map_id="fixture";
    frame.map_origin={302595,4124145,0};frame.pose_valid=true;frame.stamp_ns=frame.pose_stamp_ns=1;
    f::AuditObject box;
    for(double z:{0.,1.})for(const auto& xy:std::vector<std::array<double,2>>{
        {-79.95,-77.95},{-79.91,-77.95},{-79.91,-77.55},{-79.95,-77.55}})
      box.corners.push_back({xy[0],xy[1],z});
    const auto d=map.Evaluate(frame,box);ASSERT_TRUE(d.spatial.available);
    EXPECT_DOUBLE_EQ(d.spatial.interior_road_ratio_grid,margin==.10?.5:margin==.15?.25:0.);
  }
}

TEST(RuntimeDelivery, ObservationQualityDoesNotMutateTracksAndGpsLossPreservesLocal) {
  auto raw=ShippedRuntimeConfig(),cfg=raw;
  cfg.delivery_filters=sh::LoadRuntimeConfig(ShippedConfig(),PackageDir()).delivery_filters;
  sh::LidarRuntime first(raw,nullptr,nullptr),second(cfg,nullptr,nullptr);
  Sim reference(first),delivery(second);
  reference.gps=delivery.gps=[](int t){return t>=600&&t<1000?Mode::kInvalid:Mode::kValid;};
  reference.RunTo(1400);delivery.RunTo(1400);
  std::size_t excluded=0,unavailable=0;
  for(const auto& row:reference.scans) {
    const auto& before=row.second;const auto& after=delivery.scans.at(row.first);
    ASSERT_EQ(before.result.objects.size(),after.result.objects.size());
    EXPECT_EQ(before.result.counts.tracks,after.result.counts.tracks);
    ASSERT_EQ(after.arrays.local.objects.size(),after.arrays.local_tracking.size());
    ASSERT_EQ(after.arrays.map.objects.size(),after.arrays.map_tracking.size());
    for(std::size_t i=0;i<after.arrays.local.objects.size();++i) {
      const auto& object=after.arrays.local.objects[i];
      EXPECT_EQ(object.track_id,after.arrays.local_tracking[i].track_id);
      const auto old=std::find_if(before.arrays.local.objects.begin(),before.arrays.local.objects.end(),
          [&](const auto& o){return o.track_id==object.track_id;});
      ASSERT_NE(old,before.arrays.local.objects.end());
      EXPECT_EQ(*old,object);  // center, all corners, velocity and class unchanged
    }
    if(before.arrays.map.status==Array::POSE_UNAVAILABLE) {
      ++unavailable;EXPECT_EQ(Wire(before.arrays.local),Wire(after.arrays.local));
    } else {
      excluded+=before.arrays.local.objects.size()-after.arrays.local.objects.size();
      EXPECT_EQ(Ids(after.arrays.local),Ids(after.arrays.map));
    }
  }
  EXPECT_GT(excluded,0u);EXPECT_GT(unavailable,0u);
}

TEST(HdMapAuditRuntime, DeliveryFilterCompactsBothFramesButPreservesRawTracksAndGpsFailure) {
  auto off=ShippedRuntimeConfig(),on=off;TempDir dir;WithAudit(dir,on);
  on.hd_map_audit_mode="filter";on.delivery_preview.enabled=true;
  sh::LidarRuntime a(off,nullptr,nullptr),b(on,nullptr,nullptr);Sim base(a),filtered(b);
  base.gps=filtered.gps=[](int t){return t>=600&&t<1000?Mode::kInvalid:Mode::kValid;};
  base.RunTo(1400);filtered.RunTo(1400);std::size_t removed=0,unavailable=0;
  for(const auto& row:base.scans) {
    const auto& expected=row.second;const auto& actual=filtered.scans.at(row.first);
    EXPECT_EQ(actual.result.objects.size(),expected.result.objects.size());
    EXPECT_EQ(actual.result.counts.tracks,expected.result.counts.tracks);
    for(std::size_t i=0;i<actual.result.objects.size();++i) {
      EXPECT_EQ(actual.result.objects[i].tracking.s32_Id,expected.result.objects[i].tracking.s32_Id);
      EXPECT_EQ(actual.result.objects[i].detection.pcl_ClusterPoints.size(),expected.result.objects[i].detection.pcl_ClusterPoints.size());
      EXPECT_EQ(actual.result.objects[i].detection.f32_OBB_CenterX,expected.result.objects[i].detection.f32_OBB_CenterX);
    }
    if(expected.arrays.map.status==Array::POSE_UNAVAILABLE) {
      ++unavailable;EXPECT_EQ(Wire(expected.arrays.local),Wire(actual.arrays.local));
      EXPECT_EQ(Wire(expected.arrays.map),Wire(actual.arrays.map));
    } else {
      removed+=actual.hd_map_audit.delivery_excluded_count;
      EXPECT_TRUE(actual.arrays.local.objects.empty());EXPECT_TRUE(actual.arrays.map.objects.empty());
      EXPECT_TRUE(actual.arrays.local_tracking.empty());EXPECT_TRUE(actual.arrays.map_tracking.empty());
      EXPECT_EQ(actual.arrays.local.status,Array::OBSERVED_EMPTY);
      EXPECT_EQ(actual.arrays.map.status,Array::OBSERVED_EMPTY);
      EXPECT_EQ(actual.hd_map_audit.delivery_input_count,expected.arrays.local.objects.size());
    }
    EXPECT_EQ(actual.arrays.local.header.stamp,expected.arrays.local.header.stamp);
    for(const auto& d:actual.hd_map_audit.objects)
      EXPECT_EQ(d.delivery_excluded,d.preview_decision=="exclude_candidate");
  }
  EXPECT_GT(removed,0u);EXPECT_GT(unavailable,0u);
}

TEST(HdMapAuditRuntime, OverheadDeliveryExclusionWorksWithoutMapOrGps) {
  auto cfg=ShippedRuntimeConfig();cfg.hd_map_audit_mode="filter";
  cfg.delivery_preview.enabled=true;cfg.hd_map_audit.reset();
  sh::LidarRuntime runtime(cfg,nullptr,nullptr);Sim sim(runtime);
  // Leave the ground plane in place, lift only objects above the roof+margin.
  for(auto& p:sim.scene->points)if(p.z>-1.6)p.z+=3.;
  sim.gps=[](int){return Mode::kSilent;};sim.RunTo(600);std::size_t overhead=0;
  for(const auto& row:sim.scans)for(const auto& d:row.second.hd_map_audit.objects) {
    if(d.height_preview=="exclude_candidate") {
      ++overhead;EXPECT_TRUE(d.delivery_excluded);
      EXPECT_EQ(d.preview_reason,"overhead_clearance_candidate");
    }
    EXPECT_EQ(row.second.arrays.map.status,Array::POSE_UNAVAILABLE);
  }
  EXPECT_GT(overhead,0u);
}

TEST(HdMapAuditRuntime, PreservesEveryObjectByteAcrossGpsFaultAndRecovery) {
  auto off=ShippedRuntimeConfig(),on=off;TempDir dir;WithAudit(dir,on);
  sh::LidarRuntime a(off,nullptr,nullptr),b(on,nullptr,nullptr);Sim base(a),audit(b);
  base.gps=audit.gps=[](int t){return t>=600&&t<1000?Mode::kInvalid:Mode::kValid;};
  base.RunTo(1400);audit.RunTo(1400);
  for(const auto& row:base.scans){
    const auto& expected=row.second;const auto& actual=audit.scans.at(row.first);
    EXPECT_EQ(Wire(expected.arrays.local),Wire(actual.arrays.local));
    EXPECT_EQ(Wire(expected.arrays.map),Wire(actual.arrays.map));
    EXPECT_EQ(expected.result.counts.tracks,actual.result.counts.tracks);
    EXPECT_FALSE(expected.hd_map_audit.enabled);EXPECT_TRUE(actual.hd_map_audit.enabled);
    ASSERT_EQ(actual.hd_map_audit.objects.size(),actual.arrays.local.objects.size());
    for(const auto& d:actual.hd_map_audit.objects){
      EXPECT_EQ(d.decision,"undetermined");
      EXPECT_EQ(d.reason,actual.arrays.map.status==Array::POSE_UNAVAILABLE?
          "map_frame_or_array_status_unavailable":"pose_error_bound_unverified");
    }
    sh::trace::LineInput line;line.accepted=true;line.output=&actual;
    const auto json=YAML::Load(sh::trace::Line(line));
    EXPECT_TRUE(json["hd_map_audit"]["enabled"].as<bool>());
    EXPECT_FALSE(json["hd_map_audit"]["position_error_verified"].as<bool>());
  }
}
TEST(HdMapAuditRuntime, PointPreviewUsesFullClusterWithoutChangingOutput) {
  auto off=ShippedRuntimeConfig(),on=off;TempDir dir;WithAudit(dir,on);on.delivery_preview.enabled=true;
  sh::LidarRuntime a(off,nullptr,nullptr),b(on,nullptr,nullptr);Sim base(a),audit(b);
  base.RunTo(600);audit.RunTo(600);
  for(const auto& row:base.scans) {
    const auto& actual=audit.scans.at(row.first);
    EXPECT_EQ(Wire(row.second.arrays.local),Wire(actual.arrays.local));
    EXPECT_EQ(Wire(row.second.arrays.map),Wire(actual.arrays.map));
    for(const auto& d:actual.hd_map_audit.objects) {
      EXPECT_TRUE(d.preview_enabled);
      const auto obj=std::find_if(actual.arrays.local.objects.begin(),actual.arrays.local.objects.end(),[&](const auto& o){return o.track_id==d.track_id;});
      ASSERT_NE(obj,actual.arrays.local.objects.end());
      EXPECT_EQ(d.support_points,obj->source_point_count);
      EXPECT_EQ(d.road_points,0u); // tiny fixture is entirely non-road
      EXPECT_EQ(d.preview_decision,"exclude_candidate");
      EXPECT_EQ(d.decision,"undetermined"); // uncertainty remains unverified
    }
  }
}
TEST(HdMapAuditRuntime, MissingMapPreservesOutputAndTracks) {
  auto off=ShippedRuntimeConfig(),on=off;on.hd_map_audit_mode="audit";on.hd_map_audit_error="fixture_missing";
  sh::LidarRuntime a(off,nullptr,nullptr),b(on,nullptr,nullptr);Sim base(a),audit(b);
  base.RunTo(600);audit.RunTo(600);
  for(const auto& row:base.scans){
    const auto& actual=audit.scans.at(row.first);
    EXPECT_EQ(Wire(row.second.arrays.local),Wire(actual.arrays.local));
    EXPECT_EQ(Wire(row.second.arrays.map),Wire(actual.arrays.map));
    EXPECT_EQ(actual.hd_map_audit.reason,"map_audit_unavailable");
    for(const auto& d:actual.hd_map_audit.objects)EXPECT_EQ(d.reason,"map_audit_unavailable");
  }
}
TEST(HdMapAuditRuntime, EmptyAndInvalidInputsRemainDistinct) {
  auto cfg=ShippedRuntimeConfig();TempDir dir;WithAudit(dir,cfg);
  sh::LidarRuntime runtime(cfg,nullptr,nullptr);Sim sim(runtime);
  sim.imu=[](int){return Mode::kSilent;};sim.RunTo(300);
  EXPECT_EQ(sim.scans.at(300).arrays.local.status,Array::INPUT_INVALID);
  EXPECT_EQ(sim.scans.at(300).hd_map_audit.reason,"input_or_pose_unavailable");
  // A ground-only scan is valid and empty; it must not be mistaken for an error.
  sh::LidarRuntime empty(cfg,nullptr,nullptr);Sim ground(empty);
  ground.scene.reset(new pl::Cloud());
  for(int x=-10;x<=20;++x)for(int y=-10;y<=10;++y){pcl::PointXYZI p;p.x=x;p.y=y;p.z=-1.7;p.intensity=0;ground.scene->push_back(p);}
  ground.RunTo(500);const auto& out=ground.scans.at(500);
  EXPECT_EQ(out.arrays.map.status,Array::OBSERVED_EMPTY);
  EXPECT_EQ(out.hd_map_audit.reason,"valid_empty");EXPECT_TRUE(out.hd_map_audit.objects.empty());
}
TEST(HdMapAuditRuntime, ResetReusesImmutableMapButNotState) {
  auto cfg=ShippedRuntimeConfig();TempDir dir;WithAudit(dir,cfg);
  sh::LidarRuntime first(cfg,nullptr,nullptr),next(cfg,nullptr,nullptr);
  Sim a(first),b(next);a.RunTo(500);b.RunTo(500);
  EXPECT_EQ(Wire(a.scans.at(300).arrays.local),Wire(b.scans.at(300).arrays.local));
  EXPECT_EQ(first.config().hd_map_audit.get(),next.config().hd_map_audit.get());
  EXPECT_EQ(a.scans.at(500).hd_map_audit.bundle_sha256,b.scans.at(500).hd_map_audit.bundle_sha256);
}

// ---------------------------------------------------------------------------
// LoadRuntimeConfig
// ---------------------------------------------------------------------------

TEST(RuntimeConfig, FrontRearReleaseProfileAndLegacyYamlFallback) {
  const auto shipped=sh::LoadRuntimeConfig(ShippedConfig(),PackageDir());
  const auto& c=shipped.pipeline.clustering;
  EXPECT_FLOAT_EQ(c.f32_AdaptiveNearRange,10.f);
  EXPECT_FLOAT_EQ(c.f32_AdaptiveNearEps,1.30f);
  EXPECT_FLOAT_EQ(c.f32_AdaptiveFarEps,1.32f);
  EXPECT_FLOAT_EQ(c.f32_AdaptiveRearNearEps,1.60f);
  EXPECT_FLOAT_EQ(c.f32_AdaptiveRearFarEps,1.62f);
  TempDir dir;auto yaml=YAML::LoadFile(ShippedConfig());
  auto adaptive=yaml["lidar"]["Clustering"]["Adaptive"];
  adaptive.remove("RearNearEps");adaptive.remove("RearFarEps");
  const auto legacy=sh::LoadRuntimeConfig(dir.Write("legacy-clustering.yaml",yaml),PackageDir());
  EXPECT_FLOAT_EQ(legacy.pipeline.clustering.f32_AdaptiveRearNearEps,c.f32_AdaptiveNearEps);
  EXPECT_FLOAT_EQ(legacy.pipeline.clustering.f32_AdaptiveRearFarEps,c.f32_AdaptiveFarEps);
}

TEST(RuntimeConfig, LoadsShippedConfigWithCurrentOperatingValues) {
  const auto c = sh::LoadRuntimeConfig(ShippedConfig(),PackageDir());
  const auto& p = c.pipeline;
  EXPECT_EQ(p.vehicle.f32_Length, 4.635f);
  EXPECT_EQ(p.vehicle.f32_Width, 1.892f);
  EXPECT_EQ(p.vehicle.f32_WheelBase, 3.f);
  EXPECT_EQ(p.vehicle.f32_FrontOverhang, 0.845f);
  EXPECT_EQ(p.vehicle.f32_RearOverhang, 0.790f);
  EXPECT_EQ(p.vehicle.st_LiDARPos.f32_X, 0.80f);
  EXPECT_EQ(p.vehicle.st_LiDARPos.f32_Z, 1.45f);
  EXPECT_EQ(p.vehicle.st_GPSPos.f32_X, 0.35f);
  EXPECT_EQ(p.vehicle.st_GPSPos.f32_Z, 1.25f);
  EXPECT_EQ(p.range.f32_Min, 0.5f);
  EXPECT_EQ(p.range.f32_Max, 100.f);
  EXPECT_EQ(p.roi.f32_MinX, -30.f);
  EXPECT_EQ(p.roi.f32_MaxX, 50.f);
  EXPECT_EQ(p.roi.f32_MinY, -20.f);
  EXPECT_EQ(p.roi.f32_MaxY, 20.f);
  EXPECT_EQ(p.roi.f32_MinZ, -2.f);
  EXPECT_EQ(p.roi.f32_MaxZ, 2.f);  // Integrated live ROI requested by the user.
  EXPECT_EQ(p.clustering.f32_Eps, 0.5f);
  EXPECT_EQ(p.clustering.s32_MinPoints, 5);
  EXPECT_TRUE(p.clustering.b_Adaptive);
  EXPECT_EQ(p.clustering.f32_AdaptiveNearEps, 1.30f);
  EXPECT_EQ(p.clustering.f32_AdaptiveFarEps, 1.32f);
  EXPECT_EQ(p.tracking.f32_COST_THRESHOLD, 3.0f);
  EXPECT_EQ(p.tracking.f32_ContinuityBonus, 0.20f);
  EXPECT_EQ(p.tracking.f32_DuplicateCenterDistance, 0.60f);
  EXPECT_EQ(p.tracking.f32_DuplicateIouThreshold, 0.35f);
  EXPECT_EQ(p.tracking.f32_MergedTrackRetireDistance, 0.45f);
  EXPECT_TRUE(p.use_roll);
  EXPECT_EQ(p.voxel_leaf_m, 0.1f);
  // Integrated operating profile preserves low-object tuning and refinement.
  EXPECT_EQ(p.ground.near_threshold_m, 0.30);
  EXPECT_TRUE(p.camera_point_refinement.enabled);
  EXPECT_EQ(p.ground.far_threshold_m, 0.40);
  EXPECT_DOUBLE_EQ(p.ground.fit_threshold_m, 0.);  // Main far-band fit; explicit override stays opt-in.
  EXPECT_EQ(p.ground.seed_max_z_m, -0.65);
  EXPECT_EQ(p.ground.maximum_iterations, 80);
  EXPECT_EQ(p.ground.minimum_seed_points, 30);
  EXPECT_TRUE(p.tunnel.enabled);  // Main release keeps wall/ceiling handling enabled.
  EXPECT_TRUE(c.pose.imu_yaw_is_true_enu);
  EXPECT_EQ(c.pose.gps_max_age_s, 0.30);
  EXPECT_EQ(c.pose.imu_max_age_s, 0.12);
  EXPECT_EQ(c.pose.can_max_age_s, 0.12);
  EXPECT_EQ(c.pose.maximum_extrapolation_speed_mps, 35.0);
  EXPECT_EQ(c.pose.history_duration_s, 5.0);
  EXPECT_EQ(c.pose.gps_in_base.f32_X, 0.35f);  // vehicle GPS block
  EXPECT_EQ(c.pose.gps_in_base.f32_Y, 0.f);
  EXPECT_EQ(c.pose.gps_in_base.f32_Z, 1.25f);
  EXPECT_EQ(c.local_wait_timeout_s, 0.05);
  EXPECT_EQ(c.gps.sanity.min_x, -454.5159);
  EXPECT_EQ(c.gps.sanity.max_y, 621.3542);
  EXPECT_EQ(c.gps.sanity.maximum_speed_mps, 80.0);
  EXPECT_EQ(c.gps.map_origin, asmc::lidar::Vec(302595.0, 4124145.0, 0.0));
  EXPECT_EQ(c.gps.map_altitude_origin_m, 0.0);
  EXPECT_TRUE(c.enable_camera_fusion);
  EXPECT_TRUE(c.fusion.mission_profile);
  EXPECT_EQ(c.fusion.yolo_policy, "unique");
  EXPECT_EQ(c.fusion.max_time_offset_s, 0.05);
  EXPECT_EQ(c.fusion.sync_wait_timeout_s, 0.20);
  EXPECT_EQ(c.fusion.initial_confirmations, 5);
  EXPECT_EQ(c.fusion.iou_threshold, 0.25);
  EXPECT_EQ(c.fusion.allowed_names.size(), 15u);
  EXPECT_EQ(std::count(c.fusion.allowed_names.begin(), c.fusion.allowed_names.end(), "traffic_cone"), 0);
  EXPECT_EQ(std::count(c.fusion.allowed_names.begin(), c.fusion.allowed_names.end(), "stroller"), 1);
  EXPECT_FALSE(c.fusion.exclude_hood_y_band);
  EXPECT_EQ(c.max_points, 62500u);
  EXPECT_EQ(c.stamp_provenance, "bridge_receive_ns");
  EXPECT_EQ(c.mount_config_path, PackageDir() + "/config/autonav_v3.yaml");
  EXPECT_EQ(c.config_sha256, FileSha256(ShippedConfig()));
  EXPECT_EQ(c.mount_config_sha256, FileSha256(c.mount_config_path));
  EXPECT_EQ(p.calibration_id, "shadow:" + c.config_sha256 + ":" + c.mount_config_sha256);
}

void ExpectConfigError(const std::function<void(YAML::Node&)>& edit, const std::string& needle) {
  TempDir dir;
  YAML::Node node = YAML::LoadFile(ShippedConfig());
  edit(node);
  const std::string path = dir.Write("lidar_pipeline.yaml", node);
  try {
    sh::LoadRuntimeConfig(path, PackageDir());
    ADD_FAILURE() << "expected std::runtime_error containing '" << needle << "'";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find(needle), std::string::npos) << error.what();
  }
}

TEST(RuntimeConfig, DedicatedWallProfileLoadsGateAndSeparationSettings) {
  const auto c=sh::LoadRuntimeConfig(PackageDir()+"/config/lidar_wall_vehicle.yaml",PackageDir());
  const auto& p=c.pipeline;
  EXPECT_TRUE(p.camera_point_refinement.enabled);
  EXPECT_TRUE(p.camera_point_refinement.vehicle_history_enabled);
  EXPECT_EQ(c.fusion.views,"all");
  EXPECT_TRUE(p.tunnel.enabled);
  EXPECT_TRUE(p.tunnel_route.cluster_z.enabled);
  EXPECT_FLOAT_EQ(p.tunnel_route.cluster_z.min_sensor_z_m,-1.55f);
  EXPECT_FLOAT_EQ(p.tunnel_route.params.wall_fit_distance_m,.15f);
  EXPECT_FLOAT_EQ(p.tunnel_route.params.wall_remove_distance_m,.25f);
  EXPECT_TRUE(p.tunnel_route.params.wall_geometry_precedence);
  EXPECT_TRUE(p.shadow_wall.enabled);
  EXPECT_DOUBLE_EQ(p.shadow_wall.lead_m,30.);
  EXPECT_DOUBLE_EQ(p.shadow_wall.exit_buffer_m,10.);
  EXPECT_GE(p.shadow_wall.route.size(),2u);
  EXPECT_FALSE(p.shadow_wall.profile_sha256.empty());
}

TEST(RuntimeConfig, RejectsGpsAgeOutsideMainBound) {
  ExpectConfigError([](YAML::Node& n) { n["pose"]["lidar_pose_gps_max_age_s"] = 0.0; }, "GPS age");
  ExpectConfigError([](YAML::Node& n) { n["pose"]["lidar_pose_gps_max_age_s"] = 0.31; }, "GPS age");
  ExpectConfigError([](YAML::Node& n) { n["pose"]["lidar_pose_gps_max_age_s"] = -0.1; }, "GPS age");
}

TEST(RuntimeConfig, Mission15AndOldProfilesKeepGroupSemantics) {
  for (const std::string profile : {"mission15_no_cone_v1", "mission16_v1", "legacy"}) {
    TempDir dir;
    auto node = YAML::LoadFile(ShippedConfig());
    node["fusion"]["camera_class_profile"] = profile;
    const auto config = sh::LoadRuntimeConfig(dir.Write("shadow_pipeline.yaml", node), PackageDir());
    EXPECT_EQ(profile != "legacy", config.fusion.mission_profile);
    EXPECT_EQ(profile == "mission15_no_cone_v1" ? 15u : 0u, config.fusion.allowed_names.size());
  }
}

TEST(RuntimeConfig, RemovingLocalGroundFitRestoresLegacyConsensus) {
  TempDir dir;
  auto n = YAML::LoadFile(ShippedConfig());
  n["ground"].remove("fit_distance_threshold_m");
  const auto c = sh::LoadRuntimeConfig(dir.Write("ground-rollback.yaml", n), PackageDir());
  EXPECT_DOUBLE_EQ(c.pipeline.ground.fit_threshold_m, 0.);
  EXPECT_DOUBLE_EQ(c.pipeline.ground.near_threshold_m, .30);
  EXPECT_DOUBLE_EQ(c.pipeline.ground.far_threshold_m, .40);
}

TEST(RuntimeConfig, GroundFitAuditIsExplicitAndDoesNotChangeRemovalBands) {
  TempDir dir;
  auto n = YAML::LoadFile(ShippedConfig());
  n["ground"]["fit_distance_threshold_m"] = .08;
  const auto c = sh::LoadRuntimeConfig(dir.Write("ground-fit.yaml", n), PackageDir());
  EXPECT_DOUBLE_EQ(c.pipeline.ground.fit_threshold_m, .08);
  EXPECT_DOUBLE_EQ(c.pipeline.ground.near_threshold_m, .30);
  EXPECT_DOUBLE_EQ(c.pipeline.ground.far_threshold_m, .40);
  EXPECT_NE(c.config_sha256, ShippedRuntimeConfig().config_sha256);
  for (double value : {0., -.1, .41, std::numeric_limits<double>::infinity()})
    ExpectConfigError([value](YAML::Node& yaml) {
      yaml["ground"]["fit_distance_threshold_m"] = value;
    }, "fit_distance_threshold_m");
}

TEST(RuntimeConfig, RejectsAltitudeOriginDifferentFromMapOrigin) {
  ExpectConfigError([](YAML::Node& n) { n["map"]["map_altitude_origin_m"] = 1.0; }, "Map altitude origin");
  ExpectConfigError([](YAML::Node& n) { n["map"]["origin"][2] = 5.0; }, "Map altitude origin");
  ExpectConfigError([](YAML::Node& n) {
    YAML::Node two;
    two.push_back(302595.0);
    two.push_back(4124145.0);
    n["map"]["origin"] = two;
  }, "map.origin");
}

TEST(RuntimeConfig, RejectsMountDifferentFromVehicleLidar) {
  ExpectConfigError([](YAML::Node& n) { n["vehicle"]["LiDAR"]["x"] = 0.81; }, "LiDAR x");
  ExpectConfigError([](YAML::Node& n) { n["vehicle"]["LiDAR"]["pitch"] = 0.5; }, "LiDAR pitch");
  ExpectConfigError([](YAML::Node& n) { n["mount_config"] = "config/does_not_exist.yaml"; }, "cannot load");
}

TEST(RuntimeConfig, RejectsSchemaMissingKeysAndBadChoices) {
  ExpectConfigError([](YAML::Node& n) { n["schema_version"] = 2; }, "schema_version");
  ExpectConfigError([](YAML::Node& n) { n["ground"].remove("ground_max_tilt_deg"); },
                    "missing key ground.ground_max_tilt_deg");
  ExpectConfigError([](YAML::Node& n) { n["lidar"]["Tracking"].remove("CostThreshold"); },
                    "missing key lidar.Tracking.CostThreshold");
  ExpectConfigError([](YAML::Node& n) { n["fusion"]["camera_class_profile"] = "coco"; }, "camera_class_profile");
  ExpectConfigError([](YAML::Node& n) { n["fusion"]["fusion_yolo_policy"] = "reuse"; }, "fusion_yolo_policy");
  ExpectConfigError([](YAML::Node& n) { n["runtime"]["max_points"] = 0; }, "max_points");
  ExpectConfigError([](YAML::Node& n) { n["lidar"]["level_roll"] = "maybe"; }, "lidar.level_roll");
  EXPECT_THROW(sh::LoadRuntimeConfig(PackageDir() + "/config/no_such_file.yaml", PackageDir()),
               std::runtime_error);
}

TEST(RuntimeConfig, AuditSetupFailureDoesNotFailLidarConfiguration) {
  TempDir dir;auto n=YAML::LoadFile(ShippedConfig());n["hd_map_audit"]["mode"]="audit";
  n["hd_map_audit"]["manifest"]="/no/such/map";
  const auto cfg=sh::LoadRuntimeConfig(dir.Write("missing-map.yaml",n),PackageDir());
  EXPECT_EQ(cfg.hd_map_audit_mode,"audit");EXPECT_FALSE(cfg.hd_map_audit);
  EXPECT_FALSE(cfg.hd_map_audit_error.empty());
  ExpectConfigError([](YAML::Node& x){x["hd_map_audit"]["mode"]="exclude";},"off, audit or filter");
  ExpectConfigError([](YAML::Node& x){x["hd_map_audit"]["mode"]="filter";x["hd_map_audit"]["delivery_preview"]["enabled"]=false;},"requires delivery_preview.enabled");
}

TEST(RuntimeConfig, AppliesMainClamps) {
  TempDir dir;
  YAML::Node n = YAML::LoadFile(ShippedConfig());
  n["pose"]["lidar_pose_wait_timeout_s"] = -1.0;  // Parser.cpp:41
  n["fusion"]["fusion_max_time_offset_sec"] = 0.0;  // FusionCamera.cpp:797-798
  n["fusion"]["fusion_sync_wait_timeout_sec"] = 0.0;
  n["fusion"]["fusion_initial_confirmations"] = 0;
  n["fusion"]["fusion_class_miss_grace"] = -2;
  const auto c = sh::LoadRuntimeConfig(dir.Write("clamps.yaml", n), PackageDir());
  EXPECT_EQ(c.local_wait_timeout_s, 0.0);
  EXPECT_EQ(c.fusion.max_time_offset_s, 0.001);
  EXPECT_EQ(c.fusion.sync_wait_timeout_s, 0.001);
  EXPECT_EQ(c.fusion.initial_confirmations, 1);
  EXPECT_EQ(c.fusion.class_miss_grace, 0);
  EXPECT_NE(c.pipeline.calibration_id, ShippedRuntimeConfig().pipeline.calibration_id);
}

// ---------------------------------------------------------------------------
// ROS input converters
// ---------------------------------------------------------------------------

TEST(RosInputs, CloudFromMsgKeepsXyzIntensityAndDropsRingTime) {
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp = ros::Time(1700000000, 5);
  msg.header.frame_id = "velodyne";
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(6, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
                                sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32,
                                "intensity", 1, sensor_msgs::PointField::FLOAT32, "ring", 1,
                                sensor_msgs::PointField::UINT16, "time", 1, sensor_msgs::PointField::FLOAT32);
  modifier.resize(6);
  msg.width = 3; msg.height = 2; msg.row_step = msg.width * msg.point_step;  // organized 3 x 2
  sensor_msgs::PointCloud2Iterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z"), in(msg, "intensity"),
      time(msg, "time");
  sensor_msgs::PointCloud2Iterator<std::uint16_t> ring(msg, "ring");
  for (int i = 0; i < 6; ++i, ++x, ++y, ++z, ++in, ++ring, ++time) {
    *x = 1.5f * i; *y = -0.25f * i; *z = 0.125f * i - 1.f; *in = 7.f + i; *ring = 15 - i; *time = 0.01f * i;
  }
  const auto cloud = sh::CloudFromMsg(msg);
  ASSERT_TRUE(cloud);
  ASSERT_EQ(cloud->size(), 6u);
  EXPECT_EQ(cloud->width, 6u);
  EXPECT_EQ(cloud->height, 1u);
  EXPECT_FALSE(cloud->is_dense);
  EXPECT_TRUE(cloud->header.frame_id.empty());
  EXPECT_EQ(cloud->header.stamp, 0u);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(cloud->points[i].x, 1.5f * i);
    EXPECT_EQ(cloud->points[i].y, -0.25f * i);
    EXPECT_EQ(cloud->points[i].z, 0.125f * i - 1.f);
    EXPECT_EQ(cloud->points[i].intensity, 7.f + i);
  }
  sensor_msgs::PointCloud2 empty = msg;
  sensor_msgs::PointCloud2Modifier(empty).resize(0);
  const auto none = sh::CloudFromMsg(empty);
  ASSERT_TRUE(none);
  EXPECT_TRUE(none->empty());
}

TEST(RosInputs, ImuCanGpsFields) {
  sensor_msgs::Imu imu;
  imu.header.stamp = ros::Time(1700000001, 250);
  imu.orientation.x = 0.1; imu.orientation.y = -0.2; imu.orientation.z = 0.3; imu.orientation.w = 0.9;
  imu.linear_acceleration.x = 1; imu.linear_acceleration.y = 2; imu.linear_acceleration.z = 9.8;
  imu.angular_velocity.x = 0.01; imu.angular_velocity.y = 0.02; imu.angular_velocity.z = 0.03;
  const auto i = sh::ImuInputFromMsg(imu);
  EXPECT_EQ(i.stamp_ns, 1700000001000000250ULL);
  EXPECT_EQ(i.qx, 0.1); EXPECT_EQ(i.qy, -0.2); EXPECT_EQ(i.qz, 0.3); EXPECT_EQ(i.qw, 0.9);
  EXPECT_EQ(i.az, 9.8); EXPECT_EQ(i.wz, 0.03);

  morai_msgs::EgoVehicleStatus ego;
  ego.header.stamp = ros::Time(1700000002, 0);
  ego.velocity.x = 3.0; ego.velocity.y = -4.0; ego.velocity.z = 0.5; ego.wheel_angle = 12.5f;
  const auto c = sh::CanInputFromMsg(ego);
  EXPECT_EQ(c.stamp_ns, 1700000002000000000ULL);
  EXPECT_EQ(c.vx, 3.0); EXPECT_EQ(c.vy, -4.0); EXPECT_EQ(c.vz, 0.5); EXPECT_EQ(c.wheel_angle_deg, 12.5);

  morai_msgs::GPSMessage gps;
  gps.latitude = 37.2; gps.longitude = 126.77; gps.altitude = 31.5;
  gps.eastOffset = 302595.0; gps.northOffset = 4124145.0; gps.status = 2;
  const auto g = sh::GpsInputFromMsg(gps);
  EXPECT_EQ(g.stamp_ns, 0u);  // zero header stays zero (substituted by GpsIngest)
  EXPECT_EQ(g.latitude, 37.2); EXPECT_EQ(g.longitude, 126.77); EXPECT_EQ(g.altitude, 31.5);
  EXPECT_EQ(g.east_offset, 302595.0); EXPECT_EQ(g.north_offset, 4124145.0); EXPECT_EQ(g.status, 2);
  EXPECT_EQ(sh::StampNs(ros::Time(3, 7)), 3000000007LL);
}

// ---------------------------------------------------------------------------
// LidarRuntime
// ---------------------------------------------------------------------------

TEST(LidarRuntime, AcceptScanGate) {
  sh::LidarRuntime rt(ShippedRuntimeConfig(), nullptr, nullptr);
  std::string reason;
  EXPECT_FALSE(rt.AcceptScan(0, 10, &reason));
  EXPECT_EQ(reason, "zero_source_stamp");
  EXPECT_FALSE(rt.AcceptScan(Ros(100), 62501, &reason));
  EXPECT_EQ(reason, "raw_buffer_capacity_exceeded");
  EXPECT_TRUE(rt.AcceptScan(Ros(100), 62500, &reason));
  EXPECT_TRUE(reason.empty());
  EXPECT_FALSE(rt.AcceptScan(Ros(100), 10, &reason));
  EXPECT_EQ(reason, "duplicate_or_regressing_source_stamp");
  EXPECT_FALSE(rt.AcceptScan(Ros(100) - 1, 10, &reason));
  EXPECT_EQ(reason, "duplicate_or_regressing_source_stamp");
  EXPECT_TRUE(rt.AcceptScan(Ros(100) + 1, 10, nullptr));
  EXPECT_EQ(rt.rejected_scans(), 4u);

  // A rejected scan does not move the accepted stamp; a sub-ms stamp is
  // rejected like Main (!source_ms).
  sh::LidarRuntime fresh(ShippedRuntimeConfig(), nullptr, nullptr);
  EXPECT_FALSE(fresh.AcceptScan(999999, 10, &reason));
  EXPECT_EQ(reason, "duplicate_or_regressing_source_stamp");
  EXPECT_FALSE(fresh.AcceptScan(Ros(200), 70000, &reason));
  EXPECT_TRUE(fresh.AcceptScan(Ros(150), 10, &reason));
}

TEST(LidarRuntime, A_NormalLocalAndMapOutputs) {
  const auto config = ShippedRuntimeConfig();
  sh::LidarRuntime rt(config, nullptr, nullptr);
  Sim sim(rt);
  const auto ids = RunNormal(sim, 1000);
  ASSERT_EQ(sim.scans.size(), 8u);
  for (const auto& entry : sim.scans) {
    const int t = entry.first;
    const auto& out = entry.second;
    SCOPED_TRACE(t);
    EXPECT_TRUE(out.pose_ready);
    EXPECT_FALSE(out.fusion_ran);  // no camera source
    EXPECT_EQ(out.selected.imu_stamp_ns, static_cast<std::uint64_t>(Ros(t)));
    EXPECT_EQ(out.selected.can_stamp_ns, static_cast<std::uint64_t>(Ros(t)));
    const auto& time = out.result.context.time;
    EXPECT_EQ(time.stamp_ns, Ros(t));
    EXPECT_EQ(time.receipt_ns, Steady(t) + 2 * kMs);
    EXPECT_EQ(time.processing_ns, Steady(t) + 3 * kMs);
    EXPECT_EQ(time.clock, "ros");
    EXPECT_EQ(time.receipt_clock, "steady");
    EXPECT_EQ(time.stamp_provenance, "bridge_receive_ns");
    const auto& a = out.arrays;
    ASSERT_TRUE(a.accepted);
    EXPECT_TRUE(a.observed);
    EXPECT_EQ(a.local.header.frame_id, "base_link");
    EXPECT_EQ(a.map.header.frame_id, "map");
    EXPECT_EQ(static_cast<std::int64_t>(a.local.header.stamp.toNSec()), Ros(t));
    EXPECT_EQ(static_cast<std::int64_t>(a.map.header.stamp.toNSec()), Ros(t));
    EXPECT_EQ(a.local.stamp_provenance, "bridge_receive_ns");
    EXPECT_EQ(a.local.status, Array::OBSERVED);
    ASSERT_EQ(a.map.status, Array::OBSERVED) << a.map.reason;
    EXPECT_EQ(a.map.reason, out.result.context.map_pose.reason);
    EXPECT_EQ(Ids(a.map), ids);
    ASSERT_TRUE(out.result.context.map_pose.valid);
    const auto& map_from_base = out.result.context.map_pose.map_from_base;
    ASSERT_EQ(a.map.objects.size(), a.local.objects.size());
    for (std::size_t i = 0; i < a.local.objects.size(); ++i) {
      const auto& l = a.local.objects[i];
      const auto& m = a.map.objects[i];
      EXPECT_EQ(l.track_id, m.track_id);
      EXPECT_NEAR((ToVec(m.center) - map_from_base * ToVec(l.center)).norm(), 0., 1e-6);
      for (int k = 0; k < 8; ++k)
        EXPECT_NEAR((ToVec(m.obb_corners[k]) - map_from_base * ToVec(l.obb_corners[k])).norm(), 0., 1e-6);
    }
    // Parked car: sensor (12, 3, -0.6) -> base_link (12.8, 3, 0.85).
    bool car = false;
    for (const auto& l : a.local.objects)
      if ((ToVec(l.center) - asmc::lidar::Vec(12.8, 3.0, 0.85)).norm() < 0.3) car = true;
    EXPECT_TRUE(car);
    // Fusion enabled without camera: Main FusionLiDAR still rewrites Core to the AABB centre.
    for (const auto& o : out.result.objects) {
      if (o.detection.pcl_ClusterPoints.empty()) continue;
      float min_x = o.detection.pcl_ClusterPoints.points[0].x, max_x = min_x;
      for (const auto& p : o.detection.pcl_ClusterPoints.points) { min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x); }
      EXPECT_EQ(o.detection.f32_CoreX, 0.5f * (min_x + max_x));
      EXPECT_EQ(o.detection.f32_MinX, min_x);
    }
  }
  EXPECT_EQ(rt.output().processed(), 8u);
  EXPECT_EQ(rt.output().stamp_errors(), 0u);
  EXPECT_EQ(rt.imu_rejected(), 0u);
  EXPECT_EQ(rt.can_rejected(), 0u);
  EXPECT_EQ(rt.gps_rejected(), 0u);
  EXPECT_EQ(rt.last_gps_reason(), "accepted");
}

TEST(LidarRuntime, B_GpsSilentOrInvalidKeepsLocalTracksAndDropsMap) {
  sh::LidarRuntime rt(ShippedRuntimeConfig(), nullptr, nullptr);
  Sim sim(rt);
  const auto ids = RunNormal(sim, 1000);
  sim.gps = [](int t) { return t > 1050 ? Mode::kSilent : Mode::kValid; };
  sim.RunTo(1600);
  for (int t = 1100; t <= 1300; t += 100)  // last GPS (1050) within 300 ms
    EXPECT_EQ(sim.scans.at(t).arrays.map.status, Array::OBSERVED) << t;
  for (int t = 1400; t <= 1600; t += 100) {
    const auto& a = sim.scans.at(t).arrays;
    EXPECT_EQ(a.local.status, Array::OBSERVED) << t;
    EXPECT_EQ(Ids(a.local), ids) << t;
    EXPECT_EQ(a.map.status, Array::POSE_UNAVAILABLE) << t;
    EXPECT_EQ(a.map.reason, "gps_stale") << t;
    EXPECT_TRUE(a.map.objects.empty()) << t;
  }
  sim.gps = [](int) { return Mode::kInvalid; };  // status 0
  sim.RunTo(1800);
  for (int t = 1700; t <= 1800; t += 100) {
    const auto& a = sim.scans.at(t).arrays;
    EXPECT_EQ(Ids(a.local), ids) << t;
    EXPECT_EQ(a.map.status, Array::POSE_UNAVAILABLE) << t;
    EXPECT_EQ(a.map.reason, "gps_invalid") << t;
    EXPECT_TRUE(a.map.objects.empty()) << t;
  }
  EXPECT_EQ(rt.gps_rejected(), 2u);
  EXPECT_NE(rt.last_gps_reason(), "accepted");
}

TEST(LidarRuntime, C_GpsRecoveryNeedsThreeNormalSamples) {
  sh::LidarRuntime rt(ShippedRuntimeConfig(), nullptr, nullptr);
  Sim sim(rt);
  const auto ids = RunNormal(sim, 1000);
  // valid up to 1050, silent, status 0 at 1650/1750, valid again from 1850
  sim.gps = [](int t) {
    return t <= 1050 ? Mode::kValid : t < 1600 ? Mode::kSilent : t < 1800 ? Mode::kInvalid : Mode::kValid;
  };
  sim.RunTo(2200);
  // Valid samples at 1850, 1950, 2050: two scans recovering, then available.
  for (int t : {1900, 2000}) {
    const auto& out = sim.scans.at(t);
    EXPECT_EQ(Ids(out.arrays.local), ids) << t;
    EXPECT_EQ(out.arrays.map.status, Array::POSE_UNAVAILABLE) << t;
    EXPECT_EQ(out.arrays.map.reason, "gps_recovering") << t;
    EXPECT_TRUE(out.arrays.map.objects.empty()) << t;
  }
  for (int t : {2100, 2200}) {
    const auto& out = sim.scans.at(t);
    EXPECT_EQ(out.arrays.map.status, Array::OBSERVED) << t;
    EXPECT_EQ(Ids(out.arrays.map), ids) << t;
    // The pose comes from the new samples, never from the pre-outage pose.
    ASSERT_FALSE(out.result.context.map_pose.sample_stamps.empty());
    EXPECT_EQ(out.result.context.map_pose.sample_stamps.back(), Ros(t - 50)) << t;
  }
  // No scan between the outage and the third new sample reused the old pose.
  for (int t = 1400; t <= 2000; t += 100)
    EXPECT_FALSE(sim.scans.at(t).result.context.map_pose.valid) << t;
}

void ExpectInputInvalidWithoutTrackerAdvance(Sim& sim, int from, int to, const std::string& reason_needle) {
  auto& tracker = sim.rt.pipeline().tracker();
  ASSERT_NE(tracker.LastFrame(), nullptr);
  const auto before = *tracker.LastFrame();
  const auto next_id = tracker.NextTrackId();
  const auto history = tracker.History().size();
  sim.RunTo(to);
  for (int t = from; t <= to; t += 100) {
    const auto& out = sim.scans.at(t);
    SCOPED_TRACE(t);
    EXPECT_FALSE(out.pose_ready);
    EXPECT_FALSE(out.processed);
    EXPECT_FALSE(out.reason.empty());
    if (!reason_needle.empty()) {
      EXPECT_EQ(out.reason, reason_needle);
    }
    ASSERT_TRUE(out.arrays.accepted);
    EXPECT_FALSE(out.arrays.observed);
    EXPECT_EQ(out.arrays.local.status, Array::INPUT_INVALID);
    EXPECT_EQ(out.arrays.map.status, Array::INPUT_INVALID);
    EXPECT_EQ(out.arrays.local.reason, out.reason);
    EXPECT_EQ(out.arrays.map.reason, out.reason);
    EXPECT_TRUE(out.arrays.local.objects.empty());
    EXPECT_TRUE(out.arrays.map.objects.empty());
    EXPECT_TRUE(out.result.objects.empty());
  }
  EXPECT_EQ(tracker.NextTrackId(), next_id);
  EXPECT_EQ(tracker.History().size(), history);
  const auto after = *tracker.LastFrame();
  ASSERT_EQ(after.size(), before.size());
  for (std::size_t i = 0; i < after.size(); ++i) {
    EXPECT_EQ(after[i].tracking.s32_Id, before[i].tracking.s32_Id);
    EXPECT_EQ(after[i].tracking.s32_Age, before[i].tracking.s32_Age);  // no hidden update
    EXPECT_EQ(after[i].tracking.s32_MissCount, before[i].tracking.s32_MissCount);
  }
}

TEST(LidarRuntime, D_ImuStaleIsInputInvalidAndFreezesTracker) {
  sh::LidarRuntime rt(ShippedRuntimeConfig(), nullptr, nullptr);
  Sim sim(rt);
  const auto ids = RunNormal(sim, 1000);
  sim.imu = [](int t) { return t > 1000 && t < 1320 ? Mode::kSilent : Mode::kValid; };
  sim.RunTo(1100);  // last IMU 1000: 100 ms <= 120 ms, still ready
  EXPECT_TRUE(sim.scans.at(1100).processed);
  const int age = rt.pipeline().tracker().LastFrame()->front().tracking.s32_Age;
  ExpectInputInvalidWithoutTrackerAdvance(sim, 1200, 1300, "");
  sim.RunTo(1400);
  const auto& resumed = sim.scans.at(1400);
  EXPECT_TRUE(resumed.processed);
  EXPECT_EQ(Ids(resumed.arrays.local), ids);
  EXPECT_EQ(rt.pipeline().tracker().LastFrame()->front().tracking.s32_Age, age + 1);
  EXPECT_EQ(rt.output().invalid(), 2u);
}

TEST(LidarRuntime, D_ImuInvalidBarrierBlocksOlderSample) {
  sh::LidarRuntime rt(ShippedRuntimeConfig(), nullptr, nullptr);
  Sim sim(rt);
  const auto ids = RunNormal(sim, 1000);
  // Valid IMU up to 1080, an invalid sample at 1090, none at 1100.
  sim.imu = [](int t) { return t == 1100 ? Mode::kSilent : Mode::kValid; };
  sim.RunTo(1089);
  sim.imu = [](int) { return Mode::kInvalid; };
  sim.Imu(1090);
  sim.imu = [](int t) { return t == 1100 ? Mode::kSilent : Mode::kValid; };
  ExpectInputInvalidWithoutTrackerAdvance(sim, 1100, 1100, "imu_input_invalid");
  EXPECT_EQ(rt.imu_rejected(), 1u);
  sim.RunTo(1200);
  EXPECT_TRUE(sim.scans.at(1200).processed);
  EXPECT_EQ(Ids(sim.scans.at(1200).arrays.local), ids);
}

TEST(LidarRuntime, D_CanInvalidOrStaleIsInputInvalid) {
  sh::LidarRuntime rt(ShippedRuntimeConfig(), nullptr, nullptr);
  Sim sim(rt);
  const auto ids = RunNormal(sim, 1000);
  // Valid CAN up to 1080, an invalid sample at 1090, none at 1100.
  sim.can = [](int t) { return t == 1100 ? Mode::kSilent : Mode::kValid; };
  sim.RunTo(1089);
  pl::CanInput bad;  // NaN speed at 1090 -> rejected, newer-invalid barrier
  bad.stamp_ns = static_cast<std::uint64_t>(Ros(1090));
  bad.vx = NAN;
  rt.OnCan(bad, Steady(1090) + kMs, Ros(1090) + kMs);
  ExpectInputInvalidWithoutTrackerAdvance(sim, 1100, 1100, "speed_input_invalid");
  EXPECT_EQ(rt.can_rejected(), 1u);
  // No CAN 1120..1480: the last valid sample (1080) is stale from 1300 on.
  sim.can = [](int t) { return t > 1100 && t < 1500 ? Mode::kSilent : Mode::kValid; };
  ExpectInputInvalidWithoutTrackerAdvance(sim, 1200, 1400, "");
  EXPECT_EQ(sim.scans.at(1300).reason, "speed_input_invalid");  // Rejected input outranks age in diagnostics.
  sim.RunTo(1500);
  EXPECT_TRUE(sim.scans.at(1500).processed);
  EXPECT_EQ(Ids(sim.scans.at(1500).arrays.local), ids);
}

TEST(LidarRuntime, CameraSourceRunsFusionWithoutChangingGeometry) {
  const auto config = ShippedRuntimeConfig();
  const auto calibration = FrontCalibration();
  auto camera = std::make_shared<FakeCamera>();
  EXPECT_THROW({ sh::LidarRuntime missing(config, camera, nullptr); }, std::invalid_argument);
  sh::LidarRuntime with_camera(config, camera, &calibration);
  sh::LidarRuntime lidar_only(config, nullptr, nullptr);
  ASSERT_NE(with_camera.fusion(), nullptr);
  EXPECT_EQ(lidar_only.fusion(), nullptr);
  Sim a(with_camera), b(lidar_only);
  a.RunTo(800);
  b.RunTo(800);
  ASSERT_EQ(a.scans.size(), b.scans.size());
  for (const auto& entry : a.scans) {
    const auto& fused = entry.second;
    const auto& plain = b.scans.at(entry.first);
    SCOPED_TRACE(entry.first);
    EXPECT_TRUE(fused.fusion_ran);
    EXPECT_FALSE(plain.fusion_ran);
    ASSERT_EQ(fused.arrays.local.objects.size(), plain.arrays.local.objects.size());
    for (std::size_t i = 0; i < fused.arrays.local.objects.size(); ++i) {
      const auto& f = fused.arrays.local.objects[i];
      const auto& p = plain.arrays.local.objects[i];
      EXPECT_EQ(f.track_id, p.track_id);
      EXPECT_EQ(f.center.x, p.center.x);
      EXPECT_EQ(f.center.y, p.center.y);
      EXPECT_EQ(f.center.z, p.center.z);
      EXPECT_EQ(f.yaw, p.yaw);
      EXPECT_EQ(f.classification, p.classification);  // inactive camera: classes stay unknown
    }
  }
  EXPECT_GE(camera->reads, 1);
  EXPECT_EQ(with_camera.fusion()->counters().calls, a.scans.size());

  // Live node: fusion reads "now" right before it runs (after the local wait and
  // the pipeline), through ScanTimes::refresh_before_fusion. A clock captured at
  // scan start would make a status heartbeat received meanwhile look newer than
  // steady_now and the receiver gate would report status_stale.
  struct RecordingCamera : FakeCamera {
    pl::fusion::CameraSnapshot Read(std::int64_t reference, std::int64_t window, std::int64_t ros_now,
                                    std::int64_t steady_now) override {
      last_ros_now = ros_now;
      last_steady_now = steady_now;
      return FakeCamera::Read(reference, window, ros_now, steady_now);
    }
    std::int64_t last_ros_now = 0, last_steady_now = 0;
  };
  auto recording = std::make_shared<RecordingCamera>();
  sh::LidarRuntime live(config, recording, &calibration);
  Sim c(live);
  c.RunTo(400);
  ASSERT_TRUE(c.scans.at(400).fusion_ran);
  EXPECT_EQ(Ros(400) + 3 * kMs, recording->last_ros_now);  // no refresh: the passed values
  EXPECT_EQ(Steady(400) + 3 * kMs, recording->last_steady_now);
  int refreshed = 0;
  c.times_hook = [&refreshed](sh::ScanTimes& times) {
    times.refresh_before_fusion = [&refreshed](sh::ScanTimes& now) {
      ++refreshed;
      now.ros_now_ns += 777 * kMs;
      now.steady_now_ns += 555 * kMs;
      now.decision_ns = now.steady_now_ns;
    };
  };
  c.RunTo(500);
  ASSERT_TRUE(c.scans.at(500).fusion_ran);
  // Optional refinement is disabled in the shipped main profile.
  EXPECT_EQ(1, refreshed);
  EXPECT_EQ(Ros(500) + 3 * kMs + 777 * kMs, recording->last_ros_now);
  EXPECT_EQ(Steady(500) + 3 * kMs + 555 * kMs, recording->last_steady_now);

  // Fusion disabled: a camera source is ignored.
  auto disabled = config;
  disabled.enable_camera_fusion = false;
  sh::LidarRuntime off(disabled, camera, &calibration);
  EXPECT_EQ(off.fusion(), nullptr);
}

}  // namespace

int main(int argc, char** argv) { testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }

TEST(RuntimeIdentity, EffectiveChoicesAndCalibrationAreBoundToConfigId) {
  auto cfg = sh::LoadRuntimeConfig(ShippedConfig(), PackageDir());
  EXPECT_TRUE(cfg.pipeline.ground.remove_below_plane);
  sh::FinalizeRuntimeIdentity(cfg, "camera-a");
  const auto id=cfg.config_id, calibration=cfg.calibration_id;
  sh::FinalizeRuntimeIdentity(cfg, "camera-a"); EXPECT_EQ(cfg.config_id,id);
  cfg.pipeline.ground.remove_below_plane=false;
  sh::FinalizeRuntimeIdentity(cfg,"camera-a"); EXPECT_NE(cfg.config_id,id);
  cfg.pipeline.ground.remove_below_plane=true;
  sh::FinalizeRuntimeIdentity(cfg,"camera-a"); EXPECT_EQ(cfg.config_id,id);
  cfg.pipeline.use_roll=!cfg.pipeline.use_roll;
  sh::FinalizeRuntimeIdentity(cfg,"camera-a"); EXPECT_NE(cfg.config_id,id);
  EXPECT_EQ(cfg.calibration_id,calibration);
  sh::FinalizeRuntimeIdentity(cfg,"camera-b"); EXPECT_NE(cfg.calibration_id,calibration);
}

TEST(DeliveryDisplayHistory, BoundedActualObservationHistoryAndIdentityReset) {
  sh::DeliveryDisplayHistory h;
  asmc::lidar::filtering::AuditReport r;r.delivery_applied=true;
  asmc::lidar::filtering::AuditDecision d;d.track_id=7;d.delivery_excluded=true;r.objects={d};
  h.Update("session|1|config",100,{7},r);
  r.objects.clear();h.Update("session|1|config",200,{7,8},r);
  ASSERT_EQ(h.entries().size(),1u);EXPECT_EQ(h.entries().at(7).stamp_ns,100);
  d.delivery_excluded=false;r.objects={d};h.Update("session|1|config",300,{7},r);
  EXPECT_FALSE(h.entries().at(7).decision.delivery_excluded);
  r.objects.clear();h.Update("session|1|newconfig",400,{7},r);EXPECT_TRUE(h.entries().empty());
  r.objects={d};h.Update("session|1|newconfig",500,{7},r);
  r.objects.clear();h.Update("session|1|newconfig",600,{},r);EXPECT_TRUE(h.entries().empty());
}

// Exercise actual fusion -> typed output across GPS loss; independent camera
// headings prevent a front-only implementation from satisfying the side cases.
TEST(LidarRuntime, ThreeViewLocalClassificationSurvivesGpsLossAndMapRecovery) {
  for (const auto& selected : {std::string("camera_front"), std::string("camera_left"), std::string("camera_right")}) {
    SCOPED_TRACE(selected);
    auto cfg = ShippedRuntimeConfig();
    cfg.fusion.views = sh::LoadRuntimeConfig(ShippedConfig(), PackageDir()).fusion.views;
    ASSERT_EQ(cfg.fusion.views, "all");
    asmc::lidar::camera::Calibrations cameras;
    for (const auto& name : {"camera_front", "camera_left", "camera_right"}) {
      auto k = FrontCalibration(); k.frame_id = name; k.id = name;
      const double yaw = k.frame_id == "camera_left" ? M_PI/2 : k.frame_id == "camera_right" ? -M_PI/2 : 0;
      if (yaw != 0) { k.width=640; k.height=480; k.intrinsic << 320,0,320,0,320,240,0,0,1; }
      k.base_from_camera.linear() = asmc::lidar::rpy(0,0,yaw);
      k.base_from_camera.translation() = asmc::lidar::Vec(1.9*std::cos(yaw),1.9*std::sin(yaw),1.2);
      cameras[name]=k;
    }
    struct ProjectedCamera : pl::fusion::CameraSource {
      asmc::lidar::camera::Calibration calibration;
      pl::ScanResult previous;
      pl::fusion::CameraSnapshot Read(std::int64_t stamp, std::int64_t, std::int64_t, std::int64_t) override {
        pl::fusion::CameraSnapshot s; s.gate_active=true; s.gate_epoch=1;
        if (!previous.processed) return s;
        std::vector<pl::fusion::LidarImageBox> boxes;
        if (!pl::fusion::LidarToImage(previous.objects, previous.context, calibration, pl::fusion::FusionConfig{}, &boxes)) return s;
        pl::fusion::CameraFrame f; f.stamp_ns=stamp; f.epoch=1; f.frame_id=calibration.frame_id;
        f.width=calibration.width; f.height=calibration.height; f.identity=f.frame_id+std::to_string(stamp);
        for (const auto& b:boxes) {
          asmc::lidar::camera::Box box;
          box.xmin=b.min_x; box.ymin=b.min_y; box.xmax=b.max_x; box.ymax=b.max_y;
          box.source_class_name="car"; box.class_name="vehicle"; box.confidence=.9; box.confidence_valid=true;
          f.boxes.push_back(box);
        }
        s.by_view[f.frame_id]={f}; return s;
      }
    };
    auto camera=std::make_shared<ProjectedCamera>(); camera->calibration=cameras.at(selected);
    sh::LidarRuntime rt(cfg,camera,&cameras.at("camera_front"),cameras);
    Sim sim(rt);
    const double yaw=selected=="camera_left"?M_PI/2:selected=="camera_right"?-M_PI/2:0;
    for(auto& p:*sim.scene) { const auto x=p.x,y=p.y; p.x=std::cos(yaw)*x-std::sin(yaw)*y; p.y=std::sin(yaw)*x+std::cos(yaw)*y; }
    sim.gps=[](int t){return t>=1500&&t<2500?Mode::kInvalid:Mode::kValid;};
    bool saw_denied=false,saw_recovery=false;
    for(int t=300;t<=3300;t+=100) {
      sim.RunTo(t); const auto& out=sim.scans.at(t); camera->previous=out.result;
      ASSERT_TRUE(out.processed); ASSERT_TRUE(out.fusion_ran);
      if(t<1100)continue; // normal five positive observation confirmations
      ASSERT_FALSE(out.arrays.local.objects.empty());
      bool classified=false;
      for(const auto& o:out.arrays.local.objects) if(o.camera_class=="car"&&o.classification=="vehicle") {
        classified=true;
        EXPECT_TRUE(std::isfinite(o.center.x)); EXPECT_EQ(o.obb_corners.size(),8u);
        if(!out.result.context.map_pose.valid) { EXPECT_FALSE(o.absolute_velocity_valid); EXPECT_FALSE(o.dynamic_valid); }
      }
      EXPECT_TRUE(classified);
      EXPECT_EQ(out.arrays.local.header.frame_id,"base_link");
      EXPECT_EQ(out.arrays.local.header.stamp,out.arrays.map.header.stamp);
      if(t>=1700&&t<=2400) {
        saw_denied=true; EXPECT_EQ(out.arrays.map.status,Array::POSE_UNAVAILABLE);
        EXPECT_TRUE(out.arrays.map.objects.empty()); EXPECT_TRUE(out.arrays.map_tracking.empty());
      }
      if(t>=3000) { saw_recovery=true; EXPECT_EQ(out.arrays.map.status,Array::OBSERVED); EXPECT_EQ(Ids(out.arrays.local),Ids(out.arrays.map)); }
    }
    EXPECT_TRUE(saw_denied); EXPECT_TRUE(saw_recovery);
  }
}

TEST(LidarRuntime, PointRefinementSelectsIndependentCausalImagesForAllThreeCameras) {
  auto cfg=ShippedRuntimeConfig();cfg.pipeline.camera_point_refinement.enabled=true;cfg.fusion.views="all";
  asmc::lidar::camera::Calibrations cameras;
  for(const auto& name:{"camera_front","camera_left","camera_right"}) {
    auto k=FrontCalibration();k.frame_id=name;k.id=name;
    const double yaw=k.frame_id=="camera_left"?M_PI/2:k.frame_id=="camera_right"?-M_PI/2:0;
    k.base_from_camera.linear()=asmc::lidar::rpy(0,0,yaw);cameras[name]=k;
  }
  struct ThreeViewSource:pl::fusion::CameraSource {
    pl::fusion::CameraSnapshot Read(std::int64_t stamp,std::int64_t,std::int64_t,std::int64_t) override {
      pl::fusion::CameraSnapshot s;s.active=s.gate_active=true;s.epoch=s.gate_epoch=1;int index=0;
      for(const auto& name:{"camera_front","camera_left","camera_right"}) {
        pl::fusion::CameraFrame f;f.frame_id=name;f.width=1280;f.height=720;f.epoch=1;
        f.stamp_ns=stamp-(++index)*10*kMs;f.identity=std::string(name)+std::to_string(f.stamp_ns);
        asmc::lidar::camera::Box b;b.xmin=300;b.xmax=900;b.ymin=200;b.ymax=600;
        b.source_class_name="car";b.confidence_valid=true;b.confidence=.95;f.boxes.push_back(b);
        s.by_view[name].push_back(f);if(index==1)s.frames.push_back(f);
        f.stamp_ns=stamp+kMs;f.identity+="future";s.by_view[name].push_back(f);
      }
      return s;
    }
  };
  auto camera=std::make_shared<ThreeViewSource>();
  sh::LidarRuntime rt(cfg,camera,&cameras.at("camera_front"),cameras);Sim sim(rt);
  const auto front_scene=*sim.scene;
  for(int sign:{-1,1})for(auto p:front_scene)if(p.z> -1.5f) {
    const float x=p.x;p.x=-sign*p.y;p.y=sign*x;sim.scene->push_back(p);
  }
  sim.RunTo(800);
  const auto& out=sim.scans.at(800);ASSERT_TRUE(out.processed);
  const auto& views=out.result.camera_point_refinement.views;ASSERT_EQ(3u,views.size());
  for(const auto& v:views) {
    const int index=v.camera_id=="camera_front"?1:v.camera_id=="camera_left"?2:3;
    EXPECT_EQ(Ros(800)-index*10*kMs,v.image_stamp_ns)<<v.camera_id;
    EXPECT_GT(v.projected_points,0)<<v.camera_id;
    EXPECT_NE("no_current_synchronized_camera",v.reason)<<v.camera_id;
  }
}

TEST(RuntimeConfig, LegacyGroundConfigKeepsExplicitCompatibilityPolicy) {
  TempDir dir;
  auto yaml=YAML::LoadFile(ShippedConfig());
  yaml["ground"].remove("ground_remove_below_plane");
  auto cfg=sh::LoadRuntimeConfig(dir.Write("legacy-ground.yaml",yaml),PackageDir());
  EXPECT_FALSE(cfg.pipeline.ground.remove_below_plane);
}

TEST(LidarRuntime, MeasuredVehicleHistoryMaintainsGeometryAndClassAfterCameraMisses) {
  auto cfg=ShippedRuntimeConfig();cfg.pipeline.camera_point_refinement.enabled=true;
  cfg.pipeline.camera_point_refinement.vehicle_history_enabled=true;cfg.fusion.views="all";
  auto k=FrontCalibration();asmc::lidar::camera::Calibrations cameras{{"camera_front",k}};
  struct Source:pl::fusion::CameraSource {
    asmc::lidar::camera::Calibration calibration;pl::ScanResult previous;bool empty=false;
    pl::fusion::CameraSnapshot Read(std::int64_t stamp,std::int64_t,std::int64_t,std::int64_t) override {
      pl::fusion::CameraSnapshot s;s.active=s.gate_active=true;s.epoch=s.gate_epoch=1;
      pl::fusion::CameraFrame f;f.stamp_ns=stamp;f.epoch=1;f.frame_id="camera_front";f.width=1280;f.height=720;
      f.identity="front-"+std::to_string(stamp);
      if(previous.processed&&!empty) {
        std::vector<pl::fusion::LidarImageBox> boxes;
        pl::fusion::LidarToImage(previous.objects,previous.context,calibration,pl::fusion::FusionConfig{},&boxes);
        for(const auto& b:boxes) {
          asmc::lidar::camera::Box box;box.xmin=b.min_x;box.ymin=b.min_y;box.xmax=b.max_x;box.ymax=b.max_y;
          box.source_class_name="car";box.class_name="vehicle";box.confidence=.95;box.confidence_valid=true;f.boxes.push_back(box);
        }
      }
      s.frames={f};s.by_view["camera_front"]={f};return s;
    }
  };
  auto source=std::make_shared<Source>();source->calibration=k;
  sh::LidarRuntime rt(cfg,source,&k,cameras);Sim sim(rt);bool saw=false;
  for(int t=300;t<=4400;t+=100) {
    source->empty=t>=3300;sim.RunTo(t);const auto& out=sim.scans.at(t);source->previous=out.result;
    if(t<3800)continue;
    for(const auto& o:out.result.objects)if(o.tracking.s32_MissCount==0&&o.detection.measured_vehicle_track_id==o.tracking.s32_Id) {
      saw=true;EXPECT_EQ("vehicle",o.camera.str_ObjectClass);EXPECT_EQ(-1,o.detection.camera_point_support.box_index);
      EXPECT_GT(o.camera.class_miss_counter,3);EXPECT_GE(o.detection.pcl_ClusterPoints.size(),10u);
    }
  }
  EXPECT_TRUE(saw);
}

TEST(RuntimeConfig, CanonicalRearExtentChangesOnlyRearSelfMask) {
  auto config = ShippedRuntimeConfig();
  const auto current = config.pipeline.vehicle;
  auto legacy = current; legacy.f32_RearOverhang = .700f;
  using namespace asmc::lidar::pipeline;
  RoiClassifier now(config.pipeline.range, config.pipeline.roi, current);
  RoiClassifier before(config.pipeline.range, config.pipeline.roi, legacy);
  auto point = [&](float base_x, float base_y) {
    pcl::PointXYZI p; p.x=base_x-current.st_LiDARPos.f32_X;
    p.y=base_y-current.st_LiDARPos.f32_Y; p.z=0; p.intensity=1; return p;
  };
  EXPECT_EQ(before(point(-.745f,0)), RoiReason::Keep);
  EXPECT_EQ(now(point(-.745f,0)), RoiReason::Ego);
  EXPECT_EQ(now(point(-.791f,0)), RoiReason::Keep);
  EXPECT_EQ(now(point(-.789f,0)), RoiReason::Ego);
  EXPECT_EQ(now(point(3.844f,0)), RoiReason::Ego);
  EXPECT_EQ(now(point(3.846f,0)), RoiReason::Keep);
  EXPECT_EQ(now(point(1.f,.947f)), RoiReason::Keep);
  EXPECT_EQ(now(point(1.f,.945f)), RoiReason::Ego);
}
