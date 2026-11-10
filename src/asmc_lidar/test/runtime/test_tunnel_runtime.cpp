#include <asmc_lidar/ros/runtime.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <functional>
#include <map>
#include <string>
namespace {
namespace pl = asmc::lidar::pipeline;
namespace sh = asmc::lidar::runtime;
using Array = asmc_msgs::LidarObjectArray;
sh::RuntimeConfig Config() {
  std::string dir=__FILE__;
  for(int i=0;i<3;++i)dir=dir.substr(0,dir.find_last_of('/'));
  auto cfg=sh::LoadRuntimeConfig(dir+"/config/lidar_pipeline.yaml",dir);
  cfg.enable_camera_fusion=false;
  return cfg;
}
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


void Denied(const sh::ScanOutput& out) {
  ASSERT_TRUE(out.processed) << out.reason;
  EXPECT_TRUE(out.result.tunnel.ran);
  EXPECT_FALSE(out.result.context.map_pose.valid);
  EXPECT_EQ(out.arrays.local.header.frame_id,"base_link");
  EXPECT_NE(out.arrays.local.status,Array::INPUT_INVALID);
  EXPECT_EQ(out.arrays.map.status,Array::POSE_UNAVAILABLE);
  EXPECT_TRUE(out.arrays.map.objects.empty());
  EXPECT_TRUE(out.arrays.map_tracking.empty());
}
void Normal(const sh::ScanOutput& out) {
  ASSERT_TRUE(out.processed) << out.reason;
  EXPECT_FALSE(out.result.tunnel.ran);
  EXPECT_TRUE(out.result.context.map_pose.valid);
  EXPECT_NE(out.arrays.map.status,Array::POSE_UNAVAILABLE);
  EXPECT_EQ(out.arrays.map.header.frame_id,"map");
}
TEST(TunnelRuntime, ShippedDefaultsAutomaticallySelectDeniedAtStartup) {
  auto cfg=Config(); ASSERT_TRUE(cfg.pipeline.tunnel.enabled);
  ASSERT_TRUE(cfg.pipeline.tunnel_route.original_pipeline);
  sh::LidarRuntime runtime(cfg,nullptr,nullptr); Sim sim(runtime);
  sim.gps=[](int){return Mode::kSilent;};sim.RunTo(600);
  for(const auto& row:sim.scans)Denied(row.second);
}
TEST(TunnelRuntime, LostPacketsExpireAndThreeGoodGpsSamplesRestoreNormal) {
  sh::LidarRuntime runtime(Config(),nullptr,nullptr); Sim sim(runtime);
  sim.gps=[](int t){return t>=600 && t<1400?Mode::kSilent:Mode::kValid;};
  sim.RunTo(1900);Normal(sim.scans.at(500));
  // Last GPS at 550ms; existing 300ms age bound expires by 900ms.
  Normal(sim.scans.at(800));Denied(sim.scans.at(900));Denied(sim.scans.at(1400));
  Denied(sim.scans.at(1500));Denied(sim.scans.at(1600));Normal(sim.scans.at(1700));
}
TEST(TunnelRuntime, ZeroCoordinatesImmediatelyInvalidatePreviouslyGoodGps) {
  sh::LidarRuntime runtime(Config(),nullptr,nullptr);Sim sim(runtime);sim.RunTo(500);
  Normal(sim.scans.at(500));
  pl::GpsInput zero;zero.stamp_ns=Ros(510);zero.status=1;
  runtime.OnGps(zero,Steady(510)+kMs,Ros(510)+kMs);
  sim.gps=[](int){return Mode::kSilent;};sim.RunTo(600);
  Denied(sim.scans.at(600));
}
} // namespace
int main(int argc,char**argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}

TEST(TunnelSparseConfig, ShippedValuesValidationAndIdentity) {
  auto cfg=Config();auto& p=cfg.pipeline.tunnel_route.params;
  EXPECT_EQ(10,p.cluster_min_points);EXPECT_TRUE(p.sparse_cluster_enable);
  EXPECT_EQ(2,p.sparse_cluster_min_points);EXPECT_FLOAT_EQ(1.5f,p.sparse_cluster_wall_clearance_m);
  sh::FinalizeRuntimeIdentity(cfg,"camera");const auto original=cfg.config_id;
  p.sparse_cluster_enable=false;sh::FinalizeRuntimeIdentity(cfg,"camera");EXPECT_NE(original,cfg.config_id);
  p.sparse_cluster_enable=true;p.sparse_cluster_wall_clearance_m=1.6f;
  sh::FinalizeRuntimeIdentity(cfg,"camera");EXPECT_NE(original,cfg.config_id);
  p.sparse_cluster_wall_clearance_m=1.5f;p.sparse_cluster_min_points=3;
  sh::FinalizeRuntimeIdentity(cfg,"camera");EXPECT_NE(original,cfg.config_id);
  p.sparse_cluster_min_points=10;EXPECT_THROW(pl::TunnelRoute route(cfg.pipeline.tunnel_route),std::invalid_argument);
  p.sparse_cluster_enable=false;p.cluster_min_points=3;
  EXPECT_NO_THROW(pl::TunnelRoute route(cfg.pipeline.tunnel_route));
  p.sparse_cluster_min_points=1;EXPECT_THROW(pl::TunnelRoute route(cfg.pipeline.tunnel_route),std::invalid_argument);
  p.sparse_cluster_min_points=2;p.sparse_cluster_wall_clearance_m=NAN;
  EXPECT_THROW(pl::TunnelRoute route(cfg.pipeline.tunnel_route),std::invalid_argument);
}

TEST(TunnelScopedConfig, SelectedProfileRestoresNormalAndBindsScopeIdentity) {
  auto cfg=Config();std::string root=__FILE__;for(int i=0;i<3;++i)root=root.substr(0,root.find_last_of('/'));
  auto scoped=sh::LoadRuntimeConfig(root+"/config/lidar_wall_vehicle.yaml",root);
  EXPECT_TRUE(scoped.pipeline.tunnel_only_updates);
  EXPECT_EQ(5,scoped.pipeline.clustering.s32_MinPoints);
  EXPECT_DOUBLE_EQ(.30,scoped.pipeline.ground.near_threshold_m);
  EXPECT_TRUE(scoped.pipeline.tunnel_route.params.sparse_cluster_enable);
  EXPECT_EQ(10,scoped.pipeline.tunnel_route.params.cluster_min_points);
  sh::FinalizeRuntimeIdentity(scoped,"camera");const auto id=scoped.config_id;
  scoped.pipeline.tunnel_only_updates=false;sh::FinalizeRuntimeIdentity(scoped,"camera");EXPECT_NE(id,scoped.config_id);
  auto config=cfg.pipeline.tunnel_route;config.params.sparse_cluster_enable=true;config.params.wall_geometry_precedence=true;
  pl::TunnelRoute route(config);route.SetUpdatesAllowed(false);
  EXPECT_FALSE(route.wall_core().GetParams().sparse_cluster_enable);
  EXPECT_FALSE(route.wall_core().GetParams().wall_geometry_precedence);
  route.SetUpdatesAllowed(true);EXPECT_TRUE(route.wall_core().GetParams().sparse_cluster_enable);
}
