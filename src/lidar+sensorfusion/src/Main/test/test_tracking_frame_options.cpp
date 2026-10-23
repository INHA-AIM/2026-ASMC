// Cause tests for the stage-1 tracking options (Lidar/Tracking/TrackTrace.hpp tracking_options).
// ROS-free: no ros::init, no master. Seam: frame_boundary::ConfigureForTest (common geometry ON).
// Every scan runs the production chain PrepareScan -> StoreLeveling -> CompareCentroids
// (predictKalmanPosition, stabilizeMatchedOBB, kalman_filter_update/predict, absolute velocity)
// -> RecordTrackingFrame with IDEAL measurements: the point mean of the detection equals the exact
// object reference point in the tracking input frame (DIAGNOSTIC_ONLY, no clustering, no noise).
//
// "Fix" tests assert the corrected behaviour with the option ON. Setting the environment variable
// TRACKING_OPTIONS_FORCE_OFF=1 runs the same assertions with the option OFF (= P0 code path, proven
// output-identical to P0 by the replay identity runs) and they must FAIL: that is the before/after
// evidence. "Guard" tests pin behaviour that must hold in both modes.
#include <gtest/gtest.h>

#include <Geometry/FrameBoundary.hpp>
#include <Lidar/FrameFilter/FrameFilter.hpp>
#include <Lidar/Kalman/kalman_utils.hpp>
#include <Lidar/Tracking/TrackTrace.hpp>
#include <Lidar/Tracking/tracking_utils.hpp>
#include <asmc_lidar/projection.hpp>

#include <cmath>
#include <cstdlib>
#include <deque>
#include <functional>
#include <vector>

extern VehicleSpec st_VehicleSpec;

namespace {
using asmc::lidar::Vec;
constexpr double kDt = 0.1;
constexpr std::int64_t kSec = 1000000000LL;

bool ForceOff() { const char* v = std::getenv("TRACKING_OPTIONS_FORCE_OFF"); return v && std::string(v) == "1"; }

void ConfigureCase(bool reacquire, bool ego_translation) {
  tracking_options::Get().association_innovation_gate = false;
  frame_boundary::ConfigureForTest(true, false, "unit_test_calibration");
  st_VehicleSpec = VehicleSpec{};
  st_VehicleSpec.st_LiDARPos = {0.80f, 0.0f, 1.45f};
  st_VehicleSpec.st_LiDARRotationDeg = {0.f, 0.f, 0.f};
  st_VehicleSpec.st_GPSPos = {0.35f, 0.0f, 1.25f};
  st_Tracking.s32_NewTrackId = 100;
  st_Tracking.b_UseMapAssociation = true;
  st_Tracking.f32_COST_W_DIST = 1.0F; st_Tracking.f32_COST_W_IOU = 1.0F;
  st_Tracking.f32_COST_THRESHOLD = 3.0F; st_Tracking.f32_COST_INF = 9999.0F;
  st_Tracking.f32_ContinuityBonus = 0.20F;
  st_Tracking.f32_MergedTrackRetireDistance = 0.45F; st_Tracking.f32_DuplicateIouThreshold = 0.35F;
  st_Tracking.f32_DuplicateCenterDistance = 0.60F;
  tracking_options::Get().reacquire_elapsed_init = reacquire && !ForceOff();
  tracking_options::Get().ego_translation_comp = ego_translation && !ForceOff();
  tracking_options::Get().ego_gap_reset = false;
  tracking_options::Get().two_point_init_cov = false;
}

struct Ego { double x = 0, y = 0, yaw = 0, v = 0; };
struct World { double x = 0, y = 0, vx = 0, vy = 0; };

// Ideal detection: point mean (KF measurement) = object reference point in the leveled sensor frame
// (= sensor frame here: zero roll/pitch, zero mount rotation); a 4.6 x 1.9 m perimeter for the OBB.
LiDARObjectInfo Detection(const Ego& e, const World& o) {
  const double c = std::cos(e.yaw), s = std::sin(e.yaw);
  const double dx = o.x - e.x, dy = o.y - e.y;
  const double bx = c * dx + s * dy, by = -s * dx + c * dy;   // base_link
  const float x = float(bx - st_VehicleSpec.st_LiDARPos.f32_X), y = float(by - st_VehicleSpec.st_LiDARPos.f32_Y);
  LiDARObjectInfo d{};
  d.f32_CoreX = x; d.f32_CoreY = y; d.f32_CoreZ = 0.f;
  d.f32_CoreE = NAN; d.f32_CoreN = NAN;
  d.f32_MinX = x - 2.3f; d.f32_MaxX = x + 2.3f; d.f32_MinY = y - 0.95f; d.f32_MaxY = y + 0.95f;
  d.f32_OBB_CenterX = x; d.f32_OBB_CenterY = y; d.f32_OBB_Width = 4.6f; d.f32_OBB_Height = 1.9f;
  for (int i = 0; i <= 10; ++i) for (int side = -1; side <= 1; side += 2) {
    pcl::PointXYZI p; p.x = x - 2.3f + 0.46f * i; p.y = y + 0.95f * side; p.z = 0.f; p.intensity = 1.f;
    d.pcl_ClusterPoints.push_back(p);
  }
  d.s32_Id = -1; d.b_KfInitialized = false; d.s32_KfInitCount = 0;
  return d;
}

GPS_IMU Local(const Ego& e) {
  GPS_IMU out{};
  out.st_IMU.b_ImuReady = true;
  out.st_IMU.f32_Yaw_rad = float(e.yaw); out.st_IMU.f32_Yaw_deg = float(e.yaw * 180. / M_PI);
  out.st_CAN.f32_CurrentVel_MPS = float(e.v); out.st_CAN.f32_CurrentVel_KPH = float(e.v * 3.6);
  return out;
}

struct Result { std::vector<int> ids; std::vector<double> abs_err; LiDARObjectInfo last; };

// Runs n scans. visible(k) = whether the object is detected in scan k. ego(k)/object(k) give the truth.
Result Simulate(int n, const std::function<Ego(int)>& ego, const std::function<World(int)>& object,
           const std::function<bool(int)>& visible, bool include_anchor=true,
           const std::function<double(int)>& clock=[](int k){return k*kDt;}) {
  Result r;
  // Production FrameFilter (consolidation, first-scan births, CompareCentroids) with the global scan clock
  // exactly as LidarProcess drives it. A far stationary 'anchor' car is always visible, as in real scans,
  // so no scan has zero detections; the target detection carries u16_Idx 0.
  std::deque<std::vector<LiDARObjectInfo>> history;
  ts_PrevTime = ros::Time();
  World anchor; anchor.x = -20; anchor.y = -25;
  for (int k = 0; k < n; ++k) {
    const Ego e = ego(k); const World o = object(k);
    const GPS_IMU local = Local(e);
    const std::int64_t ns = 50 * kSec + std::int64_t(clock(k) * 1e9 + 0.5);
    ts_CurrTime.fromNSec(ns);
    asmc::lidar::PoseSelection no_map; no_map.reason = "gps_missing";
    frame_boundary::PrepareScan(asmc::lidar::Observation{ns, ns, ns, "ros", "bridge_receive_ns", ""}, local, st_VehicleSpec, no_map, k + 1);
    frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(0., 0., asmc::lidar::Rot::Identity()));
    std::vector<LiDARObjectInfo> now;
    if (visible(k)) { now.push_back(Detection(e, o)); now.back().u16_Idx = 0; }
    if(include_anchor) {now.push_back(Detection(e, anchor)); now.back().u16_Idx = 1;}
    FrameFilter(history, now, local, st_VehicleSpec);
    ts_PrevTime = ts_CurrTime;
    for (auto& t : now) if (t.s32_MissCount == 0) frame_boundary::RecordTrackingFrame(t);
    int id = -1;
    for (const auto& t : now) if (t.s32_MissCount == 0 && t.u16_Idx == 0) { id = t.s32_Id; r.last = t; }
    r.ids.push_back(id);
    if (id >= 0 && r.last.s32_KfInitCount >= 2) {
      // truth: object ground velocity in ego axes (tracking input axes = base axes here)
      const double c = std::cos(e.yaw), s = std::sin(e.yaw);
      const double tvx = c * o.vx + s * o.vy, tvy = -s * o.vx + c * o.vy;
      r.abs_err.push_back(std::hypot(r.last.f32_AbsVx - tvx, r.last.f32_AbsVy - tvy));
    } else r.abs_err.push_back(NAN);
    history.push_back(now);
    if (history.size() > 3) history.pop_front();
  }
  return r;
}

double MaxLast(const std::vector<double>& v, int count) {
  double m = 0; int seen = 0;
  for (auto it = v.rbegin(); it != v.rend() && seen < count; ++it) if (std::isfinite(*it)) { m = std::max(m, *it); ++seen; }
  return m;
}

Ego Straight(int k, double v0, double a = 0.0) {
  const double t = k * kDt; Ego e; e.v = v0 + a * t; e.x = v0 * t + 0.5 * a * t * t; return e;
}
Ego Arc(int k, double v, double w) {
  const double t = k * kDt; Ego e; e.v = v; e.yaw = w * t;
  e.x = v / w * std::sin(w * t); e.y = v / w * (1 - std::cos(w * t)); return e;
}

// ---------------------------------------------------------------------------------- reacquire (A)
// New track (one observation) missed for 2 scans, re-acquired in scan 3 while the ego drives 5 m/s
// past a parked car. P0 divides the 3-scan displacement by one scan dt: 3x the relative speed.
double ReacquireAbsError(bool option) {
  ConfigureCase(option, false);
  auto r = Simulate(4, [](int k) { return Straight(k, 5.0); }, [](int) { World w; w.x = 25; w.y = 4; return w; },
               [](int k) { return k == 0 || k == 3; });
  EXPECT_EQ(r.ids[0], r.ids[3]) << "re-acquisition must keep the id";
  return r.abs_err[3];
}
TEST(TrackingFrameOptions, FixReacquiredNewTrackVelocityUsesElapsedTime) {
  EXPECT_LT(ReacquireAbsError(true), 0.05);
}
TEST(TrackingFrameOptions, GuardP0ReacquireDefectIsReproduced) {
  if (ForceOff()) GTEST_SKIP();
  EXPECT_GT(ReacquireAbsError(false), 5.0);   // documents the P0 defect: |abs v| = 10 m/s on a parked car
}
TEST(TrackingFrameOptions, GuardReacquireOptionIsNeutralWithoutMisses) {
  ConfigureCase(false, false);
  auto a = Simulate(20, [](int k) { return Straight(k, 7.0); }, [](int) { World w; w.x = 30; w.y = -3; return w; }, [](int) { return true; });
  ConfigureCase(true, false);
  auto b = Simulate(20, [](int k) { return Straight(k, 7.0); }, [](int) { World w; w.x = 30; w.y = -3; return w; }, [](int) { return true; });
  for (int j = 0; j < 6; ++j) EXPECT_FLOAT_EQ(a.last.Eigen_KfState_CA(j), b.last.Eigen_KfState_CA(j));
}

// ---------------------------------------------------------------------------- ego translation (B)
TEST(TrackingFrameOptions, FixParkedCarHasNoFakeSpeedWhileEgoTurns) {
  ConfigureCase(true, true);
  auto r = Simulate(40, [](int k) { return Arc(k, 8.0, 0.3); }, [](int) { World w; w.x = 10; w.y = 20; return w; }, [](int) { return true; });
  for (std::size_t k = 1; k < r.ids.size(); ++k) EXPECT_EQ(r.ids[k], r.ids[0]);
  EXPECT_LT(MaxLast(r.abs_err, 20), 0.05);
}
TEST(TrackingFrameOptions, FixParkedCarHasNoFakeSpeedWhileEgoAccelerates) {
  ConfigureCase(true, true);
  auto r = Simulate(40, [](int k) { return Straight(k, 2.0, 2.0); }, [](int) { World w; w.x = 40; w.y = 5; return w; }, [](int) { return true; });
  EXPECT_LT(MaxLast(r.abs_err, 30), 0.05);
}
TEST(TrackingFrameOptions, FixMovingCarGroundVelocityWhileEgoTurns) {
  ConfigureCase(true, true);
  auto r = Simulate(40, [](int k) { return Arc(k, 8.0, 0.3); },
               [](int k) { World w; w.vx = 0.0; w.vy = 6.0; w.x = 12; w.y = -5 + 6.0 * k * kDt; return w; },
               [](int) { return true; });
  EXPECT_LT(MaxLast(r.abs_err, 20), 0.05);
}
TEST(TrackingFrameOptions, GuardStraightConstantSpeedParkedCarIsStillInBothModes) {
  for (bool on : {false, true}) {
    ConfigureCase(true, on);
    auto r = Simulate(40, [](int k) { return Straight(k, 10.0); }, [](int) { World w; w.x = 50; w.y = 6; return w; }, [](int) { return true; });
    EXPECT_LT(MaxLast(r.abs_err, 20), 0.05) << "ego_translation_comp=" << on;
  }
}
TEST(TrackingFrameOptions, GuardRotationInPlaceLeverArmHandledInBothModes) {
  for (bool on : {false, true}) {
    ConfigureCase(true, on);
    auto r = Simulate(30, [](int k) { Ego e; e.yaw = 0.4 * k * kDt; return e; }, [](int) { World w; w.x = 12; w.y = 2; return w; }, [](int) { return true; });
    EXPECT_LT(MaxLast(r.abs_err, 15), 0.05) << "ego_translation_comp=" << on;
  }
}
// The new-track prior of the ground-referenced state is 'not moving in the world': a parked car seen
// while the ego drives is predicted where it really is, so a 2-scan miss at 12 m/s keeps the id.
TEST(TrackingFrameOptions, FixFastEgoMissedParkedCarKeepsId) {
  ConfigureCase(true, true);
  auto r = Simulate(8, [](int k) { return Straight(k, 12.0); }, [](int) { World w; w.x = 45; w.y = 4; return w; },
               [](int k) { return k == 0 || k >= 3; });
  EXPECT_EQ(r.ids[3], r.ids[0]);
  EXPECT_EQ(r.ids[7], r.ids[0]);
}
TEST(TrackingFrameOptions, GuardEmptyDetectionScansKeepGroundTrackWithoutAnchor) {
  ConfigureCase(true,true);
  auto r=Simulate(8,[](int k){return Straight(k,5.);},[](int){World w;w.x=25;w.y=4;return w;},
                 [](int k){return k==0 || k>=3;},false);
  EXPECT_EQ(r.ids[0],r.ids[3]);EXPECT_EQ(r.ids[0],r.ids[7]);
}
TEST(TrackingFrameOptions, FixUnsupportedEgoGapStartsNewGeneration) {
  ConfigureCase(true,true);
  tracking_options::Get().ego_gap_reset=!ForceOff();
  auto clock=[](int k){return k<2?k*.1:1.6+(k-2)*.1;};
  auto r=Simulate(3,[&](int k){Ego e;e.v=2;e.x=2*clock(k);return e;},
                 [](int){World w;w.x=25;w.y=4;return w;},[](int){return true;},false,clock);
  EXPECT_NE(r.ids[1],r.ids[2]);EXPECT_EQ(r.last.s32_KfInitCount,1);
}
TEST(TrackingFrameOptions, GuardBWithoutGapResetReproducesFakeVelocity) {
  if(ForceOff()) GTEST_SKIP() << "isolated B control requires B to remain enabled";
  ConfigureCase(true,true); // A+B ON; only the new gap reset remains OFF.
  auto clock=[](int k){return k<2?k*.1:1.6+(k-2)*.1;};
  auto r=Simulate(3,[&](int k){Ego e;e.v=2;e.x=2*clock(k);return e;},
                 [](int){World w;w.x=25;w.y=4;return w;},[](int){return true;},false,clock);
  EXPECT_EQ(r.ids[1],r.ids[2]);
  EXPECT_GT(std::hypot(r.last.f32_AbsVx,r.last.f32_AbsVy),.5);
}
}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// Isolate yaw toggle: identical observations, A+B, Q/R, association and lifecycle.
namespace {
LiDARObjectInfo YawPair(bool enabled,double raw_yaw) {
  ConfigureCase(true,true);
  tracking_options::Get().yaw_ema_enabled=enabled;
  std::deque<std::vector<LiDARObjectInfo>> history;
  LiDARObjectInfo result{};
  ts_PrevTime=ros::Time();
  for (int k=0;k<2;++k) {
    Ego e;e.yaw=k*.1;World world;world.x=15;world.y=2;
    auto local=Local(e);
    const std::int64_t ns=80*kSec+k*100000000LL;ts_CurrTime.fromNSec(ns);
    asmc::lidar::PoseSelection no_map;no_map.reason="gps_missing";
    frame_boundary::PrepareScan(asmc::lidar::Observation{ns,ns,ns,"ros","test",""},local,st_VehicleSpec,no_map,k+1);
    frame_boundary::StoreLeveling(asmc::lidar::Rot::Identity());
    auto d=Detection(e,world);const double yaw=k?raw_yaw:.2;
    for(auto& p:d.pcl_ClusterPoints) {
      double x=p.x-d.f32_CoreX,y=p.y-d.f32_CoreY;
      p.x=d.f32_CoreX+std::cos(yaw)*x-std::sin(yaw)*y;
      p.y=d.f32_CoreY+std::sin(yaw)*x+std::cos(yaw)*y;
    }
    d.f32_OBB_YawRad=yaw;d.u16_Idx=0;
    std::vector<LiDARObjectInfo> now{d};FrameFilter(history,now,local,st_VehicleSpec);
    EXPECT_EQ(1u,now.size());
    if(now.empty())return result;
    frame_boundary::RecordTrackingFrame(now[0]);result=now[0];
    history.push_back(now);ts_PrevTime=ts_CurrTime;
  }
  tracking_options::Get().yaw_ema_enabled=true;
  return result;
}
}
TEST(YawEmaOnly, RawPiAxisAndSingleEgoRotation) {
  const auto on=YawPair(true,.6),off=YawPair(false,.6),wrapped=YawPair(false,.6+M_PI);
  EXPECT_NEAR(.225,on.f32_OBB_YawRad,1e-5); // (.2-.1)+.25*(.6-(.2-.1))
  EXPECT_NEAR(.6,off.f32_OBB_YawRad,1e-5);
  EXPECT_NEAR(0.,std::remainder(off.f32_OBB_YawRad-wrapped.f32_OBB_YawRad,M_PI),1e-5);
  EXPECT_NEAR(off.f32_OBB_CenterX,wrapped.f32_OBB_CenterX,1e-5);
  EXPECT_NEAR(off.f32_OBB_Width,wrapped.f32_OBB_Width,1e-5);
  EXPECT_EQ(on.s32_Id,off.s32_Id);EXPECT_EQ(on.s32_KfInitCount,off.s32_KfInitCount);
  EXPECT_LT((on.Eigen_KfState_CA-off.Eigen_KfState_CA).norm(),1e-6);
  EXPECT_LT((on.Eigen_KfP_CA-off.Eigen_KfP_CA).norm(),1e-6);
}

TEST(TrackingFrameOptions, SignedReverseEgoKeepsParkedGroundVelocityZero) {
  ConfigureCase(true,true);
  auto r=Simulate(30,[](int k){return Straight(k,-2.);},
      [](int){World w;w.x=15;w.y=3;return w;},[](int){return true;});
  EXPECT_LT(MaxLast(r.abs_err,10),.05);
}

#include "../src/Lidar/Tracking/ObbAxisCanonical.hpp"
TEST(ObbAxisCandidate, ExactCornersInvariantAndIdempotent) {
  struct Point {double x,y,z;};
  struct Box {double w,h,yaw;std::array<Point,4> corners;};
  Box box{1.,4.,0.,{{{-0.5,-2,0},{0.5,-2,0},{0.5,2,0},{-0.5,2,0}}}};
  std::array<Point,8> xyz;
  for(int i=0;i<8;++i){xyz[i]=box.corners[i%4];xyz[i].z=i<4?0:2;}
  const auto old=xyz;
  EXPECT_TRUE(obb_axis::Canonicalize(box,xyz));
  EXPECT_DOUBLE_EQ(4.,box.w);EXPECT_DOUBLE_EQ(1.,box.h);
  EXPECT_NEAR(M_PI/2,std::abs(box.yaw),1e-12);
  for(const auto& p:old) {
    int found=0;for(const auto& q:xyz)if(p.x==q.x&&p.y==q.y&&p.z==q.z)++found;
    EXPECT_EQ(1,found);
  }
  // Reconstruct from the new scalar representation independently of stored corners.
  for(const auto& pt:old) {
    const double u=pt.x*std::cos(box.yaw)+pt.y*std::sin(box.yaw);
    const double v=-pt.x*std::sin(box.yaw)+pt.y*std::cos(box.yaw);
    EXPECT_NEAR(box.w/2,std::abs(u),1e-12);
    EXPECT_NEAR(box.h/2,std::abs(v),1e-12);
  }
  EXPECT_FALSE(obb_axis::Canonicalize(box,xyz));
}
TEST(ObbAxisCandidate, LongAxisAndSquareRemainUnchanged) {
  struct P {double x,y;};
  struct Box {double w,h,yaw;std::array<P,4> corners;};
  for(double h:{1.,4.}) {
    Box b{4.,h,.7,{}};std::array<P,8> c{};
    EXPECT_FALSE(obb_axis::Canonicalize(b,c));EXPECT_DOUBLE_EQ(.7,b.yaw);
  }
}

#include "../src/Lidar/Tracking/AssociationInnovation.hpp"
namespace {
int InnovationPair(bool enabled, double dx, double dy, double variance=.01, double vx=0.,
                   double ego_yaw=0.) {
  ConfigureCase(true,true);
  tracking_options::Get().association_innovation_gate=enabled;
  st_Tracking.s32_NewTrackId=101;
  Ego e0; World world;world.x=20;world.y=4;
  auto previous=Detection(e0,world);
  previous.s32_Id=100;previous.s32_Age=20;previous.b_KfInitialized=true;previous.s32_KfInitCount=20;
  previous.Eigen_KfState_CA<<previous.f32_CoreX,previous.f32_CoreY,vx,0,0,0;
  previous.Eigen_KfP_CA=Eigen::Matrix<float,6,6>::Identity()*variance;
  const std::int64_t ns=100*kSec;ts_CurrTime.fromNSec(ns);
  asmc::lidar::PoseSelection no_map;no_map.reason="test";
  auto prepare=[&](std::int64_t stamp,const Ego& ego) {
    ts_CurrTime.fromNSec(stamp);
    frame_boundary::PrepareScan(asmc::lidar::Observation{stamp,stamp,stamp,"ros","test",""},Local(ego),
        st_VehicleSpec,no_map,1);
    frame_boundary::StoreLeveling(asmc::lidar::Rot::Identity());
  };
  prepare(ns,e0);frame_boundary::RecordTrackingFrame(previous);
  Ego e1;e1.yaw=ego_yaw;
  prepare(ns+100000000LL,e1);
  world.x+=vx*.1;
  auto detection=Detection(e1,world);
  detection.f32_CoreX+=dx;detection.f32_CoreY+=dy;detection.u16_Idx=0;
  for(auto& p:detection.pcl_ClusterPoints){p.x+=dx;p.y+=dy;}
  detection.f32_OBB_CenterX+=dx;detection.f32_OBB_CenterY+=dy;
  std::deque<std::vector<LiDARObjectInfo>> history{{previous}};
  std::vector<LiDARObjectInfo> now{detection};
  CompareCentroids(history,now,.1,Local(e1));
  int id=-1;
  for(const auto& t:now)if(t.s32_MissCount==0 && t.u16_Idx==0)id=t.s32_Id;
  tracking_options::Get().association_innovation_gate=false;
  return id;
}
}
TEST(AssociationInnovation, OffReproducesIncompatibleStateInheritance) {
  EXPECT_EQ(100,InnovationPair(false,.881,-1.392));
}
TEST(AssociationInnovation, OnRejectsIncompatibleStateInheritance) {
  EXPECT_NE(100,InnovationPair(true,.881,-1.392));
}
TEST(AssociationInnovation, UncertainCoastedTrackMayReacquire) {
  EXPECT_EQ(100,InnovationPair(true,.881,-1.392,1.));
}
TEST(AssociationInnovation, FastCorrectPredictionIsAccepted) {
  EXPECT_EQ(100,InnovationPair(true,.03,-.02,.01,25.));
}
TEST(AssociationInnovation, ExistingFiniteRotationIsUsedOnce) {
  EXPECT_EQ(100,InnovationPair(true,.03,-.02,.01,5.,.2));
}
TEST(AssociationInnovation, InvalidCovarianceDoesNotSilentlyDeleteDetection) {
  EXPECT_EQ(100,InnovationPair(true,.5,0.,NAN));
}
TEST(AssociationInnovation, RealAccelerationKeepsId) {
  ConfigureCase(true,true);tracking_options::Get().association_innovation_gate=true;
  auto r=Simulate(25,[](int){return Ego{};},[](int k) {
    const double t=k*kDt;World w;w.x=20+10*t+4*t*t;w.y=3;w.vx=10+8*t;return w;
  },[](int){return true;});
  for(const auto id:r.ids)EXPECT_EQ(r.ids[0],id);
  tracking_options::Get().association_innovation_gate=false;
}
TEST(AssociationInnovation, GuardSameVehicleFaceSwitchMustKeepIdentity) {
  // One physical car, two visible surfaces: centroid moves 0.8 m without physical motion.
  // This required guard intentionally exposes whether a statistical gate rejects fragment changes.
  EXPECT_EQ(100,InnovationPair(true,.8,0.));
}

namespace {
using ObservedIds=std::vector<std::vector<int>>;
ObservedIds FragmentScenario(bool split,bool occlude,bool neighbors) {
  ConfigureCase(true,true);tracking_options::Get().association_innovation_gate=true;
  std::deque<std::vector<LiDARObjectInfo>> history;
  ObservedIds ids;
  ts_PrevTime=ros::Time();
  for(int k=0;k<18;++k) {
    Ego ego;auto local=Local(ego);const std::int64_t ns=150*kSec+k*100000000LL;
    ts_CurrTime.fromNSec(ns);asmc::lidar::PoseSelection no_map;no_map.reason="test";
    frame_boundary::PrepareScan(asmc::lidar::Observation{ns,ns,ns,"ros","test",""},local,st_VehicleSpec,no_map,k+1);
    frame_boundary::StoreLeveling(asmc::lidar::Rot::Identity());
    std::vector<LiDARObjectInfo> now;
    auto add=[&](double x,double y,int index,double length=4.6,double width=1.9) {
      World w;w.x=x;w.y=y;auto d=Detection(ego,w);d.u16_Idx=index;
      for(auto& p:d.pcl_ClusterPoints) {
        p.x=d.f32_CoreX+(p.x-d.f32_CoreX)*length/4.6;
        p.y=d.f32_CoreY+(p.y-d.f32_CoreY)*width/1.9;
      }
      d.f32_OBB_Width=length;d.f32_OBB_Height=width;
      d.f32_MinX=d.f32_CoreX-length/2;d.f32_MaxX=d.f32_CoreX+length/2;
      d.f32_MinY=d.f32_CoreY-width/2;d.f32_MaxY=d.f32_CoreY+width/2;
      now.push_back(d);
    };
    if(!(occlude&&k>=8&&k<=10)) {
      // Two point sets are the left/right halves of ONE fixed 4.6 m physical rectangle.
      if(split&&k>=8&&k<=10){add(18.85,3,0,2.3);add(21.15,3,1,2.3);}
      else add(20,3,0);
    }
    if(neighbors) {add(20,6.2,2);if(k>=4)add(22,0,3,.6,.6);}
    add(-20,-20,9); // Keeps unrelated detections alive through an occlusion.
    FrameFilter(history,now,local,st_VehicleSpec);
    std::vector<int> row(4,-1);
    for(auto& t:now)if(t.s32_MissCount==0) {
      frame_boundary::RecordTrackingFrame(t);
      if(t.u16_Idx<4)row[t.u16_Idx]=t.s32_Id;
    }
    ids.push_back(row);history.push_back(now);if(history.size()>3)history.pop_front();
    ts_PrevTime=ts_CurrTime;
  }
  tracking_options::Get().association_innovation_gate=false;
  return ids;
}
}
TEST(AssociationInnovation, GuardAdjacentVehiclesPedestrianAndNormalNewBirth) {
  const auto ids=FragmentScenario(false,false,true);
  EXPECT_NE(ids[4][3],ids[4][0]);EXPECT_NE(ids[4][3],ids[4][2]);
  for(int k=1;k<18;++k){EXPECT_EQ(ids[0][0],ids[k][0]);EXPECT_EQ(ids[0][2],ids[k][2]);}
  for(int k=5;k<18;++k)EXPECT_EQ(ids[4][3],ids[k][3]);
}
TEST(AssociationInnovation, GuardShortOcclusionReacquiresSameIdentity) {
  const auto ids=FragmentScenario(false,true,false);
  EXPECT_EQ(ids[0][0],ids[11][0]);EXPECT_EQ(ids[0][0],ids[17][0]);
}
TEST(AssociationInnovation, GuardSplitRejoinRetainsPhysicalObjectIdentity) {
  const auto ids=FragmentScenario(true,false,false);
  for(int k=8;k<=10;++k)EXPECT_TRUE(ids[k][0]==ids[0][0] || ids[k][1]==ids[0][0]);
  EXPECT_EQ(ids[0][0],ids[11][0]);
}
TEST(AssociationInnovation, GuardFaceSwitchWhileEgoRotatesKeepsIdentity) {
  EXPECT_EQ(100,InnovationPair(true,.8,0.,.01,0.,.2));
}
