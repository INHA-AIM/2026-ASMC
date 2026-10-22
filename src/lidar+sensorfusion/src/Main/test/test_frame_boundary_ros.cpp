#include <gtest/gtest.h>
#include <Geometry/FrameBoundaryPublisher.hpp>
#include <Parser/SensorPoseHistory.hpp>
#include <Parser/Parser.hpp>
#include <Camera/Camera.hpp>
#include <Lidar/Tracking/tracking_utils.hpp>
#include <Lidar/Kalman/kalman_utils.hpp>
#include <Lidar/ImuRansac/ImuRansac.hpp>
#include <ros/package.h>
#include <mutex>
#include <functional>
#include <limits>
#include <algorithm>

extern VehicleSpec st_VehicleSpec;
namespace {
void Configure(bool level_roll=true) {
  ros::NodeHandle node("~"); node.setParam("enable_common_geometry",true); node.setParam("lidar_level_roll",level_roll);
  st_VehicleSpec.st_LiDARPos={.8f,0,1.45f}; st_VehicleSpec.st_LiDARRotationDeg={0,0,0};
  st_VehicleSpec.st_GPSPos={.35f,0,1.25f};
  CAMERA camera{};
  InitProjectionMatrix(camera,YAML::LoadFile(ros::package::getPath("Main")+"/setting.yaml"));
  frame_boundary::Configure(node,st_VehicleSpec,camera);
}
void Now(double time){ros::Time::setNow(ros::Time(time));}
GPS_IMU Local(double yaw=0.) {
  GPS_IMU out{}; out.st_IMU.b_ImuReady=true;
  out.st_IMU.f32_Yaw_rad=yaw; out.st_IMU.f32_Yaw_deg=yaw*180/M_PI;
  out.st_CAN.f32_CurrentVel_MPS=2.; out.st_CAN.f32_CurrentVel_KPH=7.2;
  return out;
}
LiDARObjectInfo Detection(double x,double y) {
  LiDARObjectInfo out{}; out.s32_Id=-1; out.str_ObjectClass="car"; out.str_YoloClass="car";
  out.f32_CoreX=out.f32_OBB_CenterX=x; out.f32_CoreY=out.f32_OBB_CenterY=y;
  out.f32_CoreZ=out.f32_OBB_CenterZ=-.2;
  out.f32_OBB_Width=1.; out.f32_OBB_Height=.8; out.f32_OBB_ZMin=-.5; out.f32_OBB_ZMax=.5;
  out.f32_MinX=x-.5;out.f32_MaxX=x+.5;out.f32_MinY=y-.4;out.f32_MaxY=y+.4;
  for(unsigned i=0;i<8;++i) {
    out.OBB_Corners_Body[i]={float(x+((i&1)? .5:-.5)),float(y+((i&2)? .4:-.4)),float((i&4)? .5:-.5)};
    pcl::PointXYZI p; p.x=out.OBB_Corners_Body[i].x; p.y=out.OBB_Corners_Body[i].y; p.z=out.OBB_Corners_Body[i].z;
    p.intensity=1; out.pcl_ClusterPoints.push_back(p);
  }
  return out;
}
void TrackingConfig() {
  st_Tracking.s32_NewTrackId=100; st_Tracking.b_UseMapAssociation=true;
  st_Tracking.f32_COST_W_DIST=1;st_Tracking.f32_COST_W_IOU=1;
  st_Tracking.f32_COST_THRESHOLD=3;st_Tracking.f32_COST_INF=9999;
  st_Tracking.f32_ContinuityBonus=.2;st_Tracking.f32_MergedTrackRetireDistance=.5;
  st_Tracking.f32_DuplicateIouThreshold=.35;
}
TEST(FrameBoundary, LocalReadinessDoesNotRequireGps) {
  Configure(); Now(20.);
  SensorPoseHistory history; const auto motion=Local();
  history.AddImu(20000,motion.st_IMU); history.AddCan(20000,motion.st_CAN);
  GPS_IMU resolved{}; std::string reason;
  ASSERT_TRUE(history.WaitForLocalLookup(20000,0,&resolved,&reason));
  EXPECT_EQ(reason,"local_ready"); EXPECT_FALSE(history.Lookup(20000,&resolved,&reason));
  EXPECT_EQ(history.MapAt(20000,resolved).reason,"gps_missing");
  EXPECT_FALSE(history.WaitForLocalLookup(20121,0,&resolved,&reason));
  EXPECT_TRUE(history.WaitForLocalLookup(20120,0,&resolved,&reason));
}
TEST(FrameBoundary, NewInvalidLocalSampleBlocksCachedGoodSample) {
  Configure();Now(20.);SensorPoseHistory history;auto motion=Local();
  history.AddImu(20000,motion.st_IMU);history.AddCan(20000,motion.st_CAN);
  auto invalid=motion.st_IMU;invalid.b_ImuReady=false;history.AddImu(20010,invalid);
  GPS_IMU resolved{};std::string reason;
  EXPECT_FALSE(history.WaitForLocalLookup(20020,0,&resolved,&reason));EXPECT_EQ(reason,"imu_input_invalid");
  history.AddImu(20021,motion.st_IMU);
  auto bad_speed=motion.st_CAN;bad_speed.f32_CurrentVel_MPS=NAN;history.AddCan(20022,bad_speed);
  EXPECT_FALSE(history.WaitForLocalLookup(20023,0,&resolved,&reason));EXPECT_EQ(reason,"speed_input_invalid");
  history.AddCan(20024,motion.st_CAN);
  EXPECT_TRUE(history.WaitForLocalLookup(20025,0,&resolved,&reason));
}
TEST(FrameBoundary, MapInvalidBarrierRecoveryAndSilence) {
  Configure(); SensorPoseHistory history; auto motion=Local();
  GPS_DATA gps{}; gps.f64_E=100;gps.f64_N=200;gps.f64_U=29.9;
  for(unsigned t:{20000,20200,20400}) {Now(t*.001);history.AddImu(t,motion.st_IMU);history.AddGps(t,gps);}
  Now(20.45); const auto valid=history.MapAt(20450,motion);
  ASSERT_TRUE(valid.valid); EXPECT_EQ(valid.reason,"gps_bounded_prediction");
  EXPECT_NEAR(valid.map_from_base.translation().z(),28.65,1e-6);
  Now(20.7); EXPECT_TRUE(history.MapAt(20700,motion).valid);
  Now(20.701); EXPECT_EQ(history.MapAt(20701,motion).reason,"gps_stale");
  Now(20.5); history.AddGpsInvalid(20500,20500);
  EXPECT_EQ(history.MapAt(20500,motion).reason,"gps_invalid");
  for(unsigned t:{20700,20900,21100}) {
    Now(t*.001);history.AddImu(t,motion.st_IMU);history.AddGps(t,gps);
    EXPECT_EQ(history.MapAt(t,motion).valid,t==21100);
  }
  Now(21.2);history.AddGpsInvalid(20800,21200);
  EXPECT_TRUE(history.MapAt(21200,motion).valid);
}
TEST(FrameBoundary, MapYawUsesGridWhileLocalYawStaysTrueEnu) {
  Configure(); SensorPoseHistoryConfig config; config.imu_yaw_is_true_enu=true;
  SensorPoseHistory history(config); auto motion=Local(M_PI/2.);
  GPS_DATA gps{}; gps.f64_E=100; gps.f64_N=200; gps.f64_U=29.9;
  gps.f64_Latitude=37.24; gps.f64_Longitude=126.77;
  for(unsigned t:{20000,20200,20400}) {
    Now(t*.001); history.AddImu(t,motion.st_IMU);
    history.AddCan(t,motion.st_CAN); history.AddGps(t,gps);
  }
  Now(20.45); const auto map=history.MapAt(20450,motion);
  ASSERT_TRUE(map.valid);
  const auto forward=map.map_from_base.rotation()*asmc::lidar::Vec::UnitX();
  EXPECT_NEAR(std::atan2(forward.y(),forward.x()),M_PI/2.-0.02356,0.0001);
  GPS_IMU local{}; ASSERT_TRUE(history.WaitForLocalLookup(20450,0,&local));
  EXPECT_NEAR(local.st_IMU.f32_Yaw_rad,M_PI/2.,1e-6);
  // A future GPS cannot change the correction of an earlier scan.
  auto future=gps; future.f64_Longitude=129.;
  history.AddImu(20500,motion.st_IMU); history.AddGps(20500,future);
  EXPECT_NEAR((history.MapAt(20450,motion).map_from_base.matrix()-map.map_from_base.matrix()).norm(),0.,1e-9);
  Now(20.5); history.AddGpsInvalid(20500,20500);
  EXPECT_FALSE(history.MapAt(20500,motion).valid);
}

TEST(FrameBoundary, LocalAssociationIgnoresMapCoordinatesIncludingNaN) {
  Configure(); TrackingConfig();
  auto previous=Detection(10,1);previous.s32_Id=42;previous.s32_Age=20;
  previous.b_KfInitialized=true;previous.s32_KfInitCount=2;
  previous.Eigen_KfState_CA<<10,1,-2,0,0,0;
  previous.f32_CoreE=100;previous.f32_CoreN=200;
  std::vector<int> ids;
  for(double map_x:{100.,1000000.,std::numeric_limits<double>::quiet_NaN()}) {
    auto current=Detection(9.8,1);current.f32_CoreE=map_x;current.f32_CoreN=map_x;
    std::deque<std::vector<LiDARObjectInfo>> past{{previous}};
    std::vector<LiDARObjectInfo> detections{current};
    CompareCentroids(past,detections,.1,Local());
    ASSERT_FALSE(detections.empty());ids.push_back(detections.front().s32_Id);
    EXPECT_EQ(detections.front().str_YoloClass,"car");
  }
  EXPECT_EQ(ids,std::vector<int>({42,42,42}));
}
TEST(FrameBoundary, LocalAssociationYawPredictionUsesCurrentBodyFrame) {
  Configure();TrackingConfig();
  auto previous=Detection(10,0);previous.s32_Id=42;previous.s32_Age=20;
  previous.b_KfInitialized=true;previous.s32_KfInitCount=2;
  previous.Eigen_KfState_CA<<10,0,0,0,0,0;
  previous.f32_CoreE=NAN;previous.f32_CoreN=NAN;
  auto current=Detection(10*std::cos(.1),-10*std::sin(.1));
  current.f32_CoreE=NAN;current.f32_CoreN=NAN;
  std::deque<std::vector<LiDARObjectInfo>> past{{previous}};
  std::vector<LiDARObjectInfo> detections{current};
  CompareCentroids(past,detections,.1,Local(.1));
  ASSERT_FALSE(detections.empty());EXPECT_EQ(detections.front().s32_Id,42);
}
TEST(FrameBoundary, TrackingReexpressesRollPitchAndNonzeroMountWithoutEgoDoubleCompensation) {
  Configure();TrackingConfig();Now(25.);
  VehicleSpec vehicle=st_VehicleSpec;
  vehicle.st_LiDARRotationDeg={4.f,-3.f,8.f};
  const auto mount=frame_boundary::SensorMount(vehicle);
  const asmc::lidar::Vec fixed_in_world(12,2,1);
  auto local=Local(.2);local.st_IMU.f32_Roll_rad=.1;local.st_IMU.f32_Pitch_rad=-.08;
  asmc::lidar::PoseSelection unavailable;unavailable.reason="gps_missing";
  frame_boundary::PrepareScan(ros::Time(25.),local,vehicle,unavailable);
  const auto old_level=asmc::lidar::leveling_rotation(.1,-.08,mount.rotation());
  frame_boundary::StoreLeveling(old_level);
  const auto old_point=(old_level*mount.inverse()*(frame_boundary::MapRotation(local).transpose()*fixed_in_world)).eval();
  auto previous=Detection(old_point.x(),old_point.y());previous.f32_CoreZ=old_point.z();
  previous.s32_Id=42;previous.b_KfInitialized=true;previous.s32_KfInitCount=2;
  previous.Eigen_KfState_CA<<old_point.x(),old_point.y(),0,0,0,0;
  previous.f32_PrevYaw=.2;frame_boundary::RecordTrackingFrame(previous);
  local=Local(.28);local.st_IMU.f32_Roll_rad=-.12;local.st_IMU.f32_Pitch_rad=.06;
  frame_boundary::PrepareScan(ros::Time(25.1),local,vehicle,unavailable);
  const auto new_level=asmc::lidar::leveling_rotation(-.12,.06,mount.rotation());
  frame_boundary::StoreLeveling(new_level);
  const auto new_point=(new_level*mount.inverse()*(frame_boundary::MapRotation(local).transpose()*fixed_in_world)).eval();
  auto aligned=previous;frame_boundary::AlignTrackingFrame(aligned,.28);
  EXPECT_NEAR(aligned.Eigen_KfState_CA(0),new_point.x(),1e-5);
  EXPECT_NEAR(aligned.Eigen_KfState_CA(1),new_point.y(),1e-5);
  EXPECT_NEAR(aligned.tracking_height_input,new_point.z(),1e-5);
  // CAN=2m/s does not subtract another .2m from the relative CA state.
  auto current=Detection(new_point.x(),new_point.y());current.f32_CoreZ=new_point.z();
  current.f32_CoreE=NAN;current.f32_CoreN=NAN;
  std::deque<std::vector<LiDARObjectInfo>> past{{previous}};
  std::vector<LiDARObjectInfo> detections{current};
  CompareCentroids(past,detections,.1,local);
  ASSERT_FALSE(detections.empty());EXPECT_EQ(detections.front().s32_Id,42);
  EXPECT_NEAR(detections.front().f32_CoreX,new_point.x(),1e-5);
}

// Tracker OBB geometry ownership. A world-fixed box is expressed in each scan
// with the scan model above, p=L*mount^-1*(R^T*w); ego translation is zero, so
// a previous-scan point reaches the current scan only through the frame change.
struct TrackScan {
  GPS_IMU local; asmc::lidar::Pose mount; asmc::lidar::Rot level;
  asmc::lidar::Vec Point(const asmc::lidar::Vec& world) const {
    return level*(mount.inverse()*(frame_boundary::MapRotation(local).transpose()*world));
  }
  asmc::lidar::Vec Direction(const asmc::lidar::Vec& world) const {
    return level*(mount.rotation().transpose()*(frame_boundary::MapRotation(local).transpose()*world));
  }
  asmc::lidar::Vec World(const asmc::lidar::Vec& point) const {
    return frame_boundary::MapRotation(local)*(mount*(level.transpose()*point));
  }
};
TrackScan BeginTrackScan(double time,double yaw,double roll,double pitch,const VehicleSpec& vehicle) {
  TrackScan scan; scan.local=Local(yaw);
  scan.local.st_IMU.f32_Roll_rad=roll; scan.local.st_IMU.f32_Pitch_rad=pitch;
  scan.mount=frame_boundary::SensorMount(vehicle);
  scan.level=asmc::lidar::leveling_rotation(scan.local.st_IMU.f32_Roll_rad,
      scan.local.st_IMU.f32_Pitch_rad,scan.mount.rotation(),frame_boundary::UseRoll());
  asmc::lidar::PoseSelection unavailable; unavailable.reason="gps_missing";
  Now(time); frame_boundary::PrepareScan(ros::Time(time),scan.local,vehicle,unavailable);
  frame_boundary::StoreLeveling(scan.level);
  return scan;
}
struct WorldBox {asmc::lidar::Vec center; double heading,length,width,height;};
asmc::lidar::Vec Heading(const WorldBox& box) {return {std::cos(box.heading),std::sin(box.heading),0};}
// computeOBBAtYaw order: 0->1 is the yaw (length) edge, 4..7 repeat 0..3 on the top face.
std::array<asmc::lidar::Vec,8> WorldCorners(const WorldBox& box) {
  const auto u=Heading(box); const asmc::lidar::Vec v(-u.y(),u.x(),0),up(0,0,.5*box.height);
  const double su[]={-1,1,1,-1},sv[]={-1,-1,1,1};
  std::array<asmc::lidar::Vec,8> out;
  for(unsigned i=0;i<4;++i) {
    const auto xy=(box.center+.5*su[i]*box.length*u+.5*sv[i]*box.width*v).eval();
    out[i]=xy-up; out[i+4]=xy+up;
  }
  return out;
}
LiDARObjectInfo BoxDetection(const TrackScan& scan,const WorldBox& box) {
  LiDARObjectInfo out{}; out.s32_Id=-1; out.str_ObjectClass="car"; out.str_YoloClass="car";
  const auto world=WorldCorners(box);
  double low=1e9,high=-1e9; out.f32_MinX=out.f32_MinY=1e9; out.f32_MaxX=out.f32_MaxY=-1e9;
  for(unsigned i=0;i<8;++i) {
    const auto p=scan.Point(world[i]);
    out.OBB_Corners_Body[i]={float(p.x()),float(p.y()),float(p.z())};
    low=std::min(low,p.z()); high=std::max(high,p.z());
    out.f32_MinX=std::min<float>(out.f32_MinX,p.x()); out.f32_MaxX=std::max<float>(out.f32_MaxX,p.x());
    out.f32_MinY=std::min<float>(out.f32_MinY,p.y()); out.f32_MaxY=std::max<float>(out.f32_MaxY,p.y());
  }
  // Cluster points: corners plus edge midpoints, all in this scan's leveled frame.
  static const int edges[12][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
  for(unsigned i=0;i<8;++i) {
    pcl::PointXYZI p; const auto q=scan.Point(world[i]); p.x=q.x(); p.y=q.y(); p.z=q.z(); p.intensity=1;
    out.pcl_ClusterPoints.push_back(p);
  }
  for(const auto& e:edges) {
    pcl::PointXYZI p; const auto q=scan.Point(.5*(world[e[0]]+world[e[1]])); p.x=q.x(); p.y=q.y(); p.z=q.z();
    p.intensity=1; out.pcl_ClusterPoints.push_back(p);
  }
  const auto center=scan.Point(box.center), axis=scan.Direction(Heading(box));
  out.f32_CoreX=out.f32_OBB_CenterX=center.x(); out.f32_CoreY=out.f32_OBB_CenterY=center.y();
  out.f32_CoreZ=out.f32_OBB_CenterZ=center.z();
  out.f32_OBB_Width=box.length; out.f32_OBB_Height=box.width; out.f32_OBB_ZMin=low; out.f32_OBB_ZMax=high;
  out.f32_OBB_YawRad=std::atan2(axis.y(),axis.x());
  out.f32_CoreE=out.f32_CoreN=NAN;
  return out;
}
// An OBB axis has no direction: compare with 180 degree equivalence.
double AxisError(double a,double b) {return std::abs(std::remainder(a-b,M_PI));}
double CornerYaw(const LiDARObjectInfo& o) {
  const auto& a=o.OBB_Corners_Body[0]; const auto& b=o.OBB_Corners_Body[1];
  return std::atan2(b.y-a.y,b.x-a.x);
}
double ScanYaw(const TrackScan& scan,const asmc::lidar::Vec& world_a,const asmc::lidar::Vec& world_b) {
  const auto d=(scan.Point(world_b)-scan.Point(world_a)).eval();
  return std::atan2(d.y(),d.x());
}
void ExpectObbSelfConsistent(const LiDARObjectInfo& o,const char* role) {
  EXPECT_LT(AxisError(o.f32_OBB_YawRad,CornerYaw(o)),1e-5)<<role<<" yaw axis vs corner 0->1 edge";
  asmc::lidar::Vec mean=asmc::lidar::Vec::Zero();
  for(const auto& c:o.OBB_Corners_Body) mean+=asmc::lidar::Vec(c.x,c.y,c.z)/8.;
  EXPECT_NEAR(mean.x(),o.f32_OBB_CenterX,1e-5)<<role; EXPECT_NEAR(mean.y(),o.f32_OBB_CenterY,1e-5)<<role;
  EXPECT_NEAR(mean.z(),o.f32_OBB_CenterZ,1e-5)<<role;
}
void ExpectCarriedBox(const LiDARObjectInfo& o,const TrackScan& scan,const WorldBox& box,double z_extent,
                      bool kf_at_center=true) {
  // Previous-scan geometry must reach this scan exactly once: center, corners,
  // yaw (and, for a zero relative velocity, the KF position) equal the box
  // expressed in this scan.
  EXPECT_NEAR(.5*(o.f32_OBB_ZMin+o.f32_OBB_ZMax),o.f32_OBB_CenterZ,1e-5);
  EXPECT_NEAR(o.f32_OBB_ZMax-o.f32_OBB_ZMin,z_extent,1e-5);
  const auto world=WorldCorners(box);
  for(unsigned i=0;i<8;++i) {
    const auto p=scan.Point(world[i]);
    EXPECT_NEAR(o.OBB_Corners_Body[i].x,p.x(),1e-5)<<"corner "<<i;
    EXPECT_NEAR(o.OBB_Corners_Body[i].y,p.y(),1e-5)<<"corner "<<i;
    EXPECT_NEAR(o.OBB_Corners_Body[i].z,p.z(),1e-5)<<"corner "<<i;
  }
  const auto center=scan.Point(box.center);
  EXPECT_NEAR(o.f32_OBB_CenterX,center.x(),1e-5); EXPECT_NEAR(o.f32_OBB_CenterY,center.y(),1e-5);
  EXPECT_NEAR(o.f32_OBB_CenterZ,center.z(),1e-5);
  if(kf_at_center) {EXPECT_NEAR(o.Eigen_KfState_CA(0),center.x(),1e-5); EXPECT_NEAR(o.Eigen_KfState_CA(1),center.y(),1e-5);}
  EXPECT_LT(AxisError(o.f32_OBB_YawRad,ScanYaw(scan,world[0],world[1])),1e-5)<<"carried yaw";
}
void ExpectCarriedFrom(const LiDARObjectInfo& o,const TrackScan& scan,const TrackScan& seen_scan,
                       const LiDARObjectInfo& seen) {
  // A track matched until seen_scan and then missed carries its last observed
  // (stabilized) OBB, re-expressed into this scan once per scan.
  std::array<asmc::lidar::Vec,8> expected;
  for(unsigned i=0;i<8;++i) {
    const auto& c=seen.OBB_Corners_Body[i];
    expected[i]=scan.Point(seen_scan.World(asmc::lidar::Vec(c.x,c.y,c.z)));
    EXPECT_NEAR(o.OBB_Corners_Body[i].x,expected[i].x(),1e-5)<<"handover corner "<<i;
    EXPECT_NEAR(o.OBB_Corners_Body[i].y,expected[i].y(),1e-5)<<"handover corner "<<i;
    EXPECT_NEAR(o.OBB_Corners_Body[i].z,expected[i].z(),1e-5)<<"handover corner "<<i;
  }
  const auto center=scan.Point(seen_scan.World(
      asmc::lidar::Vec(seen.f32_OBB_CenterX,seen.f32_OBB_CenterY,seen.f32_OBB_CenterZ)));
  EXPECT_NEAR(o.f32_OBB_CenterX,center.x(),1e-5); EXPECT_NEAR(o.f32_OBB_CenterY,center.y(),1e-5);
  EXPECT_NEAR(o.f32_OBB_CenterZ,center.z(),1e-5);
  const auto edge=(expected[1]-expected[0]).eval();
  EXPECT_LT(AxisError(o.f32_OBB_YawRad,std::atan2(edge.y(),edge.x())),1e-5)<<"handover yaw";
  EXPECT_NEAR(o.f32_OBB_ZMax-o.f32_OBB_ZMin,seen.f32_OBB_ZMax-seen.f32_OBB_ZMin,1e-5);
}
struct TrackCase {
  const char* name; bool level_roll; double yaw0,yaw_step; VehicleSpec vehicle;
  WorldBox matched,predicted,handover; unsigned scans;
  bool world_yaw;  // gravity-aligned leveled frame: matched yaw must equal the box heading
  double roll_scale=1.;  // 0: no roll, so pitch-only leveling is gravity-aligned too
};
constexpr unsigned kHandoverScans=3;  // the handover box is detected only in scans 0..2
void RunTrackCase(const TrackCase& c) {
  SCOPED_TRACE(c.name);
  Configure(c.level_roll); TrackingConfig();
  std::deque<std::vector<LiDARObjectInfo>> history;
  TrackScan handover_scan; LiDARObjectInfo handover_seen{}; double predicted_extent=0;
  for(unsigned s=0;s<c.scans;++s) {
    SCOPED_TRACE("scan "+std::to_string(s));
    const double yaw=std::remainder(c.yaw0+c.yaw_step*s,2*M_PI);
    const auto scan=BeginTrackScan(80.+.1*s,yaw,c.roll_scale*.06*std::sin(.7*s+.3),-.04*std::cos(.5*s),c.vehicle);
    std::vector<LiDARObjectInfo> objects{BoxDetection(scan,c.matched)};
    if(s<kHandoverScans) objects.push_back(BoxDetection(scan,c.handover));
    if(s==0) {
      objects.push_back(BoxDetection(scan,c.predicted));
      const int ids[]={42,44,43};  // matched, handover, predicted
      for(unsigned i=0;i<objects.size();++i) {
        auto& o=objects[i]; o.s32_Id=ids[i]; o.s32_Age=5; o.b_KfInitialized=true; o.s32_KfInitCount=2;
        o.Eigen_KfState_CA<<o.f32_CoreX,o.f32_CoreY,0,0,0,0; o.f32_PrevYaw=scan.local.st_IMU.f32_Yaw_rad;
      }
      predicted_extent=objects[2].f32_OBB_ZMax-objects[2].f32_OBB_ZMin;
    } else CompareCentroids(history,objects,.1,scan.local);
    // LidarProcess records the current frame for observed tracks after tracking.
    for(auto& o:objects) if(o.s32_MissCount==0) frame_boundary::RecordTrackingFrame(o);
    const LiDARObjectInfo *matched=nullptr,*predicted=nullptr,*handover=nullptr;
    for(const auto& o:objects) {
      if(o.s32_Id==42) matched=&o; if(o.s32_Id==43) predicted=&o; if(o.s32_Id==44) handover=&o;
    }
    ASSERT_NE(matched,nullptr); ASSERT_NE(predicted,nullptr); ASSERT_NE(handover,nullptr);
    EXPECT_EQ(matched->s32_MissCount,0); EXPECT_EQ(predicted->s32_MissCount,int(s));
    EXPECT_EQ(handover->s32_MissCount,s<kHandoverScans?0:int(s-kHandoverScans+1));
    ExpectObbSelfConsistent(*matched,"matched"); ExpectObbSelfConsistent(*predicted,"predicted");
    ExpectObbSelfConsistent(*handover,"handover");
    if(c.world_yaw) {
      const auto axis=scan.Direction(Heading(c.matched));
      EXPECT_LT(AxisError(matched->f32_OBB_YawRad,std::atan2(axis.y(),axis.x())),1e-5)<<"matched yaw vs box heading";
    }
    ExpectCarriedBox(*predicted,scan,c.predicted,predicted_extent);
    if(s+1==kHandoverScans) {handover_scan=scan; handover_seen=*handover;}
    if(s>=kHandoverScans) ExpectCarriedFrom(*handover,scan,handover_scan,handover_seen);
    history.clear(); history.push_back(objects);
  }
}
VehicleSpec MountedVehicle() {auto v=st_VehicleSpec; v.st_LiDARRotationDeg={4.f,-3.f,8.f}; return v;}
const WorldBox kMatchedBox{{12,2,.25},.4,4.5,1.8,1.5}, kPredictedBox{{7,-6,.1},-.9,3.,1.2,1.},
    kHandoverBox{{16,-1,.2},1.2,2.,1.1,1.3};
TEST(FrameBoundary, KalmanUpdateKeepsCurrentObbWhilePredictCarriesPreviousObb) {
  for(bool roll:{false,true}) {
    SCOPED_TRACE(roll?"roll_pitch":"pitch_only");
    Configure(roll);
    const auto before=BeginTrackScan(90.,.3,.05,-.03,st_VehicleSpec);
    auto track=BoxDetection(before,kMatchedBox); track.s32_Id=42; track.b_KfInitialized=true;
    track.s32_KfInitCount=2; track.Eigen_KfState_CA<<track.f32_CoreX,track.f32_CoreY,0,0,0,0;
    track.f32_PrevYaw=before.local.st_IMU.f32_Yaw_rad; frame_boundary::RecordTrackingFrame(track);
    const auto now=BeginTrackScan(90.1,.36,-.02,.04,st_VehicleSpec);
    // Matched: the OBB is this scan's detection; only the copied filter state is old.
    auto matched=BoxDetection(now,kMatchedBox);
    matched.Eigen_KfState_CA=track.Eigen_KfState_CA; matched.Eigen_KfP_CA=track.Eigen_KfP_CA;
    matched.b_KfInitialized=true; matched.s32_KfInitCount=2; matched.f32_PrevYaw=track.f32_PrevYaw;
    matched.tracking_frame_valid=true; matched.tracking_height_input=track.tracking_height_input;
    matched.tracking_yaw_from_input=track.tracking_yaw_from_input;
    matched.tracking_origin_in_yaw=track.tracking_origin_in_yaw;
    const auto detected=matched;
    kalman_filter_update(matched,matched.f32_CoreX,matched.f32_CoreY,matched.f32_CoreX,matched.f32_CoreY,
        .1,now.local.st_IMU.f32_Yaw_deg);
    EXPECT_FLOAT_EQ(matched.f32_OBB_YawRad,detected.f32_OBB_YawRad)<<"ego yaw applied again to a current OBB";
    EXPECT_FLOAT_EQ(matched.f32_OBB_CenterX,detected.f32_OBB_CenterX);
    EXPECT_FLOAT_EQ(matched.f32_OBB_CenterY,detected.f32_OBB_CenterY);
    EXPECT_FLOAT_EQ(matched.f32_OBB_CenterZ,detected.f32_OBB_CenterZ);
    EXPECT_FLOAT_EQ(matched.f32_OBB_ZMin,detected.f32_OBB_ZMin);
    EXPECT_FLOAT_EQ(matched.f32_OBB_ZMax,detected.f32_OBB_ZMax);
    for(unsigned i=0;i<8;++i) {
      EXPECT_FLOAT_EQ(matched.OBB_Corners_Body[i].x,detected.OBB_Corners_Body[i].x);
      EXPECT_FLOAT_EQ(matched.OBB_Corners_Body[i].y,detected.OBB_Corners_Body[i].y);
      EXPECT_FLOAT_EQ(matched.OBB_Corners_Body[i].z,detected.OBB_Corners_Body[i].z);
    }
    ExpectObbSelfConsistent(matched,"matched");
    // The copied filter state is re-expressed once: its prior equals this
    // scan's measurement of the static box, so the update keeps it.
    EXPECT_NEAR(matched.Eigen_KfState_CA(0),detected.f32_CoreX,1e-5);
    EXPECT_NEAR(matched.Eigen_KfState_CA(1),detected.f32_CoreY,1e-5);
    // Predicted-only: the whole previous-scan OBB is carried by one frame change.
    auto predicted=track; kalman_filter_predict(predicted,.1,now.local.st_IMU.f32_Yaw_deg);
    ExpectObbSelfConsistent(predicted,"predicted");
    ExpectCarriedBox(predicted,now,kMatchedBox,track.f32_OBB_ZMax-track.f32_OBB_ZMin);
    // With a relative velocity only the KF state is motion-predicted; the OBB is
    // re-expressed, not moved, so the CA displacement is not applied to it.
    auto moving=track; moving.Eigen_KfState_CA(2)=1.5; moving.Eigen_KfState_CA(3)=-.5;
    kalman_filter_predict(moving,.1,now.local.st_IMU.f32_Yaw_deg);
    ExpectCarriedBox(moving,now,kMatchedBox,track.f32_OBB_ZMax-track.f32_OBB_ZMin,false);
    const auto center=now.Point(kMatchedBox.center);
    const double dx=moving.Eigen_KfState_CA(0)-center.x(),dy=moving.Eigen_KfState_CA(1)-center.y();
    EXPECT_NEAR(std::hypot(dx,dy),.1*std::hypot(1.5,.5),1e-4)<<"KF moves by v*dt once";
    // A corner order other than computeOBBAtYaw's must not snap the yaw by 90 deg.
    auto foreign=track;
    std::rotate(foreign.OBB_Corners_Body.begin(),foreign.OBB_Corners_Body.begin()+1,foreign.OBB_Corners_Body.begin()+4);
    std::rotate(foreign.OBB_Corners_Body.begin()+4,foreign.OBB_Corners_Body.begin()+5,foreign.OBB_Corners_Body.end());
    kalman_filter_predict(foreign,.1,now.local.st_IMU.f32_Yaw_deg);
    const auto heading=now.Direction(Heading(kMatchedBox));
    EXPECT_LT(AxisError(foreign.f32_OBB_YawRad,std::atan2(heading.y(),heading.x())),roll?1e-5:5e-3)
        <<"yaw follows the rotated axis, not the foreign 0->1 edge";
  }
}
TEST(FrameBoundary, TrackerObbSingleScanMatchedAndPredicted) {
  Configure();
  RunTrackCase({"pitch_only",false,.2,.04,st_VehicleSpec,kMatchedBox,kPredictedBox,kHandoverBox,2,false});
  RunTrackCase({"roll_pitch",true,.2,.04,st_VehicleSpec,kMatchedBox,kPredictedBox,kHandoverBox,2,true});
}
TEST(FrameBoundary, TrackerObbYawDoesNotAccumulateOverScans) {
  Configure();
  RunTrackCase({"pitch_only",false,.2,.04,st_VehicleSpec,kMatchedBox,kPredictedBox,kHandoverBox,8,false});
  RunTrackCase({"pitch_only_zero_roll",false,.2,.04,st_VehicleSpec,kMatchedBox,kPredictedBox,kHandoverBox,8,true,0.});
  RunTrackCase({"roll_pitch",true,.2,.04,st_VehicleSpec,kMatchedBox,kPredictedBox,kHandoverBox,8,true});
  RunTrackCase({"roll_pitch_nonzero_mount",true,.2,.04,MountedVehicle(),kMatchedBox,kPredictedBox,kHandoverBox,8,false});
  RunTrackCase({"pitch_only_nonzero_mount",false,.2,-.05,MountedVehicle(),kMatchedBox,kPredictedBox,kHandoverBox,8,false});
}
TEST(FrameBoundary, TrackerObbNearSquareMatchedYawDoesNotDrift) {
  // Aspect < 1.2 skips the yaw EMA, so a twice-rotated prior would drift every scan.
  Configure();
  const WorldBox square{{11,3,.2},.5,1.,.9,1.2};
  RunTrackCase({"near_square_roll_pitch",true,.2,.04,st_VehicleSpec,square,kPredictedBox,kHandoverBox,8,true});
  RunTrackCase({"near_square_pitch_only_zero_roll",false,.2,.04,st_VehicleSpec,square,kPredictedBox,kHandoverBox,8,true,0.});
}
TEST(FrameBoundary, TrackerObbYawWrapAround) {
  // Ego yaw crosses +pi/-pi. The matched axis crosses the +-pi/2 branch of the
  // normalized OBB yaw; the handover and predicted raw yaws cross +-pi and -pi/2.
  Configure();
  const double yaw0=M_PI-.06;
  const WorldBox matched{{12,2,.25},yaw0+M_PI/2.+.06,4.5,1.8,1.5}, predicted{{7,-6,.1},M_PI/2.+.03,3.,1.2,1.},
      handover{{16,-1,.2},.03,2.,1.1,1.3};
  RunTrackCase({"wrap_pitch_only",false,yaw0,.04,st_VehicleSpec,matched,predicted,handover,6,false});
  RunTrackCase({"wrap_pitch_only_zero_roll",false,yaw0,.04,st_VehicleSpec,matched,predicted,handover,6,true,0.});
  RunTrackCase({"wrap_roll_pitch",true,yaw0,.04,st_VehicleSpec,matched,predicted,handover,6,true});
}
TEST(FrameBoundary, CalibrationRejectsRuntimeAndSnapshotMismatch) {
  Configure();EXPECT_TRUE(frame_boundary::MapOffsetsValid(302595.,4124145.));
  EXPECT_FALSE(frame_boundary::MapOffsetsValid(302595.1,4124145.));
  ros::NodeHandle node("~");CAMERA camera{};
  InitProjectionMatrix(camera,YAML::LoadFile(ros::package::getPath("Main")+"/setting.yaml"));
  auto wrong=st_VehicleSpec;wrong.st_LiDARPos.f32_X+=.1;
  EXPECT_THROW(frame_boundary::Configure(node,wrong,camera),std::runtime_error);
  camera.params.intrinsic_matrix_90_fov(0,0)+=1.;
  EXPECT_THROW(frame_boundary::Configure(node,st_VehicleSpec,camera),std::runtime_error);
  Configure();
}
TEST(FrameBoundary, ContinuousLocalTrackingStateIsIndependentOfGpsHistory) {
  Configure();TrackingConfig();Now(26.);
  auto simulate=[](bool denial) {
    st_Tracking.s32_NewTrackId=100;
    std::deque<std::vector<LiDARObjectInfo>> history;
    std::vector<std::vector<double>> states;
    asmc::lidar::PoseSelection map;
    for(unsigned frame=0;frame<40;++frame) {
      const double time=26.+frame*.1;Now(time);
      auto local=Local(.002*frame);
      local.st_IMU.f32_Roll_rad=.03*std::sin(frame*.1);
      local.st_IMU.f32_Pitch_rad=.02*std::cos(frame*.1);
      map.valid=!denial || frame<10 || frame>=30;
      map.reason=map.valid?"gps_bounded_prediction":"gps_stale";
      map.query_stamp_ns=ros::Time(time).toNSec();map.clock="ros";
      map.map_from_base=asmc::lidar::Pose::Identity();map.map_from_base.translation().x()=frame*.2;
      frame_boundary::PrepareScan(ros::Time(time),local,st_VehicleSpec,map,frame+1);
      frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(local.st_IMU.f32_Roll_rad,
          local.st_IMU.f32_Pitch_rad,asmc::lidar::Rot::Identity()));
      auto detection=Detection(12.-.2*frame,1.);
      detection.f32_CoreE=map.valid?100:NAN;detection.f32_CoreN=map.valid?200:NAN;
      std::vector<LiDARObjectInfo> objects{detection};
      if(history.empty()) {
        objects[0].s32_Id=42;objects[0].s32_Age=1;objects[0].b_KfInitialized=true;
        objects[0].s32_KfInitCount=2;objects[0].Eigen_KfState_CA<<12,1,-2,0,0,0;
        objects[0].f32_PrevYaw=0;
      } else CompareCentroids(history,objects,.1,local);
      EXPECT_EQ(objects.front().s32_Id,42);frame_boundary::RecordTrackingFrame(objects.front());
      const auto& o=objects.front();
      states.push_back({double(o.s32_Id),double(o.s32_Age),double(o.s32_MissCount),
          double(o.Eigen_KfState_CA(0)),double(o.Eigen_KfState_CA(1)),double(o.continuity_counter)});
      history.clear();history.push_back(objects);
    }
    return states;
  };
  const auto normal=simulate(false), denied=simulate(true);EXPECT_EQ(normal,denied);
}

TEST(FrameBoundary, ParserRejectsZeroDuplicateAndBackwardStampsBeforeTrackerInput) {
  Configure();Now(45.);
  const auto old_stamp=st_SensorData.st_RawLiDAR.u64_Timestamp;
  const auto old_source_ns=st_SensorData.st_RawLiDAR.source_stamp_ns;
  const auto old_count=st_SensorData.st_RawLiDAR.s32_Num;
  const auto old_generation=u64_LidarFrameGeneration;
  pthread_mutex_init(&st_SensorData.st_MutexLiDAR,nullptr);
  st_SensorData.st_RawLiDAR.u64_Timestamp=0;st_SensorData.st_RawLiDAR.source_stamp_ns=0;u64_LidarFrameGeneration=0;
  pcl::PointCloud<pcl::PointXYZI> cloud;cloud.push_back(pcl::PointXYZI());
  sensor_msgs::PointCloud2::Ptr message(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(cloud,*message);message->header.frame_id="velodyne";
  LiDARParser_Callback(message);
  EXPECT_EQ(st_SensorData.st_RawLiDAR.u64_Timestamp,0u);
  EXPECT_EQ(u64_LidarFrameGeneration,0u);
  message->header.stamp=ros::Time(45.);LiDARParser_Callback(message);
  EXPECT_EQ(st_SensorData.st_RawLiDAR.u64_Timestamp,45000u);
  EXPECT_EQ(u64_LidarFrameGeneration,1u);
  message->header.stamp=ros::Time(44.9);LiDARParser_Callback(message);
  EXPECT_EQ(st_SensorData.st_RawLiDAR.u64_Timestamp,45000u);
  message->header.stamp=ros::Time(45.);LiDARParser_Callback(message);
  EXPECT_EQ(u64_LidarFrameGeneration,1u);
  message->header.stamp=ros::Time(45.1);LiDARParser_Callback(message);
  EXPECT_EQ(st_SensorData.st_RawLiDAR.u64_Timestamp,45100u);
  EXPECT_EQ(u64_LidarFrameGeneration,2u);
  st_SensorData.st_RawLiDAR.u64_Timestamp=old_stamp;st_SensorData.st_RawLiDAR.source_stamp_ns=old_source_ns;
  st_SensorData.st_RawLiDAR.s32_Num=old_count;u64_LidarFrameGeneration=old_generation;
  pthread_mutex_destroy(&st_SensorData.st_MutexLiDAR);
}

TEST(FrameBoundary, FullNsLocalSelectionDoesNotAdmitFutureSameMsInput) {
  Configure();Now(60.);SensorPoseHistory history;auto motion=Local();
  history.AddImu(60000,motion.st_IMU,nullptr,60000000100ULL,1000);
  history.AddCan(60000,motion.st_CAN,nullptr,60000000100ULL,1000);
  auto future=motion;future.st_IMU.f32_Roll_rad=.1;
  history.AddImu(60000,future.st_IMU,nullptr,60000000900ULL,1100);
  GPS_IMU selected{};LocalInputSelection inputs;std::string reason;
  ASSERT_TRUE(history.WaitForLocalLookupNs(60000000500ULL,0,&selected,&inputs,&reason));
  EXPECT_EQ(inputs.imu_stamp_ns,60000000100ULL);
  EXPECT_EQ(inputs.imu_receipt_steady_ns,1000ULL);
  EXPECT_FLOAT_EQ(selected.st_IMU.f32_Roll_rad,0.f);
  ASSERT_TRUE(history.WaitForLocalLookupNs(60000000900ULL,0,&selected,&inputs,&reason));
  EXPECT_EQ(inputs.imu_stamp_ns,60000000900ULL);
}
TEST(FrameBoundary, OriginalScanNsAndDualClockSurviveCallbackAndContext) {
  Configure();Now(70.);
  pthread_mutex_init(&st_SensorData.st_MutexLiDAR,nullptr);
  st_SensorData.st_RawLiDAR.source_stamp_ns=0;
  pcl::PointCloud<pcl::PointXYZI> cloud;cloud.push_back(pcl::PointXYZI());
  sensor_msgs::PointCloud2::Ptr message(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(cloud,*message);message->header.stamp.fromNSec(70000000123ULL);
  const auto generation=u64_LidarFrameGeneration;
  LiDARParser_Callback(message);
  EXPECT_EQ(frame_boundary::SourceStamp(st_SensorData.st_RawLiDAR).toNSec(),70000000123ULL);
  const auto time=frame_boundary::ScanTime(st_SensorData.st_RawLiDAR);
  EXPECT_EQ(time.stamp_ns,70000000123LL);EXPECT_EQ(time.receipt_clock,"steady");
  EXPECT_GT(time.receipt_ns,0);EXPECT_GE(time.processing_ns,time.receipt_ns);
  EXPECT_EQ(st_SensorData.st_RawLiDAR.receipt_ros_ns,70000000000ULL);
  message->header.stamp.fromNSec(70000000234ULL);LiDARParser_Callback(message);
  EXPECT_EQ(u64_LidarFrameGeneration,generation+2);
  EXPECT_EQ(frame_boundary::SourceStamp(st_SensorData.st_RawLiDAR).toNSec(),70000000234ULL);
  pthread_mutex_destroy(&st_SensorData.st_MutexLiDAR);
}

TEST(FrameBoundary, RealGroundFilterPreservesIndependentElevatedPointsAcrossAttitude) {
  Configure();
  for(double roll:{0.,5.,-5.,10.,-10.}) for(double pitch:{-3.,0.,3.}) {
    auto local=Local();local.st_IMU.f32_Roll_rad=roll*M_PI/180.;local.st_IMU.f32_Pitch_rad=pitch*M_PI/180.;
    frame_boundary::PrepareScan(ros::Time(74.),local,st_VehicleSpec,{},1);
    const auto level=asmc::lidar::leveling_rotation(local.st_IMU.f32_Roll_rad,
        local.st_IMU.f32_Pitch_rad,asmc::lidar::Rot::Identity(),true,true);
    auto raw=pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    auto corrected=pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    auto remaining=pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    // Truth is generated from a known sloped ground before running RANSAC.
    for(unsigned i=0;i<256;++i) {
      const double x=5.+.3*(i%16),y=-2.+.3*(i/16);
      const asmc::lidar::Vec q=level.transpose()*asmc::lidar::Vec(x,y,-1.45+.02*x);
      pcl::PointXYZI p;p.x=q.x();p.y=q.y();p.z=q.z();p.intensity=0;raw->push_back(p);
    }
    for(unsigned i=0;i<24;++i) {
      const asmc::lidar::Vec q=level.transpose()*asmc::lidar::Vec(7.+.1*(i%4),1.+.1*(i/4),-.7);
      pcl::PointXYZI p;p.x=q.x();p.y=q.y();p.z=q.z();p.intensity=1;raw->push_back(p);
    }
    CorrectingPoints(raw,corrected,local);NormalVectorFiltering(corrected,remaining,local);
    ASSERT_TRUE(CurrentGroundPlaneSnapshot().valid)<<roll<<","<<pitch;
    EXPECT_EQ(remaining->size(),24U)<<roll<<","<<pitch;
    for(const auto& p:*remaining)EXPECT_EQ(p.intensity,1);
  }
}

TEST(FrameBoundary, GroundFitFailureAfterAttitudeChangePreservesUnprovenPoints) {
  Configure();Now(75.);auto local=Local();asmc::lidar::PoseSelection no_map;
  frame_boundary::PrepareScan(ros::Time(75.),local,st_VehicleSpec,no_map);
  frame_boundary::StoreLeveling(asmc::lidar::Rot::Identity());
  auto ground=pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  auto output=pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  for(int x=0;x<8;++x)for(int y=0;y<8;++y) {
    pcl::PointXYZI p;p.x=5.f+x*.3f;p.y=-1.f+y*.3f;p.z=-1.5f;ground->push_back(p);
  }
  NormalVectorFiltering(ground,output,local);
  ASSERT_TRUE(CurrentGroundPlaneSnapshot().valid);
  local.st_IMU.f32_Roll_rad=10*M_PI/180.;
  frame_boundary::PrepareScan(ros::Time(75.1),local,st_VehicleSpec,no_map);
  frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(local.st_IMU.f32_Roll_rad,0,asmc::lidar::Rot::Identity()));
  auto ambiguous=pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  for(int i=0;i<24;++i) {pcl::PointXYZI p;p.x=8.f+i*.02f;p.y=.2f;p.z=-1.5f;ambiguous->push_back(p);}
  NormalVectorFiltering(ambiguous,output,local);
  EXPECT_FALSE(CurrentGroundPlaneSnapshot().valid);
  EXPECT_EQ(output->size(),ambiguous->size());
}

bool Wait(const std::function<bool()>& condition) {
  const auto end=ros::WallTime::now()+ros::WallDuration(3.);
  while(ros::ok() && ros::WallTime::now()<end) {if(condition()) return true;ros::WallDuration(.01).sleep();}
  return condition();
}
TEST(FrameBoundary, TypedOutputsPreserveGeometryAndDistinguishInvalid) {
  Configure();Now(30.);
  ros::NodeHandle node;frame_boundary::Publisher publisher(node);
  std::mutex mutex;std::vector<asmc_msgs::LidarObjectArray> local,map;
  auto sub_local=node.subscribe<asmc_msgs::LidarObjectArray>("/perception/lidar/objects_local",10,
      [&](const asmc_msgs::LidarObjectArray::ConstPtr& m){std::lock_guard<std::mutex> lock(mutex);local.push_back(*m);});
  auto sub_map=node.subscribe<asmc_msgs::LidarObjectArray>("/perception/lidar/objects_map",10,
      [&](const asmc_msgs::LidarObjectArray::ConstPtr& m){std::lock_guard<std::mutex> lock(mutex);map.push_back(*m);});
  ASSERT_TRUE(Wait([&]{return sub_local.getNumPublishers() && sub_map.getNumPublishers();}));
  ros::WallDuration(.1).sleep();
  asmc::lidar::PoseSelection missing;missing.reason="gps_invalid";
  frame_boundary::PrepareScan(ros::Time(30.),Local(),st_VehicleSpec,missing);
  frame_boundary::StoreLeveling(asmc::lidar::Rot::Identity());
  LiDAR lidar;lidar.frame_context=frame_boundary::Current();
  auto object=Detection(10,0);object.s32_Id=42;object.s32_Age=2;lidar.vec_Objects={object};
  EXPECT_FALSE(frame_boundary::LegacyMotionReady(lidar));
  lidar.vec_Objects[0].motion_valid=true;EXPECT_TRUE(frame_boundary::LegacyMotionReady(lidar));
  publisher.Publish(lidar);
  ASSERT_TRUE(Wait([&]{std::lock_guard<std::mutex> lock(mutex);return local.size()==1 && map.size()==1;}));
  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_EQ(local[0].objects.size(),1U);EXPECT_EQ(local[0].header.frame_id,"base_link");
    EXPECT_NEAR(local[0].objects[0].center.x,10.8,1e-6);EXPECT_EQ(local[0].objects[0].track_id,42);
    EXPECT_EQ(local[0].objects[0].camera_class,"car");EXPECT_FALSE(local[0].objects[0].absolute_velocity_valid);
    EXPECT_EQ(map[0].status,map[0].POSE_UNAVAILABLE);EXPECT_TRUE(map[0].objects.empty());
  }
  Now(30.1);lidar.frame_context.time={30100000000LL,30100000000LL,30100000000LL,"ros","bridge_receive_ms"};
  lidar.vec_Objects.clear();publisher.Publish(lidar);
  ASSERT_TRUE(Wait([&]{std::lock_guard<std::mutex> lock(mutex);return local.size()==2;}));
  {
    std::lock_guard<std::mutex> lock(mutex);EXPECT_EQ(local[1].status,local[1].OBSERVED_EMPTY);
  }
  publisher.Invalid(ros::Time(30.2),"imu_stale");publisher.Invalid(ros::Time(30.2),"duplicate");
  publisher.Invalid(ros::Time(),"initial_no_input");publisher.Invalid(ros::Time(30.1),"regression");
  ASSERT_TRUE(Wait([&]{std::lock_guard<std::mutex> lock(mutex);return local.size()==3 && map.size()==3;}));
  ros::WallDuration(.1).sleep();
  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(local.size(),3U);EXPECT_EQ(local.back().status,local.back().INPUT_INVALID);
  EXPECT_EQ(local.back().reason,"imu_stale");
}
}
int main(int argc,char** argv) {
  ros::init(argc,argv,"frame_boundary_ros");testing::InitGoogleTest(&argc,argv);
  ros::AsyncSpinner spinner(1);spinner.start();
  const int result=RUN_ALL_TESTS();spinner.stop();return result;
}
