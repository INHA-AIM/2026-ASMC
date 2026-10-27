#include <asmc_lidar/experimental/confirmation_wall_evidence.hpp>
#include <asmc_lidar/experimental/tunnel_route.hpp>
#include <asmc_lidar/experimental/tunnel_motion.hpp>
#include <algorithm>
#include <asmc_lidar/pipeline/pipeline.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
using namespace asmc::lidar::pipeline;
namespace {
pcl::PointXYZI Point(float x,float y,float z) {pcl::PointXYZI p{};p.x=x;p.y=y;p.z=z;return p;}
CloudPtr Scene() {
  CloudPtr c(new Cloud);
  for(float x=5;x<35;x+=.5f)for(float y=-4.5;y<=4.5;y+=.5f)c->push_back(Point(x,y,-1.45f));
  for(float x=5;x<35;x+=.4f)for(float z=-1.1;z<2.5;z+=.2f)for(float y:{-5.f,5.f})c->push_back(Point(x,y,z));
  for(float x=5;x<35;x+=.5f)for(float y=-4;y<=4;y+=.5f)c->push_back(Point(x,y,4));
  for(float x=12;x<15;x+=.2f)for(float y=-.8f;y<.8f;y+=.2f)for(float z=-1.1f;z<.2f;z+=.2f)c->push_back(Point(x,y,z));
  return c;
}
TunnelCameraInput NoImage() {
  TunnelCameraInput cam;cam.calibrated=true;cam.calibration.width=cam.calibration.height=200;
  cam.calibration.base_from_camera.translation().z()=1000;return cam;
}
LocalState Local() {LocalState l;l.st_IMU.b_ImuReady=true;return l;}
}
TEST(TunnelRoute, GpsGateExitsAndReentryResetsGroundAndWallHistory) {
  TunnelRoute route(TunnelRouteConfig{});auto c=Scene();auto cam=NoImage();
  EXPECT_FALSE(route.Select(true));EXPECT_TRUE(route.Select(false));
  route.Process(c,Local(),1000000000,VehicleSpec{},&cam);
  route.Process(c,Local(),1100000000,VehicleSpec{},&cam);
  route.Process(c,Local(),1200000000,VehicleSpec{},&cam);
  EXPECT_GT(route.last().wall_removed,0);
  EXPECT_FALSE(route.Select(true));EXPECT_TRUE(route.Select(false));
  route.Process(c,Local(),2000000000,VehicleSpec{},&cam);
  EXPECT_EQ(0,route.last().wall_removed);
}
TEST(TunnelRoute, DynamicRoiOnlyRunsInDeniedModeAndKeepsBoundaryPoints) {
  TunnelRoute route(TunnelRouteConfig{});CloudPtr c(new Cloud);c->push_back(Point(-30,0,0));c->push_back(Point(40,0,0));c->push_back(Point(41,0,0));
  route.Select(true);route.RestrictRoi(c);EXPECT_EQ(3u,c->size());
  route.Select(false);route.RestrictRoi(c);ASSERT_EQ(2u,c->size());EXPECT_FLOAT_EQ(-30,c->at(0).x);EXPECT_FLOAT_EQ(40,c->at(1).x);
}
TEST(TunnelRoute, EntranceCeilingKeepsExactLimitAndReturnsOriginalIndices) {
  TunnelRoute route(TunnelRouteConfig{});CloudPtr c(new Cloud),out(new Cloud);std::vector<int> indices;
  c->push_back(Point(10,0,2.55f));c->push_back(Point(11,0,2.56f));c->push_back(Point(12,0,0));
  route.EntranceCeiling(c,out,indices);EXPECT_EQ((std::vector<int>{0,2}),indices);ASSERT_EQ(2u,out->size());EXPECT_FLOAT_EQ(12,out->at(1).x);
}
TEST(TunnelRoute, EntranceStructureRequiresAllThreeAxesSoLongBusSurvives) {
  TunnelRoute route(TunnelRouteConfig{});TrackedObject o;o.detection.obb_length_m=15;o.detection.obb_width_m=2.5;o.detection.f32_OBB_ZMax=3;
  EXPECT_FALSE(route.EntranceStructure(o));o.detection.obb_width_m=6;EXPECT_TRUE(route.EntranceStructure(o));
  o.detection.f32_OBB_ZMax=2.99f;EXPECT_FALSE(route.EntranceStructure(o));
}
TEST(TunnelRoute, OriginalCoreLabelsAndClustersReachTheCommonPipelineWithoutIndexOrFrameDrift) {
  TunnelRouteConfig cfg;TunnelRoute route(cfg);route.Select(false);auto cam=NoImage();auto c=Scene();
  // Reference uses the preserved original core and masks that are zero because
  // the complete scene lies behind this calibrated camera.
  tunnel_filter::TunnelPipeline reference(cfg.params);std::vector<std::uint8_t> zero(c->size(),0);
  for(int k=0;k<3;++k) {
    const auto out=route.Process(c,Local(),1000000000+k*100000000,VehicleSpec{},&cam);
    const auto& ref=reference.Process(*c,0,0,&zero,&zero,false,nullptr);
    EXPECT_EQ(ref.labels,route.last().labels);EXPECT_EQ(ref.kept_indices,out.kept_indices);
    ASSERT_EQ(out.filtered->size(),out.kept_indices.size());ASSERT_EQ(out.centroids->size(),out.clusters.size());
    for(std::size_t i=0;i<out.kept_indices.size();++i)EXPECT_FLOAT_EQ(ref.leveled->at(out.kept_indices[i]).x,out.filtered->at(i).x);
    for(const auto& cluster:out.clusters)for(int i:cluster.indices) {EXPECT_GE(i,0);EXPECT_LT(static_cast<std::size_t>(i),out.filtered->size());}
    EXPECT_TRUE(out.plane.valid);EXPECT_TRUE(out.leveling.isApprox(asmc::lidar::Rot::Identity()));
  }
}
TEST(TunnelRoute, EgoMotionUsesScanTimeYawAndSpeedAndResetsOnExit) {
  TunnelRoute route(TunnelRouteConfig{});route.Select(false);auto l=Local();auto cam=NoImage();auto c=Scene();
  l.st_CAN.f32_CurrentVel_MPS=2;route.Process(c,l,1000000000,VehicleSpec{},&cam);EXPECT_FALSE(route.last().motion.valid);
  l.st_IMU.f32_Yaw_rad=.1f;route.Process(c,l,1100000000,VehicleSpec{},&cam);
  EXPECT_TRUE(route.last().motion.valid);EXPECT_NEAR(.1,route.last().motion.dyaw_rad,1e-6);EXPECT_NEAR(.2*std::cos(.05),route.last().motion.dx_m,1e-6);
}
TEST(TunnelRoute, CameraProtectionUsesActualCalibrationAndUnavailableFrameCannotProtectByClass) {
  TunnelRoute route(TunnelRouteConfig{});route.Select(false);auto cam=NoImage();cam.calibration.base_from_camera=asmc::lidar::Pose::Identity();
  cam.calibration.base_from_camera.linear()<<0,0,1,-1,0,0,0,-1,0;
  cam.calibration.intrinsic<<100,0,100,0,100,100,0,0,1;cam.available=true;
  asmc::lidar::camera::Box box;box.xmin=50;box.xmax=150;box.ymin=60;box.ymax=140;box.source_class_name="car";cam.boxes.push_back(box);
  route.Process(Scene(),Local(),1000000000,VehicleSpec{},&cam);EXPECT_GT(route.last().protected_points,0);
  cam.available=false;route.Process(Scene(),Local(),1100000000,VehicleSpec{},&cam);EXPECT_EQ(0,route.last().protected_points);EXPECT_FALSE(route.last().protection_available);
}
TEST(TunnelRoute, OffProfileDoesNotChangeNormalClusteringOrGeometry) {
  PipelineConfig a;a.calibration_id="test";a.capture_debug=true;PipelineConfig b=a;b.tunnel_route.original_pipeline=true;
  Pipeline left(a),right(b);ScanInput scan;scan.time.stamp_ns=1000000000;scan.points=Scene();asmc::lidar::PoseSelection map;
  const auto l=left.Process(scan,Local(),map),r=right.Process(scan,Local(),map);
  EXPECT_EQ(l.digests.roi,r.digests.roi);EXPECT_EQ(l.digests.voxel,r.digests.voxel);EXPECT_EQ(l.digests.ground_kept,r.digests.ground_kept);EXPECT_EQ(l.cluster_sizes,r.cluster_sizes);EXPECT_EQ(l.objects.size(),r.objects.size());EXPECT_FALSE(r.tunnel.ran);
}
TEST(TunnelRoute, FullPipelineUsesTunnelOnlyWhenMapUnavailableAndExportsMatchingContext) {
  PipelineConfig cfg;cfg.calibration_id="test";cfg.capture_debug=true;cfg.tunnel.enabled=true;cfg.tunnel_route.original_pipeline=true;
  Pipeline pipeline(cfg);ScanInput scan;scan.points=Scene();scan.time.stamp_ns=1000000000;asmc::lidar::PoseSelection map;map.valid=true;
  const auto normal=pipeline.Process(scan,Local(),map);EXPECT_FALSE(normal.tunnel.ran);
  map.valid=false;scan.time.stamp_ns+=100000000;auto cam=std::make_shared<TunnelCameraInput>(NoImage());scan.tunnel_camera=cam;
  const auto tunnel=pipeline.Process(scan,Local(),map);ASSERT_TRUE(tunnel.processed);EXPECT_TRUE(tunnel.tunnel.ran);EXPECT_FALSE(tunnel.context.map_pose.valid);EXPECT_GT(tunnel.counts.cluster_input_points,0);
  map.valid=true;scan.time.stamp_ns+=100000000;EXPECT_FALSE(pipeline.Process(scan,Local(),map).tunnel.ran);
}
TEST(TunnelRoute, NonzeroRollPitchKeepsTheStrictCommonTransformValid) {
  PipelineConfig cfg;cfg.calibration_id="test";cfg.tunnel.enabled=true;cfg.tunnel_route.original_pipeline=true;
  Pipeline pipeline(cfg);ScanInput scan;scan.points=Scene();scan.time.stamp_ns=1000000000;
  auto local=Local();local.st_IMU.f32_Roll_rad=.028233356773853302f;local.st_IMU.f32_Pitch_rad=.0018583248602226377f;
  asmc::lidar::PoseSelection map;map.valid=false;
  const auto out=pipeline.Process(scan,local,map);ASSERT_TRUE(out.processed);ASSERT_TRUE(out.tunnel.ran);
  EXPECT_TRUE(asmc::lidar::base_from_input(out.context).has_value());
  EXPECT_TRUE((out.context.applied_leveling.transpose()*out.context.applied_leveling).isApprox(asmc::lidar::Rot::Identity(),1e-12));
}
TEST(TunnelRoute, InvalidConfigurationRejectedBeforeCoreRuns) {
  TunnelRouteConfig cfg;cfg.params.cluster_tolerance_m=0;EXPECT_THROW(TunnelRoute x(cfg),std::invalid_argument);
  cfg=TunnelRouteConfig{};cfg.params.ceiling_max_z_lidar_m=std::numeric_limits<float>::quiet_NaN();EXPECT_THROW(TunnelRoute x(cfg),std::invalid_argument);
}
int main(int argc,char**argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}

TEST(TunnelClusterZ, UsesRawSensorMeanStrictBoundaryAndRemapsPointsDuringLiveChanges) {
  TunnelRouteConfig cfg;cfg.params.wall_enable=false;cfg.params.obb_filter_enable=false;
  cfg.params.cluster_tolerance_m=.4f;cfg.params.cluster_min_points=3;
  TunnelRoute route(cfg);route.Select(false);CloudPtr cloud(new Cloud);
  for(float x=5;x<30;x+=.5f)for(float y=-4;y<4;y+=.5f)cloud->push_back(Point(x,y,-1.45f));
  for(float x=12;x<13;x+=.1f)for(float y=-1;y<0;y+=.1f)cloud->push_back(Point(x,y,-.5f));
  for(float x=20;x<21;x+=.1f)for(float y=1;y<2;y+=.1f)cloud->push_back(Point(x,y,.25f));
  auto cam=NoImage();auto local=Local();local.st_IMU.f32_Roll_rad=.03f;local.st_IMU.f32_Pitch_rad=.02f;
  auto baseline=route.Process(cloud,local,1000000000,VehicleSpec{},&cam);
  ASSERT_EQ(2u,baseline.clusters.size());
  TunnelClusterZConfig z;z.enabled=true;z.min_sensor_z_m=-.5f;route.SetClusterZConfig(z);
  auto equality=route.Process(cloud,local,1100000000,VehicleSpec{},&cam);
  EXPECT_EQ(2u,equality.clusters.size());EXPECT_EQ(0,equality.report.cluster_z_removed);
  z.min_sensor_z_m=-.49f;route.SetClusterZConfig(z);
  auto filtered=route.Process(cloud,local,1200000000,VehicleSpec{},&cam);
  ASSERT_EQ(1u,filtered.clusters.size());ASSERT_EQ(1u,filtered.centroids->size());
  EXPECT_EQ(1,filtered.report.cluster_z_removed);EXPECT_GT(filtered.report.cluster_z_removed_points,0);
  EXPECT_EQ(baseline.filtered->size()-filtered.filtered->size(),
            static_cast<std::size_t>(filtered.report.cluster_z_removed_points));
  ASSERT_EQ(filtered.kept_indices.size(),filtered.filtered->size());
  for(const auto& c:filtered.clusters)for(int i:c.indices) {
    ASSERT_GE(i,0);ASSERT_LT(static_cast<std::size_t>(i),filtered.filtered->size());
    EXPECT_FLOAT_EQ(.25f,cloud->at(filtered.kept_indices.at(i)).z);
  }
  z.enabled=false;route.SetClusterZConfig(z);
  EXPECT_EQ(2u,route.Process(cloud,local,1300000000,VehicleSpec{},&cam).clusters.size());
}
TEST(TunnelClusterZ, RejectsInvalidSettingsAndNeverChangesHealthyGpsPipeline) {
  for(float v:{1.f,-5.01f,std::numeric_limits<float>::quiet_NaN()}) {
    TunnelClusterZConfig z;z.min_sensor_z_m=v;EXPECT_THROW(ValidateTunnelClusterZ(z),std::invalid_argument);
  }
  PipelineConfig a;a.calibration_id="z-test";a.capture_debug=true;
  a.tunnel.enabled=true;a.tunnel_route.original_pipeline=true;PipelineConfig b=a;
  b.tunnel_route.cluster_z.enabled=true;b.tunnel_route.cluster_z.min_sensor_z_m=0;
  Pipeline left(a),right(b);ScanInput scan;scan.points=Scene();scan.time.stamp_ns=1000000000;
  asmc::lidar::PoseSelection map;map.valid=true;
  const auto l=left.Process(scan,Local(),map),r=right.Process(scan,Local(),map);
  EXPECT_EQ(l.digests.ground_kept,r.digests.ground_kept);EXPECT_EQ(l.cluster_sizes,r.cluster_sizes);
  EXPECT_EQ(l.objects.size(),r.objects.size());EXPECT_FALSE(r.tunnel.ran);
}

TEST(TunnelClusterZ, ThresholdEditRetiresLowPredictionsWithoutResettingTheRuntime) {
  PipelineConfig cfg;cfg.calibration_id="live-z";cfg.tunnel.enabled=true;cfg.tunnel_route.original_pipeline=true;
  cfg.tunnel_route.params.wall_enable=false;cfg.tunnel_route.params.obb_filter_enable=false;
  Pipeline pipeline(cfg);ScanInput scan;scan.points=Scene();scan.time.stamp_ns=1000000000;
  scan.tunnel_camera=std::make_shared<TunnelCameraInput>(NoImage());asmc::lidar::PoseSelection denied;
  ASSERT_TRUE(pipeline.Process(scan,Local(),denied).processed);
  TunnelClusterZConfig z;z.enabled=true;z.min_sensor_z_m=0;pipeline.SetTunnelClusterZConfig(z);
  scan.time.stamp_ns+=100000000;
  const auto out=pipeline.Process(scan,Local(),denied);
  ASSERT_TRUE(out.processed);EXPECT_EQ(1000000000,out.previous_stamp_ns);
  EXPECT_GT(out.tunnel.cluster_z_removed,0);
  for(const auto& object:out.objects) {
    const auto& d=object.detection;
    EXPECT_GE((out.context.applied_leveling.transpose()*asmc::lidar::Vec(d.f32_CoreX,d.f32_CoreY,d.f32_CoreZ)).z(),-1e-5);
  }
}

TEST(TunnelWallTuning, ChangesActualRemovalThicknessAndRestartsPlaneConfirmationOnly) {
  TunnelRouteConfig cfg;cfg.params.obb_filter_enable=false;
  TunnelRoute route(cfg);route.Select(false);auto cloud=Scene();auto cam=NoImage();
  for(float x=5;x<35;x+=.4f)cloud->push_back(Point(x,4.8f,-.5f));
  TunnelRouteOutput out;
  for(int i=0;i<4;++i)out=route.Process(cloud,Local(),1000000000+i*100000000,VehicleSpec{},&cam);
  auto nearwall=[](const CloudPtr& c) {
    return std::count_if(c->begin(),c->end(),[](const auto& p){return std::fabs(p.y-4.8f)<.001f&&std::fabs(p.z+.5f)<.001f;});
  };
  ASSERT_TRUE(route.last().walls[0].removal_allowed);EXPECT_EQ(0,nearwall(out.filtered));
  TunnelWallConfig wall;wall.fit_distance_m=.12f;wall.remove_distance_m=.05f;wall.track_max_offset_m=.4f;
  route.SetWallConfig(wall);
  out=route.Process(cloud,Local(),1500000000,VehicleSpec{},&cam);
  EXPECT_FALSE(route.last().walls[0].removal_allowed);
  for(int i=0;i<3;++i)out=route.Process(cloud,Local(),1600000000+i*100000000,VehicleSpec{},&cam);
  EXPECT_TRUE(route.last().walls[0].removal_allowed);EXPECT_GT(nearwall(out.filtered),50);
  EXPECT_FLOAT_EQ(.12f,out.report.wall_fit_distance_m);
  EXPECT_FLOAT_EQ(.05f,out.report.wall_remove_distance_m);
  EXPECT_FLOAT_EQ(.4f,out.report.wall_track_max_offset_m);
  wall.fit_distance_m=std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(route.SetWallConfig(wall),std::invalid_argument);
}

TEST(TunnelMotionHistory, ReverseCurvedMotionUsesCausalSamplesAndSensorLeverArm) {
  TunnelMotionHistory history;
  const std::int64_t begin=1000000000,end=1200000000;
  for(int i=0;i<=10;++i) {
    const auto stamp=begin+i*20000000;
    history.AddSpeed(stamp,-2,true);
    history.AddYaw(stamp,i<5?0:M_PI/2,true);
  }
  const auto m=history.Between(begin,end,0,M_PI/2,asmc::lidar::Vec(1,0,0));
  ASSERT_TRUE(m.valid);EXPECT_TRUE(m.reliable);EXPECT_TRUE(m.integrated);
  EXPECT_NEAR(-1.2,m.dx_m,1e-6);EXPECT_NEAR(.8,m.dy_m,1e-6);
  EXPECT_NEAR(M_PI/2,m.dyaw_rad,1e-6);
  // A future sample must not leak into an already complete interval.
  history.AddSpeed(end+20000000,30,false);
  const auto same=history.Between(begin,end,0,M_PI/2,asmc::lidar::Vec(1,0,0));
  EXPECT_FLOAT_EQ(m.dx_m,same.dx_m);EXPECT_TRUE(same.reliable);
}
TEST(TunnelMotionHistory, RejectedSpeedIsOnlyAnUncertainEstimateAndMissingDataNeverPretendsValid) {
  TunnelMotionHistory h;
  for(int i=0;i<=10;++i) {
    const auto stamp=1000000000LL+i*20000000;
    h.AddSpeed(stamp,2,i!=4);h.AddYaw(stamp,0,true);
  }
  const auto m=h.Between(1000000000,1200000000,0,0,asmc::lidar::Vec::Zero());
  EXPECT_TRUE(m.valid);EXPECT_FALSE(m.reliable);EXPECT_NEAR(.4,m.dx_m,1e-6);
  EXPECT_FALSE(h.Between(1000000000,1400000000,0,0,asmc::lidar::Vec::Zero()).valid);
  EXPECT_FALSE(h.Between(1000000000,2100000000,0,0,asmc::lidar::Vec::Zero()).valid);
  h.AddSpeed(1200000000,std::numeric_limits<double>::quiet_NaN(),false);
  EXPECT_FALSE(h.Between(1000000000,1200000000,0,0,asmc::lidar::Vec::Zero()).valid);
}
TEST(TunnelMotionHistory, IntegratesBothSignedBodyVelocityComponents) {
  TunnelMotionHistory h;
  for(int i=0;i<=5;++i) {
    h.AddYaw(1000000000LL+i*20000000,0,true);
    h.AddSpeed(1000000000LL+i*20000000,-3,true,1);
  }
  const auto m=h.Between(1000000000,1100000000,0,0,asmc::lidar::Vec::Zero());
  ASSERT_TRUE(m.valid);EXPECT_NEAR(-.3,m.dx_m,1e-6);EXPECT_NEAR(.1,m.dy_m,1e-6);
}
TEST(TunnelGapReanchor, RequiresAConfirmedCurrentPairWithStableWidthAndUncertainLongGap) {
  for(int scenario=0;scenario<8;++scenario) {
    TunnelRouteConfig cfg;cfg.integrated_motion=true;cfg.params.wall_gap_reanchor_enable=true;
    TunnelRoute route(cfg);route.Select(false);auto cam=NoImage();auto cloud=Scene();
    tunnel_filter::EgoMotion motion;motion.valid=true;motion.integrated=true;
    motion.reliable=true;motion.integration_steps=10;motion.dt_s=.1f;
    for(int k=0;k<3;++k)route.Process(cloud,Local(),1000000000+k*100000000,VehicleSpec{},&cam,&motion);
    ASSERT_TRUE(route.last().walls[0].removal_allowed);ASSERT_TRUE(route.last().walls[1].removal_allowed);
    CloudPtr shifted(new Cloud(*cloud));
    // scenario 0: same corridor translated laterally by 1m.
    // scenario 1: a different corridor width; 2: trusted odometry must use the
    // ordinary association gate; 3: a short frame must not use gap recovery.
    // scenario 4: no CURRENT right wall, so a previous pair cannot erase it.
    for(auto& p:shifted->points) {
      if(scenario!=1 || p.y>0)p.y+=scenario==5?2.f:1.f;
      if(scenario==6) {
        const float x=p.x,y=p.y;p.x=std::cos(.2f)*x-std::sin(.2f)*y;p.y=std::sin(.2f)*x+std::cos(.2f)*y;
      }
    }
    if(scenario==4) {
      auto& pts=shifted->points;
      pts.erase(std::remove_if(pts.begin(),pts.end(),[](const auto& p){return p.y< -3.9f&&p.z> -1.2f;}),pts.end());
      shifted->width=pts.size();
    }
    motion.reliable=scenario==2;motion.dt_s=scenario==3?.1f:.6f;motion.valid=scenario!=7;
    const auto out=route.Process(shifted,Local(),1900000000,VehicleSpec{},&cam,&motion);
    if(scenario==0) {
      EXPECT_TRUE(out.report.wall_left_reanchored);EXPECT_TRUE(out.report.wall_right_reanchored);
      EXPECT_TRUE(route.last().walls[0].removal_allowed);EXPECT_GT(out.report.wall_labeled,0);
      // Ordinary object support in the middle of the corridor is still kept.
      int object_points=0;
      for(std::size_t i=0;i<shifted->size();++i) {
        const auto& p=shifted->at(i);
        if(p.x>=12&&p.x<15&&p.y>.19f&&p.y<1.81f&&p.z> -1.2f&&p.z<.3f) {
          ++object_points;EXPECT_EQ(tunnel_filter::kKept,route.last().labels[i]);
        }
      }
      EXPECT_GT(object_points,500);
    } else {
      EXPECT_FALSE(out.report.wall_left_reanchored);EXPECT_FALSE(out.report.wall_right_reanchored);
    }
  }
}
TEST(TunnelGapReanchor, FreshEntryStillRequiresThreeFrames) {
  TunnelRouteConfig cfg;cfg.integrated_motion=true;cfg.params.wall_gap_reanchor_enable=true;
  TunnelRoute route(cfg);route.Select(false);auto cam=NoImage();
  tunnel_filter::EgoMotion m;m.valid=true;m.integrated=true;m.reliable=false;m.dt_s=.6f;m.integration_steps=20;
  const auto out=route.Process(Scene(),Local(),1000000000,VehicleSpec{},&cam,&m);
  EXPECT_FALSE(out.report.wall_left_reanchored);EXPECT_FALSE(route.last().walls[0].removal_allowed);
}

#include <asmc_lidar/experimental/tunnel_noise.hpp>
TEST(TunnelTemporalNoise, CausalHoldReleaseShapeRecoveryAndTunnelExit) {
  TunnelNoiseGate gate;TunnelNoiseConfig cfg;cfg.enabled=true;cfg.hold_frames=8;
  TrackedObject o;o.tracking.s32_Id=1;o.tracking.s32_MissCount=0;
  o.camera.str_ObjectClass="unknown";o.camera.str_YoloClass="unknown";
  o.detection.obb_length_m=4;o.detection.obb_width_m=.3;
  o.detection.f32_OBB_ZMin=-1;o.detection.f32_OBB_ZMax=2;
  GroundPlaneSnapshot plane;TunnelNoiseGate::Stats stats;
  for(int i=1;i<=9;++i) {
    auto hidden=gate.Update({o},plane,true,true,i*70000000LL,cfg,stats);
    EXPECT_EQ(i<=8,hidden.count(1)>0);EXPECT_EQ(i,stats.max_run);
  }
  o.detection.obb_width_m=2;
  EXPECT_TRUE(gate.Update({o},plane,true,true,700000000,cfg,stats).empty());
  o.detection.obb_width_m=.3;
  EXPECT_EQ(1,gate.Update({o},plane,true,true,770000000,cfg,stats).count(1));
  EXPECT_TRUE(gate.Update({o},plane,true,false,840000000,cfg,stats).empty());
  EXPECT_EQ(1,gate.Update({o},plane,true,true,910000000,cfg,stats).count(1));
}
TEST(TunnelTemporalNoise, ProtectedCompactMissingWallAndNonfiniteObjectsPass) {
  TrackedObject o;o.tracking.s32_Id=1;o.camera.str_ObjectClass="unknown";o.camera.str_YoloClass="unknown";
  o.detection.obb_length_m=4;o.detection.obb_width_m=.3;o.detection.f32_OBB_ZMin=-1;o.detection.f32_OBB_ZMax=2;
  GroundPlaneSnapshot p;
  EXPECT_EQ(2,TunnelNoiseGate::Kind(o,p,true));
  EXPECT_EQ(0,TunnelNoiseGate::Kind(o,p,false));
  o.camera.str_ObjectClass="person";EXPECT_EQ(0,TunnelNoiseGate::Kind(o,p,true));
  o.camera.str_ObjectClass="unknown";o.detection.obb_width_m=2;EXPECT_EQ(0,TunnelNoiseGate::Kind(o,p,true));
  o.detection.obb_width_m=std::numeric_limits<float>::quiet_NaN();EXPECT_EQ(0,TunnelNoiseGate::Kind(o,p,true));
  TunnelNoiseConfig c;c.hold_frames=31;EXPECT_THROW(ValidateTunnelNoise(c),std::invalid_argument);
}
TEST(TunnelTemporalNoise, GroundRequiresFlatShapeAndValidatedRoadHeight) {
  TrackedObject o;o.camera.str_ObjectClass="unknown";o.camera.str_YoloClass="unknown";
  o.detection.obb_length_m=4;o.detection.obb_width_m=.5;o.detection.f32_OBB_ZMin=-1.8;o.detection.f32_OBB_ZMax=-1.7;
  GroundPlaneSnapshot p;p.valid=true;p.coefficients={0,0,1,1.8};
  EXPECT_EQ(1,TunnelNoiseGate::Kind(o,p,false));
  p.valid=false;EXPECT_EQ(0,TunnelNoiseGate::Kind(o,p,false));
  p.valid=true;o.detection.f32_OBB_ZMax=-1;EXPECT_EQ(0,TunnelNoiseGate::Kind(o,p,false));
}

TEST(TunnelTemporalNoise, PipelineRemovesPointSupportButKeepsTrackerEvidenceForRelease) {
  PipelineConfig cfg;cfg.calibration_id="temporal-test";cfg.capture_debug=true;
  cfg.tunnel.enabled=true;cfg.tunnel_route.original_pipeline=true;cfg.tunnel_noise.enabled=true;cfg.tunnel_noise.hold_frames=2;
  cfg.tunnel_route.params.wall_enable=true;cfg.tunnel_route.params.obb_filter_enable=false;
  cfg.tunnel_route.params.cluster_tolerance_m=.4f;
  auto cloud=Scene();
  for(float x=-20;x<5;x+=.4f)for(float z=-1.1;z<2.5;z+=.2f)for(float y:{-5.f,5.f})cloud->push_back(Point(x,y,z));
  for(float x=20;x<24;x+=.1f)for(float y=2;y<2.3f;y+=.05f)for(float z=-1.13f;z< -1.09f;z+=.02f)cloud->push_back(Point(x,y,z));
  Pipeline pipe(cfg);ScanInput scan;scan.points=cloud;
  scan.tunnel_camera=std::make_shared<TunnelCameraInput>(NoImage());asmc::lidar::PoseSelection denied;
  int shadow_id=-1;
  for(int i=0;i<32;++i) {
    scan.time.stamp_ns=1000000000LL+i*70000000LL;
    const auto out=pipe.Process(scan,Local(),denied);ASSERT_TRUE(out.processed);
    if(i<29) { EXPECT_EQ(0,out.tunnel.temporal_hidden); continue; }
    if(i<31) {
      EXPECT_GT(out.tunnel.temporal_hidden,0) << "exempt=" << out.tunnel.temporal_boundary_exempt << " wall=" << out.tunnel.wall_left_state << "/" << out.tunnel.wall_right_state;EXPECT_GT(out.tunnel.temporal_removed_points,0);
      bool found=false;
      for(const auto& o:*pipe.tracker().LastFrame())if(TunnelNoiseGate::Kind(o,out.plane,false)==1){found=true;if(shadow_id<0)shadow_id=o.tracking.s32_Id;EXPECT_EQ(shadow_id,o.tracking.s32_Id);}
      EXPECT_TRUE(found);
      for(const auto& o:out.objects)EXPECT_EQ(0,TunnelNoiseGate::Kind(o,out.plane,false));
    } else {
      EXPECT_EQ(0,out.tunnel.temporal_hidden);bool released=false;
      for(const auto& o:out.objects)if(o.tracking.s32_Id==shadow_id)released=true;
      EXPECT_TRUE(released);
    }
  }
}
TEST(TunnelTemporalNoise, PredictionDoesNotAdvanceEvidenceAndLongInputGapRestartsIt) {
  TunnelNoiseGate gate;TunnelNoiseConfig cfg;cfg.enabled=true;
  TrackedObject o;o.tracking.s32_Id=3;o.camera.str_ObjectClass="unknown";o.camera.str_YoloClass="unknown";
  o.detection.obb_length_m=4;o.detection.obb_width_m=.3;o.detection.f32_OBB_ZMax=1;
  GroundPlaneSnapshot p;TunnelNoiseGate::Stats stats;
  EXPECT_EQ(1,gate.Update({o},p,true,true,1000000000,cfg,stats).count(3));
  o.tracking.s32_MissCount=1;EXPECT_EQ(1,gate.Update({o},p,true,true,1070000000,cfg,stats).count(3));EXPECT_EQ(0,stats.max_run);
  o.tracking.s32_MissCount=0;gate.Update({o},p,true,true,1140000000,cfg,stats);EXPECT_EQ(1,stats.max_run);
  gate.Update({o},p,true,true,1500000000,cfg,stats);EXPECT_EQ(1,stats.max_run);
}

TEST(TunnelTemporalNoise, VerifiedWallSupportDoesNotExtendAcrossTunnelEnds) {
  TunnelRouteConfig cfg;TunnelRoute route(cfg);route.Select(false);auto cloud=Scene();auto cam=NoImage();
  for(float x=-20;x<5;x+=.4f)for(float z=-1.1;z<2.5;z+=.2f)for(float y:{-5.f,5.f})cloud->push_back(Point(x,y,z));
  for(int i=0;i<3;++i)route.Process(cloud,Local(),1000000000LL+i*70000000LL,VehicleSpec{},&cam);
  TrackedObject o;
  for(auto& q:o.detection.obb_corners_level){q.x=20;q.y=0;}
  EXPECT_TRUE(route.InsideCurrentWalls(o));
  for(auto& q:o.detection.obb_corners_level)q.x=34;
  EXPECT_FALSE(route.InsideCurrentWalls(o));
  for(auto& q:o.detection.obb_corners_level)q.x=-19;
  EXPECT_FALSE(route.InsideCurrentWalls(o));
}

TEST(TunnelBoundaryRescue, AddsOnlyElevatedCompactSparseBoundarySupportWithoutChangingAnyPoint) {
  for(int scenario=0;scenario<5;++scenario) {
    TunnelRouteConfig a;a.params.obb_filter_enable=false;
    TunnelRouteConfig b=a;b.boundary_cluster_rescue=true;
    if(scenario==4){a.cluster_z.enabled=b.cluster_z.enabled=true;a.cluster_z.min_sensor_z_m=b.cluster_z.min_sensor_z_m=0;}
    TunnelRoute before(a),after(b);before.Select(false);after.Select(false);
    auto cloud=Scene();auto cam=NoImage();
    for(float x=-20;x<5;x+=.4f)for(float z=-1.1;z<2.5;z+=.2f)for(float y:{-5.f,5.f})cloud->push_back(Point(x,y,z));
    auto& pts=cloud->points;pts.erase(std::remove_if(pts.begin(),pts.end(),[](const auto& q){return q.x>=25.f;}),pts.end());cloud->width=pts.size();
    for(int i=0;i<7;++i) {
      const float x=(scenario==1?15.f:25.5f)+(i%3)*.2f;
      const float y=4.1f+(i/3)*(scenario==3?.01f:.2f);
      cloud->push_back(Point(x,y,scenario==2?-1.2f:-.3f));
    }
    TunnelRouteOutput left,right;
    for(int frame=0;frame<4;++frame) {
      const auto stamp=1000000000LL+frame*70000000LL;
      left=before.Process(cloud,Local(),stamp,VehicleSpec{},&cam);
      right=after.Process(cloud,Local(),stamp,VehicleSpec{},&cam);
    }
    EXPECT_EQ(left.kept_indices,right.kept_indices);
    EXPECT_EQ(CloudDigest(*left.filtered),CloudDigest(*right.filtered));
    if(scenario==0){
      int kept=0;for(const auto& q:right.filtered->points)if(q.x>25.4f&&q.y<4.8f)++kept;
      SCOPED_TRACE("kept="+std::to_string(kept)+" walls="+right.report.wall_left_state+"/"+right.report.wall_right_state);
      EXPECT_EQ(1,right.report.boundary_rescued_clusters);EXPECT_EQ(7,right.report.boundary_rescued_points);EXPECT_EQ(left.clusters.size()+1,right.clusters.size());}
    else {EXPECT_EQ(0,right.report.boundary_rescued_clusters);EXPECT_EQ(left.clusters.size(),right.clusters.size());}
    for(const auto& c:right.clusters)for(int i:c.indices){EXPECT_GE(i,0);EXPECT_LT(std::size_t(i),right.filtered->size());}
  }
}

TEST(TunnelResidualSurface, RejectedStructuralObbSupportIsRemovedFromActualCloudWithoutChangingDetections) {
  TunnelRouteConfig baseline;baseline.params.wall_enable=false;
  auto candidate=baseline;candidate.params.residual_wall_surface_enable=true;
  TunnelRoute before(baseline),after(candidate);before.Select(false);after.Select(false);
  auto cloud=Scene();auto cam=NoImage();
  auto left=before.Process(cloud,Local(),1000000000LL,VehicleSpec{},&cam);
  auto right=after.Process(cloud,Local(),1000000000LL,VehicleSpec{},&cam);
  int old_wall=0,new_wall=0;
  for(const auto& pt:left.filtered->points)if(std::fabs(pt.y)>4.9f)++old_wall;
  for(const auto& pt:right.filtered->points)if(std::fabs(pt.y)>4.9f)++new_wall;
  EXPECT_GT(old_wall,0);EXPECT_EQ(0,new_wall);
  ASSERT_EQ(left.centroids->size(),right.centroids->size());
  EXPECT_EQ(CloudDigest(*left.centroids),CloudDigest(*right.centroids));
  EXPECT_LT(right.filtered->size(),left.filtered->size());
  for(const auto& c:right.clusters)for(int i:c.indices){ASSERT_GE(i,0);ASSERT_LT(std::size_t(i),right.filtered->size());}
}

TEST(TunnelConfirmation, AdaptiveThresholdsKeepTrackerStateAndProtectKnownClasses) {
  TunnelNoiseGate gate;TunnelNoiseConfig cfg;cfg.enabled=true;cfg.confirmation_all_unknown=true;
  cfg.adaptive_confirmation=true;cfg.confirmation_frames=3;cfg.structural_confirmation_frames=5;
  std::vector<TrackedObject> v(3);
  for(int i=0;i<3;++i){v[i].tracking.s32_Id=i;v[i].camera.str_ObjectClass="unknown";v[i].camera.str_YoloClass="unknown";}
  v[2].camera.str_YoloClass="person";
  std::map<int,ConfirmationKind> kinds{{0,ConfirmationKind::General},{1,ConfirmationKind::ThinHorizontalFragment},{2,ConfirmationKind::WallFragment}};
  GroundPlaneSnapshot plane;TunnelNoiseGate::Stats stats;
  for(int i=0;i<5;++i) {
    auto hidden=gate.Update(v,plane,false,true,1000000000LL+i*70000000LL,cfg,stats,nullptr,&kinds);
    EXPECT_EQ(i<2,hidden.count(0)>0);EXPECT_EQ(i<4,hidden.count(1)>0);EXPECT_FALSE(hidden.count(2));
  }
  EXPECT_TRUE(gate.Update(v,plane,false,false,1400000000LL,cfg,stats,nullptr,&kinds).empty());
  EXPECT_EQ(2u,gate.Update(v,plane,false,true,1500000000LL,cfg,stats,nullptr,&kinds).size());
  cfg.structural_confirmation_frames=0;EXPECT_THROW(ValidateTunnelNoise(cfg),std::invalid_argument);
}
TEST(TunnelConfirmation, ActualPipelineHoldsBoxesAndPointsUntilThirdObservation) {
  PipelineConfig cfg;cfg.calibration_id="confirm-test";cfg.capture_debug=true;
  cfg.tunnel.enabled=true;cfg.tunnel_route.original_pipeline=true;
  cfg.tunnel_route.params.obb_filter_enable=false;
  cfg.tunnel_noise.enabled=true;cfg.tunnel_noise.confirmation_all_unknown=true;cfg.tunnel_noise.confirmation_frames=3;
  Pipeline pipe(cfg);ScanInput scan;scan.points=Scene();
  scan.tunnel_camera=std::make_shared<TunnelCameraInput>(NoImage());asmc::lidar::PoseSelection denied;
  for(int i=0;i<3;++i) {
    scan.time.stamp_ns=1000000000LL+i*70000000LL;
    auto out=pipe.Process(scan,Local(),denied);ASSERT_TRUE(out.processed);
    ASSERT_FALSE(pipe.tracker().LastFrame()->empty());
    if(i<2) {
      EXPECT_TRUE(out.objects.empty());EXPECT_GT(out.tunnel.temporal_hidden,0);
      EXPECT_GT(out.tunnel.temporal_removed_points,0);
    } else EXPECT_FALSE(out.objects.empty());
  }
}

TEST(TunnelRoute, CoreObbYawAlwaysFollowsDeclaredLongSideAtEveryVehicleOrientation) {
  for(int degrees=0;degrees<180;degrees+=15) {
    TunnelRouteConfig cfg;cfg.params.wall_enable=false;cfg.params.obb_filter_enable=false;
    TunnelRoute route(cfg);route.Select(false);auto cam=NoImage();CloudPtr cloud(new Cloud);
    for(float x=-10;x<=35;x+=.5f)for(float y=-5;y<=5;y+=.5f)cloud->push_back(Point(x,y,-1.45f));
    const float angle=degrees*3.14159265358979323846f/180.f;
    for(float u=-3;u<=3;u+=.2f)for(float v=-.2f;v<=.2f;v+=.1f)
      for(float z=-1;z<=0;z+=.2f)
        cloud->push_back(Point(15+u*std::cos(angle)-v*std::sin(angle),u*std::sin(angle)+v*std::cos(angle),z));
    route.Process(cloud,Local(),1000000000,VehicleSpec{},&cam);
    ASSERT_EQ(1u,route.last().objects.size())<<degrees;
    const auto& box=route.last().objects.front().obb;
    EXPECT_GE(box.length,box.width);
    EXPECT_GT(std::fabs(std::cos(box.yaw-angle)),.98)<<degrees<<" yaw="<<box.yaw;
  }
}

TEST(TunnelConfirmation, ShortCurrentWallIsSuspicionOnlyAndNeedsOppositeWallAndBoundedSupport) {
  tunnel_filter::Params p;tunnel_filter::FrameResult f;
  auto& candidate=f.walls[0].first_attempt;
  candidate.plane={0,-1,0,6};candidate.lateral_distance=6;
  candidate.inliers=100;candidate.inlier_ratio=.9;candidate.span=8;candidate.occupied_bins=4;
  candidate.vertical_span=3;candidate.beyond_fraction=0;
  candidate.intervals={{4,12}};candidate.reject_reason="short_span";
  tunnel_filter::WallSegment opposite;opposite.plane={0,1,0,5};
  opposite.valid=true;f.walls[1].segments.push_back(opposite);
  EXPECT_TRUE(ConfirmationWallSegments(f,p).empty());
  f.walls[1].removal_allowed=true;
  const auto segments=ConfirmationWallSegments(f,p);
  ASSERT_EQ(2u,segments.size());
  EXPECT_FALSE(f.walls[0].removal_allowed);
  EXPECT_TRUE(segments.front().InIntervals(8,0));
  EXPECT_FALSE(segments.front().InIntervals(20,0));
  candidate.beyond_fraction=.8;
  EXPECT_EQ(1u,ConfirmationWallSegments(f,p).size());
  candidate.beyond_fraction=0;candidate.vertical_span=1;
  EXPECT_EQ(1u,ConfirmationWallSegments(f,p).size());
}

TEST(TunnelConfirmation, ElevatedFlatUsesSixAndCannotReuseEarlierGeneralConfirmation) {
  TunnelNoiseGate gate;TunnelNoiseConfig cfg;cfg.enabled=true;cfg.confirmation_all_unknown=true;
  cfg.adaptive_confirmation=true;TrackedObject o;o.tracking.s32_Id=7;
  o.camera.str_ObjectClass="unknown";o.camera.str_YoloClass="unknown";
  GroundPlaneSnapshot p;TunnelNoiseGate::Stats stats;std::map<int,ConfirmationKind> kinds{{7,ConfirmationKind::General}};
  for(int i=0;i<3;++i)gate.Update({o},p,false,true,1000000000LL+i*70000000LL,cfg,stats,nullptr,&kinds);
  kinds[7]=ConfirmationKind::ElevatedFlatFragment;
  for(int i=0;i<6;++i) {
    const auto h=gate.Update({o},p,false,true,1210000000LL+i*70000000LL,cfg,stats,nullptr,&kinds);
    EXPECT_EQ(i<5,h.count(7)>0);EXPECT_EQ(6,stats.required_frames[7]);
  }
  cfg.low_flat_confirmation_frames=31;EXPECT_THROW(ValidateTunnelNoise(cfg),std::invalid_argument);
}

TEST(TunnelRoute, HealthyGpsBypassesAllTunnelAndHistoricalEntranceFilters) {
  PipelineConfig off;off.calibration_id="healthy-gps";off.capture_debug=true;
  PipelineConfig on=off;on.tunnel.enabled=true;on.tunnel_route.original_pipeline=true;
  Pipeline a(off),b(on);ScanInput scan;scan.points=Scene();scan.time.stamp_ns=1000000000;
  asmc::lidar::PoseSelection map;map.valid=true;
  const auto left=a.Process(scan,Local(),map),right=b.Process(scan,Local(),map);
  EXPECT_FALSE(right.tunnel.ran);
  EXPECT_EQ(left.digests.roi,right.digests.roi);
  EXPECT_EQ(left.digests.voxel,right.digests.voxel);
  EXPECT_EQ(left.digests.ground_kept,right.digests.ground_kept);
  EXPECT_EQ(left.cluster_sizes,right.cluster_sizes);
  EXPECT_EQ(left.objects.size(),right.objects.size());
}

TEST(TunnelRoute, MultiViewUsesCanonicalOpticalAxesAndEightClassesWithoutClassification) {
  for(const std::string label:{"car","bus","truck","person","dog","deer","bicycle","stroller"}) {
    TunnelRoute route(TunnelRouteConfig{});route.Select(false);
    TunnelCameraInput input;TunnelCameraView camera;camera.calibrated=true;camera.available=true;
    camera.common_projection=true;camera.calibration.frame_id="camera_front";
    camera.calibration.width=1280;camera.calibration.height=720;
    camera.calibration.intrinsic<<640,0,640,0,640,360,0,0,1;
    camera.calibration.base_from_camera=asmc::lidar::Pose::Identity();
    asmc::lidar::camera::Box box;box.xmin=550;box.xmax=730;box.ymin=290;box.ymax=440;
    box.source_class_name=label;camera.boxes={box};input.views={camera};
    route.Process(Scene(),Local(),1000000000,VehicleSpec{},&input);
    EXPECT_GT(route.last().protected_points,0)<<label;
    EXPECT_TRUE(route.last().protection_available);
    input.views[0].available=false;
    route.Process(Scene(),Local(),1100000000,VehicleSpec{},&input);
    EXPECT_EQ(route.last().protected_points,0)<<label;
  }
}
