#include <asmc_lidar/experimental/low_obstacle_evidence.hpp>
#include <asmc_lidar/geometry/scan_frame.hpp>
namespace {
struct LowFixture {
  Cloud cloud;tunnel_filter::FrameResult frame;TunnelCameraInput camera;
  TunnelRouteConfig config;VehicleSpec vehicle;
  LowFixture() {
    config.low_obstacle_evidence=true;
    frame.leveled.reset(new Cloud);frame.ground.source=tunnel_filter::PlaneSource::kCurrent;
    frame.ground.plane.d=1.8f;
    for(int side=0;side<2;++side) {
      auto& w=frame.walls[side];w.removal_allowed=true;
      tunnel_filter::WallSegment s;s.valid=true;s.plane={0,side?1.f:-1.f,0,5};s.intervals={{0,40}};
      w.segments.push_back(s);
    }
    camera.calibrated=camera.available=true;camera.identity="source/epoch/frame";camera.stamp_ns=1000000000;
    camera.image_width=camera.image_height=camera.calibration.width=camera.calibration.height=200;
    camera.calibration.id="test";camera.calibration.base_from_camera=ScanFrame::SensorMount(vehicle);
    camera.calibration.intrinsic<<100,0,100,0,100,100,0,0,1;
    camera.still_current=[](){return true;};
    asmc::lidar::camera::Box b;b.xmin=80;b.xmax=120;b.ymin=100;b.ymax=130;
    b.source_class_name="box";b.confidence=.95;b.confidence_valid=true;camera.boxes={b};
  }
  void Add(float x,float y=0,float z=-1.4f) {
    pcl::PointIndices c;
    for(int j=0;j<6;++j) {
      auto p=Point(x+.03f*j,y,z);c.indices.push_back(cloud.size());
      cloud.push_back(p);frame.leveled->push_back(p);frame.labels.push_back(tunnel_filter::kKept);
    }
    frame.clusters.push_back(c);frame.objects.emplace_back();
  }
  std::vector<LowObstacleHint> Run(){return LowObstacleEvidence(cloud,frame,vehicle,1000000000,&camera,config);}
};
}
TEST(LowObstacleEvidence, CanonicalOpticalProjectionAcceptsOnlyFrontConnectedCluster) {
  LowFixture t;t.Add(10);ASSERT_EQ("box",t.Run()[0].source_class);
  t.Add(15);const auto r=t.Run();EXPECT_EQ("box",r[0].source_class);EXPECT_TRUE(r[1].source_class.empty());
}
TEST(LowObstacleEvidence, BackgroundAloneMarkedWallCannotBeProtected) {
  LowFixture t;t.Add(10);t.frame.objects[0].flags.wall=true;t.frame.objects[0].flags.removed=true;
  EXPECT_TRUE(t.Run()[0].source_class.empty());
}
TEST(LowObstacleEvidence, InvalidNearestSurfaceMustNotFallBackToProtectingFartherBackground) {
  LowFixture t;t.Add(10);t.Add(15);t.frame.objects[0].flags.wall=true;
  const auto hint=t.Run();EXPECT_TRUE(hint[0].source_class.empty());EXPECT_TRUE(hint[1].source_class.empty());
}
TEST(LowObstacleEvidence, CeilingCannotBeProtectedButGroundResidualCan) {
  LowFixture t;t.Add(10);auto& flags=t.frame.objects[0].flags;
  flags.ground_residual=flags.removed=true;EXPECT_EQ("box",t.Run()[0].source_class);
  flags.ceiling=true;EXPECT_TRUE(t.Run()[0].source_class.empty());
}
TEST(LowObstacleEvidence, MergedForegroundAndBackgroundCannotBeProtectedAsOneObject) {
  LowFixture t;t.Add(10);t.Add(15);
  auto& a=t.frame.clusters[0].indices;const auto b=t.frame.clusters[1].indices;
  a.insert(a.end(),b.begin(),b.end());t.frame.clusters.resize(1);t.frame.objects.resize(1);
  EXPECT_TRUE(t.Run()[0].source_class.empty());
}
TEST(LowObstacleEvidence, ConnectedUnownedBackgroundInvalidatesWholeComponent) {
  LowFixture t;t.Add(10);t.Add(10.25f);const auto r=t.Run();
  EXPECT_TRUE(r[0].source_class.empty());EXPECT_TRUE(r[1].source_class.empty());
}
TEST(LowObstacleEvidence, EquallyNearDisconnectedObjectsAreAmbiguous) {
  LowFixture t;t.Add(10,-.5);t.Add(10,.5);const auto r=t.Run();
  EXPECT_TRUE(r[0].source_class.empty());EXPECT_TRUE(r[1].source_class.empty());
}
TEST(LowObstacleEvidence, MissingStaleDisabledAndRestartedCameraRestoreNothing) {
  LowFixture t;t.Add(10);t.camera.available=false;EXPECT_TRUE(t.Run()[0].source_class.empty());
  t.camera.available=true;t.camera.stamp_ns-=50000001;EXPECT_TRUE(t.Run()[0].source_class.empty());
  t.camera.stamp_ns=1000000000;t.camera.still_current=[](){return false;};EXPECT_TRUE(t.Run()[0].source_class.empty());
  int calls=0;t.camera.still_current=[&](){return ++calls==1;};EXPECT_TRUE(t.Run()[0].source_class.empty());
}
TEST(LowObstacleEvidence, InvalidDimensionsClassConfidenceOrCalibrationRestoreNothing) {
  LowFixture t;t.Add(10);t.camera.image_width=201;EXPECT_TRUE(t.Run()[0].source_class.empty());t.camera.image_width=200;
  t.camera.boxes[0].confidence=.69;EXPECT_TRUE(t.Run()[0].source_class.empty());t.camera.boxes[0].confidence=.95;
  t.camera.boxes[0].source_class_name="static_obstacle";EXPECT_TRUE(t.Run()[0].source_class.empty());
  t.camera.boxes[0].source_class_name="box";t.camera.calibration.intrinsic.setZero();EXPECT_TRUE(t.Run()[0].source_class.empty());
}
TEST(LowObstacleEvidence, GroundWallPointsAndUnconfirmedWallIntervalsRestoreNothing) {
  LowFixture t;t.Add(10);t.frame.labels[0]=tunnel_filter::kGround;EXPECT_TRUE(t.Run()[0].source_class.empty());
  t.frame.labels[0]=tunnel_filter::kWall;EXPECT_TRUE(t.Run()[0].source_class.empty());t.frame.labels[0]=tunnel_filter::kKept;
  t.frame.walls[1].removal_allowed=false;EXPECT_TRUE(t.Run()[0].source_class.empty());
}
TEST(LowObstacleEvidence, InvalidPlaneTooFewPointsAndPartiallyOutsideBoxRestoreNothing) {
  LowFixture t;t.Add(10);t.frame.ground.source=tunnel_filter::PlaneSource::kUnavailable;EXPECT_TRUE(t.Run()[0].source_class.empty());
  t.frame.ground.source=tunnel_filter::PlaneSource::kCurrent;t.camera.boxes[0].xmax=99;EXPECT_TRUE(t.Run()[0].source_class.empty());
  t.camera.boxes[0].xmax=120;t.frame.clusters[0].indices.resize(4);EXPECT_TRUE(t.Run()[0].source_class.empty());
}
TEST(LowObstacleEvidence, UnprojectedEdgePointIsDiscardedInsteadOfReceivingBlanketProtection) {
  LowFixture t;t.Add(10);
  auto& cluster=t.frame.clusters[0];
  for(int k=0;k<5;++k){auto p=Point(10.18f+.02f*k,0,-1.4f);cluster.indices.push_back(t.cloud.size());t.cloud.push_back(p);t.frame.leveled->push_back(p);t.frame.labels.push_back(tunnel_filter::kKept);}
  // Ten of eleven returns own the connected component in the box.
  t.cloud[10].y=t.frame.leveled->at(10).y=3;
  const auto hint=t.Run()[0];EXPECT_EQ("box",hint.source_class);EXPECT_EQ(10u,hint.support.size());
  EXPECT_EQ(hint.support.end(),std::find(hint.support.begin(),hint.support.end(),10));
}
TEST(TunnelCameraMotion, UsesSignedTranslationAndFullAttitudeAndRejectsUntrustedOrOldInputs) {
  TunnelMotionHistory history;const auto mount=asmc::lidar::Pose::Identity();
  for(int k=0;k<=15;++k){history.AddYaw(1000000000+k*10000000,0,true);history.AddSpeed(1000000000+k*10000000,2,true);}
  auto t=history.CameraFromScan(1000000000,1100000000,mount);ASSERT_TRUE(t);EXPECT_NEAR(.2,t->translation().x(),1e-6);
  EXPECT_FALSE(history.CameraFromScan(1000000000,1160000000,mount));
  history.AddYaw(1150000000,.1,true);EXPECT_FALSE(history.CameraFromScan(1000000000,1150000000,mount));
  history.AddYaw(1150000000,0,false);EXPECT_FALSE(history.CameraFromScan(1100000000,1150000000,mount));
}
TEST(TunnelCameraMotion, AllowsMeasuredImuCadenceButRejectsMoreThanEightyMillisecondsWithoutAttitude) {
  TunnelMotionHistory history;const auto mount=asmc::lidar::Pose::Identity();
  history.AddYaw(1000000000,0,true);history.AddSpeed(1000000000,0,true);
  EXPECT_TRUE(history.CameraFromScan(1000000000,1070000000,mount));
  EXPECT_FALSE(history.CameraFromScan(1000000000,1080000001,mount));
}
TEST(TunnelCameraMotion, FasterMotionRequiresAShorterImageDelayAndIncludesSensorLeverArm) {
  TunnelMotionHistory history;auto mount=asmc::lidar::Pose::Identity();mount.translation().x()=1;
  for(int k=0;k<=15;++k){history.AddYaw(1000000000+k*10000000,0,true);history.AddSpeed(1000000000+k*10000000,10,true);}
  const auto short_delay=history.CameraFromScan(1000000000,1050000000,mount);
  ASSERT_TRUE(short_delay);EXPECT_NEAR(.5,short_delay->translation().x(),1e-6);
  EXPECT_FALSE(history.CameraFromScan(1000000000,1100000000,mount));
  TunnelMotionHistory turn;turn.AddYaw(1000000000,0,true);turn.AddYaw(1050000000,.02,true);turn.AddSpeed(1000000000,0,true);
  auto rotated=turn.CameraFromScan(1000000000,1050000000,mount);ASSERT_TRUE(rotated);
  EXPECT_NEAR(std::cos(.02)-1,rotated->translation().x(),1e-6);EXPECT_NEAR(std::sin(.02),rotated->translation().y(),1e-6);
}
TEST(LowObstaclePipeline, PreservesExistingConfirmationAndBridgesOnlyTwoPredictionsWithSameCameraEpoch) {
  for(bool restart:{false,true}) {
    PipelineConfig cfg;cfg.calibration_id="test";cfg.capture_debug=true;
    cfg.tunnel.enabled=true;cfg.tunnel_route.original_pipeline=true;cfg.tunnel_route.low_obstacle_evidence=true;
    cfg.tunnel_route.cluster_z={true,-.9f};cfg.tunnel_route.params.cluster_min_points=5;
    cfg.tunnel_noise.enabled=true;cfg.tunnel_noise.confirmation_all_unknown=true;cfg.tunnel_noise.confirmation_frames=3;
    Pipeline pipeline(cfg);ScanInput scan;scan.points=Scene();
    for(int k=0;k<6;++k)scan.points->push_back(Point(10+(k%3)*.12f,(k/3)*.12f,-1.1f));
    auto camera=std::make_shared<TunnelCameraInput>(LowFixture{}.camera);
    camera->camera_epoch=7;camera->source_active=true;camera->source_current=[](){return true;};
    camera->boxes[0].xmin=95;camera->boxes[0].xmax=105;camera->boxes[0].ymin=108;camera->boxes[0].ymax=114;
    scan.tunnel_camera=camera;asmc::lidar::PoseSelection map;
    for(int k=0;k<8;++k) {
      scan.time.stamp_ns=1000000000+k*100000000;camera->stamp_ns=scan.time.stamp_ns;
      camera->available=k<5;if(restart&&k>=5)camera->camera_epoch=8;
      const auto result=pipeline.Process(scan,Local(),map);
      SCOPED_TRACE("scan="+std::to_string(k)+" rescued="+std::to_string(result.tunnel.low_obstacle_rescued)+
        " verified="+std::to_string(result.tunnel.low_obstacle_verified)+" clusters="+std::to_string(result.counts.clusters));
      const auto it=std::find_if(result.objects.begin(),result.objects.end(),[](const auto& o){return !o.detection.low_obstacle_cue.source_class.empty();});
      if(k<4||k==7||(restart&&k>=5))EXPECT_EQ(result.objects.end(),it)<<"scan "<<k;
      else {
        ASSERT_NE(result.objects.end(),it)<<"scan "<<k;
        EXPECT_EQ(k<5?0:k-4,it->tracking.s32_MissCount);
        EXPECT_EQ("unknown",it->camera.str_ObjectClass);
        EXPECT_EQ(1400000000LL,it->detection.low_obstacle_cue.stamp_ns);
      }
    }
  }
}
