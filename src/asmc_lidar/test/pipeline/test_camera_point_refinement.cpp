#include <asmc_lidar/detection/camera_point_refinement.hpp>
#include <asmc_lidar/experimental/tunnel_route.hpp>
#include <asmc_lidar/experimental/tunnel_motion.hpp>
#include <asmc_lidar/detection/detections.hpp>
#include <asmc_lidar/tracking/tracker.hpp>
#include <gtest/gtest.h>
#include <set>
#include <numeric>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <asmc_lidar/geometry/projection.hpp>

using namespace asmc::lidar;
using namespace asmc::lidar::pipeline;
int main(int argc,char** argv) {::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
namespace {
struct Fixture {
  CloudPtr cloud{new Cloud};Context context;TunnelCameraInput cam;
  CameraPointRefinementConfig cfg;std::vector<pcl::PointIndices> clusters;
  Fixture() {
    cfg.enabled=true;context.time.stamp_ns=1000000000;context.calibration_source="test";
    context.frame=Frame::Sensor;
    cam.available=cam.calibrated=true;cam.identity="image-1";cam.camera_epoch=1;
    cam.stamp_ns=context.time.stamp_ns;cam.image_width=cam.image_height=200;
    cam.calibration.width=cam.calibration.height=200;
    cam.calibration.intrinsic<<100,0,100,0,100,100,0,0,1;
    cam.still_current=[](){return true;};
    camera::Box b;b.xmin=88;b.xmax=112;b.ymin=88;b.ymax=112;
    b.source_class_name="car";b.confidence_valid=true;b.confidence=.95f;cam.boxes.push_back(b);
  }
  void Surface(float x,float y0=-.8f,float y1=.8f,float z0=-.8f,float z1=.8f) {
    for(float y=y0;y<=y1+.001f;y+=.2f)for(float z=z0;z<=z1+.001f;z+=.2f) {
      pcl::PointXYZI p{};p.x=x;p.y=y;p.z=z;cloud->push_back(p);
    }
  }
  void OneCluster() {pcl::PointIndices c;c.indices.resize(cloud->size());std::iota(c.indices.begin(),c.indices.end(),0);clusters={c};}
  CameraPointRefinementResult Run(const std::vector<TunnelWallEvidence>& walls={}) {
    return RefineCameraPoints(cloud,clusters,context,&cam,walls,cfg);
  }
};
}
TEST(CameraPointRefinement, SeparatesForegroundAndBackgroundWithoutDiscardingUnknownObstacle) {
  Fixture f;f.Surface(10);const auto foreground=f.cloud->size();f.Surface(20);f.OneCluster();
  const auto r=f.Run();ASSERT_TRUE(r.report.applied);ASSERT_EQ(2u,r.clusters.size());
  EXPECT_EQ(foreground,r.clusters[0].indices.size());EXPECT_EQ("car",r.support[0].source_class);
  EXPECT_EQ(0,r.parent_clusters[0]);  // preserve parent cue provenance, not its class
  std::set<int> all;for(const auto& c:r.clusters)for(int i:c.indices)EXPECT_TRUE(all.insert(i).second);
  EXPECT_EQ(f.cloud->size(),all.size());EXPECT_TRUE(r.removed_wall_indices.empty());
}
TEST(CameraPointRefinement, EqualDepthSurfacesAbstainRatherThanGuess) {
  Fixture f;f.Surface(10,-.9,-.5);f.Surface(10.2,.5,.9);f.OneCluster();
  auto r=f.Run();EXPECT_FALSE(r.report.applied);ASSERT_EQ(1u,r.report.decisions.size());
  EXPECT_EQ("ambiguous_near_surfaces",r.report.decisions[0].reason);
}
TEST(CameraPointRefinement, SparseNearerObstacleVetoesBackgroundSelection) {
  Fixture f;f.Surface(10,0,0,0,.4);f.Surface(20);f.OneCluster();
  auto r=f.Run();EXPECT_FALSE(r.report.applied);
  ASSERT_EQ(1u,r.report.decisions.size());EXPECT_EQ("unsupported_nearer_surface",r.report.decisions[0].reason);
}
TEST(CameraPointRefinement, RemovesOnlyConfirmedWallWithinMeasuredIntervals) {
  Fixture f;f.cam.boxes[0].xmin=65;f.cam.boxes[0].xmax=88;
  f.Surface(10,1.6,2.8);const auto object=f.cloud->size();
  for(float x=7;x<30;x+=.2f)f.Surface(x,3,3,-1,1);
  f.OneCluster();TunnelWallEvidence wall;wall.valid=wall.allowed=true;
  wall.plane={0,-1,0,3};wall.intervals={{5,35}};
  auto r=f.Run({wall});ASSERT_TRUE(r.report.applied);EXPECT_GT(r.report.removed_wall_points,0);
  ASSERT_FALSE(r.clusters.empty());EXPECT_EQ(object,r.clusters[0].indices.size());
  for(int i:r.removed_wall_indices)EXPECT_FLOAT_EQ(3,f.cloud->at(i).y);
  wall.allowed=false;r=f.Run({wall});EXPECT_TRUE(r.removed_wall_indices.empty());
  wall.allowed=true;wall.intervals={{40,50}};r=f.Run({wall});EXPECT_TRUE(r.removed_wall_indices.empty());
}
TEST(CameraPointRefinement, SourceResetAbortsPlanAndDoesNotMutateCloud) {
  Fixture f;f.Surface(10);f.Surface(20);f.OneCluster();int checks=0;
  f.cam.still_current=[&](){return ++checks==1;};const auto before=f.cloud->size();
  const auto r=f.Run();EXPECT_FALSE(r.report.applied);EXPECT_TRUE(r.clusters.empty());EXPECT_EQ(before,f.cloud->size());
}
TEST(CameraPointRefinement, ExtractingCarCannotEraseSparseOtherObjectAgainstTheSameWall) {
  Fixture f;f.cam.boxes[0].xmin=65;f.cam.boxes[0].xmax=88;f.Surface(10,1.6,2.8);
  for(float x=7;x<30;x+=.2f)f.Surface(x,3,3,-1,1);
  f.OneCluster();auto person=f.cam.boxes[0];person.xmin=88.5;person.xmax=92;
  person.ymin=95;person.ymax=105;person.source_class_name="person";f.cam.boxes.push_back(person);
  TunnelWallEvidence wall;wall.valid=wall.allowed=true;wall.plane={0,-1,0,3};wall.intervals={{5,35}};
  const auto r=f.Run({wall});ASSERT_TRUE(r.report.applied);EXPECT_GT(r.report.removed_wall_points,0);
  int protected_points=0;
  for(std::size_t i=0;i<f.cloud->size();++i) {
    const auto& p=f.cloud->at(i);const float u=100-100*p.y/p.x,v=100-100*p.z/p.x;
    if(u>=person.xmin&&u<=person.xmax&&v>=person.ymin&&v<=person.ymax) {
      ++protected_points;EXPECT_EQ(r.removed_wall_indices.end(),std::find(r.removed_wall_indices.begin(),r.removed_wall_indices.end(),i));
    }
  }
  EXPECT_GT(protected_points,0);
}
TEST(CameraPointRefinement, RequiresCausalMotionCompensationAndExactImageGeometry) {
  Fixture f;f.Surface(10);f.OneCluster();
  f.cam.stamp_ns=f.context.time.stamp_ns+1;EXPECT_FALSE(f.Run().report.applied);
  f.cam.stamp_ns=f.context.time.stamp_ns-10000000;EXPECT_FALSE(f.Run().report.applied);
  f.cam.motion_compensated=true;EXPECT_TRUE(f.Run().report.applied);
  f.cam.stamp_ns=f.context.time.stamp_ns-200000000;EXPECT_FALSE(f.Run().report.applied);
  f.cam.stamp_ns=f.context.time.stamp_ns;f.cam.image_width=199;EXPECT_FALSE(f.Run().report.applied);
}
TEST(CameraPointRefinement, OverlappingBoxesNeverDuplicatePoints) {
  Fixture f;f.Surface(10);f.OneCluster();f.cam.boxes.push_back(f.cam.boxes.front());
  auto r=f.Run();ASSERT_TRUE(r.report.applied);EXPECT_EQ(1,r.report.extracted_objects);
  EXPECT_EQ("overlapping_box_conflict",r.report.decisions[1].reason);
}
TEST(CameraPointRefinement, OneParentCanProduceTwoDistinctObjects) {
  Fixture f;f.Surface(10,-2,-1);f.Surface(10,1,2);f.OneCluster();
  auto& first=f.cam.boxes[0];first.xmin=108;first.xmax=122;
  auto second=first;second.xmin=78;second.xmax=92;f.cam.boxes.push_back(second);
  const auto r=f.Run();ASSERT_TRUE(r.report.applied);ASSERT_EQ(2u,r.clusters.size());
  EXPECT_EQ(2,r.report.extracted_objects);EXPECT_NE(r.support[0].box_index,r.support[1].box_index);
  std::set<int> all;for(const auto& c:r.clusters)for(int i:c.indices)EXPECT_TRUE(all.insert(i).second);
  EXPECT_EQ(f.cloud->size(),all.size());
}

namespace {
Fixture SteelFixture(double angle=0) {
  Fixture f;f.cfg.enabled=false;f.cfg.steel_fragment_merge=true;
  auto& b=f.cam.boxes[0];b.source_class_name="steel_barricade";b.confidence=.6f;
  b.xmin=78;b.xmax=122;b.ymin=90;b.ymax=102;
  for(float side:{-1.f,1.f}) {
    pcl::PointIndices cluster;
    for(float z:{.1f,.3f,.5f})for(float w:{-.04f,.04f}) {
      pcl::PointXYZI p{};p.x=10-side*1.15*std::sin(angle)+w;p.y=side*1.15*std::cos(angle);p.z=z;
      cluster.indices.push_back(f.cloud->size());f.cloud->push_back(p);
    }
    f.clusters.push_back(cluster);
  }
  // Adapt the detector's measured horizontal extent; no world-coordinate or
  // fixed heading gate is part of the algorithm.
  float lo=INFINITY,hi=-INFINITY;
  for(const auto& p:*f.cloud){const float u=100-100*p.y/p.x;lo=std::min(lo,u);hi=std::max(hi,u);}
  b.xmin=lo-1;b.xmax=hi+1;return f;
}
}
TEST(SteelFragmentMerge, JoinsOnlyMeasuredSupportsAtSeveralRotations) {
  for(double angle:{-1.3,-1.,-.5,0.,.5,1.,1.3,2.5}) {
    auto f=SteelFixture(angle);auto r=f.Run();ASSERT_TRUE(r.report.applied)<<angle;
    ASSERT_EQ(1u,r.clusters.size());EXPECT_EQ(12u,r.clusters[0].indices.size());
    EXPECT_EQ("steel_fragments_joined",r.report.decisions[0].reason);
    EXPECT_TRUE(r.removed_wall_indices.empty());EXPECT_EQ(12u,f.cloud->size());
  }
}
TEST(SteelFragmentMerge, MissingOrStaleCameraAndWeakClassLeaveOriginalParents) {
  auto f=SteelFixture();f.cam.available=false;EXPECT_FALSE(f.Run().report.applied);
  f.cam.available=true;f.cam.stamp_ns-=300000000;EXPECT_FALSE(f.Run().report.applied);
  f.cam.stamp_ns=f.context.time.stamp_ns;f.cam.boxes[0].confidence=.49f;EXPECT_FALSE(f.Run().report.applied);
}
TEST(SteelFragmentMerge, BoundedOlderEvidenceNeedsMotionAndNeverEnablesOlderGeneralRefinement) {
  auto f=SteelFixture();f.cam.stamp_ns-=200000000;
  EXPECT_FALSE(f.Run().report.applied);f.cam.motion_compensated=true;EXPECT_TRUE(f.Run().report.applied);
  f.cam.stamp_ns=f.context.time.stamp_ns-250000001;EXPECT_FALSE(f.Run().report.applied);
  Fixture car;car.cfg.steel_fragment_merge=true;car.Surface(10);car.OneCluster();car.cam.motion_compensated=true;
  car.cam.stamp_ns-=200000000;EXPECT_FALSE(car.Run().report.applied);
}
TEST(SteelWallIntegration, OlderSteelEvidenceCannotEnableGeneralWallCleanup) {
  auto f=SteelFixture();f.cfg.enabled=true;f.cfg.detached_wall_cleanup=true;
  f.cam.stamp_ns-=200000000;f.cam.motion_compensated=true;
  pcl::PointIndices background;
  for(int i=0;i<6;++i){pcl::PointXYZI p{};p.x=20;p.y=.5f+.1f*i;p.z=.2f;background.indices.push_back(f.cloud->size());f.cloud->push_back(p);}
  for(int i=0;i<11;++i){pcl::PointXYZI p{};p.x=20;p.y=3;p.z=.2f*i;background.indices.push_back(f.cloud->size());f.cloud->push_back(p);}
  f.clusters.push_back(background);TunnelWallEvidence w;w.valid=w.allowed=true;w.plane={0,-1,0,3};w.intervals={{5,30}};
  const auto r=f.Run({w});ASSERT_TRUE(r.report.applied);EXPECT_EQ(1,r.report.extracted_objects);
  EXPECT_EQ("steel_barricade",r.support[0].source_class);EXPECT_TRUE(r.removed_wall_indices.empty());
  ASSERT_EQ(2u,r.clusters.size());EXPECT_EQ(background.indices,r.clusters[1].indices);
}
TEST(SteelWallIntegration, MeasuredContinuationPreservesCarAndCannotReattachRemovedWallPoints) {
  Fixture f;f.cfg.steel_fragment_merge=true;f.cfg.detached_wall_cleanup=true;
  f.cam.source_active=true;f.cam.source_current=[](){return true;};
  f.Surface(10);pcl::PointIndices car;for(int i=0;i<int(f.cloud->size());++i)car.indices.push_back(i);f.clusters.push_back(car);
  auto steel=SteelFixture();CloudPtr measured(new Cloud);
  for(const auto& original:steel.clusters) {
    pcl::PointIndices leg;
    for(int i:original.indices){auto p=steel.cloud->at(i);p.x+=10;p.y+=5;leg.indices.push_back(f.cloud->size());f.cloud->push_back(p);measured->push_back(p);}
    f.clusters.push_back(leg);
  }
  pcl::PointIndices wall;
  for(int i=0;i<11;++i){pcl::PointXYZI p{};p.x=20;p.y=3;p.z=.2f*i;wall.indices.push_back(f.cloud->size());f.cloud->push_back(p);}
  f.clusters.push_back(wall);TunnelWallEvidence w;w.valid=w.allowed=true;w.plane={0,-1,0,3};w.intervals={{5,25}};
  auto r=f.Run({w});ASSERT_TRUE(r.report.applied);ASSERT_EQ(11,r.report.removed_wall_points);
  SteelFragmentHistory old;old.points=measured;old.support={800000000,1,7,"measured","steel_barricade"};
  auto continued=ContinueMeasuredSteelFragments(f.cloud,r.clusters,{old},f.context,&f.cam,{w},f.cfg,&r.support);
  ASSERT_TRUE(continued.report.applied);
  std::set<int> unique;int cars=0,barriers=0;
  for(std::size_t i=0;i<continued.clusters.size();++i) {
    if(continued.parent_clusters[i]>=0)continued.support[i]=r.support.at(continued.parent_clusters[i]);
    cars+=continued.support[i].source_class=="car";barriers+=continued.support[i].source_class=="steel_barricade";
    for(int p:continued.clusters[i].indices){EXPECT_TRUE(unique.insert(p).second);EXPECT_EQ(r.removed_wall_indices.end(),std::find(r.removed_wall_indices.begin(),r.removed_wall_indices.end(),p));}
  }
  EXPECT_EQ(1,cars);EXPECT_EQ(1,barriers);EXPECT_EQ(car.indices.size()+12,unique.size());
}
TEST(SteelFragmentMerge, ExtendedMeasuredMotionPreservesAllExistingBoundsAndDefaultWindow) {
  for(int mode=0;mode<4;++mode) {
    TunnelMotionHistory history;
    for(std::int64_t t=1000000000;t<=1240000000;t+=20000000) {
      history.AddYaw(t,mode==2?(t-1000000000)*1e-9:0,true);
      history.AddSpeed(t,mode==1?5:1,mode!=3);
    }
    const auto mount=Pose::Identity();
    EXPECT_FALSE(history.CameraFromScan(1000000000,1200000000,mount));
    const auto motion=history.CameraFromScan(1000000000,1200000000,mount,250000000);
    if(mode==0){ASSERT_TRUE(motion);EXPECT_NEAR(.2,motion->translation().x(),1e-6);}
    else EXPECT_FALSE(motion);
    EXPECT_FALSE(history.CameraFromScan(1000000000,1200000000,mount,1000000000));
  }
}
TEST(SteelFragmentMerge, ContinuationUsesOnlyCurrentMeasuredPointsAndKeepsOriginalImageExpiry) {
  auto f=SteelFixture();f.cam.source_active=true;f.cam.source_current=[](){return true;};
  SteelFragmentHistory old;old.points.reset(new Cloud(*f.cloud));old.support={800000000,1,0,"verified","steel_barricade"};
  auto run=[&](){return ContinueMeasuredSteelFragments(f.cloud,f.clusters,{old},f.context,&f.cam,{},f.cfg);};
  auto r=run();ASSERT_TRUE(r.report.applied);ASSERT_EQ(1u,r.clusters.size());EXPECT_EQ(12u,r.clusters[0].indices.size());
  EXPECT_EQ(800000000,r.support[0].stamp_ns);EXPECT_TRUE(r.removed_wall_indices.empty());
  old.support.stamp_ns=251000000;EXPECT_TRUE(run().report.applied);
  old.support.stamp_ns=249999999;EXPECT_FALSE(run().report.applied);
  old.support.stamp_ns=800000000;f.cam.camera_epoch=2;EXPECT_FALSE(run().report.applied);
  f.cam.camera_epoch=1;old.support.source_class="car";EXPECT_FALSE(run().report.applied);
}
TEST(SteelFragmentMerge, ContinuationCannotInventMissingLegOrResolveAmbiguousParents) {
  auto f=SteelFixture();f.cam.source_active=true;f.cam.source_current=[](){return true;};
  SteelFragmentHistory old;old.points.reset(new Cloud(*f.cloud));old.support={800000000,1,0,"verified","steel_barricade"};
  auto run=[&](){return ContinueMeasuredSteelFragments(f.cloud,f.clusters,{old},f.context,&f.cam,{},f.cfg);};
  const auto second=f.clusters.back();f.clusters.pop_back();EXPECT_FALSE(run().report.applied);
  f.clusters.push_back(second);f.clusters.push_back(second);EXPECT_FALSE(run().report.applied);
  f.clusters.pop_back();int checks=0;f.cam.source_current=[&](){return ++checks==1;};
  EXPECT_FALSE(run().report.applied);EXPECT_EQ(12u,f.cloud->size());
}
TEST(SteelFragmentMerge, ContinuationCannotOverrideCurrentInstanceOrOtherClassEvidence) {
  auto f=SteelFixture();f.cam.source_active=true;f.cam.source_current=[](){return true;};
  SteelFragmentHistory old;old.points.reset(new Cloud(*f.cloud));old.support={800000000,1,0,"verified","steel_barricade"};
  std::vector<CameraPointSupport> current(2);current[0]={1000000000,1,1,"new","person"};
  EXPECT_FALSE(ContinueMeasuredSteelFragments(f.cloud,f.clusters,{old},f.context,&f.cam,{},f.cfg,&current).report.applied);
  auto person=f.cam.boxes[0];person.source_class_name="person";person.confidence=.95;f.cam.boxes.push_back(person);
  EXPECT_FALSE(ContinueMeasuredSteelFragments(f.cloud,f.clusters,{old},f.context,&f.cam,{},f.cfg).report.applied);
}
TEST(SteelFragmentMerge, WeakSteelCannotEnableGeneralPointRefinementAfterPairAbstention) {
  Fixture f;f.cfg.enabled=true;f.cfg.steel_fragment_merge=true;
  f.cam.boxes[0].source_class_name="steel_barricade";f.cam.boxes[0].confidence=.6f;
  f.Surface(10);f.OneCluster();auto r=f.Run();EXPECT_FALSE(r.report.applied);
  ASSERT_EQ(1u,r.report.decisions.size());EXPECT_EQ("steel_insufficient_parents",r.report.decisions[0].reason);
}
TEST(SteelFragmentMerge, DoesNotUsePartialBackgroundParentOrDiscardItsPoints) {
  auto f=SteelFixture();pcl::PointIndices background;
  for(float y=-3;y<=3;y+=.2f){pcl::PointXYZI p{};p.x=12;p.y=y;p.z=.4;background.indices.push_back(f.cloud->size());f.cloud->push_back(p);}
  f.clusters.push_back(background);auto r=f.Run();ASSERT_TRUE(r.report.applied);ASSERT_EQ(2u,r.clusters.size());
  EXPECT_EQ(12u,r.clusters[0].indices.size());EXPECT_EQ(background.indices,r.clusters[1].indices);
  EXPECT_TRUE(r.removed_wall_indices.empty());
}
TEST(SteelFragmentMerge, DoesNotJoinAThirdPlausibleSupportOrOtherClass) {
  auto f=SteelFixture();auto c=f.clusters.front();
  for(auto& i:c.indices){auto p=f.cloud->at(i);p.y=0;i=f.cloud->size();f.cloud->push_back(p);}f.clusters.push_back(c);
  EXPECT_FALSE(f.Run().report.applied);
  f=SteelFixture();auto person=f.cam.boxes[0];person.source_class_name="person";f.cam.boxes.push_back(person);
  auto r=f.Run();EXPECT_FALSE(r.report.applied);EXPECT_EQ("steel_competing_other_class",r.report.decisions[0].reason);
}
TEST(SteelFragmentMerge, SharingOneLegAcrossDifferentInstancesAbstains) {
  auto f=SteelFixture();pcl::PointIndices c;
  for(int i:f.clusters[1].indices){auto p=f.cloud->at(i);p.y=3.45f;c.indices.push_back(f.cloud->size());f.cloud->push_back(p);}f.clusters.push_back(c);
  auto second=f.cam.boxes[0];second.xmin=64;second.xmax=90;f.cam.boxes.push_back(second);
  auto r=f.Run();EXPECT_FALSE(r.report.applied);EXPECT_EQ(12u+6u,f.cloud->size());
}
TEST(CameraPointRefinement, LeveledCoordinatesProduceSamePointMembership) {
  Fixture f;f.Surface(10);f.Surface(20);f.OneCluster();const auto expected=f.Run();
  f.context.frame=Frame::LeveledSensor;f.context.leveling_source="selected_imu";
  f.context.applied_leveling=rpy(.08,.04,0);
  for(auto& p:f.cloud->points) {
    const Vec q=f.context.applied_leveling*Vec(p.x,p.y,p.z);p.x=q.x();p.y=q.y();p.z=q.z();
  }
  const auto r=f.Run();ASSERT_TRUE(r.report.applied);ASSERT_EQ(expected.clusters.size(),r.clusters.size());
  for(std::size_t i=0;i<r.clusters.size();++i) {
    auto a=r.clusters[i].indices,b=expected.clusters[i].indices;std::sort(a.begin(),a.end());std::sort(b.begin(),b.end());EXPECT_EQ(a,b);
  }
}
TEST(CameraPointRefinement, RebuiltObbUsesOnlyExtractedSupport) {
  Fixture f;f.Surface(10);f.Surface(20);f.OneCluster();const auto r=f.Run();ASSERT_TRUE(r.report.applied);
  CloudPtr centers(new Cloud);
  for(const auto& c:r.clusters) {
    pcl::PointXYZI p{};for(int i:c.indices){const auto& q=f.cloud->at(i);p.x+=q.x;p.y+=q.y;p.z+=q.z;}
    p.x/=c.indices.size();p.y/=c.indices.size();p.z/=c.indices.size();centers->push_back(p);
  }
  std::vector<TrackedObject> objects;BuildDetections(objects,f.cloud,centers,r.clusters);
  ASSERT_EQ(2u,objects.size());EXPECT_FLOAT_EQ(10,objects[0].detection.f32_CoreX);
  EXPECT_LT(objects[0].detection.obb_length_m,2);EXPECT_FLOAT_EQ(10,objects[0].detection.f32_MaxX);
}
TEST(CameraPointRefinement, RetainsSparseRoofFragmentsOfTheSameLidarParent) {
  Fixture f;f.Surface(10,-.8,.8,-.8,0);f.Surface(10.3,-.4,.4,.8,1.0);const auto n=f.cloud->size();
  f.Surface(20);f.OneCluster();const auto r=f.Run();ASSERT_TRUE(r.report.applied);
  EXPECT_EQ(n,r.clusters[0].indices.size());
  bool roof=false;for(int i:r.clusters[0].indices)if(f.cloud->at(i).z>.7)roof=true;EXPECT_TRUE(roof);
}
TEST(CameraPointRefinement, TruncatedDetectorBoxCannotShrinkAnObjectLeavingTheImage) {
  Fixture f;f.Surface(10);f.OneCluster();f.cam.boxes[0].xmax=200;
  const auto r=f.Run();EXPECT_FALSE(r.report.applied);ASSERT_EQ(1u,r.report.decisions.size());
  EXPECT_EQ("truncated_image_box",r.report.decisions[0].reason);
}
TEST(CameraPointRefinement, DisabledMissingEmptyAndOverBudgetCamerasAreNoops) {
  Fixture f;f.Surface(10);f.OneCluster();f.cfg.enabled=false;EXPECT_FALSE(f.Run().report.applied);
  f.cfg.enabled=true;f.cam.available=false;EXPECT_FALSE(f.Run().report.applied);
  f.cam.available=true;f.cam.boxes.clear();EXPECT_FALSE(f.Run().report.applied);
  f.cam.boxes.resize(33);EXPECT_FALSE(f.Run().report.applied);
}
TEST(CameraPointRefinement, SameScanConsolidationCannotReattachResidualOrDifferentInstance) {
  TrackingConfig cfg;std::vector<TrackedObject> objects(2);
  for(auto& o:objects) {
    o.detection.f32_MinX=0;o.detection.f32_MaxX=2;o.detection.f32_MinY=0;o.detection.f32_MaxY=2;
    o.detection.f32_OBB_CenterX=1;o.detection.f32_OBB_CenterY=1;
  }
  auto& c=objects[0].detection.camera_point_support;c.box_index=0;c.identity="frame";
  ConsolidateCurrentDetections(objects,cfg);EXPECT_EQ(2u,objects.size());
  objects[1].detection.camera_point_support=c;objects[1].detection.camera_point_support.box_index=1;
  ConsolidateCurrentDetections(objects,cfg);EXPECT_EQ(2u,objects.size());
}

namespace {
struct RecordedFailure {
  CloudPtr cloud{new Cloud};Context context;TunnelCameraInput camera;
  std::vector<pcl::PointIndices> clusters;std::vector<TunnelWallEvidence> walls;
  RecordedFailure() {
    const auto path=std::filesystem::path(__FILE__).parent_path()/"data/wall_car_28_07.json";
    const auto n=YAML::LoadFile(path.string());context.frame=Frame::LeveledSensor;
    context.time.stamp_ns=n["stamp_ns"].as<std::int64_t>();context.calibration_source="recorded_mount";context.leveling_source="recorded_imu";
    auto pose=[](const YAML::Node& a){Pose p=Pose::Identity();for(int r=0;r<4;++r)for(int c=0;c<4;++c)p.matrix()(r,c)=a[4*r+c].as<double>();return p;};
    context.base_from_sensor=pose(n["base_from_sensor"]);
    for(int r=0;r<3;++r)for(int c=0;c<3;++c)context.applied_leveling(r,c)=n["leveling"][3*r+c].as<double>();
    camera.available=camera.calibrated=true;camera.identity="recorded_28_07";camera.camera_epoch=1;
    camera.stamp_ns=n["image_stamp_ns"].as<std::int64_t>();camera.motion_compensated=true;
    camera.image_width=camera.calibration.width=1280;camera.image_height=camera.calibration.height=720;
    camera.calibration.base_from_camera=pose(n["base_from_camera"]);camera.image_sensor_from_scan_sensor=pose(n["image_sensor_from_scan_sensor"]);
    for(int r=0;r<3;++r)for(int c=0;c<3;++c)camera.calibration.intrinsic(r,c)=n["intrinsic"][3*r+c].as<double>();
    camera.still_current=[](){return true;};
    for(const auto& p:n["points"]){pcl::PointXYZI q{};q.x=p[0].as<float>();q.y=p[1].as<float>();q.z=p[2].as<float>();cloud->push_back(q);}
    for(const auto& c:n["clusters"]){pcl::PointIndices indices;for(const auto& i:c)indices.indices.push_back(i.as<int>());clusters.push_back(std::move(indices));}
    for(const auto& w:n["walls"]){TunnelWallEvidence v;v.side=w["side"].as<int>();v.allowed=w["allowed"].as<bool>();v.valid=w["valid"].as<bool>();for(int i=0;i<4;++i)v.plane[i]=w["plane"][i].as<float>();for(const auto& interval:w["intervals"])v.intervals.emplace_back(interval[0].as<float>(),interval[1].as<float>());walls.push_back(v);}
    for(const auto& b:n["boxes"]){camera::Box box;box.xmin=b["xmin"].as<float>();box.xmax=b["xmax"].as<float>();box.ymin=b["ymin"].as<float>();box.ymax=b["ymax"].as<float>();box.class_name=b["class_name"].as<std::string>();box.source_class_name=b["source_class_name"].as<std::string>();box.confidence=b["confidence"].as<float>();box.confidence_valid=true;camera.boxes.push_back(box);}
  }
  CameraPointRefinementResult Run(bool grouping,bool cleanup) {
    CameraPointRefinementConfig c;c.enabled=true;c.vehicle_scanline_grouping=grouping;c.independent_wall_cleanup=cleanup;
    return RefineCameraPoints(cloud,clusters,context,&camera,walls,c);
  }
};
}
TEST(CameraPointRefinementV3, ExactRecordedFailureReproducesOriginalRejection) {
  RecordedFailure f;const auto r=f.Run(false,false);EXPECT_FALSE(r.report.applied);
  ASSERT_EQ(1u,r.report.decisions.size());const auto& d=r.report.decisions[0];
  EXPECT_EQ(46,d.input_points);EXPECT_EQ(2,d.components);EXPECT_EQ("no_supported_surface",d.reason);
}
TEST(CameraPointRefinementV3, ExactRecordedScanRibbonsFormVehicleBeforeValidation) {
  RecordedFailure f;const auto r=f.Run(true,true);ASSERT_TRUE(r.report.applied);
  EXPECT_EQ(1,r.report.extracted_objects);EXPECT_GT(r.report.removed_wall_points,500);
  ASSERT_EQ(1u,r.report.decisions.size());EXPECT_EQ(1,r.report.decisions[0].scanline_groups);
  EXPECT_EQ(46,r.report.decisions[0].object_points);ASSERT_EQ("car",r.support[0].source_class);
  float minx=INFINITY,maxx=-INFINITY,miny=INFINITY,maxy=-INFINITY;
  for(int i:r.clusters[0].indices){const auto& p=f.cloud->at(i);minx=std::min(minx,p.x);maxx=std::max(maxx,p.x);miny=std::min(miny,p.y);maxy=std::max(maxy,p.y);}
  EXPECT_LT(std::hypot(maxx-minx,maxy-miny),6);
}
TEST(CameraPointRefinementV3, FailedObjectValidationStillRemovesConfirmedWallOutsideBox) {
  RecordedFailure f;const auto r=f.Run(false,true);ASSERT_TRUE(r.report.applied);
  EXPECT_EQ(0,r.report.extracted_objects);EXPECT_GT(r.report.removed_wall_points,500);
  EXPECT_EQ("confirmed_wall_cleanup_only",r.report.reason);
  Context sensor=f.context;sensor.frame=Frame::Sensor;
  const auto projection=camera_projection(sensor,f.camera.calibration.base_from_camera,f.camera.calibration.intrinsic);
  Pose raw_from_level=Pose::Identity();raw_from_level.linear()=f.context.applied_leveling.transpose();
  const Projection p=(*projection)*f.camera.image_sensor_from_scan_sensor.matrix()*raw_from_level.matrix();
  const auto& b=f.camera.boxes[0];int retained=0;
  for(std::size_t i=0;i<f.cloud->size();++i) {
    const auto& q=f.cloud->at(i);const Vec v=p*Eigen::Vector4d(q.x,q.y,q.z,1);
    if(v.z()>.1&&v.x()/v.z()>=b.xmin&&v.x()/v.z()<=b.xmax&&v.y()/v.z()>=b.ymin&&v.y()/v.z()<=b.ymax) {
      ++retained;EXPECT_EQ(r.removed_wall_indices.end(),std::find(r.removed_wall_indices.begin(),r.removed_wall_indices.end(),int(i)));
    }
  }
  EXPECT_EQ(46,retained);
}
TEST(CameraPointRefinementV3, IndependentCleanupSourceResetDiscardsWallDeletion) {
  RecordedFailure f;int calls=0;f.camera.still_current=[&](){return ++calls==1;};
  const auto r=f.Run(false,true);EXPECT_FALSE(r.report.applied);EXPECT_TRUE(r.removed_wall_indices.empty());
}
namespace {
Fixture Ribbons(float second_depth=27,float second_y0=-.8f,float second_y1=.8f) {
  Fixture f;auto& b=f.cam.boxes[0];b.xmin=94;b.xmax=106;b.ymin=97;b.ymax=103;
  f.Surface(27,-.8f,.8f,-.5f,-.5f);f.Surface(second_depth,second_y0,second_y1,.5f,.5f);f.OneCluster();return f;
}
}
TEST(CameraPointRefinementV3, SparseRibbonsJoinOnlyWithinOneVehicleInstanceAndDepth) {
  auto f=Ribbons();auto r=f.Run();ASSERT_TRUE(r.report.applied);
  EXPECT_EQ(1,r.report.decisions[0].scanline_groups);EXPECT_EQ(18,r.report.decisions[0].object_points);
  f=Ribbons(29);r=f.Run();EXPECT_FALSE(r.report.applied);EXPECT_EQ(0,r.report.decisions[0].scanline_groups);
  f=Ribbons();pcl::PointIndices a,b;for(int i=0;i<18;++i)(i<9?a:b).indices.push_back(i);f.clusters={a,b};
  r=f.Run();EXPECT_FALSE(r.report.applied);EXPECT_EQ(0,r.report.decisions[0].scanline_groups);
  f=Ribbons();f.cam.boxes[0].source_class_name="person";r=f.Run();EXPECT_FALSE(r.report.applied);
  EXPECT_EQ(0,r.report.decisions[0].scanline_groups);
}
TEST(CameraPointRefinementV3, ScanlineGraphCannotGrowThroughTransitiveBridges) {
  Fixture f;auto& b=f.cam.boxes[0];b.xmin=94;b.xmax=106;b.ymin=95;b.ymax=105;
  for(float z:{-1.f,0.f,1.f})f.Surface(27,-.8,.8,z,z);f.OneCluster();
  const auto r=f.Run();EXPECT_FALSE(r.report.applied);EXPECT_EQ(0,r.report.decisions[0].scanline_groups);
}
TEST(CameraPointRefinementV3, NonoverlappingRibbonsCannotBecomeOneVehicle) {
  Fixture f;auto& b=f.cam.boxes[0];b.xmin=90;b.xmax=110;b.ymin=97;b.ymax=103;
  f.Surface(27,-2,-1,-.5,-.5);f.Surface(27,1,2,.5,.5);f.OneCluster();
  const auto r=f.Run();EXPECT_FALSE(r.report.applied);EXPECT_EQ(0,r.report.decisions[0].scanline_groups);
}
TEST(CameraPointRefinementV3, CleanupRequiresConfirmedMeasuredWallAndCurrentCamera) {
  RecordedFailure f;f.walls.clear();auto r=f.Run(false,true);EXPECT_FALSE(r.report.applied);
  f=RecordedFailure();for(auto& w:f.walls)w.allowed=false;r=f.Run(false,true);EXPECT_FALSE(r.report.applied);
  f=RecordedFailure();for(auto& w:f.walls)w.intervals.clear();r=f.Run(false,true);EXPECT_FALSE(r.report.applied);
  f=RecordedFailure();f.camera.available=false;r=f.Run(false,true);EXPECT_FALSE(r.report.applied);
  f=RecordedFailure();f.camera.boxes.clear();r=f.Run(false,true);EXPECT_FALSE(r.report.applied);
}
TEST(CameraPointRefinementV4, DetachedConfirmedTallWallNeedsForegroundAndPreservesUnresolvedInstances) {
  Fixture f;f.cfg.detached_wall_cleanup=true;f.Surface(10);const int first=f.cloud->size();
  f.Surface(20,3,3,0,2);pcl::PointIndices a,b;
  for(int i=0;i<int(f.cloud->size());++i)(i<first?a:b).indices.push_back(i);f.clusters={a,b};
  TunnelWallEvidence w;w.valid=w.allowed=true;w.plane={0,-1,0,3};w.intervals={{5,25}};
  auto r=f.Run({w});ASSERT_TRUE(r.report.applied);EXPECT_EQ(11,r.report.removed_wall_points);
  EXPECT_EQ(first,r.clusters[0].indices.size());
  auto person=f.cam.boxes[0];person.xmin=84;person.xmax=86;person.ymin=88;person.ymax=102;person.source_class_name="person";
  f.cam.boxes.push_back(person);r=f.Run({w});EXPECT_EQ(0,r.report.removed_wall_points);
  f.cam.boxes.resize(1);f.cfg.detached_wall_cleanup=false;r=f.Run({w});EXPECT_EQ(0,r.report.removed_wall_points);
}
TEST(CameraPointRefinementV4, DetachedCleanupCannotEraseNearOrUnconfirmedWall) {
  Fixture f;f.cfg.detached_wall_cleanup=true;f.Surface(10);int n=f.cloud->size();f.Surface(10.5,3,3,0,2);
  pcl::PointIndices a,b;for(int i=0;i<int(f.cloud->size());++i)(i<n?a:b).indices.push_back(i);f.clusters={a,b};
  TunnelWallEvidence w;w.valid=w.allowed=true;w.plane={0,-1,0,3};w.intervals={{5,25}};
  auto r=f.Run({w});EXPECT_EQ(0,r.report.removed_wall_points);
  w.allowed=false;r=f.Run({w});EXPECT_EQ(0,r.report.removed_wall_points);
}

TEST(CameraPointRefinementMultiView, LeftAndRightUseTheirOwnExtrinsics) {
  for(int sign:{-1,1}) {
    Fixture f;f.Surface(10);f.OneCluster();
    for(auto& p:*f.cloud){const float x=p.x;p.x=-sign*p.y;p.y=sign*x;}
    auto front=std::make_shared<TunnelCameraInput>(f.cam);front->camera_id="camera_front";
    auto side=std::make_shared<TunnelCameraInput>(f.cam);side->camera_id=sign>0?"camera_left":"camera_right";
    side->calibration.base_from_camera.linear()=rpy(0,0,sign*1.5707963267948966);
    f.cam.refinement_views={front,side};
    const auto r=f.Run();ASSERT_TRUE(r.report.applied);ASSERT_EQ(1,r.report.extracted_objects);
    ASSERT_EQ(2u,r.report.views.size());EXPECT_EQ(0,r.report.views[0].extracted_objects);
    EXPECT_EQ(1,r.report.views[1].extracted_objects);EXPECT_EQ(f.cloud->size(),r.clusters.front().indices.size());
    ASSERT_FALSE(r.report.decisions.empty());EXPECT_EQ(side->camera_id,r.report.decisions.back().camera_id);
  }
}
TEST(CameraPointRefinementMultiView, OverlapIsResolvedOnceAndResidualsArePreserved) {
  Fixture f;f.Surface(10);f.Surface(20);f.OneCluster();
  auto front=std::make_shared<TunnelCameraInput>(f.cam);front->camera_id="camera_front";
  auto left=std::make_shared<TunnelCameraInput>(f.cam);left->camera_id="camera_left";left->boxes[0].confidence=.99f;
  f.cam.refinement_views={front,left};const auto r=f.Run();ASSERT_TRUE(r.report.applied);
  EXPECT_EQ(1,r.report.extracted_objects);EXPECT_EQ(0,r.report.views[0].extracted_objects);EXPECT_EQ(1,r.report.views[1].extracted_objects);
  std::set<int> all;for(const auto& c:r.clusters)for(int i:c.indices)EXPECT_TRUE(all.insert(i).second);
  EXPECT_EQ(f.cloud->size(),all.size());EXPECT_EQ("cross_view_overlap_conflict",r.report.decisions[0].reason);
}
TEST(CameraPointRefinementMultiView, UnresolvedOtherViewVetoesWallDeletion) {
  Fixture f;f.cfg.detached_wall_cleanup=true;f.Surface(10);const int first=f.cloud->size();
  f.Surface(20,3,3,0,2);pcl::PointIndices a,b;
  for(int i=0;i<int(f.cloud->size());++i)(i<first?a:b).indices.push_back(i);f.clusters={a,b};
  auto front=std::make_shared<TunnelCameraInput>(f.cam);front->camera_id="camera_front";
  auto left=std::make_shared<TunnelCameraInput>(f.cam);left->camera_id="camera_left";
  left->boxes[0].xmin=84;left->boxes[0].xmax=86;left->boxes[0].ymin=88;left->boxes[0].ymax=102;
  left->boxes[0].source_class_name="person";
  f.cam.refinement_views={front,left};
  TunnelWallEvidence w;w.valid=w.allowed=true;w.plane={0,-1,0,3};w.intervals={{5,25}};
  const auto r=f.Run({w});ASSERT_TRUE(r.report.applied);EXPECT_EQ(1,r.report.extracted_objects);
  EXPECT_EQ(0,r.report.removed_wall_points);
}
TEST(CameraPointRefinementMultiView, StaleSideAndLowConfidenceAreReportedWithoutInventingObjects) {
  Fixture f;f.Surface(10);f.OneCluster();
  auto front=std::make_shared<TunnelCameraInput>(f.cam);front->camera_id="camera_front";front->boxes[0].confidence=.65f;
  auto left=std::make_shared<TunnelCameraInput>(f.cam);left->camera_id="camera_left";
  left->stamp_ns-=200000000;left->motion_compensated=true;
  f.cam.refinement_views={front,left};const auto r=f.Run();EXPECT_FALSE(r.report.applied);
  ASSERT_EQ(1u,r.report.decisions.size());EXPECT_EQ("below_confidence",r.report.decisions[0].reason);
  EXPECT_EQ("no_current_synchronized_camera",r.report.views[1].reason);
}
TEST(CameraPointRefinementMultiView, ResetOfAnyParticipatingViewAbortsCombinedPlan) {
  Fixture f;f.Surface(10);f.OneCluster();
  auto front=std::make_shared<TunnelCameraInput>(f.cam);front->camera_id="camera_front";
  auto left=std::make_shared<TunnelCameraInput>(f.cam);left->camera_id="camera_left";int checks=0;
  left->still_current=[&](){return ++checks<=2;};
  f.cam.refinement_views={front,left};const auto r=f.Run();EXPECT_FALSE(r.report.applied);
  EXPECT_EQ("camera_changed_during_refinement",r.report.reason);EXPECT_TRUE(r.clusters.empty());
  EXPECT_TRUE(r.removed_wall_indices.empty());EXPECT_EQ(0,r.report.extracted_objects);
}

TEST(MeasuredVehicleContinuation, SplitsCurrentVehicleFromMergedBackgroundWithoutReusingOldPoints) {
  Fixture f;f.cfg.vehicle_history_enabled=true;f.Surface(10);const int vehicle=f.cloud->size();
  MeasuredVehicleHistory h;h.track_id=42;h.confirmations=5;h.points.reset(new Cloud(*f.cloud));
  f.Surface(20);f.OneCluster();std::vector<int> ids;
  const auto r=ContinueMeasuredVehicles(f.cloud,f.clusters,{h},{},f.cfg,nullptr,nullptr,&ids);
  ASSERT_TRUE(r.report.applied);ASSERT_EQ(1,r.report.extracted_objects);ASSERT_EQ(42,ids.front());
  EXPECT_EQ(vehicle,r.clusters.front().indices.size());EXPECT_EQ(-1,r.support.front().box_index);
  std::set<int> all;for(const auto& c:r.clusters)for(int i:c.indices)EXPECT_TRUE(all.insert(i).second);
  EXPECT_EQ(f.cloud->size(),all.size());EXPECT_TRUE(r.removed_wall_indices.empty());
}
TEST(MeasuredVehicleContinuation, NeverKeepsAnObjectWithNoCurrentMeasurementsOrUnconfirmedHistory) {
  Fixture f;f.cfg.vehicle_history_enabled=true;f.Surface(10);f.OneCluster();
  MeasuredVehicleHistory h;h.track_id=42;h.confirmations=4;h.points.reset(new Cloud(*f.cloud));std::vector<int> ids;
  EXPECT_FALSE(ContinueMeasuredVehicles(f.cloud,f.clusters,{h},{},f.cfg,nullptr,nullptr,&ids).report.applied);
  h.confirmations=5;for(auto& p:*h.points)p.x+=3;
  EXPECT_FALSE(ContinueMeasuredVehicles(f.cloud,f.clusters,{h},{},f.cfg,nullptr,nullptr,&ids).report.applied);
}
TEST(MeasuredVehicleContinuation, ConflictingTracksAndFreshCameraOwnershipAbstain) {
  Fixture f;f.cfg.vehicle_history_enabled=true;f.Surface(10);f.OneCluster();
  MeasuredVehicleHistory h;h.track_id=42;h.confirmations=5;h.points.reset(new Cloud(*f.cloud));auto other=h;other.track_id=43;
  std::vector<int> ids;auto r=ContinueMeasuredVehicles(f.cloud,f.clusters,{h,other},{},f.cfg,nullptr,nullptr,&ids);
  EXPECT_FALSE(r.report.applied);EXPECT_EQ("vehicle_history_competing_tracks",r.report.decisions[0].reason);
  std::vector<CameraPointSupport> support(1);support[0].box_index=0;support[0].source_class="person";
  r=ContinueMeasuredVehicles(f.cloud,f.clusters,{h},{},f.cfg,&support,nullptr,&ids);
  EXPECT_FALSE(r.report.applied);EXPECT_EQ("vehicle_history_current_camera_ownership",r.report.decisions[0].reason);
}
TEST(MeasuredVehicleContinuation, FreshVehiclePointsReserveOriginalTrackWithoutChangingCameraSupport) {
  Fixture f;f.cfg.vehicle_history_enabled=true;f.Surface(10);f.OneCluster();
  MeasuredVehicleHistory h;h.track_id=42;h.confirmations=5;h.points.reset(new Cloud(*f.cloud));
  std::vector<CameraPointSupport> support(1);support[0].box_index=0;support[0].source_class="car";
  std::vector<int> ids;const auto r=ContinueMeasuredVehicles(f.cloud,f.clusters,{h},{},f.cfg,&support,nullptr,&ids);
  ASSERT_TRUE(r.report.applied);EXPECT_EQ(0,r.report.extracted_objects);ASSERT_EQ(1u,r.clusters.size());
  EXPECT_EQ(f.clusters[0].indices,r.clusters[0].indices);ASSERT_EQ(1u,ids.size());EXPECT_EQ(42,ids[0]);
  EXPECT_EQ(0,r.parent_clusters[0]);EXPECT_EQ("vehicle_history_current_camera_confirmed",r.report.decisions[0].reason);
}
TEST(MeasuredVehicleContinuation, VerifiedTurningMotionDoesNotRelaxCameraProjectionPolicy) {
  for(bool trusted:{true,false}) {
    TunnelMotionHistory h;
    for(int i=0;i<=5;++i) {
      const auto t=1000000000LL+i*20000000LL;h.AddYaw(t,.02*i,trusted);h.AddSpeed(t,3,true);
    }
    const auto mount=Pose::Identity();
    EXPECT_FALSE(h.CameraFromScan(1000000000LL,1100000000LL,mount,200000000LL));
    EXPECT_EQ(trusted,bool(h.CameraFromScan(1000000000LL,1100000000LL,mount,200000000LL,.2)));
    EXPECT_FALSE(h.CameraFromScan(1000000000LL,1100000000LL,mount,200000000LL,.3));
  }
}
TEST(MeasuredVehicleContinuation, MeasuredWallAndUnresolvedCameraRegionsStayDistinct) {
  Fixture f;f.cfg.vehicle_history_enabled=true;f.Surface(10);const int vehicle=f.cloud->size();
  MeasuredVehicleHistory h;h.track_id=42;h.confirmations=5;h.points.reset(new Cloud(*f.cloud));
  f.Surface(20,3,3,0,2);f.OneCluster();TunnelWallEvidence w;w.valid=w.allowed=true;w.plane={0,-1,0,3};w.intervals={{5,25}};
  std::vector<int> ids;auto r=ContinueMeasuredVehicles(f.cloud,f.clusters,{h},{w},f.cfg,nullptr,nullptr,&ids);
  ASSERT_TRUE(r.report.applied);EXPECT_EQ(f.cloud->size()-vehicle,r.removed_wall_indices.size());
  std::vector<int> veto;for(int i=vehicle;i<int(f.cloud->size());++i)veto.push_back(i);
  r=ContinueMeasuredVehicles(f.cloud,f.clusters,{h},{w},f.cfg,nullptr,&veto,&ids);
  EXPECT_TRUE(r.report.applied);EXPECT_TRUE(r.removed_wall_indices.empty());
}
