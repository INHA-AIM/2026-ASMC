#include <asmc_lidar/tracking/motion_classifier.hpp>
#include <gtest/gtest.h>
#include <cmath>
using namespace asmc::lidar::pipeline::motion_classifier;

static void Feed(MotionClassifier& classifier,std::vector<MotionObject>& objects,
                 int frame,double vx_kph,double vy_kph,double x,double y) {
  auto& o=objects[0];o.s32_Id=7;o.s32_MissCount=0;
  o.f32_AbsVx=vx_kph/3.6;o.f32_AbsVy=vy_kph/3.6;
  o.f32_CoreE=x;o.f32_CoreN=y;o.min_e=x-2;o.max_e=x+2;o.min_n=y-1;o.max_n=y+1;
  o.bounds_valid=true;o.map_vx=vx_kph/3.6;o.map_vy=vy_kph/3.6;
  classifier.Update(objects,1.+frame*.1);
}
TEST(MotionClassifier,WarmupAndStationaryLongitudinalNoise) {
  MotionClassifier c;std::vector<MotionObject> v(1);
  for(int i=0;i<3;++i){Feed(c,v,i,7,0,0,0);EXPECT_EQ(v[0].s32_MotionState,0);EXPECT_FALSE(v[0].motion_valid);}
  for(int i=3;i<20;++i)Feed(c,v,i,7,0,0,0);
  EXPECT_EQ(v[0].s32_MotionState,1);EXPECT_TRUE(v[0].motion_valid);EXPECT_FALSE(v[0].b_IsDynamic);
}
TEST(MotionClassifier,LateralAndLongitudinalConfirmedMotion) {
  for(bool lateral:{false,true}) {
    MotionClassifier c;std::vector<MotionObject> v(1);
    const double vx=lateral?0:11,vy=lateral?5:0;
    for(int i=0;i<25;++i)Feed(c,v,i,vx,vy,i*.1*vx/3.6,i*.1*vy/3.6);
    EXPECT_EQ(v[0].s32_MotionState,3);EXPECT_TRUE(v[0].b_IsDynamic);EXPECT_TRUE(v[0].motion_valid);
  }
}
TEST(MotionClassifier,LongitudinalVelocityWithoutDisplacementCannotConfirmDynamic) {
  MotionClassifier c;std::vector<MotionObject> v(1);
  for(int i=0;i<30;++i)Feed(c,v,i,12,0,0,0);
  EXPECT_NE(v[0].s32_MotionState,3);
}
TEST(MotionClassifier,MapLossResetsHistoryAndRecoveryWarmsUpAgain) {
  MotionClassifier c;std::vector<MotionObject> v(1);
  for(int i=0;i<20;++i)Feed(c,v,i,0,6,0,i*.1*6/3.6);
  ASSERT_EQ(v[0].s32_MotionState,3);
  c.InvalidateMapHistory(v);EXPECT_EQ(v[0].s32_MotionState,0);EXPECT_FALSE(v[0].motion_valid);
  Feed(c,v,21,0,6,0,4);EXPECT_EQ(v[0].s32_MotionState,0);
}
TEST(MotionClassifier,DynamicNeedsEightStaticConfirmationsToRelease) {
  MotionClassifier c;std::vector<MotionObject> v(1);
  for(int i=0;i<20;++i)Feed(c,v,i,0,6,0,i*.1*6/3.6);
  ASSERT_EQ(v[0].s32_MotionState,3);
  Feed(c,v,20,0,0,0,19*.1*6/3.6);EXPECT_EQ(v[0].s32_MotionState,3);
  for(int i=21;i<55;++i)Feed(c,v,i,0,0,0,19*.1*6/3.6);
  EXPECT_EQ(v[0].s32_MotionState,1);
}
TEST(MotionClassifier,MissedObservationsKeepThenResetDecision) {
  MotionClassifier c;std::vector<MotionObject> v(1);
  for(int i=0;i<20;++i)Feed(c,v,i,0,6,0,i*.1*6/3.6);
  ASSERT_EQ(v[0].s32_MotionState,3);
  for(int i=1;i<=3;++i){v[0].s32_MissCount=i;c.Update(v,3.+i*.1);EXPECT_EQ(v[0].s32_MotionState,3);}
  v[0].s32_MissCount=4;c.Update(v,3.4);EXPECT_EQ(v[0].s32_MotionState,0);
}
TEST(MotionClassifier, OneGrowingFaceDoesNotBecomeRigidMotion) {
  MotionClassifier c; std::vector<MotionObject> v(1);
  for(int i=0;i<30;++i) Feed(c,v,i,0,0,0,0);
  ASSERT_EQ(v[0].s32_MotionState,1);
  for(int i=0;i<15;++i) {
    auto& o=v[0]; o.f32_AbsVx=12; o.max_e=2+i*.4; o.f32_CoreE=i*.2;
    c.Update(v,4+i*.1);
  }
  EXPECT_EQ(v[0].s32_MotionState,1);
}
int main(int argc,char**argv){::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}

TEST(SemanticMotion, StaticNoiseSuppressionNeedsFreshClassAndGeometry) {
  MotionClassifierConfig cfg;cfg.semantic_static_enabled=true;MotionClassifier c(cfg);
  std::vector<MotionObject> v(1);v[0].semantic_static_eligible=true;
  for(int i=0;i<20;++i)Feed(c,v,i,0,0,0,0);
  ASSERT_TRUE(v[0].semantic_static_applied);ASSERT_EQ(v[0].s32_MotionState,1);
  for(int i=20;i<35;++i){Feed(c,v,i,i%2?40:0,0,0,0);EXPECT_EQ(v[0].s32_MotionState,1);}
  v[0].semantic_static_eligible=false;Feed(c,v,35,0,0,0,0);
  EXPECT_FALSE(v[0].semantic_static_applied);
}
TEST(SemanticMotion, MisclassifiedMovingObjectBelowTenKphReleasesOnFirstRigidEvidence) {
  MotionClassifierConfig cfg;cfg.semantic_static_enabled=true;MotionClassifier c(cfg);
  std::vector<MotionObject> v(1);v[0].semantic_static_eligible=true;
  for(int i=0;i<20;++i)Feed(c,v,i,0,0,0,0);
  int first=-1,confirmed=-1;
  for(int i=20;i<40;++i) {
    Feed(c,v,i,6,0,(i-19)*.1*6/3.6,0);
    if(v[0].semantic_reason=="rigid_motion_releases_static_prior") {
      if(first<0){first=i;EXPECT_EQ(v[0].s32_MotionState,2);}
      EXPECT_FALSE(v[0].semantic_static_applied);
    }
    if(v[0].s32_MotionState==3&&confirmed<0)confirmed=i;
  }
  ASSERT_GE(first,0);EXPECT_EQ(confirmed,first+2);
}
TEST(SemanticMotion, OneSidedGrowthCannotCertifySemanticStaticAndMapLossClearsIt) {
  MotionClassifierConfig cfg;cfg.semantic_static_enabled=true;MotionClassifier c(cfg);
  std::vector<MotionObject> v(1);v[0].semantic_static_eligible=true;
  for(int i=0;i<20;++i)Feed(c,v,i,0,0,0,0);
  for(int i=20;i<27;++i) {
    v[0].max_e=2+(i-19)*.3;v[0].f32_AbsVx=5;c.Update(v,1+i*.1);
  }
  EXPECT_FALSE(v[0].semantic_static_applied);EXPECT_EQ(v[0].s32_MotionState,0);
  c.InvalidateMapHistory(v);EXPECT_FALSE(v[0].motion_valid);EXPECT_EQ(v[0].s32_MotionState,0);
}
TEST(SemanticMotion, LabelAloneCannotOverrideAlreadyMovingAndInvalidVelocityIsNotZero) {
  MotionClassifierConfig cfg;cfg.semantic_static_enabled=true;MotionClassifier c(cfg);
  std::vector<MotionObject> v(1);
  for(int i=0;i<20;++i)Feed(c,v,i,12,0,i*.1*12/3.6,0);
  ASSERT_EQ(v[0].s32_MotionState,3);
  v[0].semantic_static_eligible=true;Feed(c,v,20,12,0,20*.1*12/3.6,0);EXPECT_EQ(v[0].s32_MotionState,3);
  Feed(c,v,21,NAN,0,7,0);EXPECT_FALSE(v[0].motion_valid);EXPECT_EQ(v[0].s32_MotionState,0);
}
