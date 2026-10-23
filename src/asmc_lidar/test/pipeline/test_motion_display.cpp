#include <asmc_lidar/tracking/motion_display.hpp>
#include <gtest/gtest.h>
using namespace asmc::lidar::pipeline::motion_display;

static void Feed(MotionClassifier& classifier,std::vector<MotionObject>& objects,
                 int frame,double vx_kph,double vy_kph,double x,double y) {
  auto& o=objects[0];o.s32_Id=7;o.s32_MissCount=0;
  o.f32_AbsVx=vx_kph/3.6;o.f32_AbsVy=vy_kph/3.6;
  o.f32_CoreE=x;o.f32_CoreN=y;o.min_e=x-2;o.max_e=x+2;o.min_n=y-1;o.max_n=y+1;
  o.bounds_valid=true;o.map_vx=vx_kph/3.6;o.map_vy=vy_kph/3.6;
  classifier.Update(objects,1.+frame*.1);
}
TEST(MotionDisplay,WarmupAndStationaryLongitudinalNoise) {
  MotionClassifier c;std::vector<MotionObject> v(1);
  for(int i=0;i<3;++i){Feed(c,v,i,7,0,0,0);EXPECT_EQ(v[0].s32_MotionState,0);EXPECT_FALSE(v[0].motion_valid);}
  for(int i=3;i<20;++i)Feed(c,v,i,7,0,0,0);
  EXPECT_EQ(v[0].s32_MotionState,1);EXPECT_TRUE(v[0].motion_valid);EXPECT_FALSE(v[0].b_IsDynamic);
}
TEST(MotionDisplay,LateralAndLongitudinalConfirmedMotion) {
  for(bool lateral:{false,true}) {
    MotionClassifier c;std::vector<MotionObject> v(1);
    const double vx=lateral?0:11,vy=lateral?5:0;
    for(int i=0;i<25;++i)Feed(c,v,i,vx,vy,i*.1*vx/3.6,i*.1*vy/3.6);
    EXPECT_EQ(v[0].s32_MotionState,3);EXPECT_TRUE(v[0].b_IsDynamic);EXPECT_TRUE(v[0].motion_valid);
  }
}
TEST(MotionDisplay,LongitudinalVelocityWithoutDisplacementCannotConfirmDynamic) {
  MotionClassifier c;std::vector<MotionObject> v(1);
  for(int i=0;i<30;++i)Feed(c,v,i,12,0,0,0);
  EXPECT_NE(v[0].s32_MotionState,3);
}
TEST(MotionDisplay,MapLossResetsHistoryAndRecoveryWarmsUpAgain) {
  MotionClassifier c;std::vector<MotionObject> v(1);
  for(int i=0;i<20;++i)Feed(c,v,i,0,6,0,i*.1*6/3.6);
  ASSERT_EQ(v[0].s32_MotionState,3);
  c.InvalidateMapHistory(v);EXPECT_EQ(v[0].s32_MotionState,0);EXPECT_FALSE(v[0].motion_valid);
  Feed(c,v,21,0,6,0,4);EXPECT_EQ(v[0].s32_MotionState,0);
}
TEST(MotionDisplay,DynamicNeedsEightStaticConfirmationsToRelease) {
  MotionClassifier c;std::vector<MotionObject> v(1);
  for(int i=0;i<20;++i)Feed(c,v,i,0,6,0,i*.1*6/3.6);
  ASSERT_EQ(v[0].s32_MotionState,3);
  Feed(c,v,20,0,0,0,19*.1*6/3.6);EXPECT_EQ(v[0].s32_MotionState,3);
  for(int i=21;i<55;++i)Feed(c,v,i,0,0,0,19*.1*6/3.6);
  EXPECT_EQ(v[0].s32_MotionState,1);
}
TEST(MotionDisplay,MissedObservationsKeepThenResetDecision) {
  MotionClassifier c;std::vector<MotionObject> v(1);
  for(int i=0;i<20;++i)Feed(c,v,i,0,6,0,i*.1*6/3.6);
  ASSERT_EQ(v[0].s32_MotionState,3);
  for(int i=1;i<=3;++i){v[0].s32_MissCount=i;c.Update(v,3.+i*.1);EXPECT_EQ(v[0].s32_MotionState,3);}
  v[0].s32_MissCount=4;c.Update(v,3.4);EXPECT_EQ(v[0].s32_MotionState,0);
}
int main(int argc,char**argv){::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
