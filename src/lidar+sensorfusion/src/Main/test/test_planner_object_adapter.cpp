#include <PlannerInterface/PlannerObjectAdapter.hpp>
#include <Msg/TrackedObject.h>
#include <gtest/gtest.h>
#include <cstdint>
#include <limits>
#include <type_traits>

static_assert(std::is_same<decltype(Msg::TrackedObject{}.id),std::int32_t>::value,
              "ROS track identity must retain the existing int32 contract");

namespace {
LiDARObjectInfo Detection(std::int32_t id) {
  LiDARObjectInfo object{};
  object.s32_Id=id; object.str_ObjectClass="static_obstacle";
  object.f32_CoreX=object.f32_OBB_CenterX=10;
  object.f32_OBB_Width=4;object.f32_OBB_Height=2;
  object.f32_OBB_ZMin=-1;object.f32_OBB_ZMax=.5;
  for(unsigned i=0;i<8;++i) {
    object.OBB_Corners_Body[i]={float(10+((i&1)?2:-2)),float((i&2)?1:-1),float((i&4)?.5:-1)};
    pcl::PointXYZI point;
    point.x=object.OBB_Corners_Body[i].x;point.y=object.OBB_Corners_Body[i].y;
    point.z=object.OBB_Corners_Body[i].z;point.intensity=1;
    object.pcl_ClusterPoints.push_back(point);
  }
  return object;
}
}

TEST(PlannerObjectAdapter,LongRunningTracksKeepFullRosIdentity) {
  GPS_IMU pose{};VehicleSpec vehicle{};
  for(std::int32_t id:{32766,32767,35604,std::numeric_limits<std::int32_t>::max()-1}) {
    LiDAR lidar{};lidar.vec_Objects.push_back(Detection(id));
    const auto out=MakePlannerObjects(lidar,pose,vehicle,20);
    ASSERT_EQ(out.size(),1u)<<id;
    EXPECT_EQ(out.front().id,id);EXPECT_DOUBLE_EQ(out.front().length,4);
    EXPECT_DOUBLE_EQ(out.front().width,2);
  }
}
TEST(PlannerObjectAdapter,LargeIdCannotEraseTheFirstObjectInACapacityLimitedFrame) {
  LiDAR lidar{};lidar.vec_Objects={Detection(35604),Detection(23)};
  const auto out=MakePlannerObjects(lidar,GPS_IMU{},VehicleSpec{},1);
  ASSERT_EQ(out.size(),1u);EXPECT_EQ(out.front().id,35604);
}
TEST(PlannerObjectAdapter,InvalidMissedAndNoiseObservationsStillStayExcluded) {
  LiDAR lidar{};auto negative=Detection(-1),missed=Detection(35604),noise=Detection(35605);
  missed.s32_MissCount=1;noise.str_ObjectClass="noise";
  lidar.vec_Objects={negative,missed,noise,Detection(35606)};
  const auto out=MakePlannerObjects(lidar,GPS_IMU{},VehicleSpec{},20);
  ASSERT_EQ(out.size(),1u);EXPECT_EQ(out.front().id,35606);
}

int main(int argc,char** argv) {
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
