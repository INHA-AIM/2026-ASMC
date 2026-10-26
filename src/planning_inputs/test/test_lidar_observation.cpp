#include <gtest/gtest.h>
#include <planning_inputs/converter.hpp>
#include <planning_inputs/observation_sequence.hpp>
#include <limits>

using namespace planning_inputs;
namespace {
asmc_msgs::LidarTrackingObservation Frame(std::uint64_t seq = 1) {
  asmc_msgs::LidarTrackingObservation m;
  auto& o=m.observation;
  o.producer_session_id="a";o.generation=1;o.scan_sequence=seq;
  o.config_id="config";o.calibration_id="calibration";
  o.objects.header.stamp=ros::Time(10,static_cast<uint32_t>(seq));o.objects.header.frame_id="map";
  o.objects.status=o.objects.OBSERVED;
  asmc_msgs::LidarObject p;
  p.track_id=7;p.observation_state=p.OBSERVED;p.age=4;
  p.classification="vehicle";p.camera_class="truck";
  p.center.x=5;p.center.z=.5;p.length=4;p.width=2;p.height=1;
  p.absolute_velocity_valid=true;p.absolute_velocity_mps.x=3;
  for(int i=0;i<8;++i){p.obb_corners[i].x=5+(i&1?2:-2);p.obb_corners[i].y=i&2?1:-1;p.obb_corners[i].z=i&4?1:0;}
  o.objects.objects.push_back(p);
  asmc_msgs::LidarTrackingState state;state.track_id=7;state.cluster_center=p.center;
  m.tracking.push_back(state);
  return m;
}
TEST(CommonLidar, PreservesGeometryClassificationAndAllFourStates) {
  for (int state=0;state<4;++state) {
    auto m=Frame();m.tracking[0].motion_state=state;
    auto& p=m.observation.objects.objects[0];p.dynamic_valid=state!=0;p.is_dynamic=state>=2;
    DetectedObjectArray out;
    ASSERT_EQ(ConversionStatus::OK,ConvertLidarObservation(m,&out));
    ASSERT_EQ(1u,out.objects.size());
    EXPECT_EQ(kObjectVehicle,out.objects[0].type);EXPECT_EQ(state,out.objects[0].motion_state);
    EXPECT_EQ(p.obb_corners,out.objects[0].obb_corners);EXPECT_EQ(p.center,out.objects[0].center);
    EXPECT_EQ(3.,out.objects[0].vx);EXPECT_EQ(m.observation.objects.header,out.header);
  }
}
TEST(CommonLidar, RawClassesAndGroupsNeverInventMotion) {
  const std::vector<std::pair<std::string,std::string>> classes={
    {"person","dynamic_obstacle"},{"dog","dynamic_obstacle"},{"deer","dynamic_obstacle"},
    {"bicycle","dynamic_obstacle"},{"stroller","dynamic_obstacle"},
    {"car","vehicle"},{"bus","vehicle"},{"truck","vehicle"},
    {"motorcycle","static_obstacle"},{"bollard","static_obstacle"},{"box","static_obstacle"},
    {"concrete_barrier","static_obstacle"},{"steel_barricade","static_obstacle"},
    {"traffic_drum","static_obstacle"},{"plastic_barrier","static_obstacle"},{"traffic_cone","static_obstacle"}};
  for(const auto& entry:classes) {
    const auto& raw=entry.first;
    auto m=Frame();auto& p=m.observation.objects.objects[0];
    p.classification=entry.second;p.camera_class=raw;
    DetectedObjectArray out;ASSERT_EQ(ConversionStatus::OK,ConvertLidarObservation(m,&out));
    EXPECT_EQ(raw=="person"?kObjectPedestrian:p.classification=="vehicle"?kObjectVehicle:kObjectUnknown,out.objects[0].type);
    EXPECT_EQ(kMotionUnknown,out.objects[0].motion_state);
  }
}
TEST(CommonLidar, RejectsInvalidInputsAtomicallyAndPreservesNormalEmpty) {
  const auto original=Frame();DetectedObjectArray out;
  ASSERT_EQ(ConversionStatus::OK,ConvertLidarObservation(original,&out));const auto saved=out;
  for(int test=0;test<8;++test) {
    auto m=original;auto& a=m.observation.objects;auto& p=a.objects[0];
    if(test==0)p.absolute_velocity_valid=false;
    if(test==1)p.absolute_velocity_mps.x=std::numeric_limits<double>::quiet_NaN();
    if(test==2)a.status=a.POSE_UNAVAILABLE;
    if(test==3)a.header.frame_id="base_link";
    if(test==4)m.tracking.clear();
    if(test==5)m.tracking[0].track_id=9;
    if(test==6){a.objects.push_back(p);m.tracking.push_back(m.tracking[0]);}
    if(test==7)m.tracking[0].motion_state=3;  // inconsistent bool validity
    EXPECT_NE(ConversionStatus::OK,ConvertLidarObservation(m,&out));EXPECT_EQ(saved,out);
  }
  auto empty=original;empty.observation.objects.objects.clear();empty.tracking.clear();
  empty.observation.objects.status=empty.observation.objects.OBSERVED_EMPTY;
  ASSERT_EQ(ConversionStatus::OK,ConvertLidarObservation(empty,&out));EXPECT_TRUE(out.objects.empty());
}
TEST(CommonLidarSequence, RejectsLateEpochsDuplicatesAndConfigChanges) {
  ObservationSequence gate;std::string reason;auto m=Frame();
  EXPECT_TRUE(gate.Accept(m.observation,&reason));EXPECT_FALSE(gate.Accept(m.observation,&reason));
  auto next=Frame(2);next.observation.config_id="unexpected";EXPECT_FALSE(gate.Accept(next.observation,&reason));
  next.observation.generation=2;EXPECT_TRUE(gate.Accept(next.observation,&reason));
  auto late=Frame(3);EXPECT_FALSE(gate.Accept(late.observation,&reason));
  next=Frame(4);next.observation.producer_session_id="b";EXPECT_TRUE(gate.Accept(next.observation,&reason));
  late=Frame(5);late.observation.generation=99;EXPECT_FALSE(gate.Accept(late.observation,&reason));
  next=Frame(6);next.observation.producer_session_id="b";
  next.observation.objects.status=next.observation.objects.POSE_UNAVAILABLE;
  EXPECT_TRUE(gate.Accept(next.observation,&reason));  // consumes the invalid epoch/stamp too
  next=Frame(5);next.observation.producer_session_id="b";EXPECT_FALSE(gate.Accept(next.observation,&reason));
}
}  // namespace
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
