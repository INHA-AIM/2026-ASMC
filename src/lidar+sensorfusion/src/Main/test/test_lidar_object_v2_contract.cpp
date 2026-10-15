// Contract test for the additive LiDAR object v2 messages (B0).
// Runs in the nested workspace CI job (planning-integration.yml run_tests), which
// is the only CI job that executes generated-message tests for these types.
#include <Msg/TrackedObject.h>
#include <Msg/TrackedObjectArray.h>
#include <asmc_msgs/LidarObject.h>
#include <asmc_msgs/LidarObjectArray.h>
#include <asmc_msgs/LidarObjectArrayV2.h>
#include <asmc_msgs/LidarObjectV2.h>
#include <asmc_msgs/TrackedObject.h>
#include <asmc_msgs/TrackedObjectArray.h>
#include <gtest/gtest.h>
#include <ros/serialization.h>

#include <cstdint>
#include <string>
#include <vector>

namespace {
using V2=asmc_msgs::LidarObjectV2;
using ArrayV2=asmc_msgs::LidarObjectArrayV2;

template <class M> std::string Md5() { return ros::message_traits::MD5Sum<M>::value(); }

template <class M> M RoundTrip(const M& input) {
  const std::uint32_t size=ros::serialization::serializationLength(input);
  std::vector<std::uint8_t> buffer(size);
  ros::serialization::OStream out(buffer.data(),size);
  ros::serialization::serialize(out,input);
  ros::serialization::IStream in(buffer.data(),size);
  M result;
  ros::serialization::deserialize(in,result);
  return result;
}

V2 Object(std::uint8_t state,int index) {
  V2 item;
  item.track_id=100+index; item.detection_index=7+index;
  item.observation_state=index%2==0?V2::OBSERVED:V2::PREDICTED;
  item.age=3+index; item.miss_count=index;
  item.classification="car"; item.camera_class="truck";
  item.center.x=1.5+index; item.center.y=-2.25; item.center.z=0.75;
  for(unsigned corner=0;corner<item.obb_corners.size();++corner) {
    item.obb_corners[corner].x=corner+0.125;
    item.obb_corners[corner].y=-double(corner)-0.5;
    item.obb_corners[corner].z=0.25*corner;
  }
  item.yaw=0.5-index; item.yaw_rate_valid=true; item.yaw_rate_rad_s=0.125*index;
  item.length=4.5; item.width=1.75; item.height=1.5;
  item.relative_velocity_valid=true; item.relative_velocity_mps.x=1.0+index;
  item.absolute_velocity_valid=index%2==1; item.absolute_velocity_mps.y=-0.5*index;
  item.source_point_count=50+index;
  item.motion_state=state;
  item.motion_valid=state!=V2::MOTION_UNKNOWN;
  item.dynamic_valid=item.motion_valid;
  item.is_dynamic=state==V2::MOTION_DYNAMIC;
  return item;
}
}  // namespace

TEST(LidarObjectV2Contract, FrozenMd5Unchanged) {
  EXPECT_EQ(Md5<asmc_msgs::LidarObject>(),"b94cf82e0f325e9ce0364bb143df9f06");
  EXPECT_EQ(Md5<asmc_msgs::LidarObjectArray>(),"5f774d07dffd4c7dfa93a8d4b62c60b4");
  EXPECT_EQ(Md5<asmc_msgs::TrackedObject>(),"6c96135eb03a3ee93bba474fa835f57d");
  EXPECT_EQ(Md5<asmc_msgs::TrackedObjectArray>(),"e127b2aabd7494ad01a5a56b005bf07e");
  EXPECT_EQ(Md5<Msg::TrackedObject>(),"6c96135eb03a3ee93bba474fa835f57d");
  EXPECT_EQ(Md5<Msg::TrackedObjectArray>(),"e127b2aabd7494ad01a5a56b005bf07e");
}

TEST(LidarObjectV2Contract, V2Md5AndTypePinned) {
  EXPECT_EQ(Md5<V2>(),"c22c9b52da848aeab0e2b4e73343d9f0");
  EXPECT_EQ(Md5<ArrayV2>(),"d728e7fd984c24641abe10dddfda6dd5");
  EXPECT_EQ(std::string(ros::message_traits::DataType<V2>::value()),"asmc_msgs/LidarObjectV2");
  EXPECT_EQ(std::string(ros::message_traits::DataType<ArrayV2>::value()),"asmc_msgs/LidarObjectArrayV2");
}

TEST(LidarObjectV2Contract, ConstantValues) {
  EXPECT_EQ(int(V2::MOTION_UNKNOWN),0);
  EXPECT_EQ(int(V2::MOTION_LOW_THREAT),1);
  EXPECT_EQ(int(V2::MOTION_DYNAMIC_PENDING),2);
  EXPECT_EQ(int(V2::MOTION_DYNAMIC),3);
  EXPECT_EQ(int(V2::OBSERVED),int(asmc_msgs::LidarObject::OBSERVED));
  EXPECT_EQ(int(V2::PREDICTED),int(asmc_msgs::LidarObject::PREDICTED));
  EXPECT_EQ(int(ArrayV2::OBSERVED),int(asmc_msgs::LidarObjectArray::OBSERVED));
  EXPECT_EQ(int(ArrayV2::OBSERVED_EMPTY),int(asmc_msgs::LidarObjectArray::OBSERVED_EMPTY));
  EXPECT_EQ(int(ArrayV2::INPUT_INVALID),int(asmc_msgs::LidarObjectArray::INPUT_INVALID));
  EXPECT_EQ(int(ArrayV2::POSE_UNAVAILABLE),int(asmc_msgs::LidarObjectArray::POSE_UNAVAILABLE));
}

TEST(LidarObjectV2Contract, DefaultObjectIsUnknownAndInvalid) {
  const V2 item;
  EXPECT_FALSE(item.motion_valid);
  EXPECT_EQ(int(item.motion_state),int(V2::MOTION_UNKNOWN));
  EXPECT_FALSE(item.dynamic_valid);
  EXPECT_FALSE(item.is_dynamic);
}

TEST(LidarObjectV2Contract, RoundTripPreservesAllFourMotionStates) {
  ArrayV2 array;
  array.header.seq=11; array.header.stamp=ros::Time(1760000000,123456789);
  array.header.frame_id="base_link";
  array.status=ArrayV2::OBSERVED; array.reason="observed"; array.stamp_provenance="bridge_receive_ns";
  const std::uint8_t states[]={V2::MOTION_UNKNOWN,V2::MOTION_LOW_THREAT,V2::MOTION_DYNAMIC_PENDING,V2::MOTION_DYNAMIC};
  for(int i=0;i<4;++i) array.objects.push_back(Object(states[i],i));

  const ArrayV2 restored=RoundTrip(array);

  EXPECT_TRUE(restored==array);
  ASSERT_EQ(restored.objects.size(),4u);
  const bool valid[]={false,true,true,true},dynamic[]={false,false,false,true};
  for(int i=0;i<4;++i) {
    EXPECT_EQ(int(restored.objects[i].motion_state),int(states[i]));
    EXPECT_EQ(restored.objects[i].motion_valid,valid[i]);
    EXPECT_EQ(restored.objects[i].dynamic_valid,valid[i]);
    EXPECT_EQ(restored.objects[i].is_dynamic,dynamic[i]);
  }
  EXPECT_EQ(restored.header.frame_id,"base_link");
  EXPECT_EQ(restored.stamp_provenance,"bridge_receive_ns");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
