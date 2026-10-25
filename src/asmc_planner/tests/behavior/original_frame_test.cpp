#include <gtest/gtest.h>
#include "Parser/Traffic/OriginalCameraFrames.hpp"
namespace {
using Index=behavior_planner::OriginalCameraFrames;
std_msgs::Header Image(unsigned seq=7) {
 std_msgs::Header h;h.frame_id="camera_extra";h.stamp=ros::Time(100,200000000);h.seq=seq;return h;
}
asmc_msgs::SensorFrameTiming Timing(unsigned seq=7) {
 asmc_msgs::SensorFrameTiming t;t.sensor_frame_id="camera_extra";
 t.sensor_topic="/camera";t.published_stamp=Image().stamp;t.sensor_wire_sequence=seq;
 t.sensor_wire_sequence_valid=true;t.bridge_session_id="bridge";t.source_valid=true;
 t.source_sec=100;t.source_nsec=0;t.source_clock_domain="morai_sensor";t.source_reference="camera_frame";return t;
}
}
TEST(OriginalFrame,RequiresRawImageAndCompleteTimingTupleNotStampAlone) {
 Index index("/camera",0,.001);Index::Original source;
 EXPECT_FALSE(index.Timing(Timing()));
 EXPECT_EQ(index.Resolve(Image(999),source),Index::Match::MISSING);
 index.Image(Image());EXPECT_EQ(index.Resolve(Image(999),source),Index::Match::VALID);
 EXPECT_EQ(source.source_ns,100000000000ULL);EXPECT_NEAR(source.ros_stamp,99.999,1e-9);
}
TEST(OriginalFrame,WrongWireSequenceOrAmbiguousSameStampCannotAuthorize) {
 Index index("/camera",0,.001);Index::Original source;
 index.Image(Image(7));index.Timing(Timing(8));
 EXPECT_EQ(index.Resolve(Image(),source),Index::Match::INVALID);
}
TEST(OriginalFrame,ExplicitInvalidCannotBeRestoredByValidDuplicate) {
 Index index("/camera",0,.001);Index::Original source;
 index.Image(Image());index.Timing(Timing());index.Invalidate(Image(77));
 index.Image(Image());index.Timing(Timing());
 EXPECT_EQ(index.Resolve(Image(999),source),Index::Match::INVALID);
}
TEST(OriginalFrame,FutureSourceAndClockRegressionAreRejected) {
 Index index("/camera",0,.001);Index::Original source;
 index.Image(Image());auto t=Timing();t.source_sec=101;
 EXPECT_TRUE(index.Timing(t));EXPECT_EQ(index.Resolve(Image(),source),Index::Match::INVALID);
 t=Timing();t.source_regression=true;EXPECT_TRUE(index.Timing(t));
 index.Image(Image());EXPECT_EQ(index.Resolve(Image(),source),Index::Match::INVALID);
}
TEST(OriginalFrame,DuplicateOriginalHasSameIdentityDespiteNewReceiveHeader) {
 Index index("/camera",0,.001);Index::Original a,b;
 index.Image(Image());index.Timing(Timing());ASSERT_EQ(index.Resolve(Image(),a),Index::Match::VALID);
 auto h=Image(8);h.stamp=ros::Time(100,300000000);auto t=Timing(8);t.published_stamp=h.stamp;t.source_duplicate=true;
 index.Image(h);index.Timing(t);ASSERT_EQ(index.Resolve(h,b),Index::Match::VALID);
 EXPECT_EQ(a.source_ns,b.source_ns);EXPECT_DOUBLE_EQ(a.ros_stamp,b.ros_stamp);
}
