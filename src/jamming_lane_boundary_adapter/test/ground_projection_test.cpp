#include <gtest/gtest.h>
#include "lane_boundary_adapter/ground_projection.hpp"
#include <limits>
#include <boost/make_shared.hpp>

namespace {
using namespace lane_boundary_adapter;
const ros::Time now(100, 0);
const ros::SteadyTime received(100, 0);
const cv::Vec3d nominal(0, 0, -.36);
cv::Vec3d tilted(double sx, double sy) {
  return {sx, sy, -.36*std::sqrt(1+sx*sx+sy*sy)};
}
struct ProjectionTest : testing::Test {
  GroundProjectionBuffer buffer{"config", {{"front","f"},{"left","l"},{"right","r"}}, nominal};
  asmc_msgs::InferenceStatus status;
  asmc_msgs::CameraGroundProjection::Ptr context{new asmc_msgs::CameraGroundProjection};
  asmc_msgs::LaneBoundaryFamily family;
  std_msgs::Header image;
  void SetUp() override {
    status.header.stamp=now; status.producer_session_id="model";
    status.generation=4; status.mode=status.ACTIVE; status.loaded=true;
    buffer.status(status,now,received);
    context->producer_session_id="model";context->generation=4;
    context->projection_session_id="family";context->config_id="config";
    context->header.stamp=now;context->header.frame_id="base_link";
    context->views={"front","left","right"};context->calibration_ids={"f","l","r"};
    context->imu_corrected=true;context->reason="corrected";
    // Family xyz belongs to the latest (right) view; raw front is 20ms older.
    for(int i=0;i<3;++i) {
      std_msgs::Header h;h.seq=100+i;h.frame_id="camera_"+context->views[i];
      h.stamp=now-ros::Duration(.02-.01*i);context->image_headers.push_back(h);
      const auto p=tilted(.01*i,-.005*i);
      context->plane_coefficients.insert(context->plane_coefficients.end(),{p[0],p[1],p[2]});
    }
    image=context->image_headers[0]; family.header=context->header;
    family.header.seq=444;context->header.seq=555; // independent publisher counters
    family.config_sha256="config";family.boundaries.resize(1);
    geometry_msgs::Point32 point;point.x=10;point.y=1;
    point.z=onPlane(tilted(.02,-.01),point.x,point.y)[2];
    family.boundaries[0].polyline.push_back(point);
  }
  ProjectionMatch match() {return buffer.match(family,image,"front",now,received);}
};

TEST_F(ProjectionTest, ReferenceXYZIsCheckedButFrontUsesItsExactImagePlane) {
  buffer.add(context,now,received);
  const auto out=match();ASSERT_TRUE(out.valid)<<out.reason;
  EXPECT_EQ(out.plane,nominal);
  image.seq++;EXPECT_EQ(match().reason,"projection_exact_image_missing");
  image=context->image_headers[0];family.header.frame_id="other";
  EXPECT_EQ(match().reason,"family_projection_frame_mismatch");family.header.frame_id="base_link";
  image=context->image_headers[0];family.boundaries[0].polyline[0].z=-.36;
  EXPECT_EQ(match().reason,"family_xyz_projection_mismatch");
  family.boundaries[0].polyline[0].x=std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(match().valid);
}
TEST_F(ProjectionTest, ContextMayArriveAfterFamilyButNearestStampIsNeverSubstituted) {
  EXPECT_FALSE(match().valid);buffer.add(context,now,received);EXPECT_TRUE(match().valid);
  family.header.stamp+=ros::Duration(.001);EXPECT_FALSE(match().valid);
}
TEST_F(ProjectionTest, WrongConfigCalibrationEpochPlaneAndArraysAreRejected) {
  context->config_id="wrong";buffer.add(context,now,received);EXPECT_EQ(buffer.last_error(),"projection_config_mismatch");
  context->config_id="config";context->generation++;
  buffer.add(context,now,received);EXPECT_EQ(buffer.last_error(),"projection_epoch_mismatch");
  context->generation--;context->calibration_ids[0]="wrong";
  buffer.add(context,now,received);EXPECT_EQ(buffer.last_error(),"projection_calibration_or_source_mismatch");
  context->calibration_ids[0]="f";context->plane_coefficients[0]=.5;
  buffer.add(context,now,received);EXPECT_EQ(buffer.last_error(),"projection_plane_out_of_bounds");
  context->plane_coefficients[0]=std::numeric_limits<double>::quiet_NaN();
  buffer.add(context,now,received);EXPECT_EQ(buffer.last_error(),"projection_nonfinite_plane");
  context->plane_coefficients.clear();buffer.add(context,now,received);
  EXPECT_EQ(buffer.last_error(),"projection_malformed_arrays");EXPECT_EQ(buffer.size(),0U);
}
TEST_F(ProjectionTest, NominalFallbackIsExplicitAndCannotContainTiltedPlanes) {
  context->imu_corrected=false;buffer.add(context,now,received);
  EXPECT_EQ(buffer.last_error(),"projection_invalid_nominal_fallback");
  context->plane_coefficients={0,0,-.36,0,0,-.36,0,0,-.36};
  family.boundaries[0].polyline[0].z=-.36;
  buffer.add(context,now,received);auto out=match();EXPECT_TRUE(out.valid);EXPECT_FALSE(out.imu_corrected);
}
TEST_F(ProjectionTest, RestartAndModeTransitionsRetireOldObservations) {
  buffer.add(context,now,received);ASSERT_TRUE(match().valid);
  status.mode=status.STANDBY;EXPECT_TRUE(buffer.status(status,now,received));
  EXPECT_EQ(match().reason,"lane_not_active");EXPECT_EQ(buffer.size(),0U);
  status.mode=status.ACTIVE;status.generation++;EXPECT_TRUE(buffer.status(status,now,received));
  buffer.add(context,now,received);EXPECT_EQ(buffer.last_error(),"projection_epoch_mismatch");
  auto old=status;old.generation--;buffer.status(old,now,received);
  EXPECT_EQ(buffer.last_error(),"retired_lane_status");
}
TEST_F(ProjectionTest, SourceAndReceiptFreshnessAndDuplicateStampsFailClosed) {
  buffer.add(context,now,received);
  EXPECT_EQ(buffer.match(family,image,"front",now,received+ros::WallDuration(.36)).reason,"lane_status_stale");
  buffer.add(context,now,received);EXPECT_EQ(buffer.last_error(),"projection_duplicate_stamp");
  EXPECT_EQ(match().reason,"projection_ambiguous_reused_stamp");
}
TEST_F(ProjectionTest, BoundedRecordsAndProjectionProducerRestart) {
  for(int i=0;i<12;++i) {
    auto next=boost::make_shared<asmc_msgs::CameraGroundProjection>(*context);
    next->header.stamp-=ros::Duration(.001*i);
    for(auto& h:next->image_headers)h.stamp-=ros::Duration(.001*i);
    buffer.add(next,now,received);
  }
  EXPECT_EQ(buffer.size(),8U);
  auto next=boost::make_shared<asmc_msgs::CameraGroundProjection>(*context);
  next->projection_session_id="new-family";
  next->header.stamp+=ros::Duration(.001);
  for(auto& h:next->image_headers)h.stamp+=ros::Duration(.001);
  const auto revision=buffer.revision();buffer.add(next,now,received);
  EXPECT_GT(buffer.revision(),revision);EXPECT_EQ(buffer.size(),1U);
  buffer.add(context,now,received);EXPECT_EQ(buffer.last_error(),"retired_projection_session");
}
TEST(GroundProjectionMath, FullXYZAndHomographyAgreeWithoutSecondAttitudeRotation) {
  const cv::Matx33d K(600,0,640,0,600,360,0,0,1);
  const cv::Matx33d R(0,-1,0,0,0,-1,1,0,0);
  const cv::Vec3d t(0,.9,-1);const auto plane=tilted(.04,-.03);
  cv::Point2d uv;ASSERT_TRUE(projectGroundPoint(K,R,t,plane,10,1,uv));
  const auto xyz=onPlane(plane,10,1);const auto direct=K*(R*xyz+t);
  const auto h=groundHomography(K,R,t,plane)*cv::Vec3d(10,1,1);
  EXPECT_NEAR(uv.x,direct[0]/direct[2],1e-10);EXPECT_NEAR(uv.y,direct[1]/direct[2],1e-10);
  EXPECT_NEAR(uv.x,h[0]/h[2],1e-10);EXPECT_NEAR(uv.y,h[1]/h[2],1e-10);
  EXPECT_FALSE(projectGroundPoint(K,R,t,plane,-5,1,uv));
}
}
int main(int argc,char** argv) {
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
