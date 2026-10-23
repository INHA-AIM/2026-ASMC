#include <asmc_lidar/ros/camera_receiver.hpp>
#include <gtest/gtest.h>
#include <functional>
using namespace asmc::lidar::camera;
using asmc::lidar::Ns;
namespace {
bool wait_for(const std::function<bool()>& predicate) {
  auto end=ros::WallTime::now()+ros::WallDuration(3);
  while(ros::WallTime::now()<end){if(predicate())return true;ros::WallDuration(.005).sleep();}return predicate();
}
Calibrations fixture() {
  Calibrations c;
  for(const std::string v:{"front","left","right"}) {
    Calibration k;k.frame_id="camera_"+v;k.id="sha256:"+v;
    k.width=v=="front"?1280:640;k.height=v=="front"?720:480;c[k.frame_id]=k;
  }return c;
}
}
TEST(CameraReceiver, TransportEpochAndIndependentViews) {
  ros::NodeHandle nh;
  Receiver receiver(nh,fixture());auto b=receiver.buffer();
  auto control=nh.advertise<asmc_msgs::InferenceStatus>("/perception/camera/objects/status",1,true);
  auto output=nh.advertise<asmc_msgs::CameraObjectObservation>("/perception/camera/objects/observations",4);
  ros::AsyncSpinner spinner(2);spinner.start();
  ASSERT_TRUE(wait_for([&]{return control.getNumSubscribers()==1&&output.getNumSubscribers()==1;}));
  asmc_msgs::InferenceStatus status;status.header.stamp=ros::Time::now();status.model="yolo";
  status.mode=status.ACTIVE;status.generation=1;status.producer_session_id="producer-one";
  control.publish(status);
  ASSERT_TRUE(wait_for([&]{return b->counters()["status_accepted"]==1;}));
  asmc_msgs::CameraObjectObservation o;o.producer_session_id=status.producer_session_id;o.generation=1;
  o.calibration_id="sha256:front";o.model_id="model-sha";o.config_id="config-sha";
  o.detections.header.seq=45;o.detections.header.stamp=ros::Time::now();o.detections.header.frame_id="camera_front";
  o.detections.image_width=1280;o.detections.image_height=720;o.detections.valid=true;
  asmc_msgs::CameraObjectBox box;box.xmin=10;box.ymin=20;box.xmax=30;box.ymax=60;
  box.class_name="dynamic_obstacle";box.source_class_name="dog";box.confidence=.85;box.confidence_valid=true;
  o.detections.boxes.push_back(box);output.publish(o);
  ASSERT_TRUE(wait_for([&]{return b->counters()["accepted"]==1;}));
  auto q=b->query(o.detections.header.stamp.toNSec(),0,ros::Time::now().toNSec(),steady_now_ns());
  ASSERT_EQ(q.views.at("camera_front").candidates.size(),1u);
  auto candidate=q.views.at("camera_front").candidates[0];const auto& saved=candidate.record->observation;
  EXPECT_EQ(saved.header.sequence,45u);EXPECT_EQ(saved.header.stamp_ns,Ns(o.detections.header.stamp.toNSec()));
  EXPECT_EQ(saved.boxes[0].source_class_name,"dog");EXPECT_EQ(saved.boxes[0].class_name,"dynamic_obstacle");
  EXPECT_FLOAT_EQ(saved.boxes[0].confidence,.85);EXPECT_TRUE(saved.boxes[0].confidence_valid);
  EXPECT_EQ(saved.model_id,"model-sha");EXPECT_EQ(saved.config_id,"config-sha");
  // Side normal-empty is a fresh observation, not an invalid/missing frame.
  auto side=o;side.detections.header.frame_id="camera_left";side.calibration_id="sha256:left";
  side.detections.image_width=640;side.detections.image_height=480;side.detections.boxes.clear();output.publish(side);
  ASSERT_TRUE(wait_for([&]{return b->counters()["accepted"]==2;}));
  q=b->query(side.detections.header.stamp.toNSec(),0,ros::Time::now().toNSec(),steady_now_ns());
  EXPECT_EQ(q.views.at("camera_left").reason,"empty");EXPECT_EQ(q.views.at("camera_right").reason,"missing");
  side.detections.valid=false;output.publish(side);
  ASSERT_TRUE(wait_for([&]{return b->counters()["invalid"]==1;}));
  EXPECT_TRUE(b->current(candidate,ros::Time::now().toNSec(),steady_now_ns()));
  status.header.stamp=ros::Time::now();status.generation=2;status.mode=status.STANDBY;control.publish(status);
  ASSERT_TRUE(wait_for([&]{return b->counters()["status_accepted"]==2;}));
  EXPECT_FALSE(b->current(candidate,ros::Time::now().toNSec(),steady_now_ns()));
  output.publish(o);ASSERT_TRUE(wait_for([&]{return b->counters()["standby"]>=1;}));
  // Restart reuses generation=1, but only the new UUID may supply observations.
  status.header.stamp=ros::Time::now();status.generation=1;status.mode=status.ACTIVE;status.producer_session_id="producer-two";control.publish(status);
  ASSERT_TRUE(wait_for([&]{return b->counters()["status_accepted"]==3;}));
  output.publish(o);ASSERT_TRUE(wait_for([&]{return b->counters()["epoch_mismatch"]>=1;}));
  receiver.close();spinner.stop();
  EXPECT_EQ(b->query(1,0,1,steady_now_ns()).views.at("camera_front").reason,"closed");
}
int main(int argc,char** argv){ros::init(argc,argv,"camera_receiver_ros_test");testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
