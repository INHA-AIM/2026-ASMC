#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "Parser/Traffic/ros_traffic_light_provider.hpp"
#include "Signal/SignalShadow.hpp"

namespace {
bool Wait(const std::function<bool()>& ready) {
  const auto deadline=ros::WallTime::now()+ros::WallDuration(3);
  while(ros::WallTime::now()<deadline) {
    ros::spinOnce();if(ready())return true;ros::WallDuration(.005).sleep();
  }
  return false;
}
TEST(SignalJournal, PreservesIntermediateRedAndDoesNotChangeOperationalSample) {
  ros::NodeHandle nh;
  behavior_planner::RosTrafficLightProvider provider(nh,"/signal_journal/frames","",1,1,true);
  auto pub=nh.advertise<asmc_msgs::TrafficLightDetection>("/signal_journal/frames",64);
  ASSERT_TRUE(Wait([&]{return pub.getNumSubscribers()>0;}));
  const auto stamp=ros::Time::now();
  for(int i=0;i<3;++i) {
    asmc_msgs::TrafficLightDetection m;m.header.stamp=stamp+ros::Duration(i*.001);
    m.header.frame_id="camera";m.valid=true;m.stale=false;m.label=i==1 ? "red" : "green";
    pub.publish(m);
  }
  std::vector<asmc_planner::signal::Observation> events;bool overflow=false;
  ASSERT_TRUE(Wait([&]{auto batch=provider.takeSignalObservations(overflow);
    events.insert(events.end(),batch.begin(),batch.end());return events.size()==3;}));
  EXPECT_FALSE(overflow);EXPECT_EQ(events[1].label,"red");
  EXPECT_EQ(events[1].original_stamp_ns,(stamp+ros::Duration(.001)).toNSec());
  EXPECT_EQ(provider.getState("id",behavior_planner::SignalMovement::STRAIGHT),behavior_planner::TrafficLightState::GREEN);
  provider.takeSignalObservations(overflow);
  EXPECT_EQ(provider.getRawLabel(),"green");
}
TEST(SignalJournal, OverflowIsExplicitAndBounded) {
  ros::NodeHandle nh;
  behavior_planner::RosTrafficLightProvider provider(nh,"/signal_overflow/frames","",1,1,true);
  auto pub=nh.advertise<asmc_msgs::TrafficLightDetection>("/signal_overflow/frames",64);
  ASSERT_TRUE(Wait([&]{return pub.getNumSubscribers()>0;}));
  for(int i=0;i<132;++i) {
    asmc_msgs::TrafficLightDetection m;m.header.stamp=ros::Time::now();m.valid=true;m.stale=false;
    m.label="frame_"+std::to_string(i);pub.publish(m);
    ASSERT_TRUE(Wait([&]{return provider.getRawLabel()==m.label;}));
  }
  ros::spinOnce();bool overflow=false;const auto events=provider.takeSignalObservations(overflow);
  EXPECT_TRUE(overflow);EXPECT_EQ(events.size(),128u);
}
TEST(SignalJournal, StopTransitionWakesAfterJournalUnlockWithoutWakingEveryRedFrame) {
  ros::NodeHandle nh;
  behavior_planner::RosTrafficLightProvider provider(nh,"/signal_wakeup/frames","",1,1,true);
  unsigned wakes=0,consumed=0;bool overflow=false;
  provider.setStopObservationWakeup([&] {
    ++wakes;
    const auto batch=provider.takeSignalObservations(overflow);
    ASSERT_FALSE(batch.empty());
    EXPECT_TRUE(batch.back().label!="green" || !batch.back().valid || batch.back().stale);
    consumed+=batch.size();
    // A policy re-evaluation also reads the operational sample; no lock can
    // remain held across this callback.
    provider.getRawLabel();
  });
  auto pub=nh.advertise<asmc_msgs::TrafficLightDetection>("/signal_wakeup/frames",64);
  ASSERT_TRUE(Wait([&]{return pub.getNumSubscribers()>0;}));
  unsigned received=0;
  for(const auto* label:{"green","yellow","red","red","green","red"}) {
    asmc_msgs::TrafficLightDetection m;m.header.stamp=ros::Time::now();m.header.seq=++received;
    m.header.frame_id="camera";m.valid=true;m.stale=false;m.label=label;pub.publish(m);
    ASSERT_TRUE(Wait([&]{return provider.getRawLabel()==label;}));ros::spinOnce();
  }
  EXPECT_EQ(wakes,2u);EXPECT_GE(consumed,5u);EXPECT_FALSE(overflow);
  // Same-original invalid notification must also wake after a green sample.
  asmc_msgs::TrafficLightDetection m;m.header.stamp=ros::Time::now();m.valid=true;m.stale=false;m.label="green";
  pub.publish(m);ASSERT_TRUE(Wait([&]{return provider.getRawLabel()=="green";}));
  m.valid=false;m.stale=true;pub.publish(m);
  ASSERT_TRUE(Wait([&]{return wakes==3;}));
}
TEST(SignalJournal, ActiveModeIsRejectedBeforeAnyControlConnection) {
  ros::NodeHandle nh,pnh("/signal_active_rejection");pnh.setParam("signal_policy/mode","active");
  EXPECT_THROW(behavior_planner::SignalShadow shadow(nh,pnh),std::runtime_error);
  pnh.deleteParam("signal_policy/mode");
}
TEST(SignalJournal, DefaultShadowReportsMissingInputsWithoutControlPermission) {
  ros::NodeHandle nh,pnh("/signal_default_shadow");
  behavior_planner::SignalShadow shadow(nh,pnh);
  std::string diagnostic;
  auto subscriber=nh.subscribe<std_msgs::String>("/planning/signal_policy_shadow",1,
      [&](const std_msgs::String::ConstPtr& m){diagnostic=m->data;});
  ASSERT_TRUE(Wait([&]{return subscriber.getNumPublishers()>0;}));
  behavior_planner::RouteManager route;behavior_planner::EgoState ego;
  shadow.Update(ego,false,false,nullptr,route,nullptr);
  ASSERT_TRUE(Wait([&]{return !diagnostic.empty();}));
  EXPECT_NE(diagnostic.find("\"status\":\"unknown_input\""),std::string::npos);
  EXPECT_NE(diagnostic.find("\"actual_control_allowed\":false"),std::string::npos);
}
}  // namespace
int main(int argc,char** argv) {
  ros::init(argc,argv,"signal_journal_test");testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
