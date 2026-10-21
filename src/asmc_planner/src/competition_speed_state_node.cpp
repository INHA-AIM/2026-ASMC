// ROS input assembly only: GPS/IMU owns map pose, Competition status owns vx.
// Ego status, gRPC actor truth, lateral velocity and oracle position are unused.
#include <asmc_msgs/VehicleState.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdexcept>
#include "Planning/CompetitionSpeedAlignment.hpp"

class CompetitionSpeedStateNode {
 public:
  CompetitionSpeedStateNode():private_("~") {
    planning::CompetitionSpeedAlignment::Config config;
    if(!private_.getParam("source_age_s",config.source_age_s) ||
       !private_.getParam("receive_age_s",config.receive_age_s) ||
       !private_.getParam("restart_jump_s",config.restart_jump_s) ||
       !private_.getParam("stationary_deadband_mps",config.stationary_deadband_mps) ||
       !std::isfinite(config.source_age_s)||config.source_age_s<=0||config.source_age_s>.3 ||
       !std::isfinite(config.receive_age_s)||config.receive_age_s<=0||config.receive_age_s>.3 ||
       !std::isfinite(config.restart_jump_s)||config.restart_jump_s<=config.source_age_s ||
       !std::isfinite(config.stationary_deadband_mps)||config.stationary_deadband_mps<0||config.stationary_deadband_mps>.1)
      throw std::runtime_error("Missing/invalid competition speed alignment configuration");
    alignment_=planning::CompetitionSpeedAlignment(config);
    std::string input,output,speed;
    private_.param<std::string>("pose_topic",input,"/planning/vehicle/gps_imu_state");
    private_.param<std::string>("state_topic",output,"/planning/vehicle/state");
    private_.param<std::string>("speed_topic",speed,"/Competition_topic");
    private_.param<std::string>("speed_callerid",caller_,"/rosbridge_websocket");
    publisher_=nh_.advertise<asmc_msgs::VehicleState>(output,1);
    diagnostics_=nh_.advertise<std_msgs::String>("/planning/vehicle/speed_diagnostics",1);
    speed_=nh_.subscribe(speed,32,&CompetitionSpeedStateNode::speedCallback,this);
    pose_=nh_.subscribe(input,1,&CompetitionSpeedStateNode::poseCallback,this);
  }
 private:
  void speedCallback(const ros::MessageEvent<morai_msgs::EgoVehicleStatus const>& event) {
    if(event.getPublisherName()!=caller_)return;
    const auto m=event.getMessage();
    // ROS Competition status carries velocity.x in m/s. No other fields enter
    // the estimator, even if a producer accidentally fills privileged fields.
    alignment_.observe(m->header.stamp.toSec(),ros::SteadyTime::now().toSec(),m->velocity.x);
  }
  void poseCallback(const asmc_msgs::VehicleState::ConstPtr& m) {
    auto state=*m;
    const auto speed=alignment_.align(m->header.stamp.toSec(),m->acceleration,
                                       ros::SteadyTime::now().toSec());
    state.valid=m->valid && m->header.frame_id=="map" &&
        m->source==asmc_msgs::VehicleState::SOURCE_GPS_IMU && speed.has_value();
    if(speed)state.speed=*speed;
    // Preserve map pose, its source stamp and base-estimator source enum. The
    // existing message has no hybrid-source enum; expose the speed provenance
    // explicitly in diagnostics rather than inventing a wire enum.
    publisher_.publish(state);
    std_msgs::String diagnostic;
    diagnostic.data=std::string("pose=GPS_IMU speed=Competition.velocity.x caller=")+caller_+
        " valid="+(state.valid?"true":"false")+" pose_speed_mps="+std::to_string(m->speed)+
        " speed_mps="+(speed?std::to_string(*speed):"unavailable");
    diagnostics_.publish(diagnostic);
  }
  ros::NodeHandle nh_,private_;
  ros::Subscriber speed_,pose_;
  ros::Publisher publisher_,diagnostics_;
  std::string caller_;
  planning::CompetitionSpeedAlignment alignment_;
};
int main(int argc,char** argv) {
  ros::init(argc,argv,"planning_competition_speed_state");
  CompetitionSpeedStateNode node;ros::spin();
}
