#include <asmc_lidar/core.hpp>
#include <asmc_lidar/imu_selection.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>
#include <chrono>
#include <deque>
#include <cmath>
#include <memory>

namespace {
class Node {
 public:
  Node():nh_(),private_("~") {
    std::string mode="reference",calibration,reference;
    private_.param("mode",mode,mode);
    private_.param("calibration_path",calibration,std::string());
    private_.param("reference_path",reference,std::string());
    if(mode!="reference"&&mode!="ground_review")throw std::runtime_error("mode must be reference or ground_review");
    std::string profile="tunnel";private_.param("profile",profile,profile);
    if(profile!="tunnel"&&profile!="general")throw std::runtime_error("profile must be tunnel or general");
    profile_=profile=="tunnel"?asmc_lidar::ReferenceProfile::Tunnel:asmc_lidar::ReferenceProfile::General;
    if(calibration.empty()||reference.empty())throw std::runtime_error("calibration/reference path missing");
    processor_.reset(new asmc_lidar::Processor(mode=="reference"?asmc_lidar::Mode::Reference:asmc_lidar::Mode::GroundReview,
        asmc_lidar::LoadCalibration(calibration),asmc_lidar::LoadReferenceParams(reference)));
    std::string cloud="/velodyne_points",imu="/imu";
    private_.param("cloud_topic",cloud,cloud);private_.param("imu_topic",imu,imu);
    input_pub_=nh_.advertise<sensor_msgs::PointCloud2>("/lidar_candidate/input_base",1);
    ground_pub_=nh_.advertise<sensor_msgs::PointCloud2>("/lidar_candidate/ground",1);
    nonground_pub_=nh_.advertise<sensor_msgs::PointCloud2>("/lidar_candidate/nonground",1);
    uncertain_pub_=nh_.advertise<sensor_msgs::PointCloud2>("/lidar_candidate/uncertain",1);
    marker_pub_=nh_.advertise<visualization_msgs::MarkerArray>("/lidar_candidate/reference_obbs",1);
    diagnostic_pub_=nh_.advertise<diagnostic_msgs::DiagnosticArray>("/lidar_candidate/status",1);
    imu_sub_=nh_.subscribe(imu,64,&Node::OnImu,this);
    cloud_sub_=nh_.subscribe(cloud,2,&Node::OnCloud,this);
    timer_=nh_.createTimer(ros::Duration(1.0),&Node::OnTimer,this);
    ROS_WARN("LiDAR candidate REVIEW ONLY: reference and Patchwork++ outputs are not planner inputs");
  }
 private:
  void OnImu(const sensor_msgs::Imu::ConstPtr& msg) {
    if(msg->header.stamp.isZero()) {++imu_zero_stamp_count_;return;}
    const auto& q=msg->orientation;
    Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
    const double norm=rotation.squaredNorm();
    asmc_lidar::ImuSample sample;
    sample.stamp_ns=msg->header.stamp.toNSec();
    sample.valid=std::isfinite(norm)&&norm>=.9&&norm<=1.1;
    if(sample.valid) {
      const auto r=rotation.normalized().toRotationMatrix();
      sample.roll_rad=std::atan2(r(2,1),r(2,2));
      sample.pitch_rad=std::asin(std::max(-1.,std::min(1.,-r(2,0))));
    }
    imu_.push_back(sample);while(imu_.size()>100)imu_.pop_front();
  }
  void Cloud(ros::Publisher& pub,const asmc_lidar::Cloud& cloud,const std_msgs::Header& h) {
    sensor_msgs::PointCloud2 msg;pcl::toROSMsg(cloud,msg);msg.header=h;msg.header.frame_id="base_link";pub.publish(msg);
  }
  void Diagnostics(const std::string& code,const std_msgs::Header& header,
                   std::size_t raw,std::size_t roi,std::size_t voxel,double elapsed) {
    diagnostic_msgs::DiagnosticArray message;message.header.stamp=ros::Time::now();
    diagnostic_msgs::DiagnosticStatus item;item.name="asmc_lidar_candidate";item.hardware_id="Autonav v3";
    item.level=(code=="valid"&&!last_imu_newer_invalid_)?item.OK:item.WARN;
    item.message=code=="valid"&&last_imu_newer_invalid_?
        "valid_with_newer_invalid_imu":code;
    auto field=[&](std::string key,std::string value) {
      diagnostic_msgs::KeyValue kv;kv.key=std::move(key);kv.value=std::move(value);item.values.push_back(kv);
    };
    field("source_stamp_ns",std::to_string(header.stamp.toNSec()));
    field("receipt_ros_ns",std::to_string(last_message_.toNSec()));
    field("stamp_provenance","bridge_receive_not_beam_time");
    field("imu_selection",last_imu_selection_);
    field("imu_newer_invalid_seen",last_imu_newer_invalid_?"true":"false");
    field("imu_zero_stamp_count",std::to_string(imu_zero_stamp_count_));
    field("frame","base_link");field("raw_points",std::to_string(raw));
    field("roi_points",std::to_string(roi));field("voxel_points",std::to_string(voxel));
    field("camera","missing");field("map_pose","unavailable_in_L2_A1");
    field("processing_ms",std::to_string(elapsed));
    message.status.push_back(item);diagnostic_pub_.publish(message);
  }
  void ClearMarkers(const std_msgs::Header& h,visualization_msgs::MarkerArray* out) {
    visualization_msgs::Marker clear;clear.header=h;clear.header.frame_id="base_link";
    clear.action=visualization_msgs::Marker::DELETEALL;out->markers.push_back(clear);
  }
  void OnCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    last_message_=ros::Time::now();
    const auto begin=std::chrono::steady_clock::now();
    asmc_lidar::FrameInput input;input.stamp_ns=msg->header.stamp.toNSec();input.profile=profile_;
    input.receipt_ns=last_message_.toNSec();
    asmc_lidar::Cloud::Ptr raw(new asmc_lidar::Cloud);
    pcl::fromROSMsg(*msg,*raw);input.cloud=raw;
    const auto selected=asmc_lidar::SelectImu(imu_,std::uint64_t(input.stamp_ns));
    last_imu_selection_=asmc_lidar::ImuSelectionName(selected.status);
    last_imu_newer_invalid_=selected.newer_invalid_seen;
    if(selected.status==asmc_lidar::ImuSelectionStatus::Valid) {
      input.roll_rad=selected.sample.roll_rad;
      input.pitch_rad=selected.sample.pitch_rad;
      input.imu_valid=true;
    }
    asmc_lidar::FrameOutput output;
    try{output=processor_->Process(input);}
    catch(const std::exception& e){output.status=std::string("processing_error:")+e.what();ROS_ERROR_THROTTLE(2.0,"%s",e.what());}
    std_msgs::Header h=msg->header;h.frame_id="base_link";
    visualization_msgs::MarkerArray markers;ClearMarkers(h,&markers);
    if(output.status=="valid") {
      Cloud(input_pub_,*output.input_base,h);
      Cloud(ground_pub_,*output.ground_base,h);
      Cloud(nonground_pub_,*output.nonground_base,h);
      Cloud(uncertain_pub_,*output.uncertain_base,h);
      constexpr int edge[][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
      int id=0;
      for(const auto& obj:output.reference_objects_base) {
        visualization_msgs::Marker marker;marker.header=h;marker.ns="frame_obbs";marker.id=id++;
        marker.type=visualization_msgs::Marker::LINE_LIST;marker.action=visualization_msgs::Marker::ADD;
        marker.pose.orientation.w=1;marker.scale.x=.06;
        marker.color.r=1;marker.color.g=.3;marker.color.b=.9;marker.color.a=1;
        marker.lifetime=ros::Duration(.5);
        for(const auto& pair:edge)for(int index:{pair[0],pair[1]}) {
          geometry_msgs::Point p;p.x=obj.obb.corners[index].x;p.y=obj.obb.corners[index].y;
          p.z=obj.obb.corners[index].z;marker.points.push_back(p);
        }
        markers.markers.push_back(marker);
      }
    }
    marker_pub_.publish(markers);
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    Diagnostics(output.status,h,output.raw_points,output.roi_points,output.voxel_points,ms);
  }
  void OnTimer(const ros::TimerEvent&) {
    if(!last_message_.isZero()&&(ros::Time::now()-last_message_).toSec()<1.0)return;
    std_msgs::Header h;h.stamp=last_message_;
    Diagnostics("sensor_missing",h,0,0,0,0);
  }
  ros::NodeHandle nh_,private_;
  ros::Subscriber cloud_sub_,imu_sub_;
  ros::Publisher input_pub_,ground_pub_,nonground_pub_,uncertain_pub_,marker_pub_,diagnostic_pub_;
  ros::Timer timer_;
  ros::Time last_message_;
  std::deque<asmc_lidar::ImuSample> imu_;
  std::string last_imu_selection_="missing";
  bool last_imu_newer_invalid_=false;
  std::size_t imu_zero_stamp_count_=0;
  asmc_lidar::ReferenceProfile profile_=asmc_lidar::ReferenceProfile::Tunnel;
  std::unique_ptr<asmc_lidar::Processor> processor_;
};
}  // namespace
int main(int argc,char** argv) {
  ros::init(argc,argv,"asmc_lidar_review");
  try{Node node;ros::spin();return 0;}
  catch(const std::exception& e){ROS_FATAL("%s",e.what());return 1;}
}
