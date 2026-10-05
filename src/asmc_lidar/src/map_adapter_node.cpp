#include <asmc_bridge/gps_transform.hpp>
#include <frame_time/core.hpp>
#include <asmc_msgs/DetectedObjectArray.h>
#include <asmc_msgs/LidarObjectArray.h>
#include <asmc_msgs/TrackedObjectArray.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/GPSMessage.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <chrono>
#include <string>

namespace {
using asmc::lidar_eval::Ns;
struct Fix {morai_msgs::GPSMessage message;};
struct Attitude {sensor_msgs::Imu message;};
Eigen::Vector3d Vector(const geometry_msgs::Point& p){return {p.x,p.y,p.z};}
geometry_msgs::Point Point(const Eigen::Vector3d& p){geometry_msgs::Point q;q.x=p.x();q.y=p.y();q.z=p.z();return q;}
bool Fresh(Ns sample,Ns scan,Ns max_gap){return sample>0&&scan>=sample&&scan-sample<=max_gap;}
class Node {
 public:
  Node():p_("~"),poses_(100000000){
    p_.param("map_origin_x",origin_x_,302595.);p_.param("map_origin_y",origin_y_,4124145.);
    p_.param("gps_lever_x",lever_x_,.35);p_.param("gps_lever_y",lever_y_,0.);
    p_.param("gps_lever_z",lever_z_,1.25);
    local_=nh_.subscribe("/perception/lidar/objects_local",2,&Node::Local,this);
    gps_=nh_.subscribe("/gps",20,&Node::Gps,this);
    imu_=nh_.subscribe("/imu",100,&Node::Imu,this);
    ego_=nh_.subscribe("/Ego_topic",100,&Node::Ego,this);
    map_=nh_.advertise<asmc_msgs::LidarObjectArray>("/perception/lidar/objects_map",2);
    compat_=nh_.advertise<asmc_msgs::TrackedObjectArray>("/perception/lidar/tracked_objects",2);
    planner_=nh_.advertise<asmc_msgs::DetectedObjectArray>("/perception/objects_lidar_candidate",2);
    diagnostics_=nh_.advertise<diagnostic_msgs::DiagnosticArray>("/perception/lidar/map_status",2);
    markers_=nh_.advertise<visualization_msgs::MarkerArray>("/lidar_candidate/unified/map_tracks",1);
    timer_=nh_.createWallTimer(ros::WallDuration(.02),&Node::Timer,this);
  }
 private:
  void Gps(const morai_msgs::GPSMessage::ConstPtr& m){
    const Ns stamp=m->header.stamp.toNSec();
    if(m->header.stamp.isZero()||m->status<=0||!std::isfinite(m->altitude)||
       !std::isfinite(m->latitude)||!std::isfinite(m->longitude)||
       std::abs(m->latitude)<1e-8||std::abs(m->longitude)<1e-8||
       !std::isfinite(m->eastOffset)||!std::isfinite(m->northOffset)||
       std::abs(m->eastOffset-origin_x_)>1||std::abs(m->northOffset-origin_y_)>1){
      // Invalid GPS is a pose-history barrier, even if a fresh fix follows
      // quickly enough that two Ego samples would otherwise bracket a scan.
      gps_healthy_=false;gps_invalid_stamp_=stamp;fix_.message=morai_msgs::GPSMessage();
      poses_.clear();poses_latest_stamp_=0;last_base_stamp_=0;return;
    }
    gps_healthy_=true;fix_.message=*m;
  }
  void Imu(const sensor_msgs::Imu::ConstPtr& m){
    const auto& q=m->orientation;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if(m->header.stamp.isZero()||!std::isfinite(norm)||norm<.9||norm>1.1||
       m->orientation_covariance[0]==-1||!std::isfinite(m->angular_velocity.z))return;
    attitude_.message=*m;
    const Ns stamp=m->header.stamp.toNSec();
    if(!angular_rates_.empty()&&stamp<=angular_rates_.back().first)angular_rates_.clear();
    angular_rates_.push_back({stamp,m->angular_velocity.z});
    while(!angular_rates_.empty()&&angular_rates_.front().first<stamp-500000000)
      angular_rates_.pop_front();
  }
  void Ego(const morai_msgs::EgoVehicleStatus::ConstPtr& m){
    const Ns stamp=m->header.stamp.toNSec();
    if(last_base_stamp_&&stamp<=last_base_stamp_){
      poses_.clear();poses_latest_stamp_=0;last_base_stamp_=0;
      ROS_WARN_THROTTLE(3,"LiDAR map pose source clock reset; waiting for fresh bracket");
    }
    if(!Fresh(fix_.message.header.stamp.toNSec(),stamp,600000000)){
      ++gps_stale_rejections_;return;
    }
    if(!Fresh(attitude_.message.header.stamp.toNSec(),stamp,100000000)){
      ++imu_stale_rejections_;return;
    }
    if(!std::isfinite(m->position.x)||!std::isfinite(m->position.y)||
       !std::isfinite(m->position.z)){++invalid_ego_rejections_;return;}
    double e,n;const auto& gps=fix_.message;
    GpsToUtm52N(gps.latitude,gps.longitude,e,n);
    const auto& q=attitude_.message.orientation;
    const Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
    const auto r=rotation.normalized().toRotationMatrix();
    const Eigen::Vector3d lever(lever_x_,lever_y_,lever_z_);
    // MORAI offsets are already the HD-map origin. Subtract exactly once.
    const Eigen::Vector3d gps_rear(e-gps.eastOffset,n-gps.northOffset,gps.altitude);
    const Eigen::Vector3d base_gps=gps_rear-r*lever;
    // GPS is sampled at ~5 Hz while Ego is ~50 Hz. Check the actual age of
    // the GPS fix rather than comparing old GPS XY with current Ego XY at a
    // fixed 1 m threshold (which rejects normal high-speed driving).
    const double age_s=(stamp-gps.header.stamp.toNSec())*1e-9;
    const double speed=std::hypot(m->velocity.x,m->velocity.y);
    if(!std::isfinite(speed)||
       std::hypot(m->position.x-base_gps.x(),m->position.y-base_gps.y())>
           1.+std::min(30.,1.5*speed*age_s)||
       std::abs(m->position.z-base_gps.z())>1.5){
      ++position_mismatch_rejections_;return;
    }
    asmc::lidar_eval::PoseSample sample;
    sample.stamp_ns=stamp;sample.arrival_ns=ros::Time::now().toNSec();
    sample.clock="ros";sample.valid=true;
    sample.map_from_reference=Eigen::Isometry3d::Identity();
    sample.map_from_reference.linear()=r;
    // Ego provides high-rate 3D vehicle pose, GPS validates map origin and
    // antenna/height datum. /vehicle/state alone is only 2D and is not used.
    sample.map_from_reference.translation()=Eigen::Vector3d(m->position.x,m->position.y,m->position.z);
    if(last_base_stamp_&&
       (stamp-last_base_stamp_>1000000000LL||
        (sample.map_from_reference.translation()-last_base_position_).head<2>().norm()>55.*(stamp-last_base_stamp_)*1e-9+2.)){
      sample.valid=false;ROS_WARN_THROTTLE(2,"LiDAR map pose jump; map unavailable across discontinuity");
    }
    last_base_stamp_=stamp;last_base_position_=sample.map_from_reference.translation();
    const auto reason=poses_.append(sample);
    if(!reason.empty()){
      ++pose_append_rejected_;
      ROS_WARN_THROTTLE(3,"LiDAR map pose ignored: %s",reason.c_str());
    }
    else {++pose_accepted_;poses_latest_stamp_=stamp;poses_.trim_before(stamp-500000000);}
    TryProcess();
  }
  void Local(const asmc_msgs::LidarObjectArray::ConstPtr& message){
    // A high-rate Ego pose stamped after the cloud may arrive a few ms later.
    // Wait briefly for a causal bracket; never extrapolate or restamp the cloud.
    pending_.push_back({*message,std::chrono::steady_clock::now()});
    TryProcess();
  }
  void Timer(const ros::WallTimerEvent&){
    TryProcess();
    diagnostic_msgs::DiagnosticArray array;array.header.stamp=ros::Time::now();
    diagnostic_msgs::DiagnosticStatus state;state.name="asmc_lidar/map_pose";
    state.hardware_id="Autonav v3";state.level=diagnostic_msgs::DiagnosticStatus::OK;
    state.message=gps_healthy_?"gps_available":"gps_invalid_or_missing";
    auto add=[&](const std::string& key,std::uint64_t value){
      diagnostic_msgs::KeyValue entry;entry.key=key;entry.value=std::to_string(value);
      state.values.push_back(entry);
    };
    add("pose_accepted",pose_accepted_);
    add("pose_append_rejected",pose_append_rejected_);
    add("gps_stale_rejections",gps_stale_rejections_);
    add("imu_stale_rejections",imu_stale_rejections_);
    add("position_mismatch_rejections",position_mismatch_rejections_);
    add("invalid_ego_rejections",invalid_ego_rejections_);
    add("latest_pose_stamp_ns",poses_latest_stamp_);
    array.status.push_back(state);diagnostics_.publish(array);
  }
  void TryProcess(){
    while(!pending_.empty()){
      const auto& front=pending_.front();
      const auto age=std::chrono::duration<double>(std::chrono::steady_clock::now()-front.second).count();
      const auto newest=front.first.header.stamp.toNSec();
      if(age<.12 && (poses_latest_stamp_<=newest || poses_latest_stamp_-newest>100000000))return;
      Process(front.first);
      poses_.trim_before(front.first.header.stamp.toNSec()-500000000);
      pending_.pop_front();
    }
  }
  void Process(const asmc_msgs::LidarObjectArray& input){
    const auto* message=&input;
    asmc_msgs::LidarObjectArray output;output.header=message->header;output.header.frame_id="map";
    output.stamp_provenance=message->stamp_provenance;
    if(message->header.frame_id!="base_link"||message->header.stamp.isZero()||
       (message->status!=message->OBSERVED&&message->status!=message->OBSERVED_EMPTY)){
      output.status=output.INPUT_INVALID;output.reason="local_input_invalid:"+message->reason;
      map_.publish(output);ClearMarkers(output.header);return;
    }
    asmc::lidar_eval::Observation observation;
    observation.stamp_ns=message->header.stamp.toNSec();
    observation.processing_ns=ros::Time::now().toNSec();
    observation.receipt_ns=observation.processing_ns;observation.clock="ros";
    observation.stamp_provenance=message->stamp_provenance;
    if(!gps_healthy_&&gps_invalid_stamp_&&observation.stamp_ns>=gps_invalid_stamp_){
      output.status=output.POSE_UNAVAILABLE;output.reason="gps_invalid";
      map_.publish(output);ClearMarkers(output.header);return;
    }
    if(!Fresh(fix_.message.header.stamp.toNSec(),observation.stamp_ns,600000000)){
      output.status=output.POSE_UNAVAILABLE;output.reason="gps_missing_or_stale";
      map_.publish(output);ClearMarkers(output.header);return;
    }
    const auto pose=poses_.at(observation);
    if(!pose.valid){output.status=output.POSE_UNAVAILABLE;output.reason=pose.reason;
      map_.publish(output);ClearMarkers(output.header);return;}
    output.status=message->status;output.reason=message->reason;
    asmc::lidar_eval::Context context;context.time=observation;
    context.frame=asmc::lidar_eval::Frame::Base;context.map_pose=pose;
    for(const auto& source:message->objects){asmc_msgs::LidarObject o=source;
      asmc::lidar_eval::Object object;
      object.id=source.track_id;object.classification=source.classification;
      object.center=Vector(source.center);
      object.long_axis=Eigen::Vector3d(std::cos(source.yaw),std::sin(source.yaw),0);
      for(int i=0;i<8;++i)object.corners[i]=Vector(source.obb_corners[i]);
      if(source.absolute_velocity_valid){
        object.velocity_meaning=asmc::lidar_eval::VelocityMeaning::AbsoluteInputVector;
        object.velocity=Eigen::Vector3d(source.absolute_velocity_mps.x,
                         source.absolute_velocity_mps.y,source.absolute_velocity_mps.z);
      }
      const auto converted=asmc::lidar_eval::transform(object,context);
      if(!converted.valid||!converted.map||!converted.map->yaw||
         (source.absolute_velocity_valid&&!converted.absolute_velocity_map)){
        output.objects.clear();output.status=output.INPUT_INVALID;
        output.reason="invalid_object_geometry_or_velocity";
        map_.publish(output);ClearMarkers(output.header);return;
      }
      o.center=Point(converted.map->center);
      for(int i=0;i<8;++i)o.obb_corners[i]=Point(converted.map->corners[i]);
      o.yaw=*converted.map->yaw;
      if(converted.absolute_velocity_map){const auto& v=*converted.absolute_velocity_map;
        o.absolute_velocity_mps.x=v.x();o.absolute_velocity_mps.y=v.y();o.absolute_velocity_mps.z=v.z();}
      if(o.relative_velocity_valid){const auto v=pose.map_from_base.rotation()*Eigen::Vector3d(
         source.relative_velocity_mps.x,source.relative_velocity_mps.y,source.relative_velocity_mps.z);
        o.relative_velocity_mps.x=v.x();o.relative_velocity_mps.y=v.y();o.relative_velocity_mps.z=v.z();}
      output.objects.push_back(std::move(o));}
    map_.publish(output);
    MapMarkers(output);
    asmc_msgs::TrackedObjectArray tracked;tracked.header=output.header;
    asmc_msgs::DetectedObjectArray detected;detected.header=output.header;
    bool publish_legacy=false;
    double observed_yaw_rate=0;
    for(auto it=angular_rates_.rbegin();it!=angular_rates_.rend();++it)
      if(Fresh(it->first,observation.stamp_ns,100000000)){
        observed_yaw_rate=it->second;publish_legacy=true;break;
      }
    for(const auto& o:output.objects){
      if(o.observation_state!=o.OBSERVED)continue;
      if(!o.absolute_velocity_valid||!o.dynamic_valid||!o.yaw_rate_valid||
         o.length<=.05||o.width<=.05||o.height<=.05){publish_legacy=false;break;}
      asmc_msgs::TrackedObject tr;tr.id=o.track_id;tr.classification=o.classification;
      tr.yolo_class=o.camera_class;tr.is_dynamic=o.is_dynamic;tr.age=o.age;
      tr.miss_count=o.miss_count;tr.cluster_center=o.center;tr.obb_center=o.center;
      tr.obb_corners=o.obb_corners;tr.obb_yaw=o.yaw;tr.velocity_mps=o.absolute_velocity_mps;
      tr.speed_kph=std::hypot(o.absolute_velocity_mps.x,o.absolute_velocity_mps.y)*3.6;
      tr.length=o.length;tr.width=o.width;tr.height=o.height;
      tr.type=o.classification=="person"?1:o.classification=="car"?2:3;
      tracked.objects.push_back(tr);
      asmc_msgs::DetectedObject d;d.id=o.track_id;d.type=tr.type;d.center=o.center;
      d.yaw=o.yaw;d.vx=o.absolute_velocity_mps.x;d.vy=o.absolute_velocity_mps.y;
      d.yaw_rate=o.yaw_rate_rad_s+observed_yaw_rate;
      d.width=o.width;d.length=o.length;d.height=o.height;d.obb_corners=o.obb_corners;
      detected.objects.push_back(d);
    }
    if(publish_legacy){compat_.publish(tracked);planner_.publish(detected);}
  }
  void ClearMarkers(const std_msgs::Header& header){
    visualization_msgs::MarkerArray result;visualization_msgs::Marker clear;
    clear.header=header;clear.action=visualization_msgs::Marker::DELETEALL;
    result.markers.push_back(clear);markers_.publish(result);
  }
  void MapMarkers(const asmc_msgs::LidarObjectArray& output){
    visualization_msgs::MarkerArray result;visualization_msgs::Marker clear;
    clear.header=output.header;clear.action=visualization_msgs::Marker::DELETEALL;
    result.markers.push_back(clear);
    static constexpr int edges[][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},
                                    {0,4},{1,5},{2,6},{3,7}};
    for(const auto& object:output.objects){
      visualization_msgs::Marker line;line.header=output.header;
      line.ns=object.observation_state==object.OBSERVED?"map_observed":"map_predicted";
      line.id=object.track_id;line.type=visualization_msgs::Marker::LINE_LIST;
      line.action=visualization_msgs::Marker::ADD;line.pose.orientation.w=1;
      line.scale.x=.09;line.lifetime=ros::Duration(.5);
      line.color.r=object.observation_state==object.OBSERVED?.15f:1.f;
      line.color.g=object.observation_state==object.OBSERVED?1.f:.6f;
      line.color.b=.15f;line.color.a=object.observation_state==object.OBSERVED?1.f:.4f;
      for(const auto& edge:edges){line.points.push_back(object.obb_corners[edge[0]]);
                                  line.points.push_back(object.obb_corners[edge[1]]);}
      result.markers.push_back(line);
      visualization_msgs::Marker label;label.header=output.header;label.ns="map_track_ids";
      label.id=object.track_id;label.type=visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.action=visualization_msgs::Marker::ADD;label.pose.position=object.center;
      label.pose.position.z+=std::max(.5,object.height/2+.3);label.pose.orientation.w=1;
      label.scale.z=.55;label.color.r=1;label.color.g=1;label.color.b=1;label.color.a=1;
      label.lifetime=ros::Duration(.5);
      label.text=(object.observation_state==object.OBSERVED?"#":"P#")+
                 std::to_string(object.track_id);result.markers.push_back(label);
    }
    markers_.publish(result);
  }
  ros::NodeHandle nh_,p_;
  ros::Subscriber local_,gps_,imu_,ego_;
  ros::Publisher map_,compat_,planner_,diagnostics_,markers_;
  ros::WallTimer timer_;
  asmc::lidar_eval::PoseHistory poses_;
  std::deque<std::pair<Ns,double>> angular_rates_;
  std::deque<std::pair<asmc_msgs::LidarObjectArray,std::chrono::steady_clock::time_point>> pending_;
  Ns poses_latest_stamp_=0;
  Ns last_base_stamp_=0;
  Eigen::Vector3d last_base_position_=Eigen::Vector3d::Zero();
  Fix fix_;Attitude attitude_;
  bool gps_healthy_=false;Ns gps_invalid_stamp_=0;
  std::uint64_t pose_accepted_=0,gps_stale_rejections_=0,imu_stale_rejections_=0;
  std::uint64_t position_mismatch_rejections_=0,invalid_ego_rejections_=0;
  std::uint64_t pose_append_rejected_=0;
  double origin_x_,origin_y_,lever_x_,lever_y_,lever_z_;
};
}
int main(int argc,char** argv){ros::init(argc,argv,"asmc_lidar_map_adapter");Node node;ros::spin();}
