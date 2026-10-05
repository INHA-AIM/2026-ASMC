#include <asmc_lidar/ground_live.hpp>
#include <asmc_lidar/imu_selection.hpp>
#include <asmc_lidar/unified.hpp>
#include <asmc_lidar/tracking.hpp>
#include <asmc_msgs/LidarObjectArray.h>
#include <asmc_msgs/CameraObjectBoxArray.h>
#include <asmc_msgs/VehicleState.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <iomanip>
#include <sstream>

namespace {
using Clock=std::chrono::steady_clock;
double Ms(Clock::time_point t){return std::chrono::duration<double,std::milli>(Clock::now()-t).count();}
class Node {
 public:
  Node():p_("~") {
    std::string method,calibration,cloud,imu,comparison;
    p_.param<std::string>("comparison_config",comparison,"");
    p_.param<std::string>("method",method,"all");
    p_.param<std::string>("patchwork_variant",variant_,"guard25");
    p_.param<std::string>("calibration_path",calibration,"");
    p_.param<std::string>("cloud_topic",cloud,"/velodyne_points");
    p_.param<std::string>("imu_topic",imu,"/imu");
    p_.param<std::string>("expected_lidar_frame",expected_lidar_frame_,"velodyne");
    p_.param<std::string>("stamp_provenance",stamp_provenance_,"bridge_receive");
    p_.param<bool>("publish_unified",publish_unified_,false);
    // An existing bridge may have been started with a different timestamp mode.
    std::string active_mode;
    if(cloud=="/velodyne_points"&&nh_.getParam("/morai_sensor_bridge/timestamp_mode",active_mode))
      stamp_provenance_=active_mode=="source"?"simulator_source":
                        active_mode=="receive"?"bridge_receive":"unknown_bridge_mode";
    cal_=asmc_lidar::LoadCalibration(calibration);
    model_.reset(new asmc_lidar::GroundComparison(method,variant_,asmc_lidar::LoadGroundComparisonConfig(comparison)));
    for(const auto& name:{"jaeho","patchwork","linefit"}) {
      if(method!="all"&&method!=name)continue;
      for(const auto& kind:{"ground","nonground","uncertain","structural_removed","native_retained"})
        pubs_[std::string(name)+"/"+kind]=nh_.advertise<sensor_msgs::PointCloud2>(
            "/lidar_candidate/ground_live/"+std::string(name)+"/"+kind,1);
    }
    status_=nh_.advertise<diagnostic_msgs::DiagnosticArray>("/lidar_candidate/ground_live/status",10);
    if(publish_unified_){
      unified_points_=nh_.advertise<sensor_msgs::PointCloud2>("/lidar_candidate/unified/nonground",1);
      unified_structure_=nh_.advertise<sensor_msgs::PointCloud2>("/lidar_candidate/unified/structure_evidence",1);
      unified_markers_=nh_.advertise<visualization_msgs::MarkerArray>("/lidar_candidate/unified/markers",1);
      track_markers_=nh_.advertise<visualization_msgs::MarkerArray>("/lidar_candidate/unified/tracks",1);
      unified_status_=nh_.advertise<diagnostic_msgs::DiagnosticArray>("/lidar_candidate/unified/status",10);
      local_objects_=nh_.advertise<asmc_msgs::LidarObjectArray>("/perception/lidar/objects_local",2);
      camera_=nh_.subscribe("/perception/camera/objects",4,&Node::Camera,this);
      vehicle_=nh_.subscribe("/vehicle/state",4,&Node::Vehicle,this);
      ego_=nh_.subscribe("/Ego_topic",4,&Node::Ego,this);
    }
    if(model_->config().leveling_mode=="roll_pitch")
      fit_markers_=nh_.advertise<visualization_msgs::MarkerArray>("/lidar_candidate/ground_live/fit_markers",1);
    cloud_=nh_.subscribe(cloud,1,&Node::Enqueue,this);
    imu_=nh_.subscribe(imu,100,&Node::Imu,this);
    timer_=nh_.createWallTimer(ros::WallDuration(.25),&Node::Timer,this);
    worker_=std::thread(&Node::Work,this);
    ROS_WARN("LiDAR candidate: V4 Jaeho ground; base_link; unified=%s, structure auto suppression OFF, Patchwork comparison is research only",
             publish_unified_?"on":"off");
  }
  ~Node(){
    {std::lock_guard<std::mutex> lock(mutex_);stop_=true;}
    cv_.notify_all();worker_.join();
  }
 private:
  struct Request {
    sensor_msgs::PointCloud2::ConstPtr cloud;
    asmc_lidar::ImuSelection imu;
    asmc_lidar::CameraObservation camera;
    asmc_lidar::EgoTwist ego_twist;
    Clock::time_point arrival;
    std::uint64_t epoch=0,receipt_ns=0;
    std::string error;
  };
  void Imu(const sensor_msgs::Imu::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(mutex_);
    Eigen::Quaterniond q(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z);
    asmc_lidar::ImuSample s;s.stamp_ns=msg->header.stamp.toNSec();
    if(!s.stamp_ns)return;
    const auto norm=q.squaredNorm();s.valid=std::isfinite(norm)&&norm>=.9&&norm<=1.1;
    s.valid=s.valid&&msg->orientation_covariance[0]!=-1;
    if(s.valid){
      const auto r=q.normalized().toRotationMatrix();
      s.roll_rad=std::atan2(r(2,1),r(2,2));
      s.pitch_rad=std::asin(std::max(-1.,std::min(1.,-r(2,0))));
      s.yaw_rad=std::atan2(r(1,0),r(0,0));
      s.yaw_rate_rad_s=msg->angular_velocity.z;
      s.valid=s.valid&&std::isfinite(s.yaw_rate_rad_s);
    }
    history_.push_back(s);while(history_.size()>100)history_.pop_front();
  }
  void Camera(const asmc_msgs::CameraObjectBoxArray::ConstPtr& message){
    std::lock_guard<std::mutex> lock(mutex_);
    asmc_lidar::CameraObservation c;
    c.stamp_ns=message->header.stamp.toNSec();c.frame_id=message->header.frame_id;
    c.valid=message->valid;c.width=message->image_width;c.height=message->image_height;
    for(const auto& box:message->boxes){
      if(std::isfinite(box.xmin)&&std::isfinite(box.ymin)&&std::isfinite(box.xmax)&&
         std::isfinite(box.ymax)&&box.xmax>box.xmin&&box.ymax>box.ymin)
        c.boxes.push_back({box.xmin,box.ymin,box.xmax,box.ymax,box.class_name});
      else c.valid=false;
    }
    latest_camera_=std::move(c);
  }
  void Vehicle(const asmc_msgs::VehicleState::ConstPtr& message){
    std::lock_guard<std::mutex> lock(mutex_);latest_vehicle_=*message;
  }
  void Ego(const morai_msgs::EgoVehicleStatus::ConstPtr& message){
    std::lock_guard<std::mutex> lock(mutex_);latest_ego_=*message;
  }
  void Enqueue(const sensor_msgs::PointCloud2::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(mutex_);
    auto r=std::make_shared<Request>();r->cloud=msg;r->arrival=Clock::now();
    r->receipt_ns=ros::Time::now().toNSec();++received_;
    const auto ns=msg->header.stamp.toNSec();
    if(ns&&last_stamp_&&ns<last_stamp_){
      ++epoch_;++resets_;history_.clear();reset_needed_=true;
      if(pending_){++dropped_;pending_.reset();}
    }
    r->epoch=epoch_;
    if(!ns)r->error="zero_stamp";
    else if(ns==last_stamp_)r->error="duplicate_stamp";
    else {
      last_stamp_=ns;r->imu=asmc_lidar::SelectImu(history_,ns);
      if(r->imu.status!=asmc_lidar::ImuSelectionStatus::Valid)
        r->error=std::string("imu_")+asmc_lidar::ImuSelectionName(r->imu.status);
      r->camera=latest_camera_;
      const auto& v=latest_vehicle_;
      if(v.valid&&v.header.frame_id=="map"&&!v.header.stamp.isZero()&&
         v.header.stamp.toNSec()<=ns&&ns-v.header.stamp.toNSec()<=100000000&&
         std::isfinite(v.speed)&&v.speed>=0){
        r->ego_twist.valid=true;
        r->ego_twist.linear_base.x()=v.speed;
        r->ego_twist.angular_base.z()=r->imu.sample.yaw_rate_rad_s;
      }
      // The GPS/IMU VehicleState estimator is published at 50 Hz but its
      // source stamp can be the older ~5 Hz GPS fix. Use the actual EgoStatus
      // observation as an optional, time-checked speed source when available.
      // Neither EgoStatus nor GPS is required for local detection/tracking.
      const auto& ego=latest_ego_;
      const auto ego_ns=ego.header.stamp.toNSec();
      if(!r->ego_twist.valid&&ego_ns&&ego_ns<=ns&&ns-ego_ns<=100000000&&
         std::isfinite(ego.velocity.x)&&std::isfinite(ego.velocity.y)){
        r->ego_twist.valid=true;
        // MORAI Ego.velocity.x is signed vehicle-forward velocity. The norm
        // would mislabel reverse motion as forward ego motion.
        r->ego_twist.linear_base.x()=ego.velocity.x;
        r->ego_twist.linear_base.y()=ego.velocity.y;
        r->ego_twist.angular_base.z()=r->imu.sample.yaw_rate_rad_s;
      }
    }
    last_arrival_=r->arrival;last_header_=msg->header;stale_=false;
    if(pending_)++dropped_;
    pending_=r;cv_.notify_one();
  }
  void PublishTracked(const std::vector<asmc_lidar::TrackedDetection>& tracks,
                      const std_msgs::Header& header){
    asmc_msgs::LidarObjectArray array;array.header=header;array.header.frame_id="base_link";
    array.status=std::any_of(tracks.begin(),tracks.end(),[](const auto& t){return t.observed;})
                 ?array.OBSERVED:array.OBSERVED_EMPTY;
    array.stamp_provenance=stamp_provenance_;
    visualization_msgs::MarkerArray markers;visualization_msgs::Marker clear;
    clear.header=array.header;clear.action=visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);
    static constexpr int edges[][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},
                                     {0,4},{1,5},{2,6},{3,7}};
    for(const auto& track:tracks){asmc_msgs::LidarObject o;
      o.track_id=track.id;o.detection_index=track.geometry.detection_index;
      o.observation_state=track.observed?o.OBSERVED:o.PREDICTED;
      o.age=track.age;o.miss_count=track.miss_count;
      o.classification=track.classification;o.camera_class=track.camera_class;
      o.center.x=track.geometry.center_base.x();o.center.y=track.geometry.center_base.y();
      o.center.z=track.geometry.center_base.z();o.yaw=track.geometry.yaw_base_rad;
      o.yaw_rate_valid=track.yaw_rate_valid;o.yaw_rate_rad_s=track.yaw_rate_rad_s;
      for(std::size_t i=0;i<8;++i){const auto& p=track.geometry.corners_base[i];
        o.obb_corners[i].x=p.x();o.obb_corners[i].y=p.y();o.obb_corners[i].z=p.z();}
      const auto& c=track.geometry.corners_base;
      o.length=(c[1]-c[0]).norm();o.width=(c[3]-c[0]).norm();
      o.height=(c[4]-c[0]).norm();
      o.relative_velocity_valid=track.relative_velocity_valid;
      o.relative_velocity_mps.x=track.relative_velocity_base.x();
      o.relative_velocity_mps.y=track.relative_velocity_base.y();
      o.absolute_velocity_valid=track.absolute_velocity_valid;
      o.absolute_velocity_mps.x=track.absolute_velocity_base.x();
      o.absolute_velocity_mps.y=track.absolute_velocity_base.y();
      o.dynamic_valid=track.dynamic_valid;o.is_dynamic=track.is_dynamic;
      o.source_point_count=track.observed?track.geometry.source_indices.size():0;
      visualization_msgs::Marker line;line.header=array.header;
      line.ns=track.observed?"observed_tracks":"predicted_tracks";
      line.id=track.id;line.type=visualization_msgs::Marker::LINE_LIST;
      line.action=visualization_msgs::Marker::ADD;line.pose.orientation.w=1;
      line.scale.x=.09;line.color.a=track.observed?1.f:.35f;
      line.color.r=track.observed?.12f:1.f;
      line.color.g=track.observed?1.f:.65f;line.color.b=.15f;
      for(const auto& edge:edges)for(int k:edge){geometry_msgs::Point p;
        p.x=track.geometry.corners_base[k].x();p.y=track.geometry.corners_base[k].y();
        p.z=track.geometry.corners_base[k].z();line.points.push_back(p);}
      markers.markers.push_back(line);
      visualization_msgs::Marker label;label.header=array.header;label.ns="track_ids";
      label.id=track.id;label.type=visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.action=visualization_msgs::Marker::ADD;label.pose.orientation.w=1;
      label.pose.position=o.center;label.pose.position.z+=std::max(.5,o.height/2+.3);
      label.scale.z=.55;label.color.r=1;label.color.g=1;label.color.b=1;label.color.a=1;
      std::ostringstream caption;caption<<(track.observed?"#":"P#")<<track.id<<" "<<o.classification;
      if(o.absolute_velocity_valid)caption<<" "<<std::fixed<<std::setprecision(1)
         <<std::hypot(o.absolute_velocity_mps.x,o.absolute_velocity_mps.y)*3.6<<"km/h";
      else caption<<" v=?";
      label.text=caption.str();markers.markers.push_back(label);
      array.objects.push_back(std::move(o));}
    local_objects_.publish(array);
    track_markers_.publish(markers);
  }
  void PublishInvalid(const std_msgs::Header& header,const std::string& reason){
    if(!publish_unified_)return;
    asmc_msgs::LidarObjectArray message;message.header=header;
    message.header.frame_id="base_link";message.status=message.INPUT_INVALID;
    message.reason=reason;message.stamp_provenance=stamp_provenance_;
    local_objects_.publish(message);
    visualization_msgs::MarkerArray markers;visualization_msgs::Marker clear;
    clear.header=message.header;clear.action=visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);track_markers_.publish(markers);
  }
  sensor_msgs::PointCloud2 Cloud(const asmc_lidar::Cloud& input,
      const std::vector<std::size_t>& ids,const std_msgs::Header& header){
    sensor_msgs::PointCloud2 out;out.header=header;out.header.frame_id="base_link";
    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2Fields(5,"x",1,sensor_msgs::PointField::FLOAT32,
      "y",1,sensor_msgs::PointField::FLOAT32,"z",1,sensor_msgs::PointField::FLOAT32,
      "intensity",1,sensor_msgs::PointField::FLOAT32,"source_index",1,sensor_msgs::PointField::UINT32);
    modifier.resize(ids.size());
    sensor_msgs::PointCloud2Iterator<float> x(out,"x"),y(out,"y"),z(out,"z"),intensity(out,"intensity");
    sensor_msgs::PointCloud2Iterator<std::uint32_t> index(out,"source_index");
    for(auto i:ids){
      const auto& p=input[i];
      // Same as inverse(leveling) then mount, without the float round trip.
      const auto b=cal_.base_from_sensor*Eigen::Vector3d(p.x,p.y,p.z);
      *x=float(b.x());*y=float(b.y());*z=float(b.z());*intensity=p.intensity;*index=i;
      ++x;++y;++z;++intensity;++index;
    }
    out.is_dense=false;return out;
  }
  void Clear(const std_msgs::Header& header){
    const auto empty=Cloud(asmc_lidar::Cloud{}, {},header);
    for(auto& pub:pubs_)pub.second.publish(empty);
    if(publish_unified_){
      unified_points_.publish(empty);unified_structure_.publish(empty);
      visualization_msgs::MarkerArray array;visualization_msgs::Marker clear;
      clear.header=header;clear.header.frame_id="base_link";clear.action=visualization_msgs::Marker::DELETEALL;
      array.markers.push_back(clear);unified_markers_.publish(array);
      track_markers_.publish(array);
    }
    if(fit_markers_) {
      visualization_msgs::MarkerArray array;visualization_msgs::Marker clear;
      clear.header=header;clear.header.frame_id="base_link";clear.action=visualization_msgs::Marker::DELETEALL;
      array.markers.push_back(clear);fit_markers_.publish(array);
    }
  }
  void UnifiedPublish(const asmc_lidar::Cloud& cloud,const asmc_lidar::UnifiedResult& result,
                      const std_msgs::Header& header){
    // Keep review topics advertised, but avoid copying two full point clouds
    // when the runtime runs headless. RViz subscribers receive the next scan.
    if(unified_points_.getNumSubscribers())
      unified_points_.publish(Cloud(cloud,result.retained_raw_indices,header));
    if(unified_structure_.getNumSubscribers())
      unified_structure_.publish(Cloud(cloud,result.structure_candidate_raw_indices,header));
    if(!unified_markers_.getNumSubscribers())return;
    visualization_msgs::MarkerArray array;visualization_msgs::Marker clear;
    clear.header=header;clear.header.frame_id="base_link";clear.action=visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(clear);
    static constexpr int edges[][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},
                                    {0,4},{1,5},{2,6},{3,7}};
    for(const auto& d:result.detections){
      visualization_msgs::Marker m;m.header=header;m.header.frame_id="base_link";
      m.ns="frame_detections_unclassified";m.id=int(d.detection_index);
      m.type=visualization_msgs::Marker::LINE_LIST;m.action=visualization_msgs::Marker::ADD;
      m.pose.orientation.w=1;m.scale.x=.07;
      m.color.r=1;m.color.g=.56;m.color.b=.12;m.color.a=1;
      for(const auto& e:edges)for(int k:e){
        geometry_msgs::Point p;p.x=d.corners_base[k].x();p.y=d.corners_base[k].y();p.z=d.corners_base[k].z();
        m.points.push_back(p);
      }
      array.markers.push_back(m);
    }
    for(std::size_t i=0;i<result.structure_evidence.size();++i){
      const auto& e=result.structure_evidence[i];
      visualization_msgs::Marker m;m.header=header;m.header.frame_id="base_link";
      m.ns="unconfirmed_structure_support";m.id=int(i);
      m.type=visualization_msgs::Marker::CUBE;m.action=visualization_msgs::Marker::ADD;
      m.pose.position.x=(e.min_base.x()+e.max_base.x())/2;
      m.pose.position.y=(e.min_base.y()+e.max_base.y())/2;
      m.pose.position.z=(e.min_base.z()+e.max_base.z())/2;
      m.pose.orientation.w=1;
      m.scale.x=std::max(.05,e.max_base.x()-e.min_base.x());
      m.scale.y=std::max(.05,e.max_base.y()-e.min_base.y());
      m.scale.z=std::max(.05,e.max_base.z()-e.min_base.z());
      m.color.r=.1;m.color.g=.8;m.color.b=1;m.color.a=.18;
      array.markers.push_back(m);
    }
    unified_markers_.publish(array);
  }
  void UnifiedStatus(const std::string& state,const asmc_lidar::UnifiedResult* result){
    if(!publish_unified_)return;
    diagnostic_msgs::DiagnosticArray array;array.header.stamp=ros::Time::now();
    diagnostic_msgs::DiagnosticStatus d;d.name="asmc_lidar/unified";d.hardware_id="Autonav v3";
    d.level=(state=="valid"||state=="observed_empty"||state=="roi_empty")?d.OK:d.WARN;
    d.message=state;
    auto add=[&](const std::string& key,const auto& value){
      diagnostic_msgs::KeyValue kv;kv.key=key;std::ostringstream text;text<<value;kv.value=text.str();
      d.values.push_back(kv);
    };
    add("frame","base_link");add("ground_config_id",model_->config().id);
    add("ground_config_sha256",model_->config().sha256);
    add("observation_stamp_provenance",stamp_provenance_);
    add("structure_auto_suppression",false);add("gps_required",false);
    if(result){
      add("observation_stamp_ns",result->observation_ns);
      add("map_context",result->map_context_status);
      add("camera_context",result->camera_context_status);
      add("frame_detections",result->detections.size());
      add("structure_hypotheses",result->structure_evidence.size());
      add("ground_raw_points",result->ground_raw_points);
      add("retained_raw_points",result->retained_raw_indices.size());
      add("clustering_ms",result->clustering_ms);add("obb_ms",result->obb_ms);
    }
    array.status.push_back(d);unified_status_.publish(array);
  }
  void FitMarkers(const asmc_lidar::GroundLiveResult& result,const std_msgs::Header& header){
    if(!fit_markers_)return;
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker clear;clear.header=header;clear.header.frame_id="base_link";
    clear.action=visualization_msgs::Marker::DELETEALL;array.markers.push_back(clear);
    const auto it=result.methods.find("jaeho");
    if(it!=result.methods.end()){
      Eigen::Matrix3d level;
      for(int i=0;i<3;++i)for(int j=0;j<3;++j)
        level(i,j)=result.prepared.leveling[3*i+j];
      for(const auto& plane:it->second.planes){
        if(!plane.valid||!plane.inlier_points)continue;
        const Eigen::Vector3d center(plane.centroid[0],plane.centroid[1],plane.centroid[2]);
        const Eigen::Vector3d normal(plane.normal[0],plane.normal[1],plane.normal[2]);
        const auto base=cal_.base_from_sensor*(level.transpose()*center);
        const auto direction=cal_.base_from_sensor.linear()*(level.transpose()*normal);
        visualization_msgs::Marker marker;
        marker.header=header;marker.header.frame_id="base_link";marker.ns="ground_plane_attempt";
        marker.id=plane.attempt;marker.type=visualization_msgs::Marker::ARROW;
        marker.action=visualization_msgs::Marker::ADD;marker.pose.orientation.w=1;
        marker.scale.x=.08;marker.scale.y=.18;marker.scale.z=.22;
        marker.color.r=plane.accepted_ground?.1f:1.f;
        marker.color.g=plane.accepted_ground?1.f:.2f;
        marker.color.b=.15f;marker.color.a=.95f;
        geometry_msgs::Point a,b;
        a.x=base.x();a.y=base.y();a.z=base.z();
        b.x=base.x()+direction.x();b.y=base.y()+direction.y();b.z=base.z()+direction.z();
        marker.points={a,b};array.markers.push_back(marker);
      }
    }
    fit_markers_.publish(array);
  }
  void Status(const std::string& state,const Request* r,
      const asmc_lidar::GroundLiveResult* result,double wait,double compute,double output_ms){
    diagnostic_msgs::DiagnosticArray array;array.header.stamp=ros::Time::now();
    diagnostic_msgs::DiagnosticStatus d;d.name="asmc_lidar/ground_live";d.hardware_id="Autonav v3";
    d.level=(state=="valid"||state=="observed_empty"||state=="roi_empty")?d.OK:d.WARN;d.message=state;
    auto add=[&](const std::string& k,const auto& value){
      diagnostic_msgs::KeyValue v;v.key=k;std::ostringstream s;s<<std::setprecision(17)<<value;
      v.value=s.str();d.values.push_back(v);
    };
    add("received",received_);add("processed",processed_);add("invalid",invalid_);
    add("queue_dropped",dropped_);add("time_resets",resets_);
    add("source_stamp_ns",r?r->cloud->header.stamp.toNSec():last_header_.stamp.toNSec());
    add("receipt_ros_ns",r?r->receipt_ns:0);
    add("stamp_provenance",stamp_provenance_);add("frame","base_link");
    add("patchwork_variant",variant_);add("config_id",model_->config().id);add("config_sha256",model_->config().sha256);
    add("leveling_mode",model_->config().leveling_mode);
    add("queue_wait_ms",wait);add("compute_ms",compute);add("publish_ms",output_ms);
    add("receipt_to_publish_ms",r?Ms(r->arrival):0.);
    if(r){add("imu_stamp_ns",r->imu.sample.stamp_ns);add("imu_pitch",r->imu.sample.pitch_rad);
      add("imu_roll",r->imu.sample.roll_rad);add("imu_newer_invalid_seen",r->imu.newer_invalid_seen);}
    if(result){
      add("prepare_ms",result->prepared.prepare_ms);add("voxel_points",result->prepared.voxel->size());
      for(const auto& item:result->methods){
        add(item.first+"_segment_ms",item.second.segment_ms);
        add(item.first+"_copy_ms",item.second.copy_ms);
        add(item.first+"_guard_ms",item.second.guard_ms);
        add(item.first+"_restore_ms",item.second.restore_ms);
        for(const auto& event:item.second.events)add(item.first+"_"+event.first,event.second);
        std::size_t unknown=0,guard_restored=0;
        for(std::size_t i=0;i<item.second.labels.size();++i){
          unknown+=item.second.labels[i]==3;
          if(item.first=="patchwork"&&!item.second.reasons.empty())guard_restored+=(item.second.reasons[i]&2048)!=0;
        }
        add(item.first+"_unclassified_raw_points",unknown);
        if(item.first=="patchwork")add("patchwork_guard_restored_raw_points",guard_restored);
      }
    }
    array.status.push_back(d);status_.publish(array);
  }
  void Work(){
    for(;;){
      std::shared_ptr<Request> r;bool reset;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock,[&]{return stop_||bool(pending_);});if(stop_)return;
        r=std::move(pending_);reset=reset_needed_;reset_needed_=false;
      }
      const double wait=Ms(r->arrival);const auto start=Clock::now();
      asmc_lidar::Cloud cloud;asmc_lidar::GroundLiveResult result;
      asmc_lidar::UnifiedResult unified;
      std::vector<asmc_lidar::TrackedDetection> tracks;std::string state=r->error;
      try{
        if(reset){model_->Reset();tracker_.Reset();}
        if(state.empty()&&Ms(r->arrival)>500)state="processing_stale";
        if(state.empty()){
          if(publish_unified_&&r->cloud->header.frame_id!=expected_lidar_frame_)
            throw std::runtime_error("lidar_frame_mismatch:"+r->cloud->header.frame_id);
          for(const auto& field:{"x","y","z","intensity"})
            if(std::none_of(r->cloud->fields.begin(),r->cloud->fields.end(),[&](const auto& f){
              return f.name==field&&f.datatype==sensor_msgs::PointField::FLOAT32&&f.count==1;}))
              throw std::runtime_error(std::string("missing_float32_field:")+field);
          pcl::fromROSMsg(*r->cloud,cloud);
          result=model_->Process(cloud,cal_,float(r->imu.sample.roll_rad),float(r->imu.sample.pitch_rad));
          state=cloud.empty()?"observed_empty":result.prepared.voxel->empty()?"roi_empty":"valid";
          if(publish_unified_){
            unified=asmc_lidar::BuildUnified(cloud,result,cal_,r->cloud->header.stamp.toNSec());
            tracks=tracker_.Update(unified,r->imu.sample.yaw_rad,r->ego_twist,&r->camera);
          }
        }
      }catch(const std::exception& e){state=std::string("processing_error:")+e.what();}
      const double compute=Ms(start);const auto output=Clock::now();
      std::lock_guard<std::mutex> lock(mutex_);
      if(r->epoch!=epoch_){++dropped_;continue;}
      if(Ms(r->arrival)>500){++invalid_;Clear(r->cloud->header);PublishInvalid(r->cloud->header,"processing_stale");Status("processing_stale",r.get(),nullptr,wait,compute,0);UnifiedStatus("processing_stale",nullptr);continue;}
      if(state!="valid"&&state!="observed_empty"&&state!="roi_empty"){
        ++invalid_;Clear(r->cloud->header);PublishInvalid(r->cloud->header,state);Status(state,r.get(),nullptr,wait,compute,0);UnifiedStatus(state,nullptr);continue;
      }
      ++processed_;
      for(const auto& method:result.methods){
        // The ground-comparison launch still exposes every topic. In the
        // unified headless runtime do not build five redundant clouds if no
        // reviewer/recorder actually subscribes to them.
        bool review_subscriber=false;
        for(const auto& kind:{"ground","nonground","uncertain","structural_removed","native_retained"})
          review_subscriber|=pubs_.at(method.first+"/"+kind).getNumSubscribers()>0;
        if(!review_subscriber)continue;
        std::map<std::string,std::vector<std::size_t>> ids;
        for(const auto& name:{"ground","nonground","uncertain","structural_removed","native_retained"})ids[name]={};
        for(std::size_t i=0;i<cloud.size();++i){
          const auto label=method.second.labels[i];if(!label)continue;
          ids[label==1?"ground":label==2?"nonground":"uncertain"].push_back(i);
          if(method.second.structural_removed[i])ids["structural_removed"].push_back(i);
          else if(label==2)ids["native_retained"].push_back(i);
        }
        for(const auto& item:ids){auto& publisher=pubs_.at(method.first+"/"+item.first);
          if(publisher.getNumSubscribers())publisher.publish(Cloud(cloud,item.second,r->cloud->header));}
      }
      FitMarkers(result,r->cloud->header);
      if(publish_unified_){
        UnifiedPublish(cloud,unified,r->cloud->header);
        PublishTracked(tracks,r->cloud->header);
      }
      Status(state,r.get(),&result,wait,compute,Ms(output));
      UnifiedStatus(state,publish_unified_?&unified:nullptr);
      ROS_INFO_THROTTLE(5.,"Ground comparison rx=%lu processed=%lu invalid=%lu queue_drop=%lu compute=%.2fms",
          received_,processed_,invalid_,dropped_,compute);
    }
  }
  void Timer(const ros::WallTimerEvent&){
    std::lock_guard<std::mutex> lock(mutex_);
    if(last_arrival_!=Clock::time_point{}&&Ms(last_arrival_)<500)return;
    if(!stale_){Clear(last_header_);PublishInvalid(last_header_,"sensor_missing");stale_=true;}
    Status("sensor_missing",nullptr,nullptr,0,0,0);
    UnifiedStatus("sensor_missing",nullptr);
  }
  ros::NodeHandle nh_,p_;ros::Subscriber cloud_,imu_;ros::Publisher status_,fit_markers_;
  ros::Subscriber camera_,vehicle_,ego_;ros::Publisher local_objects_;
  ros::Publisher unified_points_,unified_structure_,unified_markers_,track_markers_,unified_status_;
  ros::WallTimer timer_;std::map<std::string,ros::Publisher> pubs_;
  asmc_lidar::Calibration cal_;std::unique_ptr<asmc_lidar::GroundComparison> model_;
  asmc_lidar::TrackingEngine tracker_;
  asmc_lidar::CameraObservation latest_camera_;
  asmc_msgs::VehicleState latest_vehicle_;
  morai_msgs::EgoVehicleStatus latest_ego_;
  std::deque<asmc_lidar::ImuSample> history_;
  std::mutex mutex_;std::condition_variable cv_;std::thread worker_;
  std::shared_ptr<Request> pending_;bool stop_=false,reset_needed_=false,stale_=false;
  bool publish_unified_=false;
  std::string expected_lidar_frame_="velodyne";
  Clock::time_point last_arrival_;std_msgs::Header last_header_;
  std::uint64_t epoch_=0,last_stamp_=0,received_=0,processed_=0,invalid_=0,dropped_=0,resets_=0;
  std::string variant_,stamp_provenance_;
};
}
int main(int argc,char** argv){
  ros::init(argc,argv,"ground_compare_live");
  try{Node node;ros::spin();return 0;}catch(const std::exception& e){ROS_FATAL("%s",e.what());return 1;}
}
