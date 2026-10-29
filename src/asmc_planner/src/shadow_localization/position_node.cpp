#include "shadow_localization/static_planner.hpp"
#include "shadow_localization/local_recovery.hpp"
#include <set>
#include "shadow_localization/paint_window.hpp"
#include "Planning/JammingDetector.hpp"
#include "shadow_localization/input_alignment.hpp"
#include "shadow_localization/input_epoch.hpp"
#include "shadow_localization/gps_mount.hpp"
#include "local/journal.hpp"
#include <asmc_msgs/LidarTrackingObservation.h>
#include <asmc_msgs/LocalTrajectory.h>
#include <asmc_geometry/geodesy.hpp>
#include <morai_msgs/GPSMessage.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <iomanip>
#include <memory>
#include <sstream>

namespace shadow_localization {
using namespace local_planning;
class PositionNode {
  ros::NodeHandle nh,p{"~"};
  ros::Subscriber state_sub,gps_sub,imu_sub,objects_sub,mode_sub,prepare_sub,local_recovery_sub;
  bool local_recovery_enabled=false;
  LocalRecoveryPolicy local_recovery;
  asmc_msgs::LocalTrajectory::ConstPtr local_candidate;
  std::set<std::string> retired_local_frames;
  std::string local_candidate_frame;
  uint64_t local_candidate_sequence=0;
  double local_candidate_received=0;
  void on_local_recovery(const asmc_msgs::LocalTrajectory::ConstPtr& m){
    if(m->run_id!=run||retired_local_frames.count(m->header.frame_id))return;
    if(m->header.frame_id!=local_candidate_frame){
      if(!local_candidate_frame.empty())retired_local_frames.insert(local_candidate_frame);
      // Bound restart tracking by refusing new sources until this node restarts.
      if(retired_local_frames.size()>64)return;
      local_candidate_frame=m->header.frame_id;local_candidate_sequence=0;
      local_candidate.reset();local_recovery.reset();
    }
    if(m->plan_sequence<=local_candidate_sequence)return;
    local_candidate_sequence=m->plan_sequence;local_candidate=m;
    local_candidate_received=ros::SteadyTime::now().toSec();
  }
  ros::Publisher trajectory_pub,path_pub,trace_pub,recovery_ready_pub,anchor_ready_pub,debug_pub;
  double last_debug_=-1;
  std::string trial_id;
  ros::SteadyTimer timer;
  InputAlignment alignment;
  InputEpoch objects_identity;
  Anchor anchor;
  std::unique_ptr<StaticPlanner> planner;
  std::unique_ptr<Journal> journal;
  Config config;
  nav_msgs::Odometry::ConstPtr state;
  struct GpsPending { morai_msgs::GPSMessage::ConstPtr value; double received; };
  std::deque<GpsPending> gps;
  asmc_msgs::LidarTrackingObservation::ConstPtr pending_objects;
  double pending_received=0,gps_offset_x=.35,gps_offset_y=0,gps_offset_z=1.25;
  std::vector<Obstacle> world_objects;
  std::string frame,run,input_reason="initializing",objects_reason="missing",gps_reason="missing";
  double state_received=0,objects_received=0,object_stamp=0,last_gps_stamp=0,imu_received=0;
  bool objects_valid=false,attitude_valid=false;
  bool mode_gated=false,selected=false,preparing=false,static_avoidance=false;
  bool shadow_lane_rules=false,dynamic_obstacles=false,paint_exempt=false;
  std::unique_ptr<asmc_planner::ShadowZone> paint_zone;
  PaintWindow paint_window;
  uint64_t sequence=0,last_state_stamp=0,rejected_inputs=0,obb_transforms=0;
  unsigned alignment_epoch=0;
  void epoch_changed() {
    paint_window.reset();paint_exempt=false;
    if(alignment_epoch==alignment.epoch)return;
    local_candidate.reset();local_recovery.reset();
    const bool previous=alignment_epoch!=0;alignment_epoch=alignment.epoch;
    if(previous)anchor.invalidate("motion_epoch_changed");else anchor.reset();
    gps.clear();pending_objects.reset();world_objects.clear();objects_valid=false;
    last_state_stamp=0;last_gps_stamp=0;planner->reset_reference();
  }
  void drain_gps() {
    const double steady=ros::SteadyTime::now().toSec();
    while(!gps.empty()) {
      const auto& pending=gps.front();const auto& g=pending.value;const double t=g->header.stamp.toSec();
      auto paired=alignment.pose_at(t),attitude=alignment.attitude_at(t);
      if(!paired || !attitude) {
        if(steady-pending.received<=.25)break;
        gps_reason="gps_pair_deadline";++rejected_inputs;gps.pop_front();continue;
      }
      double x,y;asmc::geometry::GpsToUtm52N(g->latitude,g->longitude,x,y);
      const double yaw=attitude->yaw+asmc::geometry::Utm52NYawOffset(g->latitude,g->longitude);
      const auto lever=GpsMountXY(gps_offset_x,gps_offset_y,gps_offset_z,
                                 attitude->roll,attitude->pitch,yaw);
      x-=g->eastOffset+lever[0];
      y-=g->northOffset+lever[1];
      const auto generation=anchor.generation;
      anchor.observe(*paired,x,y,yaw);gps_reason=anchor.reason;
      if(generation!=anchor.generation)planner->reset_reference();
      gps.pop_front();
    }
  }
  void on_state(const nav_msgs::Odometry::ConstPtr& m) {
    if(m->child_frame_id!="base_link") {input_reason="motion_frame_invalid";++rejected_inputs;return;}
    const auto& q=m->pose.pose.orientation;const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if(!std::isfinite(norm)||std::abs(norm-1)>.05) {input_reason="motion_quaternion_invalid";++rejected_inputs;return;}
    Motion v;v.t=m->header.stamp.toSec();v.x=m->pose.pose.position.x;v.y=m->pose.pose.position.y;
    v.yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    v.v=m->twist.twist.linear.x;v.w=m->twist.twist.angular.z;
    const bool accepted=alignment.pose(m->header.frame_id,v);epoch_changed();
    input_reason=alignment.reason;
    if(!accepted) {++rejected_inputs;return;}
    frame=m->header.frame_id;state=m;state_received=ros::SteadyTime::now().toSec();
    anchor.motion(v);drain_gps();process_objects();
  }
  void on_gps(const morai_msgs::GPSMessage::ConstPtr& m) {
    const double t=m->header.stamp.toSec(),age=ros::Time::now().toSec()-t;
    if(!asmc::geometry::ValidGpsFix(m->status,!m->header.stamp.isZero(),m->latitude,m->longitude,m->eastOffset,m->northOffset)) {
      gps_reason="gps_fix_invalid";return;
    }
    if(t<=last_gps_stamp || age<-.05 || age>.3) {gps_reason="gps_time_rejected";++rejected_inputs;return;}
    last_gps_stamp=t;gps.push_back({m,ros::SteadyTime::now().toSec()});
    while(gps.size()>32) {gps.pop_front();++rejected_inputs;}
    drain_gps();
  }
  void on_imu(const sensor_msgs::Imu::ConstPtr& m) {
    const auto& q=m->orientation;const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    const double age=(ros::Time::now()-m->header.stamp).toSec();
    if(m->header.frame_id!="imu"||!std::isfinite(norm)||std::abs(norm-1)>.05||m->orientation_covariance[0]<0||age<-.05||age>.3) {
      attitude_valid=false;input_reason="imu_invalid_or_stale";++rejected_inputs;return;
    }
    Motion a;a.t=m->header.stamp.toSec();
    a.yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    a.roll=std::atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y));
    a.pitch=std::asin(std::clamp(2*(q.w*q.y-q.z*q.x),-1.,1.));
    if(std::abs(a.pitch)>.5) {attitude_valid=false;input_reason="imu_tilt_invalid";return;}
    a.w=(std::sin(a.roll)*m->angular_velocity.y+std::cos(a.roll)*m->angular_velocity.z)/std::cos(a.pitch);
    const bool accepted=alignment.imu(a);epoch_changed();
    if(!accepted) {input_reason=alignment.reason;++rejected_inputs;return;}
    attitude_valid=true;imu_received=ros::SteadyTime::now().toSec();
    drain_gps();process_objects();
  }
  void on_objects(const asmc_msgs::LidarTrackingObservation::ConstPtr& m) {
    const auto& o=m->observation;const auto& a=o.objects;const double t=a.header.stamp.toSec();
    const double age=ros::Time::now().toSec()-t;
    // Bad source times must not advance the accepted producer sequence.
    if(age<-.05||age>.3) {objects_reason="scan_time_rejected";++rejected_inputs;return;}
    const auto identity=objects_identity.accept(o.producer_session_id,o.generation,o.scan_sequence,o.config_id,o.calibration_id,t);
    if(identity==InputEpoch::Result::rejected) {++rejected_inputs;objects_reason=objects_identity.reason;return;}
    if(identity==InputEpoch::Result::changed) {
      anchor.invalidate("lidar_identity_changed");planner->reset_reference();
      world_objects.clear();objects_valid=false;pending_objects.reset();
    }
    if(a.header.frame_id!="base_link"||
       (a.status!=a.OBSERVED && a.status!=a.OBSERVED_EMPTY)||
       (a.status==a.OBSERVED_EMPTY && !a.objects.empty())||
       (a.status==a.OBSERVED && a.objects.empty())) {
      objects_valid=false;world_objects.clear();pending_objects.reset();
      objects_reason="invalid_scan_contract";++rejected_inputs;return;
    }
    if(identity==InputEpoch::Result::duplicate)return;
    pending_objects=m;pending_received=ros::SteadyTime::now().toSec();process_objects();
  }
  void process_objects() {
    if(!pending_objects)return;
    const auto& a=pending_objects->observation.objects;const double t=a.header.stamp.toSec();
    auto motion=alignment.paired(t);
    if(!motion) {
      if(ros::SteadyTime::now().toSec()-pending_received>.25) {
        pending_objects.reset();objects_valid=false;objects_reason="scan_pair_deadline";++rejected_inputs;
      }
      return;
    }
    std::vector<Obstacle> next;const auto rotation=AttitudeRotation(motion->roll,motion->pitch);
    const double cs=std::cos(motion->yaw),sn=std::sin(motion->yaw);
    auto reject=[&](const char* reason){pending_objects.reset();objects_valid=false;world_objects.clear();objects_reason=reason;++rejected_inputs;};
    for(const auto& o:a.objects) {
      // Invalid/predicted geometry cannot silently become observed free space.
      if(o.observation_state!=o.OBSERVED) {reject("object_not_observed");return;}
      Obstacle ob;ob.id=o.track_id;ob.stamp=t;ob.camera_class=o.camera_class;ob.classification=o.classification;
      auto& observed=ob.observation;
      observed.id=o.track_id;observed.classification=o.classification;observed.camera_class=o.camera_class;
      observed.width=o.width;observed.length=o.length;observed.height=o.height;
      observed.geometry_valid=observed.z_valid=true;
      // This consumer does not copy the producer's applied KF vector. Do
      // not expose its default numeric zero as a valid speed; StaticPlanner
      // qualifies a separate finite velocity from aligned hull history.
      // The producer bit/vector remain unchanged in the raw observation bag.
      observed.velocity_unavailable=true;
      observed.motion_state=o.dynamic_valid&&o.is_dynamic?ObjectMotionState::DYNAMIC:ObjectMotionState::UNKNOWN;
      const auto center=Rotate(rotation,o.center.x,o.center.y,o.center.z);
      observed.x=motion->x+cs*center[0]-sn*center[1];
      observed.y=motion->y+sn*center[0]+cs*center[1];observed.z=center[2];
      const auto axis=Rotate(rotation,std::cos(o.yaw),std::sin(o.yaw),0.);
      observed.heading=motion->yaw+std::atan2(axis[1],axis[0]);
      size_t corner_index=0;
      for(const auto& c:o.obb_corners) {
        if(!std::isfinite(c.x+c.y+c.z)) {reject("object_nonfinite");return;}
        const auto q=Rotate(rotation,c.x,c.y,c.z);
        const double x=motion->x+cs*q[0]-sn*q[1],y=motion->y+sn*q[0]+cs*q[1];
        observed.corners[corner_index++]={x,y,q[2]};ob.polygon.push_back({x,y});
      }
      ob.polygon=geometry::ComputeConvexHull(ob.polygon);
      if(ob.polygon.size()<3) {reject("object_degenerate");return;}
      next.push_back(std::move(ob));
    }
    world_objects=std::move(next);object_stamp=t;objects_received=pending_received;
    objects_valid=true;objects_reason="aligned_scan";++obb_transforms;pending_objects.reset();
  }
  void tick(const ros::SteadyTimerEvent&){
    drain_gps();process_objects();
    if(!state)return;
    const double now=ros::Time::now().toSec(),steady=ros::SteadyTime::now().toSec();
    const double t=state->header.stamp.toSec();auto origin=alignment.pose_at(t);if(!origin)return;
    Result r;bool computed=false;auto transform=anchor.at(t);
    const bool fresh=steady-state_received<=.15&&now-t>=0&&now-t<=.15;
    if(mode_gated&&!selected&&!preparing)r.reason="inactive_normal";
    else if(!fresh)r.reason="position_state_stale";
    else if(!attitude_valid || steady-imu_received>.15)r.reason="position_imu_stale";
    else if(!transform.valid)r.reason=anchor.reason;
    else if(!objects_valid||steady-objects_received>.3||t-object_stamp>.3||object_stamp-t>.05)
      r.reason="position_objects_stale";
    else if(state->header.stamp.toNSec()==last_state_stamp)return;
    else {
      Scene scene;scene.speed=origin->v;scene.yaw_rate=origin->w;scene.acceleration=origin->a;scene.age=std::max(now-t,now-object_stamp);
      const double cs=std::cos(origin->yaw),sn=std::sin(origin->yaw);
      for(auto ob:world_objects){
        for(auto& point:ob.polygon){double x=point.x-origin->x,y=point.y-origin->y;
          point.x=cs*x+sn*y;point.y=-sn*x+cs*y;}
        auto& o=ob.observation;
        const double x=o.x-origin->x,y=o.y-origin->y;
        o.x=cs*x+sn*y;o.y=-sn*x+cs*y;o.heading-=origin->yaw;
        for(auto& corner:o.corners){const double x=corner.x-origin->x,y=corner.y-origin->y;
          corner.x=cs*x+sn*y;corner.y=-sn*x+cs*y;}
        scene.obstacles.push_back(std::move(ob));
      }
      const auto forward_speed=ForwardPlanningSpeed(scene.speed);
      if(!forward_speed)r.reason="position_reverse_not_supported";
      else {computed=true;
        scene.speed=*forward_speed;
        const auto map_pose=transform.apply(origin->x,origin->y);
        paint_exempt=shadow_lane_rules&&paint_zone&&paint_window.update(t,selected,true,
            paint_zone->Contains(map_pose[0],map_pose[1],origin->yaw+transform.yaw));
        planner->set_paint_exemption(paint_exempt);
        planner->set_wall_permission(shadow_lane_rules&&selected&&paint_zone&&
          paint_zone->Contains(map_pose[0],map_pose[1],origin->yaw+transform.yaw));
        r=static_avoidance?planner->plan(scene,config,*origin,transform):planner->reference(scene,config,*origin,transform);}
    }
    if(!computed){paint_window.reset();paint_exempt=false;planner->set_paint_exemption(false);planner->set_wall_permission(false);}
    last_state_stamp=state->header.stamp.toNSec();++sequence;
    asmc_msgs::LocalTrajectory output;output.header=state->header;output.ego_at_plan=*state;
    output.valid=r.valid;output.reason=r.reason;output.run_id=run;output.plan_sequence=sequence;
    output.scene_stamp.fromSec(object_stamp);output.dt=.05;
    nav_msgs::Path path;path.header=output.header;
    const double cs=std::cos(origin->yaw),sn=std::sin(origin->yaw);
    for(const auto& q:r.trajectory){asmc_msgs::TrajectoryPoint point;
      point.x=origin->x+cs*q[0]-sn*q[1];point.y=origin->y+sn*q[0]+cs*q[1];
      point.yaw=origin->yaw+q[2];point.curvature=q[3];point.velocity=q[4];point.acceleration=q[5];
      output.points.push_back(point);geometry_msgs::PoseStamped p;p.header=output.header;
      p.pose.position.x=point.x;p.pose.position.y=point.y;
      p.pose.orientation.z=std::sin(.5*point.yaw);p.pose.orientation.w=std::cos(.5*point.yaw);
      path.poses.push_back(p);}
    bool selected_local=false;
    if(local_recovery_enabled){
      std::optional<asmc_msgs::LocalTrajectory> candidate;
      if(local_candidate){
        auto paired=alignment.pose_at(local_candidate->header.stamp.toSec());
        if(paired)candidate=ReframeLocalRecovery(*local_candidate,*paired,frame,run,now,steady-local_candidate_received);
      }
      const bool risk=(computed&&(planner->moving_objects()||planner->pending_motion_objects()))||
          r.reason.find("dynamic")!=std::string::npos||r.reason.find("motion_pending")!=std::string::npos;
      const bool allowed=selected&&fresh&&attitude_valid&&steady-imu_received<=.15&&transform.valid&&
          objects_valid&&steady-objects_received<=.3&&t-object_stamp>=-.05&&t-object_stamp<=.3;
      const bool progress=r.valid&&r.chosen_cap>.8&&r.path.size()>2;
      const bool recoverable=RecoverableLocalFailure(r.reason)||r.valid;
      selected_local=local_recovery.update(steady,allowed,progress,recoverable,bool(candidate),std::abs(origin->v)<.5,risk);
      if(selected_local){
        output=std::move(*candidate);output.plan_sequence=sequence;
        output.reason="local_recovery:"+local_candidate->reason;
        path.header=output.header;path.poses.clear();
        for(const auto& point:output.points){geometry_msgs::PoseStamped pose;pose.header=output.header;
          pose.pose.position.x=point.x;pose.pose.position.y=point.y;
          pose.pose.orientation.z=std::sin(point.yaw*.5);pose.pose.orientation.w=std::cos(point.yaw*.5);path.poses.push_back(pose);}
      }
    }
    trajectory_pub.publish(output);path_pub.publish(path);
    double observer_ms=0;size_t observer_bytes=0;
    if(debug_pub.getNumSubscribers()&&steady-last_debug_>=.2){
      const auto observer_start=std::chrono::steady_clock::now();
      last_debug_=steady;
      const bool display_objects_fresh=objects_valid&&steady-objects_received<=.3&&t-object_stamp<=.3&&object_stamp-t<=.05;
      std::ostringstream d;d<<std::setprecision(15)<<"{\"frame\":\"map\",\"stamp\":"<<t
       <<",\"run_id\":"<<std::quoted(run)<<",\"motion_frame\":"<<std::quoted(frame)
       <<",\"trial_id\":"<<std::quoted(trial_id)
       <<",\"plan_sequence\":"<<sequence<<",\"valid\":"<<(output.valid?"true":"false")
       <<",\"mode_selected\":"<<(selected?"true":"false")<<",\"reason\":"<<std::quoted(output.reason)
       <<",\"native_valid\":"<<(r.valid?"true":"false")<<",\"native_reason\":"<<std::quoted(r.reason)
       <<",\"selected_source\":"<<std::quoted(selected_local?"local_recovery":"localization")
       <<",\"selected_valid\":"<<(output.valid?"true":"false")<<",\"recovery_policy\":"<<std::quoted(local_recovery.reason)
       <<",\"escape_requested\":"<<(r.escape_requested?"true":"false")<<",\"stationary_windows\":"<<r.stationary_windows
       <<",\"phase\":"<<std::quoted(computed?planner->phase():"not_computed")<<",\"source\":"<<std::quoted(transform.source)
       <<",\"anchor_ready\":"<<(transform.valid?"true":"false")<<",\"anchor_samples\":"<<anchor.samples()
       <<",\"recovery_state\":"<<std::quoted(anchor.state)<<",\"recovery_reason\":"<<std::quoted(anchor.reason)
       <<",\"input_reason\":"<<std::quoted(input_reason)<<",\"objects_reason\":"<<std::quoted(objects_reason)
       <<",\"objects_fresh\":"<<(display_objects_fresh?"true":"false")
       <<",\"lane_paint_exempt\":"<<(paint_exempt?"true":"false")
       <<",\"moving_objects\":"<<(computed?planner->moving_objects():0)
       <<",\"pending_motion_objects\":"<<(computed?planner->pending_motion_objects():0)
       <<",\"rest_motion_disagreements\":"<<(computed?planner->rest_motion_disagreements():0)
       <<",\"path_reused\":"<<(computed&&planner->reused_path()?"true":"false")
       <<",\"continuity_reason\":"<<std::quoted(planner->continuity_reason())
       <<",\"gps_reason\":"<<std::quoted(gps_reason)
       <<",\"speed_kph\":"<<origin->v*3.6<<",\"selected_kph\":"<<r.chosen_cap*3.6
       <<",\"horizon_m\":"<<(r.path.empty()?0:r.path.back().s)
       <<",\"horizon_s\":"<<(output.points.empty()?0:(output.points.size()-1)*.05)
       <<",\"transition_m\":"<<(computed&&planner->native_calls()?planner->transition_distance():0)
       <<",\"return_m\":"<<(computed&&planner->native_calls()?planner->return_distance():0)
       <<",\"generated\":"<<(computed?planner->generated():0)<<",\"feasible\":"<<(computed?planner->feasible():0)
       <<",\"ego\":";
      const auto e=transform.apply(origin->x,origin->y);
      if(transform.valid&&fresh&&attitude_valid&&steady-imu_received<=.15)d<<"{\"x\":"<<e[0]<<",\"y\":"<<e[1]<<",\"yaw\":"<<origin->yaw+transform.yaw<<",\"valid\":true}";
      else d<<"null";
      d<<",\"candidates\":[";size_t retained=0;
      if(computed&&planner->native_calls()){
        const auto& paths=planner->candidates();const auto& validity=planner->candidate_validity();
        std::vector<size_t> indices;const size_t count=std::min(size_t(24),paths.size());
        bool shows_valid=false;
        for(size_t j=0;j<count;++j){const size_t i=count<2?0:j*(paths.size()-1)/(count-1);
          indices.push_back(i);shows_valid=shows_valid||(i<validity.size()&&validity[i]);}
        // Keep a representative feasible path even when downsampling a large
        // lattice. The actual MPC input remains the separate selected path.
        if(!shows_valid&&!indices.empty())for(size_t i=0;i<paths.size()&&i<validity.size();++i)
          if(validity[i]){indices.back()=i;break;}
        for(size_t i:indices){const auto& c=paths[i];
          if(c.x.size()<2||c.y.size()!=c.x.size())continue;
          if(retained++)d<<',';
          d<<"{\"feasible\":"<<(i<validity.size()&&validity[i]?"true":"false")<<",\"points\":[";
          bool first=true;const size_t step=std::max(size_t(1),(c.x.size()+63)/64);
          for(size_t j=0;j<c.x.size();j+=step){if(!std::isfinite(c.x[j]+c.y[j]))continue;
            if(!first)d<<',';
            first=false;d<<'['<<c.x[j]<<','<<c.y[j]<<']';}
          d<<"]}";
        }
      }
      d<<"],\"candidate_limit\":24,\"selected\":[";
      bool first=true;
      const size_t selected_step=std::max(size_t(2),(output.points.size()+1023)/1024);
      if(output.valid&&transform.valid)for(size_t i=0;i<output.points.size();i+=selected_step){const auto& point=output.points[i];
        const auto q=transform.apply(point.x,point.y);if(!first)d<<',';first=false;d<<'['<<q[0]<<','<<q[1]<<']';}
      d<<"],\"obstacles\":[";first=true;
      size_t shown_objects=0;
      if(transform.valid&&display_objects_fresh)
      for(const auto& ob:world_objects){if(shown_objects++>=512)break;if(!first)d<<',';first=false;
        d<<"{\"id\":"<<ob.id<<",\"points\":[";bool start=true;
        for(const auto& point:ob.polygon){const auto q=transform.apply(point.x,point.y);if(!start)d<<',';start=false;d<<'['<<q[0]<<','<<q[1]<<']';}d<<"]}";}
      d<<"]}";std_msgs::String debug;debug.data=d.str();debug_pub.publish(debug);
      observer_bytes=debug.data.size();observer_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-observer_start).count();
    }
    std_msgs::Bool ready;
    ready.data=!selected_local&&r.valid&&fresh&&transform.valid&&transform.source=="gps_anchored"&&anchor.state=="gps_anchored";
    if(shadow_lane_rules&&static_avoidance&&planner->maneuver_active())ready.data=false;
    recovery_ready_pub.publish(ready);
    std_msgs::Bool anchored;anchored.data=fresh&&attitude_valid&&steady-imu_received<=.15&&
        transform.valid&&transform.source=="gps_anchored"&&anchor.state=="gps_anchored";
    anchor_ready_pub.publish(anchored);
    auto xy=transform.apply(origin->x,origin->y);
    double obb_xmin=0,obb_xmax=0;
    if(!world_objects.empty()) {
      obb_xmin=INFINITY;obb_xmax=-INFINITY;
      for(const auto& point:world_objects.front().polygon) {
        obb_xmin=std::min(obb_xmin,point.x);obb_xmax=std::max(obb_xmax,point.x);
      }
    }
    std::ostringstream s;s<<std::setprecision(17)<<"{\"run_id\":"<<std::quoted(run)
      <<",\"stamp\":"<<t<<",\"valid\":"<<(output.valid?"true":"false")<<",\"reason\":"<<std::quoted(output.reason)
      <<",\"native_valid\":"<<(r.valid?"true":"false")<<",\"native_reason\":"<<std::quoted(r.reason)
       <<",\"selected_source\":"<<std::quoted(selected_local?"local_recovery":"localization")
      <<",\"selected_valid\":"<<(output.valid?"true":"false")<<",\"selected_reason\":"<<std::quoted(output.reason)
      <<",\"selected_scene_stamp\":"<<output.scene_stamp.toSec()<<",\"recovery_policy\":"<<std::quoted(local_recovery.reason)
      <<",\"source\":"<<std::quoted(transform.source)<<",\"map_x\":"<<xy[0]<<",\"map_y\":"<<xy[1]
      <<",\"map_yaw\":"<<origin->yaw+transform.yaw<<",\"anchor_stamp\":"<<transform.stamp
      <<",\"speed_kph\":"<<origin->v*3.6<<",\"selected_kph\":"<<r.chosen_cap*3.6
      <<",\"phase\":"<<std::quoted(planner->phase())<<",\"static_avoidance\":"<<(static_avoidance?"true":"false")
      <<",\"path_reused\":"<<(computed&&planner->reused_path()?"true":"false")
      <<",\"following_target\":"<<(computed?planner->following_target():-1)
      <<",\"following_actual_bumper_gap_m\":"<<(computed?planner->following_gap():-1.)
      <<",\"continuity_reason\":"<<std::quoted(computed?planner->continuity_reason():"not_computed")
      <<",\"confirmed_rest_objects\":"<<(computed?planner->confirmed_rest():0)
      <<",\"rest_motion_disagreements\":"<<(computed?planner->rest_motion_disagreements():0)
      <<",\"static_retargets\":"<<(computed?planner->retargets():0)
      <<",\"native_generated\":"<<(computed?planner->generated():0)<<",\"native_feasible\":"<<(computed?planner->feasible():0)
      <<",\"native_candidate_reason\":"<<std::quoted(computed?planner->candidate_reason():"not_computed")
      <<",\"processing_ms\":"<<r.elapsed_ms<<",\"observer_encode_ms\":"<<observer_ms<<",\"observer_payload_bytes\":"<<observer_bytes<<",\"objects\":"<<world_objects.size()
      <<",\"blocking_id\":"<<r.blocking_id<<",\"guide_lateral_m\":"<<(r.path.empty()?0:r.path.front().y)
      <<",\"guide_heading_rad\":"<<(r.path.empty()?0:r.path.front().yaw)
      <<",\"guide_map_x\":"<<(r.path.empty()?xy[0]:xy[0]+std::cos(origin->yaw+transform.yaw)*r.path.front().x-std::sin(origin->yaw+transform.yaw)*r.path.front().y)
      <<",\"guide_map_y\":"<<(r.path.empty()?xy[1]:xy[1]+std::sin(origin->yaw+transform.yaw)*r.path.front().x+std::cos(origin->yaw+transform.yaw)*r.path.front().y)
      <<",\"guide_map_yaw\":"<<(origin->yaw+transform.yaw+(r.path.empty()?0:r.path.front().yaw))
      <<",\"anchor_ready\":"<<(transform.valid?"true":"false")<<",\"anchor_samples\":"<<anchor.samples()
      <<",\"mode_gated\":"<<(mode_gated?"true":"false")<<",\"mode_selected\":"<<(selected?"true":"false")
      <<",\"recovery_state\":"<<std::quoted(anchor.state)<<",\"recovery_reason\":"<<std::quoted(anchor.reason)
      <<",\"innovation_m\":"<<anchor.innovation_m<<",\"innovation_rad\":"<<anchor.innovation_rad
      <<",\"raw_gps_innovation_m\":"<<anchor.raw_innovation_m
      <<",\"recovery_window_validation\":"<<(anchor.validate_recovery_window?"true":"false")
      <<",\"motion_epoch\":"<<alignment.epoch<<",\"motion_frame\":"<<std::quoted(frame)
      <<",\"anchor_epoch\":"<<anchor.generation<<",\"imu_stamp\":"<<alignment.imu_stamp
      <<",\"object_stamp\":"<<object_stamp<<",\"object_session\":"<<std::quoted(objects_identity.session)
      <<",\"object_generation\":"<<objects_identity.generation<<",\"object_sequence\":"<<objects_identity.sequence
      <<",\"object_config\":"<<std::quoted(objects_identity.config)<<",\"object_calibration\":"<<std::quoted(objects_identity.calibration)
      <<",\"input_reason\":"<<std::quoted(input_reason)<<",\"objects_reason\":"<<std::quoted(objects_reason)
      <<",\"gps_reason\":"<<std::quoted(gps_reason)<<",\"rejected_inputs\":"<<rejected_inputs
      <<",\"obb_scan_transforms\":"<<obb_transforms<<",\"obb_attitude_applications\":1"
      <<",\"first_obb_world_xmin\":"<<obb_xmin<<",\"first_obb_world_xmax\":"<<obb_xmax
      <<",\"integration_calls\":0,\"inference_calls\":0,\"lattice_calls\":"<<(computed?planner->native_calls():0)
      <<",\"speed_profiles\":"<<(computed?planner->speed_profiles():0)<<"}";
    std_msgs::String trace;trace.data=s.str();trace_pub.publish(trace);journal->push(trace.data);
  }
 public:
  PositionNode(){
    std::string parameters,state_topic,gps_topic,imu_topic,objects_topic,log;
    p.param<std::string>("parameters",parameters,"");p.param<std::string>("run_id",run,"");
    p.param<std::string>("trial_id",trial_id,"");
    if(parameters.empty()||run.empty())throw std::runtime_error("position parameters/run_id required");
    p.param<std::string>("state_topic",state_topic,"/planning/local/odometry");
    p.param("mode_gated",mode_gated,false);
    p.param("local_recovery_enabled",local_recovery_enabled,false);
    if(local_recovery_enabled)local_recovery_sub=nh.subscribe("/planning/local/trajectory",1,&PositionNode::on_local_recovery,this);
    p.param("static_avoidance",static_avoidance,false);
    p.param("shadow_lane_rules",shadow_lane_rules,false);
    p.param("dynamic_obstacles",dynamic_obstacles,false);
    if(shadow_lane_rules){
      std::string zone_path;double start=0,end=0,lateral=6.,heading=1.2;
      if(!nh.getParam("/planning_mode/shadow_zone/path_file",zone_path)||
         !nh.getParam("/planning_mode/physical_tunnel/station_start_m",start)||
         !nh.getParam("/planning_mode/physical_tunnel/station_end_m",end))
        throw std::runtime_error("paint exemption requires the configured known shadow zone");
      nh.param("/planning_mode/shadow_zone/lateral_tolerance_m",lateral,lateral);
      nh.param("/planning_mode/shadow_zone/heading_tolerance_rad",heading,heading);
      paint_zone=std::make_unique<asmc_planner::ShadowZone>(zone_path,start,end,lateral,heading);
    }
    p.param("validate_recovery_window",anchor.validate_recovery_window,false);
    p.param("input_reorder_tolerance_s",alignment.reorder_tolerance_s,0.);
    if(!std::isfinite(alignment.reorder_tolerance_s)||alignment.reorder_tolerance_s<0||alignment.reorder_tolerance_s>.1)
      throw std::runtime_error("invalid bounded source reorder tolerance");
    if(mode_gated) {
      mode_sub=nh.subscribe<std_msgs::UInt8>("/planning/mode",1,
        [this](const std_msgs::UInt8::ConstPtr& m){selected=m->data==1;if(!selected){local_recovery.reset();local_candidate.reset();}});
      prepare_sub=nh.subscribe<std_msgs::Bool>("/planning/jamming/prepare",1,
        [this](const std_msgs::Bool::ConstPtr& m){preparing=m->data;});
    }
    p.param<std::string>("gps_topic",gps_topic,"/gps");
    p.param<std::string>("imu_topic",imu_topic,"/imu");
    p.param("gps_sensor_offset/x",gps_offset_x,.35);p.param("gps_sensor_offset/y",gps_offset_y,0.);
    p.param("gps_sensor_offset/z",gps_offset_z,1.25);
    if(!std::isfinite(gps_offset_x+gps_offset_y+gps_offset_z)||
       std::abs(gps_offset_x)>5||std::abs(gps_offset_y)>5||gps_offset_z<0||gps_offset_z>5)
      throw std::runtime_error("invalid localization GPS mounting coordinates");
    p.param<std::string>("objects_topic",objects_topic,"/perception/lidar/tracks_local");
    p.param<std::string>("journal_path",log,"");journal=std::make_unique<Journal>(log);
    p.param("speed_cap_kph",config.speed_cap,20.);config.speed_cap/=3.6;
    p.param("vehicle_width",config.width,1.892);
    p.param("vehicle_length",config.length,4.635);
    p.param("rear_axle_to_bumper",config.rear,.790);
    p.param("cruise_speed_kph",config.cruise_speed,15.);config.cruise_speed/=3.6;
    p.param("escape_speed_cap_kph",config.recovery_speed,5.);config.recovery_speed/=3.6;
    if(!std::isfinite(config.width+config.length+config.rear+config.cruise_speed+config.recovery_speed)||
       config.width!=Config{}.width || config.length!=Config{}.length || config.rear!=Config{}.rear ||
       config.cruise_speed<=0 || config.cruise_speed>20./3.6 ||
       config.recovery_speed<3./3.6 || config.recovery_speed>5./3.6)
      throw std::runtime_error("invalid shared localization vehicle/speed policy");
    p.param("longitudinal/acceleration_limit_mps2",config.accel,2.);
    p.param("longitudinal/jerk_limit_mps3",config.jerk,4.8);
    if(!std::isfinite(config.accel+config.jerk)||config.accel<=0||config.accel>2.||config.jerk<=0||config.jerk>4.8)
      throw std::runtime_error("invalid experimental longitudinal limits");
    config.max_range=35.;config.recovery=config.blind_assist=false;
    if(!std::isfinite(config.speed_cap)||config.speed_cap<=0||config.speed_cap>20./3.6)
      throw std::runtime_error("localization trial cap must be in (0,20] km/h");
    planner=std::make_unique<StaticPlanner>(parameters);
    std::string wall_file;p.param<std::string>("wall_corridor_file",wall_file,"");
    if(!wall_file.empty())planner->load_wall_corridor(wall_file);
    planner->enable_dynamic(dynamic_obstacles);
    bool continuity=false;p.param("reuse_verified_path",continuity,false);
    planner->enable_path_continuity(continuity);
    double following_gap=3.;p.param("following_bumper_gap_m",following_gap,3.);
    if(!std::isfinite(following_gap)||following_gap<2.||following_gap>8.)
      throw std::runtime_error("invalid localization following bumper gap");
    planner->set_following_bumper_gap(following_gap);
    trajectory_pub=nh.advertise<asmc_msgs::LocalTrajectory>("/planning/localization/trajectory",1);
    path_pub=nh.advertise<nav_msgs::Path>("/planning/localization/path",1);
    trace_pub=nh.advertise<std_msgs::String>("/planning/localization/trace",1);
    debug_pub=nh.advertise<std_msgs::String>("/planning/localization/debug",1);
    recovery_ready_pub=nh.advertise<std_msgs::Bool>("/planning/localization/recovery_ready",1);
    anchor_ready_pub=nh.advertise<std_msgs::Bool>("/planning/localization/anchor_ready",1);
    state_sub=nh.subscribe(state_topic,64,&PositionNode::on_state,this,ros::TransportHints().tcpNoDelay());
    gps_sub=nh.subscribe(gps_topic,32,&PositionNode::on_gps,this,ros::TransportHints().tcpNoDelay());
    imu_sub=nh.subscribe(imu_topic,64,&PositionNode::on_imu,this,ros::TransportHints().tcpNoDelay());
    objects_sub=nh.subscribe(objects_topic,4,&PositionNode::on_objects,this,ros::TransportHints().tcpNoDelay());
    timer=nh.createSteadyTimer(ros::WallDuration(.05),&PositionNode::tick,this);
  }
};
} // namespace shadow_localization
int main(int argc,char** argv){ros::init(argc,argv,"localization");
  try{shadow_localization::PositionNode node;ros::spin();}catch(const std::exception& e){ROS_FATAL("POSITION: %s",e.what());return 1;}return 0;}
