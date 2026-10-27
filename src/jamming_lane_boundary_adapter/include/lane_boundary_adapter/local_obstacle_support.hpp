#pragma once
// Consumer-side support for the opt-in shadow experiment. Does not infer a
// class or advance fusion confirmation; geometry must already be published by
// asmc_lidar. Empty input is never evidence that an unresolved image is clear.
#include <asmc_msgs/LidarTrackingObservation.h>
#include <lane_boundary_adapter/CameraBBox.h>
#include <geometry_msgs/Polygon.h>
#include <opencv2/core.hpp>
#include <deque>
#include <cmath>
#include <limits>
#include <algorithm>
namespace lane_boundary_adapter {
class LocalObstacleSupport {
 public:
  using Scan=asmc_msgs::LidarTrackingObservation;
  struct Entry { Scan::ConstPtr message; ros::SteadyTime received; };
  void accept(const Scan::ConstPtr& m,const ros::Time& now,const ros::SteadyTime& receipt) {
    const auto& o=m->observation;const auto& a=o.objects;const auto stamp=a.header.stamp;
    if(stamp.isZero()||a.header.frame_id!="base_link"||o.producer_session_id.empty()||
       o.config_id.empty()||o.calibration_id.empty()||
       (now-stamp).toSec()>.35||(stamp-now).toSec()>.05){entries_.clear();return;}
    if(!last_stamp_.isZero() && stamp<last_stamp_){entries_.clear();return;}
    if((a.status!=a.OBSERVED && a.status!=a.OBSERVED_EMPTY)||a.objects.size()>256){entries_.clear();return;}
    if(stamp==last_stamp_ && session_==o.producer_session_id && generation_==o.generation &&
       config_==o.config_id && calibration_==o.calibration_id)return;
    if(session_==o.producer_session_id && o.generation<generation_){entries_.clear();return;}
    if(session_!=o.producer_session_id||generation_!=o.generation||config_!=o.config_id||calibration_!=o.calibration_id)
      entries_.clear();
    session_=o.producer_session_id;generation_=o.generation;config_=o.config_id;calibration_=o.calibration_id;last_stamp_=stamp;
    if((a.status!=a.OBSERVED && a.status!=a.OBSERVED_EMPTY)||a.objects.size()>256){entries_.clear();return;}
    entries_.push_back({m,receipt});while(entries_.size()>8)entries_.pop_front();
  }
  Scan::ConstPtr select(const ros::Time& image,const ros::Time& now,const ros::SteadyTime& steady)const {
    Scan::ConstPtr best;double distance=.050000001;
    for(const auto& e:entries_){const auto stamp=e.message->observation.objects.header.stamp;
      if((steady-e.received).toSec()>.35||(now-stamp).toSec()>.35||(stamp-now).toSec()>.05)continue;
      const double d=std::abs((image-stamp).toSec());
      if(d<distance){distance=d;best=e.message;}
    }return best;
  }
  static bool footprint(const asmc_msgs::LidarObject& o,geometry_msgs::Polygon& out,double padding=.25) {
    out.points.clear();
    if(o.observation_state!=o.OBSERVED||o.miss_count||o.source_point_count<3||!std::isfinite(o.length)||!std::isfinite(o.width)||!std::isfinite(o.height)||o.length<=0||o.width<=0||o.height<=0)return false;
    double xmin=1e100,xmax=-1e100,ymin=1e100,ymax=-1e100;
    for(const auto& p:o.obb_corners){if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return false;
      xmin=std::min(xmin,p.x);xmax=std::max(xmax,p.x);ymin=std::min(ymin,p.y);ymax=std::max(ymax,p.y);}
    if(xmax-xmin<1e-5||ymax-ymin<1e-5)return false;
    for(const auto& p:std::vector<cv::Point2d>{{xmin-padding,ymin-padding},{xmax+padding,ymin-padding},
        {xmax+padding,ymax+padding},{xmin-padding,ymax+padding}}){geometry_msgs::Point32 q;q.x=p.x;q.y=p.y;out.points.push_back(q);}return true;
  }
  static bool projectedBox(const asmc_msgs::LidarObject& o,const cv::Matx33d& K,const cv::Matx33d& R,
                           const cv::Vec3d& t,CameraBBox& b){
    b.xmin=b.ymin=std::numeric_limits<float>::infinity();b.xmax=b.ymax=-b.xmin;
    for(const auto& p:o.obb_corners){const auto c=R*cv::Vec3d(p.x,p.y,p.z)+t;
      if(!std::isfinite(c[2])||c[2]<=.01)return false;
      const auto u=K*c;
      const double x=u[0]/u[2],y=u[1]/u[2];if(!std::isfinite(x)||!std::isfinite(y))return false;
      b.xmin=std::min(b.xmin,float(x));b.xmax=std::max(b.xmax,float(x));b.ymin=std::min(b.ymin,float(y));b.ymax=std::max(b.ymax,float(y));}
    return b.xmax>b.xmin&&b.ymax>b.ymin;
  }
  static double iou(const CameraBBox&a,const CameraBBox&b){
    const double intersection=std::max(0.f,std::min(a.xmax,b.xmax)-std::max(a.xmin,b.xmin))*
      std::max(0.f,std::min(a.ymax,b.ymax)-std::max(a.ymin,b.ymin));
    const double area=(a.xmax-a.xmin)*(a.ymax-a.ymin)+(b.xmax-b.xmin)*(b.ymax-b.ymin)-intersection;
    return area>0?intersection/area:0.;
  }
  // Unique BOTH ways. Same-class 2D overlap only verifies the existing fusion
  // association; it does not label a new track or count another confirmation.
  static std::vector<int> associate(const std::vector<CameraBBox>& boxes,const Scan& scan,
      const cv::Matx33d& K,const cv::Matx33d& R,const cv::Vec3d& t){
    const auto& objects=scan.observation.objects.objects;std::vector<int> result(boxes.size(),-1),uses(objects.size(),0);
    for(std::size_t i=0;i<boxes.size();++i){int count=0;
      for(std::size_t j=0;j<objects.size();++j){geometry_msgs::Polygon f;CameraBBox projected;
        if(boxes[i].label.empty()||boxes[i].label!=objects[j].camera_class||!footprint(objects[j],f)||
           !projectedBox(objects[j],K,R,t,projected)||iou(boxes[i],projected)<.25)continue;
        result[i]=int(j);++count;++uses[j];}
      if(count!=1)result[i]=-1;}
    for(int& j:result)if(j>=0&&uses[j]!=1)j=-1;
    return result;
  }
 private:
  std::deque<Entry> entries_;ros::Time last_stamp_;std::string session_,config_,calibration_;uint64_t generation_=0;
};
} // namespace lane_boundary_adapter
