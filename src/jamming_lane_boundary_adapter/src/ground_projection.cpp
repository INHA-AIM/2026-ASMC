#include "lane_boundary_adapter/ground_projection.hpp"
#include <algorithm>
#include <cmath>
#include <set>

namespace lane_boundary_adapter {
namespace {
bool fresh(ros::Time stamp, ros::Time now) {
  return !stamp.isZero() && (now-stamp).toSec() >= -.05 && (now-stamp).toSec() <= .35;
}
bool sameImage(const std_msgs::Header& a, const std_msgs::Header& b) {
  return a.stamp==b.stamp && a.seq==b.seq && a.frame_id==b.frame_id;
}
std::string identity(const asmc_msgs::CameraGroundProjection& m) {
  return m.producer_session_id+":"+std::to_string(m.generation)+":"+m.projection_session_id;
}
}
GroundProjectionBuffer::GroundProjectionBuffer(std::string config_id,
    std::map<std::string,std::string> ids, cv::Vec3d nominal)
    :config_id_(std::move(config_id)),calibration_ids_(std::move(ids)),nominal_(nominal) {}

void GroundProjectionBuffer::clear() {
  ++revision_;
  for(const auto& row:records_) retired_stamps_.push_back(row.message->header.stamp);
  while(retired_stamps_.size()>32) retired_stamps_.pop_front();
  records_.clear();
}
std::string GroundProjectionBuffer::gate(ros::Time now, ros::SteadyTime receipt) const {
  if(!has_status_) return "waiting_for_lane_status";
  if(!fresh(status_.header.stamp,now) || (receipt-status_receipt_).toSec()>.35)
    return "lane_status_stale";
  if(status_.mode!=asmc_msgs::InferenceStatus::ACTIVE || !status_.loaded)
    return "lane_not_active";
  return "";
}
bool GroundProjectionBuffer::status(const asmc_msgs::InferenceStatus& m, ros::Time now,
                                    ros::SteadyTime receipt) {
  if(m.producer_session_id.empty() || !fresh(m.header.stamp,now)) {
    last_error_="invalid_lane_status"; return false;
  }
  const std::string next=m.producer_session_id+":"+std::to_string(m.generation);
  if(std::find(retired_sessions_.begin(),retired_sessions_.end(),next)!=retired_sessions_.end()) {
    last_error_="retired_lane_status"; return false;
  }
  if(has_status_ && next==session_ && (m.header.stamp<status_.header.stamp ||
      m.source_cutoff<status_.source_cutoff)) { last_error_="regressed_lane_status"; return false; }
  const bool changed=has_status_ && (next!=session_ || m.mode!=status_.mode || m.source_cutoff!=status_.source_cutoff);
  if(changed) {
    if(next!=session_) retired_sessions_.push_back(session_);
    while(retired_sessions_.size()>16) retired_sessions_.pop_front();
    clear(); projection_session_.clear();
  }
  session_=next; status_=m; has_status_=true; status_receipt_=receipt;
  return changed;
}
bool GroundProjectionBuffer::add(const asmc_msgs::CameraGroundProjection::ConstPtr& m,
                                 ros::Time now, ros::SteadyTime receipt) {
  last_error_=gate(now,receipt);
  if(!last_error_.empty()) return false;
  auto reject=[&](const std::string& why){last_error_=why;return false;};
  if(m->config_id!=config_id_) return reject("projection_config_mismatch");
  if(m->producer_session_id!=status_.producer_session_id || m->generation!=status_.generation)
    return reject("projection_epoch_mismatch");
  if(m->projection_session_id.empty() || m->header.frame_id!="base_link" ||
      !fresh(m->header.stamp,now) || m->header.stamp<status_.source_cutoff)
    return reject("projection_invalid_reference");
  const auto n=calibration_ids_.size();
  if(m->views.size()!=n || m->image_headers.size()!=n || m->calibration_ids.size()!=n ||
      m->plane_coefficients.size()!=3*n) return reject("projection_malformed_arrays");
  std::set<std::string> views; ros::Time earliest=m->header.stamp,latest;
  for(std::size_t i=0;i<n;++i) {
    const auto& v=m->views[i]; const auto id=calibration_ids_.find(v);
    const auto& h=m->image_headers[i];
    if(!views.insert(v).second || id==calibration_ids_.end() || id->second!=m->calibration_ids[i] ||
        h.frame_id!="camera_"+v || !fresh(h.stamp,now) || h.stamp<status_.source_cutoff)
      return reject("projection_calibration_or_source_mismatch");
    earliest=std::min(earliest,h.stamp); latest=std::max(latest,h.stamp);
    const cv::Vec3d p(m->plane_coefficients[3*i],m->plane_coefficients[3*i+1],m->plane_coefficients[3*i+2]);
    for(double value:{p[0],p[1],p[2]}) if(!std::isfinite(value)) return reject("projection_nonfinite_plane");
    if(std::hypot(p[0],p[1])>std::tan(8.*std::acos(-1.)/180.)+1e-9 ||
        std::abs(p[2]/std::sqrt(1+p[0]*p[0]+p[1]*p[1])-nominal_[2])>1e-5)
      return reject("projection_plane_out_of_bounds");
    if(!m->imu_corrected && cv::norm(p-nominal_)>1e-9) return reject("projection_invalid_nominal_fallback");
  }
  if(latest!=m->header.stamp || (latest-earliest).toSec()>.035)
    return reject("projection_bundle_time_mismatch");
  const auto next=identity(*m);
  if(std::find(retired_sessions_.begin(),retired_sessions_.end(),next)!=retired_sessions_.end())
    return reject("retired_projection_session");
  const bool changed=!projection_session_.empty() && next!=projection_session_;
  if(changed) { retired_sessions_.push_back(projection_session_); clear(); }
  while(retired_sessions_.size()>16) retired_sessions_.pop_front();
  projection_session_=next;
  if(std::find(retired_stamps_.begin(),retired_stamps_.end(),m->header.stamp)!=retired_stamps_.end())
    return reject("projection_ambiguous_reused_stamp");
  for(auto it=records_.begin();it!=records_.end();) {
    if((receipt-it->receipt).toSec()>.35) it=records_.erase(it); else ++it;
  }
  for(const auto& r:records_) if(r.message->header.stamp==m->header.stamp) {
    retired_stamps_.push_back(m->header.stamp);
    while(retired_stamps_.size()>32)retired_stamps_.pop_front();
    last_error_="projection_duplicate_stamp"; return changed;
  }
  records_.push_back({m,receipt}); while(records_.size()>8) records_.pop_front();
  last_error_.clear(); return changed;
}
ProjectionMatch GroundProjectionBuffer::match(const asmc_msgs::LaneBoundaryFamily& family,
    const std_msgs::Header& image,const std::string& view,ros::Time now,ros::SteadyTime receipt) const {
  ProjectionMatch out; out.reason=gate(now,receipt); if(!out.reason.empty())return out;
  if(family.header.frame_id!="base_link") {out.reason="family_projection_frame_mismatch";return out;}
  if(!fresh(image.stamp,now) || !fresh(family.header.stamp,now) ||
      image.stamp<status_.source_cutoff || family.header.stamp<status_.source_cutoff) {
    out.reason="projection_source_expired"; return out;
  }
  if(std::find(retired_stamps_.begin(),retired_stamps_.end(),family.header.stamp)!=retired_stamps_.end()) {
    out.reason="projection_ambiguous_reused_stamp";return out;
  }
  for(auto it=records_.rbegin();it!=records_.rend();++it) {
    const auto& m=*it->message;
    if(m.header.stamp!=family.header.stamp) continue;
    if((receipt-it->receipt).toSec()>.35 || m.config_id!=family.config_sha256) {
      out.reason="projection_context_expired_or_config_mismatch";return out;
    }
    std::size_t reference=0,selected=m.views.size();
    for(std::size_t i=0;i<m.views.size();++i) {
      if(m.image_headers[i].stamp>m.image_headers[reference].stamp)reference=i;
      if(m.views[i]==view && sameImage(m.image_headers[i],image))selected=i;
    }
    if(selected==m.views.size()) {out.reason="projection_exact_image_missing";continue;}
    const cv::Vec3d reference_plane(m.plane_coefficients[3*reference],m.plane_coefficients[3*reference+1],m.plane_coefficients[3*reference+2]);
    for(const auto& b:family.boundaries) for(const auto& p:b.polyline) {
      if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          std::abs(p.z-onPlane(reference_plane,p.x,p.y)[2])>5e-5) {
        out.reason="family_xyz_projection_mismatch";return out;
      }
    }
    out.plane={m.plane_coefficients[3*selected],m.plane_coefficients[3*selected+1],m.plane_coefficients[3*selected+2]};
    out.valid=true;out.imu_corrected=m.imu_corrected;out.reason=m.reason;return out;
  }
  if(out.reason.empty()) out.reason="waiting_for_projection_context";
  return out;
}
cv::Vec3d onPlane(const cv::Vec3d& p,double x,double y) {return {x,y,p[2]+p[0]*x+p[1]*y};}
cv::Matx33d groundHomography(const cv::Matx33d& K,const cv::Matx33d& R,const cv::Vec3d& t,const cv::Vec3d& p) {
  cv::Matx33d plane;
  for(int i=0;i<3;++i){plane(i,0)=R(i,0)+p[0]*R(i,2);plane(i,1)=R(i,1)+p[1]*R(i,2);plane(i,2)=t[i]+p[2]*R(i,2);}
  return K*plane;
}
bool projectGroundPoint(const cv::Matx33d& K,const cv::Matx33d& R,const cv::Vec3d& t,
    const cv::Vec3d& plane,double x,double y,cv::Point2d& pixel) {
  const auto optical=R*onPlane(plane,x,y)+t;
  if(!std::isfinite(optical[2]) || optical[2]<=.01)return false;
  const auto p=K*optical;pixel={p[0]/p[2],p[1]/p[2]};
  return std::isfinite(pixel.x)&&std::isfinite(pixel.y);
}
} // namespace lane_boundary_adapter
