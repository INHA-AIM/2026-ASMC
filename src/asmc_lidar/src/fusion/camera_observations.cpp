#include <asmc_lidar/fusion/camera_observations.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace asmc::lidar::camera {
namespace {
bool fresh(Ns source, Ns now, Ns age, Ns future) {
  return source>0 && now>0 && (source<=now ? now-source<=age : source-now<=future);
}
bool within(Ns a, Ns b, Ns limit) { return a>=b ? a-b<=limit : b-a<=limit; }
}
Buffer::Buffer(Calibrations c, Limits l):calibrations_(std::move(c)),limits_(l) {
  if (calibrations_.empty() || !l.capacity || l.retention_ns<=0 || l.status_freshness_ns<=0 || l.future_tolerance_ns<0)
    throw std::invalid_argument("invalid camera buffer limits/calibrations");
  for (const auto& entry:calibrations_) {
    const auto& k=entry.second;
    if (entry.first!=k.frame_id || k.id.empty() || !k.width || !k.height ||
        !pose_valid(k.base_from_camera) || !k.intrinsic.allFinite() || k.intrinsic(0,0)<=0 || k.intrinsic(1,1)<=0)
      throw std::invalid_argument("invalid camera calibration");
    views_.emplace(entry.first,View{});
  }
}
std::string Buffer::count_locked(const std::string& reason) { ++counts_[reason]; return reason; }
void Buffer::clear_locked(const std::string& reason) {
  ++epoch_;
  for (auto& entry:views_) {entry.second.records.clear();entry.second.watermark=0;++entry.second.revision;entry.second.reason=reason;}
}
std::string Buffer::gate_locked(Ns ros_now, Ns steady_now) const {
  if(closed_) return "closed";
  if(!has_status_) return "status_missing";
  if(steady_now<status_receipt_ || steady_now-status_receipt_>limits_.status_freshness_ns ||
     !fresh(status_.stamp_ns,ros_now,limits_.status_freshness_ns,limits_.future_tolerance_ns)) return "status_stale";
  if(status_.mode!=2) return status_.mode==1 ? "prepare" : "standby";
  if(identity_conflict_) return "identity_conflict";
  return {};
}
void Buffer::expire_locked(Ns ros_now, Ns steady_now) {
  for(auto& entry:views_) {
    auto& v=entry.second;
    while(!v.records.empty()) {
      const auto& r=*v.records.front();
      if(steady_now>=r.receipt_ns && steady_now-r.receipt_ns<=limits_.retention_ns &&
         fresh(r.observation.header.stamp_ns,ros_now,limits_.retention_ns,limits_.future_tolerance_ns)) break;
      v.records.pop_front();count_locked("expired");
      if(v.records.empty()) v.reason="stale";
    }
  }
}
std::string Buffer::status(const Status& s, Ns ros_now, Ns steady_now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(closed_) return count_locked("closed");
  if(s.session.empty() || s.model!="yolo" || s.mode>2 || s.cutoff_ns<0 || steady_now<0 ||
     !fresh(s.stamp_ns,ros_now,limits_.status_freshness_ns,limits_.future_tolerance_ns)) return count_locked("bad_status");
  if(has_status_ && s.session!=status_.session &&
     std::find(retired_sessions_.begin(),retired_sessions_.end(),s.session)!=retired_sessions_.end()) return count_locked("retired_session");
  if(has_status_ && s.session==status_.session && (s.generation<status_.generation ||
     (s.generation==status_.generation && s.stamp_ns<status_.stamp_ns))) return count_locked("old_status");
  if(has_status_ && s.session==status_.session && s.generation==status_.generation &&
     (s.mode!=status_.mode || s.cutoff_ns!=status_.cutoff_ns)) return count_locked("status_conflict");
  const bool transition=!has_status_ || s.session!=status_.session || s.generation!=status_.generation;
  if(transition) {
    if(has_status_ && s.session!=status_.session) {
      retired_sessions_.push_back(status_.session);
      if(retired_sessions_.size()>32) retired_sessions_.pop_front();
    }
    clear_locked("missing");model_id_.clear();config_id_.clear();identity_conflict_=false;
    count_locked("transition");
  } else if(!gate_locked(ros_now,steady_now).empty()) {
    // A heartbeat after silence must not resurrect a previously returned candidate.
    clear_locked("missing");
  }
  status_=s;status_receipt_=steady_now;has_status_=true;
  return count_locked("status_accepted");
}
std::string Buffer::observe(Observation o, Ns ros_now, Ns steady_now) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto gate=gate_locked(ros_now,steady_now);
  if(!gate.empty()) return count_locked(gate);
  expire_locked(ros_now,steady_now);
  auto it=views_.find(o.header.frame_id);
  if(it==views_.end()) return count_locked("unknown_camera");
  auto& v=it->second;
  if(o.session!=status_.session || o.generation!=status_.generation) return count_locked("epoch_mismatch");
  if(o.header.stamp_ns<=status_.cutoff_ns || steady_now<0 ||
     !fresh(o.header.stamp_ns,ros_now,limits_.retention_ns,limits_.future_tolerance_ns)) return count_locked("source_stale_or_cutoff");
  // Invalidations may reuse the last valid source header; older ones cannot erase new data.
  if(o.header.stamp_ns<v.watermark || (o.valid && o.header.stamp_ns==v.watermark)) return count_locked("duplicate_or_old");
  const auto& c=calibrations_.at(o.header.frame_id);
  if((o.valid && (o.width!=c.width || o.height!=c.height)) || o.calibration_id!=c.id) {
    v.records.clear();++v.revision;v.reason="calibration_mismatch";
    return count_locked(v.reason);
  }
  if(o.model_id.empty() || o.config_id.empty()) return count_locked("missing_identity");
  if(!model_id_.empty() && (o.model_id!=model_id_ || o.config_id!=config_id_)) {
    identity_conflict_=true;clear_locked("identity_conflict");return count_locked("identity_conflict");
  }
  model_id_=o.model_id;config_id_=o.config_id;
  if(!o.valid) {
    v.records.clear();++v.revision;v.watermark=o.header.stamp_ns;v.reason="invalid";
    return count_locked("invalid");
  }
  for(const auto& b:o.boxes) {
    if(!std::isfinite(b.xmin)||!std::isfinite(b.ymin)||!std::isfinite(b.xmax)||!std::isfinite(b.ymax)||
       b.xmin<0||b.ymin<0||b.xmax>o.width||b.ymax>o.height||b.xmin>=b.xmax||b.ymin>=b.ymax||
       !std::isfinite(b.confidence)||b.confidence<0||b.confidence>1||b.source_class_name.empty()||
       (b.class_name!="vehicle" && b.class_name!="dynamic_obstacle" && b.class_name!="static_obstacle")) {
      v.records.clear();++v.revision;v.reason="bad_box";return count_locked("bad_box");
    }
  }
  v.watermark=o.header.stamp_ns;v.reason=o.boxes.empty()?"empty":"ok";
  v.records.push_back(std::make_shared<const Stored>(Stored{std::move(o),steady_now,epoch_,v.revision}));
  while(v.records.size()>limits_.capacity) {v.records.pop_front();count_locked("capacity_drop");}
  return count_locked("accepted");
}
Query Buffer::query(Ns scan_stamp, Ns max_difference, Ns ros_now, Ns steady_now) {
  std::lock_guard<std::mutex> lock(mutex_);
  Query q;expire_locked(ros_now,steady_now);
  const auto gate=gate_locked(ros_now,steady_now);
  for(const auto& entry:views_) {
    auto& out=q.views[entry.first];const auto& v=entry.second;
    out.reason=!gate.empty()?gate:(scan_stamp<=0||max_difference<0?"bad_query":v.reason);
    if(!gate.empty()||scan_stamp<=0||max_difference<0) continue;
    for(auto it=v.records.rbegin();it!=v.records.rend();++it) {
      if(within((*it)->observation.header.stamp_ns,scan_stamp,max_difference))
        out.candidates.push_back({*it,(*it)->observation.header.stamp_ns-scan_stamp});
    }
    std::stable_sort(out.candidates.begin(),out.candidates.end(),[](const auto& a,const auto& b){
      return std::abs(a.image_minus_scan_ns)<std::abs(b.image_minus_scan_ns);
    });
    if(!out.candidates.empty()) out.reason=out.candidates.front().record->observation.boxes.empty()?"empty":"ok";
    else if(!v.records.empty()) out.reason="no_time_match";
  }
  return q;
}
bool Buffer::current(const Candidate& candidate, Ns ros_now, Ns steady_now) {
  std::lock_guard<std::mutex> lock(mutex_);expire_locked(ros_now,steady_now);
  if(!candidate.record || !gate_locked(ros_now,steady_now).empty()) return false;
  const auto& r=*candidate.record;auto it=views_.find(r.observation.header.frame_id);
  if(it==views_.end() || r.epoch!=epoch_ || r.view_revision!=it->second.revision) return false;
  const auto& records=it->second.records;
  return std::find(records.begin(),records.end(),candidate.record)!=records.end();
}
std::map<std::string,std::uint64_t> Buffer::counters() const {std::lock_guard<std::mutex> l(mutex_);return counts_;}
std::map<std::string,std::size_t> Buffer::sizes() const {
  std::lock_guard<std::mutex> l(mutex_);std::map<std::string,std::size_t> out;
  for(const auto& e:views_) out[e.first]=e.second.records.size();
  return out;
}
void Buffer::close() {std::lock_guard<std::mutex> l(mutex_);closed_=true;clear_locked("closed");}
} // namespace asmc::lidar::camera
