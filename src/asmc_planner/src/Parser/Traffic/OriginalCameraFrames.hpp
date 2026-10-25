#pragma once
#include <asmc_msgs/SensorFrameTiming.h>
#include <std_msgs/Header.h>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <limits>

namespace behavior_planner {
// The classifier publisher rewrites Header.seq in ROS1. Resolve its unchanged
// frame/stamp to EXACTLY ONE original image + timing-sidecar tuple, including
// the original image wire sequence and topic. Ambiguity never authorizes entry.
class OriginalCameraFrames {
 public:
  enum class Match { MISSING, INVALID, VALID };
  struct Original {
    std::uint64_t source_ns=0,epoch=0;
    double ros_stamp=0;
  };
  OriginalCameraFrames(std::string topic,double offset,double uncertainty)
      :topic_(std::move(topic)),offset_(offset),uncertainty_(uncertainty) {
    if(topic_.empty() || !std::isfinite(offset_) || !std::isfinite(uncertainty_) ||
       uncertainty_<0 || uncertainty_>.05)throw std::runtime_error("invalid original-camera clock calibration");
  }
  bool Image(const std_msgs::Header& h) {
    auto& r=RecordFor(h.frame_id,h.stamp.toNSec(),h.seq);r.image=true;
    return records_.at({h.frame_id,h.stamp.toNSec()}).size()!=1;
  }
  bool Timing(const asmc_msgs::SensorFrameTiming& t) {
    if(t.sensor_topic!=topic_)return false;
    if(t.bridge_session_id.empty()) {
      Tombstone({t.sensor_frame_id,t.published_stamp.toNSec()});return true;
    }
    bool reset=false;
    if(!session_.empty() && session_!=t.bridge_session_id) {
      if(retired_.count(t.bridge_session_id))return false;
      retired_.insert(session_);records_.clear();++epoch_;reset=true;
    }
    session_=t.bridge_session_id;
    if(t.source_regression) {++epoch_;records_.clear();reset=true;}
    auto& r=RecordFor(t.sensor_frame_id,t.published_stamp.toNSec(),t.sensor_wire_sequence);
    r.timing=true;r.source_ns=t.source_sec*1000000000ULL+t.source_nsec;r.epoch=epoch_;
    r.valid=t.sensor_wire_sequence_valid && t.source_valid && !t.source_conflict &&
        !t.source_regression && t.source_nsec<1000000000 && !t.bridge_session_id.empty() &&
        t.source_sec>0 && t.source_sec<=std::numeric_limits<std::uint64_t>::max()/1000000000ULL &&
        t.source_clock_domain=="morai_sensor" && t.source_reference=="camera_frame";
    r.ros_stamp=static_cast<double>(r.source_ns)*1e-9+offset_-uncertainty_;
    // A transformed capture may not occur after its bridge publication. Use
    // the oldest possible capture for freshness; never renew it on duplicates.
    if(!std::isfinite(r.ros_stamp) || r.ros_stamp>t.published_stamp.toSec()+1e-6)r.valid=false;
    if(!r.valid)Tombstone({t.sensor_frame_id,t.published_stamp.toNSec()});
    return reset || !r.valid || records_.at({t.sensor_frame_id,t.published_stamp.toNSec()}).size()!=1;
  }
  void Invalidate(const std_msgs::Header& h) {
    Tombstone({h.frame_id,h.stamp.toNSec()});
  }
  Match Resolve(const std_msgs::Header& h,Original& out) const {
    const Key key{h.frame_id,h.stamp.toNSec()};
    const auto at=records_.find(key);
    if(at==records_.end())return invalid_.count(key)?Match::INVALID:Match::MISSING;
    if(at->second.size()!=1)return Match::INVALID;
    const auto& r=at->second.front();
    if(!r.image || !r.timing)return invalid_.count(key)?Match::INVALID:Match::MISSING;
    out={r.source_ns,r.epoch,r.ros_stamp};
    return !invalid_.count(key) && r.valid && r.epoch==epoch_ ? Match::VALID:Match::INVALID;
  }
  std::uint64_t epoch() const {return epoch_;}
 private:
  using Key=std::pair<std::string,std::uint64_t>;
  void Tombstone(const Key& key) {
    invalid_.insert(key);
    while(invalid_.size()>1024)invalid_.erase(invalid_.begin());
  }
  struct Record {
    std::uint32_t sequence=0;
    std::uint64_t source_ns=0,epoch=0;
    double ros_stamp=0;
    bool image=false,timing=false,valid=false;
  };
  Record& RecordFor(const std::string& frame,std::uint64_t stamp,std::uint32_t seq) {
    const Key key{frame,stamp};
    auto& group=records_[key];
    for(auto& r:group)if(r.sequence==seq)return r;
    if(group.size()>=2)return group.back();
    group.push_back({});group.back().sequence=seq;
    while(records_.size()>512) {
      auto first=records_.begin();
      if(first->first==key) {auto last=std::prev(records_.end());records_.erase(last);}
      else records_.erase(first);
    }
    return group.back();
  }
  std::string topic_,session_;
  double offset_=0,uncertainty_=0;
  std::uint64_t epoch_=1;
  std::map<Key,std::vector<Record>> records_;
  std::set<Key> invalid_;
  std::set<std::string> retired_;
};
}  // namespace behavior_planner
