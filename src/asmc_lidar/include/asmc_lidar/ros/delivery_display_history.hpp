#pragma once
// Display-only history. Never feeds tracking, fusion or delivery decisions.
#include <asmc_lidar/filtering/hd_map_audit.hpp>
#include <map>
#include <set>
#include <string>

namespace asmc::lidar::runtime {
class DeliveryDisplayHistory {
 public:
  struct Entry { filtering::AuditDecision decision; std::int64_t stamp_ns=0; };
  void Update(const std::string& identity, std::int64_t stamp_ns,
              const std::set<int>& alive, const filtering::AuditReport& report) {
    if(identity!=identity_ || stamp_ns<last_stamp_) entries_.clear();
    identity_=identity; last_stamp_=stamp_ns;
    for(auto i=entries_.begin();i!=entries_.end();)
      if(!alive.count(i->first)) i=entries_.erase(i); else ++i;
    // A disabled/audit-only filter cannot carry a previous delivery verdict.
    if(!report.delivery_applied) {entries_.clear();return;}
    for(const auto& d:report.objects)
      if(alive.count(d.track_id)) entries_[d.track_id]={d,stamp_ns};
  }
  const std::map<int,Entry>& entries() const {return entries_;}
 private:
  std::string identity_;
  std::int64_t last_stamp_=0;
  std::map<int,Entry> entries_;
};
}
