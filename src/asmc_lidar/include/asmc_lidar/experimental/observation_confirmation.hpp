#pragma once
#include <cstdint>
#include <algorithm>
#include <map>
#include <set>
#include <vector>
namespace asmc::lidar::pipeline {
// Output-only confirmation: input tracker history must remain unfiltered.
class ObservationConfirmation {
 public:
  struct Input { int id; bool observed; bool protected_class; int required_frames=0; };
  std::set<int> Update(const std::vector<Input>& objects, bool active,
                       std::int64_t stamp, int required_frames) {
    std::set<int> hidden;
    if (!active) { Reset(); return hidden; }
    if (last_stamp_ <= 0 || stamp <= last_stamp_ || stamp-last_stamp_ > 250000000)
      states_.clear();
    std::map<int, State> next;
    for (const auto& o : objects) {
      if (o.id < 0) continue;
      auto it=states_.find(o.id);
      State s=it==states_.end()?State{}:it->second;
      // Scan traffic is not evidence that this particular object still exists.
      // Predictions never renew a structural candidate's confirmation lease.
      // General obstacles retain confirmation while the raw tracker retains the ID.
      const bool structural=s.structural || o.required_frames>required_frames;
      if(structural && s.last_observed>0 && stamp-s.last_observed>kStructuralObservationLeaseNs)s=State{};
      s.structural=s.structural || o.required_frames>required_frames;
      const int requested=o.required_frames>0?o.required_frames:required_frames;
      // A previously general ID cannot bypass stronger newly measured evidence.
      if(s.confirmed && !o.protected_class && requested>s.required) {
        s.confirmed=false;s.run=0;
      }
      if (o.protected_class) s.confirmed=true;
      if (!s.confirmed) s.required=std::max(s.required,o.required_frames>0?o.required_frames:required_frames);
      if (o.observed) {
        s.last_observed=stamp;
        if (!s.confirmed && ++s.run >= s.required) s.confirmed=true;
      } else if (!s.confirmed) s.run=0;
      if (!s.confirmed) hidden.insert(o.id);
      next[o.id]=s;
    }
    states_=std::move(next); last_stamp_=stamp; return hidden;
  }
  int Required(int id) const { auto it=states_.find(id);return it==states_.end()?0:it->second.required; }
  int Run(int id) const { auto it=states_.find(id);return it==states_.end()?0:it->second.run; }
  void Reset() { states_.clear(); last_stamp_=0; }
 private:
  static constexpr std::int64_t kStructuralObservationLeaseNs=250000000;
  struct State { int run=0,required=0; bool confirmed=false,structural=false; std::int64_t last_observed=0; };
  std::map<int,State> states_; std::int64_t last_stamp_=0;
};
}
