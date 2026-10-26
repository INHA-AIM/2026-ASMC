#pragma once
#include "Signal/SignalPolicy.hpp"

namespace asmc_planner::signal {
// A validated preview qualifies an owner permit. Issuing that permit changes
// the wire token, so allow bounded time for its OWN plan/Controller proof to
// arrive without alternately revoking/reissuing the command every policy tick.
// This cache is never a trajectory ownership proof; Controller still requires
// the exact current request and (for recovery) its complete exit path.
class ReleaseCommit {
 public:
  bool Authorize(const Key& key,bool recovery,bool eligible,bool current_proof,
                 bool current_request_authorized,double steady_now) {
    if(key!=key_ || recovery!=recovery_) {Reset();key_=key;recovery_=recovery;}
    if(!eligible || !key.valid() || !std::isfinite(steady_now)) {Reset();return false;}
    if(!authorized_ && current_proof) {authorized_=true;started_=steady_now;}
    if(!authorized_)return false;
    if(current_request_authorized && current_proof)return true;
    if(steady_now>=started_ && steady_now-started_<=.5)return true;
    Reset();return false;
  }
  void Reset() {authorized_=false;started_=-1;}
 private:
  Key key_;bool recovery_=false,authorized_=false;double started_=-1;
};
}  // namespace asmc_planner::signal
