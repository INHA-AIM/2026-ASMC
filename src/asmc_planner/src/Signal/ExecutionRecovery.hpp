#pragma once

namespace asmc_planner::signal {
// Current-token status only. The policy owner authorizes path-only recovery;
// a raw Controller safe flag cannot authorize it on that owner's behalf.
class ExecutionRecovery {
 public:
  void Observe(bool failed,unsigned mask,bool linked,bool acknowledgment_requested) {
    if(failed) {
      failed_=true;
      if(mask!=2 || !acknowledgment_requested)pending_=false;
    } else if(linked && acknowledgment_requested)failed_=pending_=false;
  }
  void Authorize(bool policy_accepted) {if(policy_accepted)pending_=true;}
  bool failed() const {return failed_;}
  bool pending() const {return pending_;}
 private:
  bool failed_=false,pending_=false;
};
}
