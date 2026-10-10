#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include "planner_baseline_fsm.hpp"
#include "Behavior/Decision/fsm.hpp"
using namespace behavior_planner;
int main() {
  std::mt19937 rng(20261008);
  std::size_t operations = 0;
  for (int scenario=0; scenario<200; ++scenario) {
    BaselineFsm before; Fsm after;
    const double delay = static_cast<int>(rng()%10)-2;
    const double exit_ttc = static_cast<int>(rng()%8)-1;
    const double hold = (static_cast<int>(rng()%10)-2)*0.1;
    const int grace = static_cast<int>(rng()%8)-2;
    before.setFollowReleaseDelay(delay); after.setFollowReleaseDelay(delay);
    before.setEmergencyRelease(exit_ttc,hold); after.setEmergencyRelease(exit_ttc,hold);
    before.setAvoidReleaseGraceCycles(grace); after.setAvoidReleaseGraceCycles(grace);
    double timestamp=0;
    uint32_t command=1;
    for (int step=0; step<1000; ++step) {
      timestamp += (static_cast<int>(rng()%8)-2)*0.1;
      double t = rng()%40==0 ? std::numeric_limits<double>::quiet_NaN() : timestamp;
      if (rng()%9==0) command=rng()%5;
      const auto candidate = static_cast<BehaviorType>(rng()%9);
      assert(before.update(candidate)==after.update(candidate));
      const bool emergency = rng()%12==0;
      const double ttc = rng()%40==0 ? std::numeric_limits<double>::quiet_NaN() : (rng()%80)*0.1;
      assert(before.updateEmergency(emergency,ttc,t)==after.updateEmergency(emergency,ttc,t));
      assert(before.emergencyLatched()==after.emergencyLatched());
      const bool observed=rng()%3==0;
      const int id=static_cast<int>(rng()%6)-1;
      assert(before.updateFollowTarget(observed,id,t)==after.updateFollowTarget(observed,id,t));
      assert(before.followReleaseGraceActive()==after.followReleaseGraceActive());
      PlannerFeedbackData feedback;
      feedback.valid=rng()%4!=0;
      feedback.command_id=rng()%3==0 ? rng()%5 : command;
      feedback.execution_state=static_cast<PlannerExecutionState>(rng()%8);
      const bool stop=rng()%4==0, moving=rng()%10==0, follow=rng()%10==0;
      const auto x=before.updateManeuver(candidate,feedback,command,stop,moving,follow);
      const auto y=after.updateManeuver(candidate,feedback,command,stop,moving,follow);
      if (x.behavior!=y.behavior || x.state!=y.state || x.phase!=y.phase) {
        std::cerr << "FSM mismatch at scenario " << scenario << " step " << step << '\n';
        return 1;
      }
      ++operations;
    }
  }
  std::cout << operations << " identical FSM cycles (legacy mapping, emergency, FOLLOW, lifecycle)\n";
}
