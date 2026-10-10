#include "Behavior/Decision/behavior_scorer.hpp"
namespace behavior_planner {

BehaviorScore BehaviorScorer::score(const UrbanFeature& feature,
                                    const HardConstraint& hard,
                                    bool persisted_follow_target) const {
  BehaviorScore score;
  score.keep = 70.0;
  score.follow = feature.surrounding.has_front_vehicle
                     ? 65.0
                     : (persisted_follow_target ? 75.0 : -1e6);
  if (feature.surrounding.has_front_vehicle) {
    const double ttc = feature.surrounding.front_vehicle.ttc;
    score.keep -= ttc < 3.0 ? 35.0 : 10.0;
    score.follow += ttc < 3.0 ? 20.0 : 5.0;
  }
  // Static obstacles take precedence over KEEP/FOLLOW. Frenet owns the
  // lateral direction and rejects unsafe candidates before COMMIT.
  score.avoid = feature.risk.static_obstacle_ahead ? 90.0 : -1e6;
  // Temporary FOLLOW/AVOID verification gate: lane-change remains disabled
  // until its Behavior FSM transitions are implemented.
  score.left_change = -1e6;
  score.right_change = -1e6;
  score.stop = (hard.planned_stop || hard.force_stop) ? 1e5 : -1e6;
  score.emergency_stop = hard.emergency_stop ? 1e6 : -1e6;
  return score;
}

}  // namespace behavior_planner
