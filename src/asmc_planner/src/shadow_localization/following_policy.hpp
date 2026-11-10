#pragma once
#include <cmath>
#include <optional>
namespace shadow_localization {
// Frenet FOLLOW measures rear-axle-to-object-centre distance. Convert the
// requested bumper gap explicitly; feeding 3m directly would overlap bodies.
inline std::optional<double> FollowingReferenceGap(double ego_front,double leader_center,
                                                  double leader_rear,double bumper_gap) {
  if(!std::isfinite(ego_front+leader_center+leader_rear+bumper_gap)||
     ego_front<=0||bumper_gap<2.||bumper_gap>8.||leader_rear>leader_center)return {};
  return ego_front+(leader_center-leader_rear)+bumper_gap;
}
}
