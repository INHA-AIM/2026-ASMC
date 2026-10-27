#pragma once
#include <algorithm>
#include <cmath>
namespace asmc::lidar::pipeline {
struct ConfirmationEvidence {
  double length=0,width=0,height=0;
  bool ground_valid=false;
  double median_ground_clearance=0;
  bool wall_valid=false;
  double wall_support_fraction=0;
  double wall_axis_sine=1;
};
enum class ConfirmationKind { General=0, GroundFragment=1, WallFragment=2, ThinHorizontalFragment=3, ElevatedFlatFragment=4 };
inline ConfirmationKind ClassifyConfirmation(const ConfirmationEvidence& e) {
  const double L=std::max(e.length,e.width), W=std::min(e.length,e.width);
  if(!std::isfinite(L)||!std::isfinite(W)||!std::isfinite(e.height)||L<=0||W<=0||e.height<0)
    return ConfirmationKind::General;
  // Flat, elongated support at the measured road, not arbitrary negative z.
  if(e.ground_valid && std::isfinite(e.median_ground_clearance) &&
     e.height<=.25 && e.median_ground_clearance>=-.10 &&
     e.median_ground_clearance<=.15 && L/W>=3)
    return ConfirmationKind::GroundFragment;
  // A sparse horizontal sliver may sit above the estimated road (curb/slab,
  // distant ground error). Delay confirmation; never call it semantic ground.
  if(e.ground_valid && std::isfinite(e.median_ground_clearance) &&
     e.height<=.08 && L>=1.4 && L/W>=6 &&
     e.median_ground_clearance>.15 && e.median_ground_clearance<=.60)
    return ConfirmationKind::ThinHorizontalFragment;
  // A broad low plane above the extrapolated road is still suspicious;
  // increasing its wait is not a semantic ground deletion decision.
  if(e.ground_valid && std::isfinite(e.median_ground_clearance) &&
     e.height<=.30 && L>=6 && L/W>=3 &&
     e.median_ground_clearance>.15 && e.median_ground_clearance<=.60)
    return ConfirmationKind::ElevatedFlatFragment;
  // Require actual point support on a current verified wall segment.
  // Axis alone is unreliable for short tall columns.
  const bool strip=L>=1.5 && W<=.8 && L/W>=6 &&
                   std::isfinite(e.wall_axis_sine) && e.wall_axis_sine<=.173648177667;
  const bool column=e.height>=2 && L<=3 && W<=.8;
  if(e.wall_valid && std::isfinite(e.wall_support_fraction) &&
     e.wall_support_fraction>=.8 && (strip||column))return ConfirmationKind::WallFragment;
  return ConfirmationKind::General;
}
}
