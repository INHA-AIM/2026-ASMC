#pragma once
#include <asmc_lidar/pipeline/pipeline.hpp>

namespace asmc::lidar::pipeline {
// The CA tracker first re-expresses the previous position and derivatives into
// today's sensor orientation (ScanFrame::AlignState). Consequently its velocity
// already excludes ego angular motion: adding omega x p here would count yaw
// compensation twice. Rotate its translation-relative estimate into base axes,
// add ego translation once, then rotate into map. This is a planar estimate.
inline std::optional<Vec> PlanningVelocity(const TrackedObject& object,
                                         const ScanResult& scan) {
  const auto& t = object.tracking;
  const auto& c = scan.context;
  const auto base = base_from_input(c);
  if (!base || !c.map_pose.valid || c.map_pose.query_stamp_ns != c.time.stamp_ns ||
      !t.b_KfInitialized || t.s32_KfInitCount < 2 || t.s32_MissCount != 0 ||
      !t.tracking_frame_valid || !std::isfinite(scan.dt_s) || scan.dt_s <= .01 || scan.dt_s > .3 ||
      !t.Eigen_KfState_CA.allFinite() || !c.ego_linear_base.allFinite()) return std::nullopt;
  const Vec relative(t.Eigen_KfState_CA(2), t.Eigen_KfState_CA(3), 0.);
  const Vec velocity = c.map_pose.map_from_base.linear() *
      (base->linear() * relative + c.ego_linear_base);
  if (!velocity.allFinite() || velocity.norm() > 60.) return std::nullopt;
  return velocity;
}

}  // namespace asmc::lidar::pipeline
