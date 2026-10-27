#pragma once
#include <asmc_lidar/experimental/tunnel_filter_core.hpp>
#include <vector>
#include <cmath>
namespace asmc::lidar::pipeline {
// Evidence for a longer CONFIRMATION wait only, never permission to erase.
// A short current fit needs all other geometric checks and an independently
// confirmed opposite wall. Its measured intervals remain bounded.
inline std::vector<tunnel_filter::WallSegment> ConfirmationWallSegments(
    const tunnel_filter::FrameResult& frame,const tunnel_filter::Params& params) {
  std::vector<tunnel_filter::WallSegment> segments;
  for(std::size_t side=0;side<2;++side) {
    const auto& wall=frame.walls[side];
    if(wall.removal_allowed)for(const auto& seg:wall.segments)if(seg.valid)segments.push_back(seg);
    if(!frame.walls[1-side].removal_allowed || frame.walls[1-side].segments.empty())continue;
    auto candidate=wall.first_attempt;
    if(candidate.reject_reason!="short_span" && candidate.reject_reason!="few_bins")continue;
    if(!tunnel_filter::ValidateWallSegment(&candidate,params,-1.f,4.f,2))continue;
    const auto& opposite=frame.walls[1-side].segments.front();
    const float dot=(candidate.plane.a*opposite.plane.a+candidate.plane.b*opposite.plane.b)/
      (std::hypot(candidate.plane.a,candidate.plane.b)*std::hypot(opposite.plane.a,opposite.plane.b));
    if(dot>-.984807753f)continue; // within ten degrees of opposite normals
    segments.push_back(candidate);
  }
  return segments;
}
}
