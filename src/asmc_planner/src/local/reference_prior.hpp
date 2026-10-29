#pragma once
#include "local/planner.hpp"
#include <cmath>

namespace local_planning {
// Navigation preference only: a short, source-aged family reference in the
// existing relative-odometry frame. Never supplies road/obstacle clearance.
class ReferencePrior {
  std::vector<std::array<double,2>> points_;
  Pose origin_;
  double stamp_ = -1.;
public:
  void clear() { points_.clear(); stamp_=-1.; }
  void observe(const std::vector<std::array<double,2>>& points,
               const Pose& origin, double source_stamp) {
    if (points.size()<2 || !std::isfinite(source_stamp) || source_stamp<=stamp_) return;
    points_=points;origin_=origin;stamp_=source_stamp;
  }
  double age(double now) const { return stamp_<0 ? -1. : now-stamp_; }
  std::vector<std::array<double,2>> aligned(const Pose& origin,double now) const {
    if (points_.empty() || age(now)<0 || age(now)>2. ||
        std::hypot(origin.x-origin_.x,origin.y-origin_.y)>6. ||
        std::abs(std::remainder(origin.yaw-origin_.yaw,2*M_PI))>.2) return {};
    std::vector<std::array<double,2>> out;out.reserve(points_.size());
    const double c=std::cos(origin.yaw),s=std::sin(origin.yaw),
                 a=std::cos(origin_.yaw),b=std::sin(origin_.yaw);
    for (const auto& p:points_) {
      const double dx=origin_.x+a*p[0]-b*p[1]-origin.x,
                   dy=origin_.y+b*p[0]+a*p[1]-origin.y;
      out.push_back({c*dx+s*dy,-s*dx+c*dy});
    }
    return out;
  }
};
}
