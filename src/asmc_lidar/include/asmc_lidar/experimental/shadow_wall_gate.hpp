#pragma once
#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <asmc_lidar/core/frame_time.hpp>
namespace asmc::lidar::pipeline {
struct ShadowWallConfig {
  bool enabled=false;
  double entry_m=0,exit_m=0,lead_m=30,exit_buffer_m=10,lateral_m=6;
  std::string profile_sha256;
  std::vector<std::array<double,3>> route; // official station, map x, map y
};
struct ShadowWallDecision {
  bool active=false;
  double station_m=-1,distance_to_entry_m=-1,lateral_m=-1;
  std::string reason="disabled";
};
inline void ValidateShadowWall(const ShadowWallConfig& c) {
  if(!c.enabled)return;
  if(c.route.size()<2||!std::isfinite(c.entry_m)||!std::isfinite(c.exit_m)||c.entry_m>=c.exit_m||
     !std::isfinite(c.lead_m)||c.lead_m<5||c.lead_m>100||
     !std::isfinite(c.exit_buffer_m)||c.exit_buffer_m<0||c.exit_buffer_m>50||
     !std::isfinite(c.lateral_m)||c.lateral_m<=0||c.lateral_m>10)
    throw std::invalid_argument("invalid shadow wall gate profile");
  double last=-INFINITY;
  for(const auto& p:c.route) {
    if(!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2])||p[0]<=last)
      throw std::invalid_argument("invalid shadow wall route sample");
    last=p[0];
  }
}
inline ShadowWallDecision ShadowWallGate(const ShadowWallConfig& c,const PoseSelection& pose,bool was_active) {
  ShadowWallDecision out;if(!c.enabled)return out;
  out.reason="map_pose_unavailable";if(!pose.valid||!pose.map_from_base.matrix().allFinite())return out;
  const double x=pose.map_from_base.translation().x(),y=pose.map_from_base.translation().y();
  double xmin=INFINITY,xmax=-INFINITY,ymin=INFINITY,ymax=-INFINITY;
  for(const auto& p:c.route){xmin=std::min(xmin,p[1]);xmax=std::max(xmax,p[1]);ymin=std::min(ymin,p[2]);ymax=std::max(ymax,p[2]);}
  out.reason="far_from_shadow_route";
  if(x<xmin-c.lateral_m||x>xmax+c.lateral_m||y<ymin-c.lateral_m||y>ymax+c.lateral_m)return out;
  double best=INFINITY;out.reason="off_route_or_heading";
  for(std::size_t i=1;i<c.route.size();++i) {
    const auto& a=c.route[i-1];const auto& b=c.route[i];const double dx=b[1]-a[1],dy=b[2]-a[2],l2=dx*dx+dy*dy;
    if(l2<1e-8)continue;
    // The reviewed route has a direction. Opposite/crossing roads cannot arm it.
    if((dx*pose.map_from_base.linear()(0,0)+dy*pose.map_from_base.linear()(1,0))/std::sqrt(l2)<.5)continue;
    const double u=std::clamp(((x-a[1])*dx+(y-a[2])*dy)/l2,0.,1.);
    const double distance=std::hypot(x-a[1]-u*dx,y-a[2]-u*dy);
    if(distance<best){best=distance;out.station_m=a[0]+u*(b[0]-a[0]);}
  }
  if(best>c.lateral_m)return out;
  out.lateral_m=best;out.distance_to_entry_m=c.entry_m-out.station_m;
  const double hysteresis=was_active?3.:0.;
  out.active=out.station_m>=c.entry_m-c.lead_m-hysteresis&&out.station_m<=c.exit_m+c.exit_buffer_m+hysteresis;
  out.reason=out.active?"shadow_wall_window":"outside_shadow_wall_window";return out;
}
}
