#pragma once
#include "Frenet/Geometry/CartesianTypes.hpp"
#include <cmath>
#include <algorithm>
#include <optional>
#include <string>
#include <utility>

namespace shadow_localization {
// Spatial history only: never replays an old command, speed profile or solver.
// The consumer must revalidate the returned path against every CURRENT input.
class PathMemory {
 public:
  void clear(){path_={};stamp_=0;}
  void remember(CartesianPath path,double stamp){path_=std::move(path);stamp_=stamp;}
  double age(double now)const{return now-stamp_;}
  std::optional<CartesianPath> remaining(const CartesianState& ego,double now,std::string& reason)const {
    if(path_.x.size()<3){reason="no_previous_path";return {};}
    if(!std::isfinite(now)||now<stamp_||now-stamp_>.6){reason="previous_path_expired";return {};}
    size_t closest=0;double distance=INFINITY;
    for(size_t i=0;i<path_.x.size();++i){
      const double d=std::hypot(path_.x[i]-ego.x,path_.y[i]-ego.y);
      if(d<distance){distance=d;closest=i;}
    }
    if(distance>.35||std::abs(std::remainder(path_.yaw[closest]-ego.yaw,2*M_PI))>.25){
      reason="previous_path_ego_mismatch";return {};
    }
    if(closest+2>=path_.x.size()){reason="previous_path_exhausted";return {};}
    CartesianPath out;
    out.x.push_back(ego.x);out.y.push_back(ego.y);out.yaw.push_back(ego.yaw);
    out.kappa.push_back(path_.kappa[closest]);out.v.push_back(std::max(0.,ego.v));out.a.push_back(ego.a);
    for(size_t i=closest+1;i<path_.x.size();++i){
      out.x.push_back(path_.x[i]);out.y.push_back(path_.y[i]);out.yaw.push_back(path_.yaw[i]);
      out.kappa.push_back(path_.kappa[i]);out.v.push_back(path_.v[i]);out.a.push_back(0.);
    }
    reason="requires_current_scene_validation";return out;
  }
 private:
  CartesianPath path_;double stamp_=0;
};
}
