#include "shadow_localization/static_planner.hpp"
#include <chrono>

namespace shadow_localization {
using namespace local_planning;
Result StaticPlanner::reference(const Scene& scene,const Config& cfg,
                               const Motion& origin,const Transform& map,bool recovery) {
  const auto start=std::chrono::steady_clock::now();
  Result out;out.recovery=recovery;phase_="REFERENCE";native_calls_=speed_profiles_=0;
  if(!map.valid){out.reason="shadow_anchor_unavailable";return out;}
  const auto xy=map.apply(origin.x,origin.y);
  CartesianState ego{xy[0],xy[1],origin.yaw+map.yaw,0,scene.speed,scene.acceleration};
  double s=0,d=0;
  const bool projected=std::isfinite(reference_s_)
      ? core_->ProjectPointToFrenet(ego.x,ego.y,reference_s_,30.,4.,s,d)
      : core_->ProjectEgoToS(ego,s);
  CartesianState guide;
  if(!projected||!core_->ReferencePoseAt(s,guide)||
     std::hypot(ego.x-guide.x,ego.y-guide.y)>1.5||
     std::abs(std::remainder(ego.yaw-guide.yaw,2*M_PI))>.65){
    out.reason="reference_localization_mismatch";return out;
  }
  reference_s_=s;
  const double cs=std::cos(ego.yaw),sn=std::sin(ego.yaw);
  auto clear=[&](const Pose& p){ return footprint(scene,cfg,p,out,origin,map); };
  if(!clear(Pose{}))return out;
  for(double ds=0;ds<=cfg.max_range;ds+=.25){
    CartesianState point;
    if(!core_->ReferencePoseAt(s+ds,point))break;
    const double dx=point.x-ego.x,dy=point.y-ego.y;
    Pose p{cs*dx+sn*dy,-sn*dx+cs*dy,
      std::remainder(point.yaw-ego.yaw,2*M_PI),point.kappa,0};
    if(std::abs(p.k)>cfg.max_k)break;
    if(!out.path.empty()){
      const auto& previous=out.path.back();
      const double distance=std::hypot(p.x-previous.x,p.y-previous.y);
      if(distance<1e-5)break; // open reference endpoint, no wrapping
      p.s=previous.s+distance;
    }
    const Pose from=out.path.empty()?Pose{}:out.path.back();
    p.yaw=from.yaw+std::remainder(p.yaw-from.yaw,2*M_PI);
    const int n=std::max(1,int(std::ceil((std::hypot(p.x-from.x,p.y-from.y)+
                              cfg.length*std::abs(p.yaw-from.yaw))/.1)));
    bool valid=true;
    for(int i=1;i<=n;++i){const double q=double(i)/n;
      if(!clear(Pose{from.x+q*(p.x-from.x),from.y+q*(p.y-from.y),
                     from.yaw+q*(p.yaw-from.yaw),p.k,0})){valid=false;break;}}
    if(!valid)break;
    out.path.push_back(p);
  }
  if(out.path.size()<3){if(out.reason=="no_path")out.reason="reference_no_verified_prefix";return out;}
  // This baseline verifies map road + current local OBB occupancy; it makes
  // no claim of semantic free-space observation. Empty-scene trial ownership
  // is a separate explicit boundary. Obstacles/unknown are never erased here.
  out.search_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  ++speed_profiles_;return TimeParameterize(scene,cfg,std::move(out));
}
} // namespace shadow_localization
