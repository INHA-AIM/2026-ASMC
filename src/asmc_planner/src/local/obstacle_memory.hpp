#pragma once
#include "local/planner.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace local_planning {
// Planning-only bounded retention of an already measured hull. No association,
// KF, new class/motion decision, point cloud, or new LiDAR publication. Original
// source age is retained; this is not a fresh detection or a static-velocity claim.
class ObstacleMemory {
  struct Entry { Obstacle world; double stamp=0; unsigned scans=0; };
  std::map<int,Entry> entries;
  double last_source=0;
  static bool asset(const Obstacle &o) {
    if(o.classification!="static_obstacle" || o.moving) return false;
    const auto &n=o.camera_class;
    // motorcycle belongs to the mission static group, but is not an
    // immovable asset. Unknown/vehicle/dynamic are not silently made static.
    return n=="box" || n=="bollard" || n=="traffic_drum" ||
        n=="plastic_barrier" || n=="concrete_barrier" || n=="steel_barricade";
  }
  static void transform(geometry::Polygon2D &poly,const Pose &origin,bool inverse) {
    const double c=std::cos(origin.yaw),s=std::sin(origin.yaw);
    for(auto &p:poly) {
      const double x=p.x-(inverse?origin.x:0.), y=p.y-(inverse?origin.y:0.);
      p={c*x+(inverse?s:-s)*y+(inverse?0.:origin.x),
         (inverse?-s:s)*x+c*y+(inverse?0.:origin.y)};
    }
  }
public:
  void clear() { entries.clear();last_source=0; }
  size_t size() const { return entries.size(); }
  void apply(std::vector<Obstacle> &current,const Pose &origin,double source,double now) {
    if(source<last_source || !std::isfinite(source+now) || source>now) {clear();return;}
    for(auto it=entries.begin();it!=entries.end();) {
      if(now-it->second.stamp>.20) it=entries.erase(it); else ++it;
    }
    std::set<int> seen;
    for(const auto &o:current) {
      seen.insert(o.id);
      if(!asset(o)) { entries.erase(o.id); continue; }
      if(source==last_source) continue; // Replanning does not confirm/refresh a scan.
      auto &e=entries[o.id];
      e.scans=(e.stamp>0 && e.world.camera_class==o.camera_class)?e.scans+1:1;
      e.world=o;e.world.retained=false;e.stamp=source;
      transform(e.world.polygon,origin,false);
    }
    for(const auto &item:entries) {
      const auto &e=item.second;
      const double age=now-e.stamp;
      if(seen.count(item.first) || e.scans<3 || age<0 || age>.20) continue;
      auto o=e.world;
      o.retained=true;o.source_age=age;
      o.uncertainty=age*o.motion_budget;
      transform(o.polygon,origin,true);
      current.push_back(std::move(o));
    }
    last_source=source;
    // Bounded state: prune the oldest if an implausibly crowded scan exceeds64.
    while(entries.size()>64) {
      auto oldest=std::min_element(entries.begin(),entries.end(),[](const auto &a,const auto &b){return a.second.stamp<b.second.stamp;});
      entries.erase(oldest);
    }
  }
};
} // namespace local_planning
