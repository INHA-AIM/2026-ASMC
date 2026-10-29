#pragma once
#include "local/planner.hpp"
namespace local_planning {
// Optional one-frame speckle evidence: never changes raw probability/coverage.
// Only tiny enclosed far-field holes with fresh preceding clear-road evidence
// qualify. RoadMemory's newer-nonroad veto prevents persistent suppression.
inline unsigned BuildSemanticNoiseMask(Grid &g,const Config &c,const std::vector<Obstacle> &objects) {
  g.transient_nonroad.clear();
  if(!c.semantic_speckle_guard || g.recent_road_age.size()!=g.coverage.size())return 0;
  g.transient_nonroad.assign(g.rows*g.cols,0);
  std::vector<uint8_t> seen(g.coverage.size(),0);unsigned total=0,components=0;
  for(int seed=0;seed<g.rows*g.cols;++seed) {
    if(seen[seed] || g.coverage[seed]!=255 || g.probability[seed]>=153)continue;
    std::vector<int> todo{seed};seen[seed]=1;bool ok=true;
    for(size_t k=0;k<todo.size();++k) {
      const int i=todo[k],r=i/g.cols,col=i%g.cols;
      const double x=g.xmax-r*g.resolution;
      if(x<8. || x>16. || g.recent_road_age[i]<=0. || g.recent_road_age[i]>.35)ok=false;
      for(int dr=-1;dr<=1;++dr)for(int dc=-1;dc<=1;++dc) {
        const int rr=r+dr,cc=col+dc;
        if(rr<0 || rr>=g.rows || cc<0 || cc>=g.cols){ok=false;continue;}
        const int j=rr*g.cols+cc;
        if(g.coverage[j]!=255){ok=false;continue;}
        if(g.probability[j]<153) {if(!seen[j]){seen[j]=1;todo.push_back(j);}}
        else if(g.probability[j]<204)ok=false;
      }
    }
    if(!ok || todo.size()>3)continue;
    for(int i:todo) {
      const double x=g.xmax-(i/g.cols)*g.resolution,y=g.ymax-(i%g.cols)*g.resolution;
      geometry::Polygon2D cell;
      for(auto d:std::vector<std::array<double,2>>{{-.5*g.resolution,-.5*g.resolution},{.5*g.resolution,-.5*g.resolution},{.5*g.resolution,.5*g.resolution},{-.5*g.resolution,.5*g.resolution}}) {
        const auto q=GridToPlan(g,x+d[0],y+d[1]);cell.push_back({q[0],q[1]});
      }
      for(const auto &o:objects)
        if(geometry::ConvexPolygonDistance(cell,o.polygon)<=.5+o.uncertainty+o.extra_clearance)ok=false;
    }
    if(ok){++components;for(int i:todo){g.transient_nonroad[i]=1;++total;}}
  }
  if(components>2){std::fill(g.transient_nonroad.begin(),g.transient_nonroad.end(),0);return 0;}
  return total;
}
}
