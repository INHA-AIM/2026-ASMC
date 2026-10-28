#pragma once
#include "local/planner.hpp"
#include <algorithm>
#include <cmath>

namespace local_planning {
// Experimental navigation cue only. Never fills semantic cells, replaces
// collision checks, marks an invalid family valid, or invents wall positions.
inline std::vector<std::array<double,2>> SemanticReference(const Grid& grid,
                                                         const Config& c) {
  std::vector<std::array<double,2>> out;
  if (grid.rows <= 0 || grid.cols <= 0 || grid.probability.size()!=size_t(grid.rows*grid.cols) ||
      grid.coverage.size()!=grid.probability.size()) return out;
  double previous_y = 0;
  for (int row=grid.rows-1;row>=0;row-=5) {
    const double x=grid.xmax-row*grid.resolution;
    // Near-body ego occlusion splits the road into side strips. Those strips
    // cannot determine the forward navigation reference.
    if (x<c.length-c.rear+c.margin || x>c.max_range) continue;
    double best=1e9, chosen=0; bool found=false;
    for (int col=0;col<grid.cols;) {
      auto free=[&](int col) { const auto i=row*grid.cols+col;
        return grid.coverage[i]==255 && grid.probability[i]>=180; };
      if (!free(col)) { ++col;continue; }
      const int first=col;
      while (col+1<grid.cols && free(col+1)) ++col;
      // One-lane lateral preference envelope prevents a distant side-camera
      // patch from becoming the navigation cue. This is NOT a collision ROI;
      // the planner still checks/uses the full observed raster independently.
      const double left=std::min(3.5,grid.ymax-first*grid.resolution),
                   right=std::max(-3.5,grid.ymax-col*grid.resolution);
      ++col;
      const double half=.5*c.width+c.margin;
      if (left-right < 2*half) continue;
      const double y=std::clamp(right+1.75,right+half,left-half);
      const double cost=std::abs(y-previous_y);
      if (cost<best) { best=cost;chosen=y;found=true; }
    }
    if (!found) continue;
    // Gaps in this navigation cue are not a statement of free space. The
    // independent dense body sweep rejects unknown/blocked intermediate cells.
    if (out.empty() && x>10.) return {};
    auto point=GridToPlan(grid,x,chosen);
    if (!out.empty() && point[0]<=out.back()[0]) break;
    out.push_back({point[0],point[1]});previous_y=chosen;
  }
  if (out.size()<6 || out.back()[0]<8.) return {};
  const auto raw=out;
  for (size_t i=1;i+1<out.size();++i)
    out[i][1]=(raw[i-1][1]+raw[i][1]+raw[i+1][1])/3.;
  // Soft reference at axle only. Untouched raster/road-memory footprint checks
  // still decide physical space, including every obstacle/unknown gap.
  if (out.front()[0]>0.) out.insert(out.begin(),{0.,out.front()[1]});
  return out;
}
}
