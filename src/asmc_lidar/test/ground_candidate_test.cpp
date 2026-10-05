#include <asmc_lidar/general.hpp>
#include <asmc_lidar/road_anchor.hpp>
#include <asmc_lidar/ground_live.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
using namespace asmc_lidar;
void Add(Cloud& c,float x,float y,float z) {
  pcl::PointXYZI p;p.x=x;p.y=y;p.z=z;p.intensity=42;c.push_back(p);
}
int main() {
  Cloud scene;
  for(int x=-20;x<=20;++x)for(int y=-15;y<=15;++y)Add(scene,x*.5f,y*.5f,-1.8f);
  const auto floor_points=scene.size();
  for(int x=-50;x<=50;++x)for(int y=-30;y<=30;++y)Add(scene,x*.25f,y*.25f,2.f);
  std::vector<GroundPlaneAttempt> before,after;
  const auto old=ProcessGeneralGroundLeveled(scene,&before);
  const auto candidate=ProcessGeneralGroundLeveled(scene,&after,true);
  assert(before.front().accepted_ground&&before.front().height_at_origin_m>0);
  assert(!after.front().accepted_ground&&after.back().accepted_ground);
  // PCL optimized FLOAT32 plane accumulation is not the double transform core:
  // measured flat-scene intercept error is 2.31e-5 m. Allow 0.1 mm here only.
  assert(std::abs(after.back().height_at_origin_m+1.8)<1e-4);
  for(std::size_t i=0;i<floor_points;++i)assert(candidate.labels[i]==tunnel_filter::kGround);
  for(std::size_t i=floor_points;i<scene.size();++i)assert(candidate.labels[i]==tunnel_filter::kKept);
  assert(candidate.kept_points==scene.size()-floor_points);
  Cloud slope;
  for(int x=-30;x<=30;++x)for(int y=-20;y<=20;++y)Add(slope,x*.4f,y*.4f,-1.8f+.08f*x*.4f+.03f*y*.4f);
  auto anchor=FitRoadAnchor(slope);assert(anchor.valid);
  assert(std::abs(anchor.HeightAt(10,2)-(-1.8+.8+.06))<1e-4);
  pcl::PointXYZI p;p.x=10;p.y=2;p.z=0;assert(anchor.Supports(p));
  p.x=40;assert(!anchor.Supports(p)); // no unlimited extrapolation
  Cloud false_surface;
  for(int x=-20;x<=20;++x)for(int y=-20;y<=20;++y)Add(false_surface,x*.5f,y*.5f,-.8f);
  assert(!FitRoadAnchor(false_surface).valid);
  assert(!FitRoadAnchor(Cloud{}).valid);
  Cloud layered=scene;
  for(float height:{3.f,4.f})for(int x=-50;x<=50;++x)for(int y=-30;y<=30;++y)
    Add(layered,x*.25f,y*.25f,height);
  GroundPrepared prepared;prepared.leveled=layered.makeShared();prepared.voxel=layered.makeShared();
  for(std::size_t i=0;i<layered.size();++i)prepared.raw_to_voxel.push_back(int(i));
  GroundComparisonConfig cfg;cfg.id="test_support";cfg.leveling_mode="roll_pitch";
  cfg.jaeho_ground_policy="height_support";
  GroundComparison supported("jaeho","guard25",cfg);
  const auto result=supported.ProcessPrepared(prepared,layered.size()).methods.at("jaeho");
  assert(result.events.at("ground_anchor_used")==1);
  assert(result.events.at("ground_fallback")==0);
  assert(result.labels.size()==layered.size());
  assert(result.planes.back().attempt==4&&result.planes.back().accepted_ground);
  assert(result.events.at("ground_anchor_supported_points")>30);
  std::cout<<"ceiling search exclusion, point retention, slope anchor, bounded support and invalid input passed\n";
}
