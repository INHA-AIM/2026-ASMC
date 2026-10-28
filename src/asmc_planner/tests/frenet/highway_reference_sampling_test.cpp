#include "Execution/HighwayReferenceExecution.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace asmc_planner::lane_reference;
using asmc_planner::Point2D;
namespace {
int failures=0;
void expect(bool ok,const char* label) {
  std::cout<<(ok?"PASS ":"FAIL ")<<label<<'\n';failures+=!ok;
}
std::vector<Point2D> rectangle(double x0,double x1,double y0,double y1) {
  return {{x0,y0},{x1,y0},{x1,y1},{x0,y1}};
}
LaneMapData fixture(bool dense=false) {
  LaneMapData data;data.identity={"map","sampling-v1","sampling",std::string(64,'1')};
  data.complete=true;
  LaneData a,b;a.id="A";a.link_id="a";a.traversable=true;a.successors={"B"};
  b.id="B";b.link_id="b";b.traversable=true;b.predecessors={"A"};
  for(int i=0;i<60;++i)a.centerline.push_back({double(i),0});
  a.centerline.push_back({59.902,0});a.centerline.push_back({60,0});
  for(int i=0;i<=60;++i)b.centerline.push_back({60.+i,-.08*i});
  for(auto* lane:{&a,&b}) {
    lane->polygon=rectangle(-10,130,-10,10);
    if(dense) {
      std::vector<Point2D> points;
      for(std::size_t i=1;i<lane->centerline.size();++i) {
        const auto p=lane->centerline[i-1],q=lane->centerline[i];
        for(int n=0;n<8;++n)points.push_back({p.x+(q.x-p.x)*n/8.,p.y+(q.y-p.y)*n/8.});
      }
      points.push_back(lane->centerline.back());lane->centerline=std::move(points);
    }
  }
  data.lanes={a,b};data.areas={{"road",rectangle(-10,130,-10,10),true}};
  return data;
}
LaneReferenceOptions options() {
  LaneReferenceOptions o;o.reference={.05,10,.5,300,.3,30};o.vehicle={3.845,.79,.95};
  o.max_curvature=.23;o.max_lateral_accel=2.6;o.max_join_gap_m=.05;
  o.sample_distance_m=.2;o.sample_yaw_rad=.02;o.max_samples=30000;return o;
}
double joinPeak(const RefLine& ref) {
  double peak=0;
  for(const auto& p:ref.points)if(p.x>60&&p.y<-200&&p.y>-235)
    peak=std::max(peak,std::abs(p.kappa));
  return peak;
}
void densityAndSafety() {
  std::string error;const auto data=fixture();
  const auto sparse=HdLaneMap::create(data,error),dense=HdLaneMap::create(fixture(true),error);
  if(!sparse||!dense)throw std::runtime_error("sampling fixture: "+error);
  const auto o=options();
  const auto first=BuildLaneReference(sparse,sparse->identity(),{"A","B"},o,LaneReferenceSampling::UNIFORM_ARC_LENGTH);
  const auto second=BuildLaneReference(dense,dense->identity(),{"A","B"},o,LaneReferenceSampling::UNIFORM_ARC_LENGTH);
  expect(bool(first)&&bool(second),"same connected geometry prepares at different source densities");
  if(!first||!second)throw std::runtime_error(first.detail+" / "+second.detail);
  const auto& a=first.candidate->line();const auto& b=second.candidate->line();
  bool equal=a.points.size()==b.points.size();
  for(std::size_t i=0;equal&&i<a.points.size();++i)
    equal=std::hypot(a.points[i].x-b.points[i].x,a.points[i].y-b.points[i].y)<1e-8&&
          std::abs(a.points[i].kappa-b.points[i].kappa)<1e-8;
  expect(equal,"position and curvature are invariant to added collinear source vertices");
  std::vector<double> x,y;
  for(const auto& lane:data.lanes)for(const auto& p:lane.centerline)
    if(x.empty()||std::hypot(p.x-x.back(),p.y-y.back())>1e-9){x.push_back(p.x);y.push_back(p.y);}
  const auto baseline=BuildRefLine(x,y,o.reference,o.max_curvature);
  const auto legacy=BuildLaneReference(sparse,sparse->identity(),{"A","B"},o);
  bool unchanged=bool(legacy)&&legacy.candidate->line().points.size()==baseline.points.size();
  for(std::size_t i=0;unchanged&&i<baseline.points.size();++i) {
    const auto& p=legacy.candidate->line().points[i];const auto& q=baseline.points[i];
    unchanged=p.x==q.x&&p.y==q.y&&p.kappa==q.kappa;
  }
  expect(unchanged,"existing callers retain the original reference construction exactly");
  expect(!BuildLaneReference(sparse,sparse->identity(),{"B","A"},o,LaneReferenceSampling::UNIFORM_ARC_LENGTH),
    "sampling never invents a disconnected successor");
  auto small=o;small.max_samples=3;
  expect(!BuildLaneReference(sparse,sparse->identity(),{"A","B"},small,LaneReferenceSampling::UNIFORM_ARC_LENGTH),
    "input resampling remains bounded by the sample budget");
  auto blocked=data;blocked.markings={{"solid",{{30,-10},{30,10}},.15,PaintPattern::SOLID,false,false}};
  auto map=HdLaneMap::create(blocked,error);
  expect(map&&!BuildLaneReference(map,map->identity(),{"A","B"},o,LaneReferenceSampling::UNIFORM_ARC_LENGTH),
    "normalized reference still rejects solid-line contact");
  blocked=data;blocked.areas={{"narrow",rectangle(-10,130,-.2,.2),true}};
  map=HdLaneMap::create(blocked,error);
  expect(map&&!BuildLaneReference(map,map->identity(),{"A","B"},o,LaneReferenceSampling::UNIFORM_ARC_LENGTH),
    "normalized reference still requires the complete vehicle footprint on road");
}
void officialHighway() {
  FrenetPlanner mission;std::string error;
  if(!mission.Init(FRENET_TEST_CONFIG)||!mission.EnableHighwayMerge(error))throw std::runtime_error(error);
  const auto unchanged=mission.reference();const auto route=mission.highway_route();
  asmc_planner::execution::ManeuverState state;
  HighwayReferenceExecution execution(mission,FRENET_TEST_CONFIG,state);
  for(std::size_t z=0;z<route->zones.size();++z) {
    const auto& reference=execution.target(z);
    const auto old=BuildLaneReference(route->map,route->map->identity(),route->zones[z].config.target_lanes,
                                      reference.options());
    if(!old)throw std::runtime_error(old.detail);
    const double before=joinPeak(old.candidate->line()),after=joinPeak(reference.line());
    std::cout<<"JOIN zone="<<z<<" before="<<before<<" after="<<after<<'\n';
    expect(before>.03&&after>.005&&after<.025&&after<before*.6,
      "official highway join no longer acquires the source-sampling curvature spike");
    expect(reference.usableEnd()>reference.usableBegin()&&!reference.line().closed_loop,
      "normalized lane retains a finite certified driving interval");
  }
  bool same=mission.reference().points.size()==unchanged.points.size();
  for(std::size_t i=0;same&&i<unchanged.points.size();++i) {
    const auto& p=mission.reference().points[i];const auto& q=unchanged.points[i];
    same=p.x==q.x&&p.y==q.y&&p.kappa==q.kappa;
  }
  expect(same,"highway reference preparation leaves the mission global reference unchanged");
}
}
int main() {
  try{densityAndSafety();officialHighway();}catch(const std::exception& e){++failures;std::cerr<<e.what()<<'\n';}
  std::cout<<"failures="<<failures<<'\n';return failures?1:0;
}
