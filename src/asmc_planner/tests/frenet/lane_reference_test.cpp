#include "Execution/ReferenceHandover.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

using namespace asmc_planner;
using namespace asmc_planner::lane_reference;
namespace {
int checks=0,failures=0;
void expect(bool yes,const std::string& name) {
  ++checks;
  if (!yes) { ++failures; std::cerr<<"FAIL: "<<name<<'\n'; }
}
std::vector<Point2D> rectangle(double x0,double x1,double y0,double y1) {
  return {{x0,y0},{x1,y0},{x1,y1},{x0,y1}};
}
LaneData lane(const std::string& id,double x0,double x1,double y) {
  LaneData l; l.id=id; l.link_id="link:"+id; l.traversable=true;
  for (double x=x0;x<=x1;x+=1.0) l.centerline.push_back({x,y});
  l.polygon=rectangle(x0,x1,y-1.75,y+1.75);
  return l;
}
LaneMapData fixture() {
  LaneMapData d; d.identity={"map","fixture-v1","test-map",std::string(64,'1')}; d.complete=true;
  auto a=lane("A",0,100,0), b=lane("B",0,100,3.5), c=lane("C",100,160,3.5);
  a.left_adjacent="B"; b.right_adjacent="A"; b.successors={"C"}; c.predecessors={"B"};
  d.lanes={a,b,c}; d.areas={{"road",rectangle(-10,170,-3,7),true}};
  d.markings={{"divider",{{0,1.75},{160,1.75}},.15,PaintPattern::BROKEN,false,false},
    {"left_edge",{{-10,5.25},{170,5.25}},.15,PaintPattern::SOLID,false,false},
    {"stop",{{30,-3},{30,7}},.4,PaintPattern::SOLID,true,false}};
  return d;
}
LaneReferenceOptions options() {
  LaneReferenceOptions o;
  o.reference={.05,10.,.5,300,.3,30.};
  o.vehicle={3.845,.79,.95};
  o.max_curvature=.23; o.max_lateral_accel=2.6; o.max_join_gap_m=.05;
  o.sample_distance_m=.2; o.sample_yaw_rad=.02; o.footprint_margin_m=0.; o.max_samples=30000;
  return o;
}
HandoverOptions gateOptions() { return {.4,.1,.2,.5,.3,.3,.21}; }
HandoverObservation observation(double stamp,double y=3.5,double yaw=0.) {
  HandoverObservation o; o.now_s=stamp; o.ego_stamp_s=stamp; o.objects_stamp_s=stamp;
  o.ego_valid=true; o.objects_valid=true; o.ego={50.,y,yaw,0.,8.,0.}; return o;
}
void mapTests() {
  std::string error;
  const auto good=HdLaneMap::create(fixture(),error);
  expect(bool(good)&&error.empty(),"complete global map accepted");
  expect(good->laneCount()==3,"all lanes retained");
  auto d=fixture(); d.complete=false;
  expect(!HdLaneMap::create(d,error),"partial local crop rejected");
  d=fixture(); d.identity.frame_id="base_link";
  expect(!HdLaneMap::create(d,error),"wrong coordinate frame rejected");
  d=fixture(); d.identity.geometry_sha256="missing";
  expect(!HdLaneMap::create(d,error),"invalid geometry identity rejected");
  d=fixture(); d.lanes.push_back(d.lanes[0]);
  expect(!HdLaneMap::create(d,error),"duplicate lane ID rejected");
  d=fixture(); d.lanes[1].successors={"missing"};
  expect(!HdLaneMap::create(d,error),"dangling successor rejected");
  d=fixture(); d.lanes[2].predecessors.clear();
  expect(!HdLaneMap::create(d,error),"non-reciprocal topology rejected");
  d=fixture(); d.lanes[1].centerline[2].x=std::numeric_limits<double>::quiet_NaN();
  expect(!HdLaneMap::create(d,error),"non-finite centerline rejected");
  d=fixture(); d.areas[0].validated=false;
  expect(!HdLaneMap::create(d,error),"provisional road rejected");
  expect(good->identity()==fixture().identity&&good->laneCount()==3,"failed replacement leaves held snapshot intact");
  const auto corridor=good->corridor({"B","C"},error);
  expect(corridor&&corridor->coversFootprint(100,3.5,0,options().vehicle,0),"full footprint can cross an approved longitudinal seam");
  expect(!good->corridor({"missing"},error),"unknown corridor lane rejected");
  expect(!good->touchesSolid(30,3.5,0,options().vehicle,0),"stop line is not a solid lane wall");
  expect(good->touchesSolid(50,4.5,0,options().vehicle,0),"paint half width included in contact");

  // v7 compatibility is exact topology conversion, never a buffer or a
  // blanket acceptance of self-intersecting polygons.
  d=fixture();d.areas={{"spur",{{0,0},{10,0},{10,10},{5,10},{5,11},{5,10},{0,10}},true}};
  auto spur=HdLaneMap::create(d,error);
  expect(spur&&spur->road()->coversPoint(5,9)&&!spur->road()->coversPoint(5,10.5),
         "zero-area A-B-A spur is removed without expanding the road");
  d.areas={{"pinched_hole",{{0,0},{10,0},{10,10},{5,10},{7,7},{3,7},{5,10},{0,10}},true}};
  auto hole=HdLaneMap::create(d,error);
  expect(hole&&hole->road()->coversPoint(1,8)&&!hole->road()->coversPoint(5,8),
         "exact inner loop remains a forbidden hole rather than drivable fill");
  if(hole)expect(!hole->road()->coversFootprint(5,8,0,{.2,.2,.2},0),
                "the entire footprint respects a converted interior hole");
  d.areas={{"crossed",{{0,0},{10,10},{0,10},{10,0}},true}};
  expect(!HdLaneMap::create(d,error),"an un-noded crossing is still rejected");

  // A narrow interior island is invisible to a four-corner-only check.
  d=fixture();
  d.areas={{"west",rectangle(-10,1,-3,7),true},{"east",rectangle(1.1,170,-3,7),true},
    {"north",rectangle(1,1.1,.1,7),true},{"south",rectangle(1,1.1,-3,-.1),true}};
  const auto island=HdLaneMap::create(d,error);
  expect(bool(island),"polygon union with interior island accepted");
  expect(island->road()->coversPoint(-.79,-.95)&&island->road()->coversPoint(3.845,.95),"car corners lie on road around island");
  expect(!island->road()->coversFootprint(0,0,0,options().vehicle,0),"complete footprint detects interior island");
}
void referenceTests() {
  std::string error;
  auto map=HdLaneMap::create(fixture(),error);
  auto result=BuildLaneReference(map,map->identity(),{"B","C"},options());
  expect(bool(result),"connected target-lane reference built");
  if (!result) { std::cerr<<result.detail<<'\n'; return; }
  const auto& r=*result.candidate;
  expect(!r.line().closed_loop&&r.lanes().size()==2,"local lane reference is an explicit open sequence");
  expect(r.maximumCurvature()<1e-9,"straight target reference remains straight");
  expect(r.usableBegin()>0&&r.usableEnd()<r.line().length,"footprint end caps excluded from certified interval");
  auto id=map->identity(); id.build_id="other";
  expect(!BuildLaneReference(map,id,{"B"},options()),"mixed map identity rejected");
  expect(!BuildLaneReference(map,map->identity(),{"A","C"},options()),"unconnected nodes never bridged by proximity");
  expect(!BuildLaneReference(map,map->identity(),{"B","B"},options()),"repeated lane rejected");
  auto cfg=options(); cfg.max_samples=3;
  expect(!BuildLaneReference(map,map->identity(),{"B"},cfg),"sample budget bounds reference work");
  cfg=options(); cfg.reference.resample_ds=0;
  expect(!BuildLaneReference(map,map->identity(),{"B"},cfg),"zero resampling step rejected");
  auto d=fixture(); d.lanes[2].centerline.front().x+=1.0;
  auto gap=HdLaneMap::create(d,error);
  expect(!BuildLaneReference(gap,gap->identity(),{"B","C"},options()),"geometric seam gap rejected despite matching successor ID");
  d=fixture(); d.markings.push_back({"new_solid",{{60,2},{60,5}},.15,PaintPattern::SOLID,false,false});
  auto blocked=HdLaneMap::create(d,error);
  expect(!BuildLaneReference(blocked,blocked->identity(),{"B"},options()),"actual solid geometry blocks a reference independently of lane boundary IDs");
  auto p=ProjectOnLaneReference(r,50,3.5,0,8);
  expect(p.valid&&std::abs(p.d)<1e-8,"ego projected from physical map pose onto new lane");
  auto source_object=ProjectOnLaneReference(r,60,0,0,8);
  auto target_object=ProjectOnLaneReference(r,60,3.5,0,8);
  expect(source_object.valid&&std::abs(source_object.d+3.5)<1e-8,"old-lane object gets new-reference lateral coordinate");
  expect(target_object.valid&&std::abs(target_object.d)<1e-8,"target-lane object remains centered on new reference");
  auto crossing=ProjectOnLaneReference(r,60,3.5,0,0,2);
  expect(crossing.valid&&std::abs(crossing.lateral_speed_mps-2)<1e-8,"object velocity is independent of its body heading");
  expect(!ProjectOnLaneReference(r,60,3.5,0,0,std::numeric_limits<double>::quiet_NaN()).valid,"invalid object velocity rejected");
  expect(!ProjectOnLaneReference(r,50,3.5,std::numeric_limits<double>::quiet_NaN(),8).valid,"invalid projection input rejected");
  d=fixture(); d.lanes[1].polygon=rectangle(0,100,3.,4.);
  const auto narrow=HdLaneMap::create(d,error);
  const auto through=BuildLaneReference(narrow,narrow->identity(),{"B"},options());
  expect(bool(through),"physical road containment is separate from lane classification at a convergence");
  if (through) {
    ReferenceHandoverGate gate(gateOptions());
    gate.begin(4,1,through.candidate,20);
    HandoverGeometryResult verdict;
    for (int i=0;i<8;++i) verdict=gate.observe(4,1,narrow->identity(),observation(10+i*.1));
    expect(verdict.state==HandoverGeometryState::ALIGNING,"passing a drivable seam never bypasses full-footprint target-lane arrival");
  }
  d=fixture(); d.lanes[1].polygon=rectangle(0,100,4.,5.);
  const auto displaced=HdLaneMap::create(d,error);
  expect(!BuildLaneReference(displaced,displaced->identity(),{"B"},options()),"reference center outside requested lane still rejected");
}
void regionCoverageTests() {
  std::string error;
  auto d=fixture();
  d.areas={{"west",rectangle(-10,1,-3,7),true},{"east",rectangle(1,170,-3,7),true}};
  auto map=HdLaneMap::create(d,error);
  expect(map&&map->road()->coversFootprint(0,0,0,options().vehicle,0),"footprint crosses a shared polygon boundary without a global union");
  expect(!map->corridor({},error)&&!error.empty(),"empty corridor rejected explicitly");
  d.areas[1].polygon=rectangle(1.0001,170,-3,7);
  map=HdLaneMap::create(d,error);
  expect(map&&!map->road()->coversPoint(1.00005,0),"narrow uncovered gap remains outside the road");
  expect(map&&!map->road()->coversFootprint(0,0,0,options().vehicle,0),"small gap under footprint is never removed by an area tolerance");
  d.areas.push_back({"gap_fill",rectangle(1,1.0001,-3,7),true});
  map=HdLaneMap::create(d,error);
  expect(map&&map->road()->coversFootprint(0,0,0,options().vehicle,0),"a third source polygon can cover the seam remainder");
  std::reverse(d.areas.begin(),d.areas.end());map=HdLaneMap::create(d,error);
  expect(map&&map->road()->coversFootprint(0,0,0,options().vehicle,0),"covered multi-polygon seam survives reversed source order");
  d.areas={{"overlap_west",rectangle(-10,2,-3,7),true},{"overlap_east",rectangle(1,170,-3,7),true}};
  map=HdLaneMap::create(d,error);
  expect(map&&map->road()->coversFootprint(0,0,0,options().vehicle,0),"overlapping valid polygons cover the complete footprint");
  d.areas={{"wide_west",rectangle(-10,1,-3,7),true},{"narrow_east",rectangle(1,170,-1,1),true}};
  map=HdLaneMap::create(d,error);
  expect(map&&map->road()->coversFootprint(0,0,0,options().vehicle,0),"local union cache can retain a drivable region");
  expect(map&&!map->road()->coversFootprint(0,.5,0,options().vehicle,0),"same cached polygon set still tests the current vehicle footprint");
  d.areas={{"exact",rectangle(-.79,3.845,-.95,.95),true}};
  map=HdLaneMap::create(d,error);
  expect(map&&map->road()->coversFootprint(0,0,0,options().vehicle,0),"source boundary itself is covered");
  expect(map&&!map->road()->coversFootprint(0,0,0,options().vehicle,.001),"requested footprint margin is preserved");
  // Two interior holes touch at one point. Store their surrounding tiles as
  // independent valid polygons instead of constructing a fragile hole ring.
  d.areas.clear();const double xs[]={-10,0,1,2,170},ys[]={-3,0,1,2,7};
  for(int x=0;x<4;++x)for(int y=0;y<4;++y){
    if((x==1&&y==1)||(x==2&&y==2))continue;
    d.areas.push_back({std::to_string(x)+":"+std::to_string(y),rectangle(xs[x],xs[x+1],ys[y],ys[y+1]),true});
  }
  map=HdLaneMap::create(d,error);
  expect(bool(map),"valid source regions around point-touching holes load");
  expect(map&&map->road()->coversPoint(1,1),"shared hole-boundary point retains source coverage");
  expect(map&&!map->road()->coversFootprint(0,0,0,options().vehicle,0),"touching holes remain non-drivable under the car");
  d=fixture();d.areas[0].polygon={{0,0},{1,1},{0,1},{1,0}};
  expect(!HdLaneMap::create(d,error),"self-intersecting source polygon is still rejected");
}
void handoverTests() {
  std::string error; auto map=HdLaneMap::create(fixture(),error);
  auto result=BuildLaneReference(map,map->identity(),{"B","C"},options());
  if (!result) { expect(false,"handover setup reference"); return; }
  ReferenceHandoverGate g(gateOptions());
  expect(g.begin(7,1,result.candidate,20),"handover bound to command/epoch/target");
  expect(!g.begin(7,2,result.candidate,20),"same command cannot change source epoch");
  auto a=g.observe(7,1,map->identity(),observation(1,0));
  expect(a.state==HandoverGeometryState::ALIGNING,"source-lane driving cannot complete target handover");
  for (int i=1;i<=4;++i) {
    a=g.observe(7,1,map->identity(),observation(1+i*.1));
    expect(a.state==HandoverGeometryState::ALIGNING,"alignment requires consecutive fresh source poses");
  }
  expect(g.begin(7,1,result.candidate,20),"same-ID heartbeat preserves settling interval");
  auto duplicate=g.observe(7,1,map->identity(),observation(1.4));
  expect(duplicate.state==HandoverGeometryState::DUPLICATE_POSE,"reissued pose never advances settling");
  g.observe(7,1,map->identity(),observation(1.5));
  a=g.observe(7,1,map->identity(),observation(1.6));
  expect(a.state==HandoverGeometryState::GEOMETRY_ELIGIBLE,"stable target pose becomes geometry eligible");
  expect(a.detail.find("dynamic_safety_still_required")!=std::string::npos,"geometry eligibility never claims public READY or COMPLETED");
  auto stale=observation(1.7); stale.objects_stamp_s=1.;
  expect(g.observe(7,1,map->identity(),stale).state==HandoverGeometryState::INVALID,"stale objects invalidate geometric readiness");
  a=g.observe(7,1,map->identity(),observation(1.8));
  expect(a.state==HandoverGeometryState::ALIGNING,"invalid input resets stability history");
  expect(g.observe(8,1,map->identity(),observation(1.9)).state==HandoverGeometryState::INVALID,"old readiness cannot acknowledge a new command");
  expect(g.observe(7,2,map->identity(),observation(1.9)).state==HandoverGeometryState::INVALID,"old readiness cannot acknowledge a new reference epoch");
  auto id=map->identity(); id.geometry_sha256=std::string(64,'2');
  expect(g.observe(7,1,id,observation(1.9)).state==HandoverGeometryState::INVALID,"changed map invalidates readiness");
  expect(g.begin(8,1,result.candidate,20),"new command starts a new observation history");
  a=g.observe(8,1,map->identity(),observation(2,3.5,.2));
  expect(a.state==HandoverGeometryState::ALIGNING,"target position alone cannot complete heading/lateral-speed alignment");
  g.observe(8,1,map->identity(),observation(2.1));
  a=g.observe(8,1,map->identity(),observation(3.));
  expect(a.state==HandoverGeometryState::ALIGNING,"pose gap resets settling instead of counting missing data as time aligned");
  expect(g.observe(8,1,map->identity(),observation(2.9)).state==HandoverGeometryState::INVALID,"backward source time invalidates readiness");
}
std::vector<Point2D> annulus(double inner,double outer,double begin,double end) {
  std::vector<Point2D> p;
  for (int i=0;i<=240;++i) { const double a=begin+(end-begin)*i/240.; p.push_back({outer*std::cos(a),outer*std::sin(a)}); }
  for (int i=240;i>=0;--i) { const double a=begin+(end-begin)*i/240.; p.push_back({inner*std::cos(a),inner*std::sin(a)}); }
  return p;
}
void curvedReferenceTests() {
  auto d=fixture(); d.lanes.clear(); d.markings.clear();
  for (const double radius:{80.,83.5}) {
    LaneData l; l.id=radius==80.?"inner":"outer"; l.link_id="link:"+l.id; l.traversable=true;
    for (int i=0;i<=240;++i) { const double a=1.2*i/240.; l.centerline.push_back({radius*std::cos(a),radius*std::sin(a)}); }
    l.polygon=annulus(radius-1.75,radius+1.75,0,1.2);
    if (radius==80.) l.right_adjacent="outer"; else l.left_adjacent="inner";
    d.lanes.push_back(std::move(l));
  }
  d.areas={{"curved_road",annulus(76,88,-.2,1.4),true}};
  std::string error; const auto map=HdLaneMap::create(d,error);
  expect(bool(map),"curved adjacent lanes load");
  if (!map) { std::cerr<<error<<'\n'; return; }
  const auto target=BuildLaneReference(map,map->identity(),{"outer"},options());
  expect(bool(target),"curved target reference satisfies footprint and curvature");
  if (!target) { std::cerr<<target.detail<<'\n'; return; }
  expect(target.candidate->maximumCurvature()>.01,"curved reference retains road curvature");
  ReferenceHandoverGate gate(gateOptions());
  expect(gate.begin(11,1,target.candidate,20),"curved handover binds target");
  HandoverGeometryResult result;
  for (int i=0;i<=6;++i) {
    const double theta=.6+i*.001;
    auto obs=observation(5+i*.1);
    obs.ego.x=83.5*std::cos(theta); obs.ego.y=83.5*std::sin(theta);
    obs.ego.yaw=theta+1.5707963267948966; obs.ego.kappa=1/83.5;
    result=gate.observe(11,1,map->identity(),obs);
  }
  expect(result.state==HandoverGeometryState::GEOMETRY_ELIGIBLE,"curved alignment uses target tangent rather than a straight/global heading");
}
}
int main() {
  mapTests(); regionCoverageTests(); referenceTests(); handoverTests(); curvedReferenceTests();
  std::cout<<checks<<" checks, "<<failures<<" failures\n";
  return failures?1:0;
}
