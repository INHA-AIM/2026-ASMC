#include "Parser/HdMapLaneInput.hpp"
#include <iostream>

using namespace asmc_planner::lane_reference;
namespace {
int checks=0,failures=0;
void expect(bool yes,const char* name) { ++checks; if (!yes) { ++failures; std::cerr<<name<<'\n'; } }
template<class Layer> void identity(Layer& l) {
  l.header.frame_id="map"; l.build_id="test-v1"; l.map_id="test"; l.geometry_sha256=std::string(64,'1'); l.complete=true;
}
geometry_msgs::Point point(double x,double y) { geometry_msgs::Point p; p.x=x; p.y=y; return p; }
}
int main() {
  asmc_msgs::HdMapMeta meta; identity(meta); meta.schema_major=1; meta.scope=asmc_msgs::HdMapMeta::SCOPE_GLOBAL;
  meta.operational=true; meta.lane_count=1; meta.lane_marking_count=1; meta.drivable_area_count=1;
  asmc_msgs::LaneArray lanes; identity(lanes);
  asmc_msgs::Lane lane; lane.id="lane:A"; lane.link_id="A"; lane.lane_type=asmc_msgs::Lane::TYPE_DRIVING;
  lane.direction=asmc_msgs::Lane::DIRECTION_FORWARD;
  lane.centerline={point(0,0),point(80,0)};
  lane.polygon={point(0,-1.75),point(80,-1.75),point(80,1.75),point(0,1.75)};
  lanes.lanes={lane};
  asmc_msgs::LaneMarkingArray markings; identity(markings);
  asmc_msgs::LaneMarking paint; paint.id="broken"; paint.width_m=.15; paint.pattern=asmc_msgs::LaneMarking::PATTERN_BROKEN;
  paint.physical_type=asmc_msgs::LaneMarking::PHYSICAL_PAINTED;
  paint.polyline={point(0,1.75),point(80,1.75)}; markings.markings={paint};
  asmc_msgs::DrivableAreaArray areas; identity(areas);
  asmc_msgs::DrivableArea road; road.id="road"; road.validity=asmc_msgs::DrivableArea::VALIDITY_VALIDATED;
  road.polygon={point(-10,-3),point(100,-3),point(100,3),point(-10,3)}; areas.areas={road};
  std::string error;
  auto good=ConvertHdMapLaneSnapshot(meta,lanes,markings,areas,error);
  expect(bool(good)&&error.empty(),"matching global ROS messages convert");
  auto wrong=markings; wrong.geometry_sha256=std::string(64,'2');
  expect(!ConvertHdMapLaneSnapshot(meta,lanes,wrong,areas,error),"mixed geometry SHA rejected");
  wrong=markings; wrong.build_id="other";
  expect(!ConvertHdMapLaneSnapshot(meta,lanes,wrong,areas,error),"mixed build rejected");
  wrong=markings; wrong.header.frame_id="base_link";
  expect(!ConvertHdMapLaneSnapshot(meta,lanes,wrong,areas,error),"mixed frame rejected");
  auto partial=lanes; partial.complete=false;
  expect(!ConvertHdMapLaneSnapshot(meta,partial,markings,areas,error),"local lane crop rejected");
  auto bad_meta=meta; bad_meta.lane_count=2;
  expect(!ConvertHdMapLaneSnapshot(bad_meta,lanes,markings,areas,error),"layer count mismatch rejected");
  bad_meta=meta; bad_meta.operational=false;
  expect(!ConvertHdMapLaneSnapshot(bad_meta,lanes,markings,areas,error),"non-operational metadata rejected");
  bad_meta=meta; bad_meta.scope=asmc_msgs::HdMapMeta::SCOPE_LOCAL_CROP;
  expect(!ConvertHdMapLaneSnapshot(bad_meta,lanes,markings,areas,error),"local metadata rejected");
  bad_meta=meta; bad_meta.schema_major=2;
  expect(!ConvertHdMapLaneSnapshot(bad_meta,lanes,markings,areas,error),"unsupported schema rejected");
  auto provisional=areas; provisional.areas[0].validity=asmc_msgs::DrivableArea::VALIDITY_PROVISIONAL;
  expect(!ConvertHdMapLaneSnapshot(meta,lanes,markings,provisional,error),"provisional areas rejected");
  expect(good&&good->laneCount()==1,"previous immutable snapshot remains usable");
  std::cout<<checks<<" ROS adapter checks, "<<failures<<" failures\n";
  return failures?1:0;
}
