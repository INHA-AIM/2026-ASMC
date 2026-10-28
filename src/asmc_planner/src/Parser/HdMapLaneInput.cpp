#include "Parser/HdMapLaneInput.hpp"

namespace asmc_planner::lane_reference {
namespace {
template<class Layer>
bool matches(const Layer& layer,const MapIdentity& identity) {
  return layer.complete && layer.header.frame_id==identity.frame_id &&
    layer.build_id==identity.build_id && layer.map_id==identity.map_id &&
    layer.geometry_sha256==identity.geometry_sha256;
}
template<class Points>
std::vector<Point2D> xy(const Points& points) {
  std::vector<Point2D> result; result.reserve(points.size());
  for (const auto& p:points) result.push_back({p.x,p.y});
  return result;
}
}
std::shared_ptr<const HdLaneMap> ConvertHdMapLaneSnapshot(
    const asmc_msgs::HdMapMeta& meta,const asmc_msgs::LaneArray& lanes,
    const asmc_msgs::LaneMarkingArray& markings,const asmc_msgs::DrivableAreaArray& areas,
    std::string& error) {
  LaneMapData data;
  data.identity={meta.header.frame_id,meta.build_id,meta.map_id,meta.geometry_sha256};
  if (meta.schema_major!=1 || !meta.operational || !meta.complete || meta.scope!=asmc_msgs::HdMapMeta::SCOPE_GLOBAL ||
      !data.identity.valid() || !matches(lanes,data.identity) || !matches(markings,data.identity) || !matches(areas,data.identity) ||
      meta.lane_count!=lanes.lanes.size() || meta.lane_marking_count!=markings.markings.size() || meta.drivable_area_count!=areas.areas.size()) {
    error="incomplete_or_mixed_global_hd_map_layers"; return {};
  }
  data.complete=true;
  for (const auto& input:lanes.lanes) {
    LaneData l; l.id=input.id; l.link_id=input.link_id;
    l.left_adjacent=input.left_adjacent_lane_id; l.right_adjacent=input.right_adjacent_lane_id;
    l.predecessors=input.predecessor_lane_ids; l.successors=input.successor_lane_ids;
    l.centerline=xy(input.centerline); l.polygon=xy(input.polygon);
    l.traversable=input.lane_type!=asmc_msgs::Lane::TYPE_UNKNOWN && input.lane_type!=asmc_msgs::Lane::TYPE_SHOULDER &&
      input.direction!=asmc_msgs::Lane::DIRECTION_UNKNOWN;
    data.lanes.push_back(std::move(l));
  }
  for (const auto& input:markings.markings) {
    MarkingData m; m.id=input.id; m.points=xy(input.polyline); m.width_m=input.width_m;
    m.pattern=input.pattern==asmc_msgs::LaneMarking::PATTERN_SOLID?PaintPattern::SOLID:
      input.pattern==asmc_msgs::LaneMarking::PATTERN_BROKEN?PaintPattern::BROKEN:PaintPattern::UNKNOWN;
    m.stop_line=input.role==asmc_msgs::LaneMarking::ROLE_STOP_LINE;
    m.virtual_line=input.physical_type==asmc_msgs::LaneMarking::PHYSICAL_VIRTUAL;
    data.markings.push_back(std::move(m));
  }
  for (const auto& input:areas.areas)
    data.areas.push_back({input.id,xy(input.polygon),input.validity==asmc_msgs::DrivableArea::VALIDITY_VALIDATED});
  return HdLaneMap::create(std::move(data),error);
}
}  // namespace asmc_planner::lane_reference
