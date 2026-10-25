#include "planning_inputs/filters/PlanningObjectFilter.hpp"
#include "planning_inputs/filters/PlannerObjectTypes.hpp"
#include <std_msgs/String.h>
#include <sstream>
#include <stdexcept>
#include <utility>
namespace planning_inputs {
PlanningObjectFilter::PlanningObjectFilter(ros::NodeHandle& p) {
  std::string image, metadata, catalog, zones, map;
  p.param<std::string>("drivable_mask_image", image, "");
  p.param<std::string>("drivable_mask_metadata", metadata, "");
  p.param<std::string>("known_static_catalog", catalog, "");
  p.param<std::string>("static_structure_exclusion_config", zones, "");
  p.param<std::string>("drivable_map_id", map, "waize_katri_2025-10-23");
  double east, north, altitude, margin, ratio, tolerance, height, bottom;
  int samples;
  p.param("drivable_origin_easting", east, 302595.0);
  p.param("drivable_origin_northing", north, 4124145.0);
  p.param("drivable_origin_altitude", altitude, 0.0);
  p.param("drivable_margin_m", margin, 0.0);
  p.param("drivable_ratio_threshold", ratio, 0.15);
  p.param("drivable_samples", samples, 5);
  p.param("known_static_center_tolerance_m", tolerance, 2.5);
  p.param("static_structure_minimum_height_m", height, 2.5);
  p.param("static_structure_minimum_bottom_z_m", bottom, 2.0);
  std::vector<KnownStaticApplicationRegion> regions;
  const auto region = [&p, &regions](const std::string& prefix,
                                    KnownStaticApplicationRegion value) {
    bool enabled;
    p.param(prefix + "_enabled", enabled, true);
    p.param(prefix + "_min_x", value.min_x, value.min_x);
    p.param(prefix + "_max_x", value.max_x, value.max_x);
    p.param(prefix + "_min_y", value.min_y, value.min_y);
    p.param(prefix + "_max_y", value.max_y, value.max_y);
    if(enabled) regions.push_back(value);
  };
  region("known_static_application_region", {-35.0,15.0,108.0,180.0});
  region("known_static_guardrail_extension", {-107.0,15.0,175.0,345.0});
  const std::array<double,3> origin{{east,north,altitude}};
  bool road_enabled, known_enabled, structure_enabled;
  p.param("drivable_filter_enabled", road_enabled, true);
  p.param("known_static_filter_enabled", known_enabled, true);
  p.param("static_structure_exclusion_enabled", structure_enabled, true);
  if(road_enabled && !road_.Configure(image,metadata,map,origin,margin,ratio,samples))
    throw std::runtime_error("Planning road filter: " + road_.status_message());
  if(known_enabled && !known_.ConfigureWithRegions(catalog,map,origin,tolerance,regions))
    throw std::runtime_error("Planning known-static filter: " + known_.status_message());
  if(structure_enabled && !structure_.Configure(zones,height,bottom))
    throw std::runtime_error("Planning structure filter: " + structure_.status_message());
  ros::NodeHandle node;
  debug_=node.advertise<std_msgs::String>("/planning/objects_debug",2);
  ROS_INFO("Planning publication filters ready: road=%d known=%d references=%zu structure=%d; validated mask and UNKNOWN-only static regions",
           road_.ready(),known_.ready(),known_.reference_count(),structure_.ready());
}
void PlanningObjectFilter::Apply(const asmc_msgs::TrackedObjectArray& source,
                                DetectedObjectArray* output) {
  // 이관 인계: 현재 main_raw 선택 경로의 legacy publication 필터다.
  // asmc_lidar의 인지 소유 필터가 검증·활성화된 경로에는 road/known-static/structure와
  // 아래 OBB clipping을 다시 적용하지 않는다. 기존 Main 소비·복구 경로가 남아 있는
  // 동안 이 구현을 먼저 삭제하지 말고, 판제 입력 전환 PR에서 호출부·설정을 함께 정리한다.
  // 실제 제외가 OFF인 audit-only 구현을 필터 완료로 간주하지 않는다.
  // 정본: docs/handoffs/lidar-migration.md#판제-소비-전환-시-정리할-중복-처리
  // Conversion is atomic and preserves source order; validate before filtering.
  if(!output || source.objects.size()!=output->objects.size())
    throw std::runtime_error("Planning filter source/output alignment mismatch");
  std::ostringstream debug;
  debug << "{\"stamp\":" << source.header.stamp.toSec()
        << ",\"raw_count\":" << source.objects.size() << ",\"objects\":[";
  std::vector<DetectedObject> kept;
  kept.reserve(output->objects.size());
  for(std::size_t i=0;i<source.objects.size();++i) {
    const auto& raw=source.objects[i];
    auto result=output->objects[i];
    PlannerObject object;
    object.id=raw.id; object.classification=raw.classification;
    object.yolo_class=raw.yolo_class;
    // The private classifier is authoritative when known. Preserve upstream
    // motion evidence while private state is UNKNOWN (e.g. missing ego heading).
    object.motion_state=result.motion_state;
    object.is_dynamic=result.motion_state==DetectedObject::MOTION_DYNAMIC ||
      (result.motion_state==DetectedObject::MOTION_UNKNOWN && raw.is_dynamic);
    object.cluster_center_map={raw.cluster_center.x,raw.cluster_center.y,raw.cluster_center.z};
    object.obb_center_map={result.center.x,result.center.y,result.center.z};
    object.velocity_map_mps={result.vx,result.vy,raw.velocity_mps.z};
    object.length=result.length; object.width=result.width; object.height=result.height;
    object.obb_yaw_map_rad=result.yaw;
    for(std::size_t j=0;j<object.obb_corners_map.size();++j)
      object.obb_corners_map[j]={result.obb_corners[j].x,result.obb_corners[j].y,result.obb_corners[j].z};
    const auto road=road_.Evaluate(object);
    const auto known=known_.Evaluate(object);
    const auto structure=structure_.Evaluate(object);
    const bool keep=road.keep && known.keep && structure.keep;
    bool clipped=false;
    if(keep) {
      PlannerObject planning;
      road_.BuildPlanningObject(object,road,&planning,&clipped);
      result.center.x=planning.obb_center_map.x;result.center.y=planning.obb_center_map.y;
      result.center.z=planning.obb_center_map.z;
      result.length=planning.length;result.width=planning.width;result.height=planning.height;
      result.yaw=planning.obb_yaw_map_rad;
      for(std::size_t j=0;j<planning.obb_corners_map.size();++j) {
        result.obb_corners[j].x=planning.obb_corners_map[j].x;
        result.obb_corners[j].y=planning.obb_corners_map[j].y;
        result.obb_corners[j].z=planning.obb_corners_map[j].z;
      }
      kept.push_back(std::move(result));
    }
    if(i) debug << ',';
    debug << "{\"id\":" << raw.id << ",\"keep\":" << (keep?"true":"false")
          << ",\"road_ratio\":" << road.road_ratio
          << ",\"known_static_keep\":" << (known.keep?"true":"false")
          << ",\"reference_id\":" << known.reference_track_id
          << ",\"geometry_clipped\":" << (clipped?"true":"false")
          << ",\"reason\":\"" << (!road.keep?road.reason:!known.keep?known.reason:!structure.keep?structure.reason:"keep") << "\"}";
  }
  debug << "],\"kept_count\":" << kept.size() << '}';
  std_msgs::String message;message.data=debug.str();debug_.publish(message);
  output->objects=std::move(kept);
}
}
