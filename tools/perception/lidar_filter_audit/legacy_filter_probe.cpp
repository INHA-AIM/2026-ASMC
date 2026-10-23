// Reuse each current filter implementation, not a Python rewrite of old policy.
// JSONL in/out on stdin/stdout; no ROS, network, tracker or publication side effects.
#ifdef USE_MAIN_FILTERS
#include <PlannerInterface/DrivableBoundaryFilter.hpp>
#include <PlannerInterface/KnownStaticObjectFilter.hpp>
#include <PlannerInterface/StaticStructureExclusionFilter.hpp>
#include <PlannerInterface/PlannerObjectTypes.hpp>
#include <PlannerInterface/PlanningRelevanceFilter.hpp>
#else
#include <planning_inputs/filters/DrivableBoundaryFilter.hpp>
#include <planning_inputs/filters/KnownStaticObjectFilter.hpp>
#include <planning_inputs/filters/StaticStructureExclusionFilter.hpp>
#include <planning_inputs/filters/PlannerObjectTypes.hpp>
#endif
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>

template<class T> T Value(const YAML::Node& n, const char* key, T fallback) {
  return n[key] && !n[key].IsNull() ? n[key].as<T>() : fallback;
}
void String(std::ostream& s, const std::string& v) { s << std::quoted(v); }
PlannerPoint3D Point(const YAML::Node& n) {
  if(!n.IsSequence() || n.size()!=3) throw std::runtime_error("point needs XYZ");
  return {n[0].as<double>(),n[1].as<double>(),n[2].as<double>()};
}
int main(int argc, char** argv) {
  try {
    if(argc!=7) throw std::runtime_error("usage: probe MASK METADATA CATALOG ZONES PARAMS EXPECTED_MAP_ID");
    const auto p=YAML::LoadFile(argv[5]);
    const std::array<double,3> origin{{p["drivable_origin_easting"].as<double>(),p["drivable_origin_northing"].as<double>(),p["drivable_origin_altitude"].as<double>()}};
    DrivableBoundaryFilter road; KnownStaticObjectFilter known; StaticStructureExclusionFilter structure;
    if(!road.Configure(argv[1],argv[2],argv[6],origin,p["drivable_margin_m"].as<double>(),p["drivable_ratio_threshold"].as<double>(),p["drivable_samples"].as<int>())) throw std::runtime_error(road.status_message());
    std::vector<KnownStaticApplicationRegion> regions;
    for(const auto& prefix: {std::string("known_static_application_region"),std::string("known_static_guardrail_extension")}) {
      if(p[prefix+"_enabled"].as<bool>()) regions.push_back({p[prefix+"_min_x"].as<double>(),p[prefix+"_max_x"].as<double>(),p[prefix+"_min_y"].as<double>(),p[prefix+"_max_y"].as<double>()});
    }
    if(!known.ConfigureWithRegions(argv[3],argv[6],origin,p["known_static_center_tolerance_m"].as<double>(),regions)) throw std::runtime_error(known.status_message());
    if(!structure.Configure(argv[4],p["static_structure_minimum_height_m"].as<double>(),p["static_structure_minimum_bottom_z_m"].as<double>())) throw std::runtime_error(structure.status_message());
#ifdef USE_MAIN_FILTERS
    PlanningRelevanceFilter relevance; relevance.Configure(0.10,3,5);
#endif
    std::string line, last_frame;
    std::cout << std::setprecision(17) << std::boolalpha << std::unitbuf;
    while(std::getline(std::cin,line)) {
      const auto n=YAML::Load(line); const std::string key=n["key"].as<std::string>();
      const auto start=std::chrono::steady_clock::now();
      if(!Value(n,"baseline_complete",false)) {
        std::cout << "{\"key\":";String(std::cout,key);
        std::cout << ",\"available\":false,\"reason\":\"legacy_input_evidence_missing\"}\n";continue;
      }
      const auto v=n["object"]; PlannerObject o;
      o.id=v["track_id"].as<int>();o.classification=Value(v,"classification",std::string("unknown"));o.yolo_class=Value(v,"camera_class",std::string("unknown"));
      o.cluster_center_map=Point(v["cluster_center"]);o.obb_center_map=Point(v["center"]);o.velocity_map_mps=Point(v["legacy_velocity_mps"]);
      o.is_dynamic=v["legacy_is_dynamic"].as<bool>();o.motion_state=v["legacy_motion_state"].as<int>();
      o.length=v["length"].as<double>();o.width=v["width"].as<double>();o.height=v["height"].as<double>();o.obb_yaw_map_rad=v["yaw"].as<double>();
      if(v["corners"].size()!=8) throw std::runtime_error("needs eight corners");
      for(int i=0;i<8;++i)o.obb_corners_map[i]=Point(v["corners"][i]);
      const auto r=road.Evaluate(o);const auto k=known.Evaluate(o);const auto s=structure.Evaluate(o);
      bool keep=r.keep && k.keep && s.keep, clipped=false;
      std::string reason=!r.keep?r.reason:!k.keep?k.reason:!s.keep?s.reason:"keep";
      bool full=false; std::string relevance_reason="not_in_planning_inputs_chain";
#ifdef USE_MAIN_FILTERS
      const auto frame=n["frame_key"].as<std::string>();
      if(frame!=last_frame){relevance.BeginFrame();last_frame=frame;}
      if(v["ground_evidence"]) {
        const auto g=v["ground_evidence"];o.ground_evidence.valid=g["valid"].as<bool>();o.ground_evidence.sample_count=g["samples"].as<int>();
        o.ground_evidence.median_clearance_m=g["median_clearance"].as<double>();o.ground_evidence.rejection_band_m=g["band"].as<double>();
      }
      full=Value(n,"relevance_complete",false);
      if(full){
        const auto e=n["ego"];const auto d=relevance.Evaluate(o,keep,e["x"].as<double>(),e["y"].as<double>(),e["speed_mps"].as<double>(),e["length_m"].as<double>());
        keep=d.keep;relevance_reason=d.reason;if(!keep && reason=="keep")reason=d.reason;
      }else relevance_reason="ego_or_ground_evidence_missing";
#endif
      PlannerObject output=o;if(keep)road.BuildPlanningObject(o,r,&output,&clipped);
      const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      std::cout << "{\"key\":";String(std::cout,key);std::cout << ",\"available\":true,\"full_main_chain\":"<<full<<",\"keep\":"<<keep<<",\"reason\":";String(std::cout,reason);
      std::cout<<",\"road_ratio\":"<<r.road_ratio<<",\"predicted_road_ratio\":"<<r.predicted_road_ratio<<",\"road_reason\":";String(std::cout,r.reason);
      std::cout<<",\"known_static_keep\":"<<k.keep<<",\"known_reason\":";String(std::cout,k.reason);std::cout<<",\"reference_id\":"<<k.reference_track_id<<",\"structure_keep\":"<<s.keep<<",\"structure_reason\":";String(std::cout,s.reason);
      std::cout<<",\"relevance_reason\":";String(std::cout,relevance_reason);
      std::cout<<",\"geometry_clipped\":"<<clipped<<",\"output_center\":["<<output.obb_center_map.x<<','<<output.obb_center_map.y<<','<<output.obb_center_map.z<<"],\"output_corners\":[";
      for(int i=0;i<8;++i){if(i)std::cout<<',';const auto& c=output.obb_corners_map[i];std::cout<<'['<<c.x<<','<<c.y<<','<<c.z<<']';}
      std::cout<<"],\"comparison_ms\":"<<ms<<"}\n";
    }
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
