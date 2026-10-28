// Offline/shadow only. No ROS node, trajectory output, or control publication.
#include "Execution/ReferenceHandover.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace asmc_planner;
using namespace asmc_planner::lane_reference;
namespace pt=boost::property_tree;
namespace {
std::vector<Point2D> points(const pt::ptree& input) {
  std::vector<Point2D> result;
  for (const auto& row:input) {
    auto i=row.second.begin();
    if (i==row.second.end()) throw std::runtime_error("missing x");
    const double x=i++->second.get_value<double>();
    if (i==row.second.end()) throw std::runtime_error("missing y");
    result.push_back({x,i->second.get_value<double>()});
  }
  return result;
}
std::vector<std::string> strings(const pt::ptree& p) {
  std::vector<std::string> out;
  for (const auto& x:p) out.push_back(x.second.get_value<std::string>());
  return out;
}
LaneMapData mapData(const pt::ptree& input) {
  LaneMapData d; const auto& identity=input.get_child("identity");
  d.identity={identity.get<std::string>("frame_id"),identity.get<std::string>("build_id"),
    identity.get<std::string>("map_id"),identity.get<std::string>("geometry_sha256")};
  d.complete=input.get<bool>("complete");
  for (const auto& x:input.get_child("lanes")) {
    const auto& p=x.second; LaneData l;
    l.id=p.get<std::string>("id"); l.link_id=p.get<std::string>("link_id");
    l.left_adjacent=p.get<std::string>("left_adjacent"); l.right_adjacent=p.get<std::string>("right_adjacent");
    l.predecessors=strings(p.get_child("predecessors")); l.successors=strings(p.get_child("successors"));
    l.centerline=points(p.get_child("centerline")); l.polygon=points(p.get_child("polygon"));
    l.traversable=p.get<bool>("traversable"); d.lanes.push_back(std::move(l));
  }
  for (const auto& x:input.get_child("areas")) {
    const auto& p=x.second;
    d.areas.push_back({p.get<std::string>("id"),points(p.get_child("polygon")),p.get<bool>("validated")});
  }
  for (const auto& x:input.get_child("markings")) {
    const auto& p=x.second; MarkingData m;
    m.id=p.get<std::string>("id"); m.points=points(p.get_child("points")); m.width_m=p.get<double>("width_m");
    const auto pattern=p.get<std::string>("pattern");
    m.pattern=pattern=="solid"?PaintPattern::SOLID:pattern=="broken"?PaintPattern::BROKEN:PaintPattern::UNKNOWN;
    m.stop_line=p.get<bool>("stop_line"); m.virtual_line=p.get<bool>("virtual_line");
    d.markings.push_back(std::move(m));
  }
  return d;
}
LaneReferenceOptions options(const pt::ptree& p) {
  LaneReferenceOptions o; const auto& r=p.get_child("reference");
  o.reference={r.get<double>("closure_tolerance_m"),r.get<double>("smoothing_window_m"),r.get<double>("resample_ds"),
    r.get<int>("relax_max_iter"),r.get<double>("relax_step"),r.get<double>("ego_projection_search_window")};
  o.vehicle={p.get<double>("front_m"),p.get<double>("rear_m"),p.get<double>("half_width_m")};
  o.max_curvature=p.get<double>("max_curvature"); o.max_lateral_accel=p.get<double>("max_lateral_accel");
  o.max_join_gap_m=p.get<double>("max_join_gap_m"); o.sample_distance_m=p.get<double>("sample_distance_m");
  o.sample_yaw_rad=p.get<double>("sample_yaw_rad"); o.footprint_margin_m=p.get<double>("footprint_margin_m");
  o.max_samples=p.get<std::size_t>("max_samples"); return o;
}
HandoverOptions gateOptions(const pt::ptree& p) {
  return {p.get<double>("lateral_tolerance_m"),p.get<double>("heading_tolerance_rad"),p.get<double>("lateral_speed_tolerance_mps"),
    p.get<double>("settle_duration_s"),p.get<double>("ego_max_age_s"),p.get<double>("objects_max_age_s"),p.get<double>("max_pose_gap_s")};
}
}
int main(int argc,char** argv) {
  if (argc!=2) { std::cerr<<"usage: lane_reference_audit snapshot.json\n"; return 2; }
  try {
    pt::ptree input; pt::read_json(argv[1],input);
    std::string error;
    const auto start=std::chrono::steady_clock::now();
    const auto map=HdLaneMap::create(mapData(input.get_child("map")),error);
    if (!map) { std::cerr<<"map rejected: "<<error<<'\n'; return 1; }
    pt::ptree output;
    output.put("mode","offline_geometry_only_no_control_output");
    output.put("build_id",map->identity().build_id); output.put("lanes",map->laneCount());
    output.put("map_load_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
    pt::ptree reports;
    for (const auto& q:input.get_child("requests")) {
      const auto& request=q.second;
      const auto before=std::chrono::steady_clock::now();
      const auto result=BuildLaneReference(map,map->identity(),strings(request.get_child("lane_ids")),options(input.get_child("options")));
      pt::ptree report; report.put("name",request.get<std::string>("name"));
      report.put("static_reference_valid",bool(result)); report.put("detail",result.detail);
      if (result.rejected_point) {
        const auto& p=*result.rejected_point;
        report.put("rejected_s",p.s); report.put("rejected_x",p.x); report.put("rejected_y",p.y); report.put("rejected_yaw",p.theta);
      }
      report.put("build_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-before).count());
      if (result) {
        const auto& r=*result.candidate;
        report.put("reference_length_m",r.line().length); report.put("max_curvature",r.maximumCurvature());
        report.put("usable_begin_s",r.usableBegin()); report.put("usable_end_s",r.usableEnd());
        if (std::isfinite(r.curveSpeedLimit())) report.put("curve_speed_limit_mps",r.curveSpeedLimit());
        ReferenceHandoverGate gate(gateOptions(input.get_child("handover")));
        const auto command_id=request.get<std::uint32_t>("command_id");
        if (!gate.begin(command_id,1,result.candidate,r.usableBegin())) throw std::runtime_error("invalid handover configuration");
        std::size_t count=0,invalid=0,duplicate=0,aligning=0,eligible=0;
        double max_abs_d=0.0;
        auto observe=[&](const HandoverObservation& o) {
          const auto g=gate.observe(command_id,1,map->identity(),o); ++count;
          if (g.projection.valid) max_abs_d=std::max(max_abs_d,std::abs(g.projection.d));
          if (g.state==HandoverGeometryState::INVALID) ++invalid;
          if (g.state==HandoverGeometryState::DUPLICATE_POSE) ++duplicate;
          if (g.state==HandoverGeometryState::ALIGNING) ++aligning;
          if (g.state==HandoverGeometryState::GEOMETRY_ELIGIBLE) ++eligible;
        };
        if (request.get<bool>("synthetic_target",false)) {
          const double start_s=r.usableBegin()+10.0;
          if (start_s+10.0>=r.usableEnd()) throw std::runtime_error("reference too short for the synthetic tracking fixture");
          for (int i=0;i<30;++i) {
            const double speed=std::min(8.0,(r.usableEnd()-start_s-1.0)/3.0);
            const auto p=Interpolate(r.line(),start_s+i*.1*speed);
            HandoverObservation o; o.now_s=o.ego_stamp_s=o.objects_stamp_s=100+i*.1;
            o.ego_valid=o.objects_valid=true; o.ego={p.x,p.y,p.theta,p.kappa,speed,0.0};
            observe(o);
          }
          report.put("fixture","synthetic_target_tracking_not_vehicle_motion");
        }
        if (auto observations=request.get_child_optional("observations")) {
          for (const auto& obs:*observations) {
            const auto& p=obs.second; HandoverObservation o;
            o.now_s=p.get<double>("now_s"); o.ego_stamp_s=p.get<double>("ego_stamp_s");
            o.objects_stamp_s=p.get<double>("objects_stamp_s"); o.ego_valid=p.get<bool>("ego_valid"); o.objects_valid=p.get<bool>("objects_valid");
            o.ego={p.get<double>("x"),p.get<double>("y"),p.get<double>("yaw"),p.get<double>("kappa"),p.get<double>("speed"),p.get<double>("acceleration")};
            observe(o);
          }
        }
        report.put("pose_count",count); report.put("invalid",invalid); report.put("duplicate",duplicate);
        report.put("aligning",aligning); report.put("geometry_eligible",eligible); report.put("max_abs_d_m",max_abs_d);
      }
      reports.push_back({"",report});
    }
    output.add_child("reports",reports); pt::write_json(std::cout,output,false);
    return 0;
  } catch (const std::exception& e) { std::cerr<<"lane reference audit: "<<e.what()<<'\n'; return 2; }
}
