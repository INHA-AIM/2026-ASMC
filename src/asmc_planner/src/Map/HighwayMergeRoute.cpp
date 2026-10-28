#include "Map/HighwayMergeRoute.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace asmc_planner::lane_reference;
using asmc_planner::Point2D;
namespace pt = boost::property_tree;
namespace {
std::vector<Point2D> Points(const pt::ptree& rows) {
    std::vector<Point2D> out;
    for (const auto& row: rows) {
        auto p=row.second.begin();
        if (row.second.size()<2) throw std::runtime_error("map coordinate needs x/y");
        const double x=p++->second.get_value<double>();
        out.push_back({x,p->second.get_value<double>()});
    }
    return out;
}
std::vector<std::string> Ids(const pt::ptree& rows) {
    std::vector<std::string> out;
    for (const auto& row:rows) out.push_back(row.second.get_value<std::string>());
    return out;
}
}

std::shared_ptr<const HighwayMergeRoute> PrepareHighwayMergeRoute(
    const RefLine& ref,const ReferenceConfig& reference_config,
    const HighwayMergeConfig& config,const VehicleShape& vehicle,
    double max_curvature,const std::string& path,double approach_decel,std::string& error,
    const std::string& markings_path) {
    error.clear();
    try {
        if (!config.enabled || ref.points.size()<3 || config.zones.empty()||
            !std::isfinite(approach_decel)||approach_decel<=0)
            throw std::runtime_error("highway route/geometry disabled or missing");
        for (const double v:{config.speed_cap_mps,config.forward_accel_mps2,config.provisional_lateral_speed_mps,config.max_horizon_s,
            config.post_merge_horizon_s,config.source_max_age_s,config.confirmation_gap_s,
            config.max_rear_decel_mps2,config.completion_settle_s,
            config.completion_heading_rad,config.completion_lateral_m,
            config.collision_max_step_m,config.planning_budget_ms,
            config.transition_tracking_position_m,config.transition_tracking_heading_rad,
            config.transition_tracking_curvature_inv_m})
            if (!std::isfinite(v)||v<=0) throw std::runtime_error("invalid highway policy scalar");
        if(config.transition_tracking_heading_rad>=M_PI/2)
            throw std::runtime_error("invalid highway transition tracking heading");
        if (!std::isfinite(config.curve_lateral_accel_safety_factor)||
            config.curve_lateral_accel_safety_factor<=0||config.curve_lateral_accel_safety_factor>1||
            !std::isfinite(config.curve_comfortable_decel_factor)||
            config.curve_comfortable_decel_factor<=0||config.curve_comfortable_decel_factor>1)
            throw std::runtime_error("invalid highway curve policy factor");
        if (!std::isfinite(config.speed_error_mps)||config.speed_error_mps<0||
            !std::isfinite(config.speed_error_floor_mps)||config.speed_error_floor_mps<0||
            config.speed_error_floor_mps>config.speed_error_mps||
            !std::isfinite(config.speed_error_correlation_s)||config.speed_error_correlation_s<.1||
            config.speed_error_correlation_s>1||
            !std::isfinite(config.motion_fit_min_span_s)||config.motion_fit_min_span_s<.1||
            config.motion_fit_min_span_s>.5||
            config.max_horizon_s>15||config.max_horizon_s<=config.post_merge_horizon_s||
            config.planning_budget_ms>40||config.max_collision_samples<2||
            config.max_collision_samples>50000||config.safe_confirm_cycles<1)
            throw std::runtime_error("invalid highway prediction/search bound");
        pt::ptree product;pt::read_json(path,product);
        LaneMapData data;
        data.identity={product.get<std::string>("frame_id"),product.get<std::string>("build_id"),
            product.get<std::string>("map_id"),product.get<std::string>("reviewed_geometry_sha256")};
        data.complete=true;
        for (const auto& row:product.get_child("lanes")) {
            const auto& p=row.second;LaneData l;
            l.id=p.get<std::string>("id");l.link_id=p.get<std::string>("link_id");
            l.left_adjacent=p.get<std::string>("left_adjacent_lane_id","");
            l.right_adjacent=p.get<std::string>("right_adjacent_lane_id","");
            l.predecessors=Ids(p.get_child("predecessor_lane_ids"));
            l.successors=Ids(p.get_child("successor_lane_ids"));
            l.centerline=Points(p.get_child("centerline"));l.polygon=Points(p.get_child("polygon"));
            const auto type=p.get<std::string>("lane_type");
            l.traversable=type!="unknown"&&type!="shoulder"&&p.get<std::string>("direction")!="unknown";
            data.lanes.push_back(std::move(l));
        }
        for(const auto& row:product.get_child("drivable_areas")) {
            const auto& p=row.second;
            data.areas.push_back({p.get<std::string>("id"),Points(p.get_child("polygon")),
                p.get<std::string>("validity")=="validated"});
        }
        if(!markings_path.empty()) {
            pt::ptree markings;pt::read_json(markings_path,markings);
            // Drivable and curated-paint products have distinct product IDs.
            // Their shared source geometry and map origin are the contract.
            if(markings.get<std::string>("sources.current_link_set.sha256")!=
                   product.get<std::string>("sources.current_link_set.sha256")||
               markings.get<std::string>("frame.frame_id")!=data.identity.frame_id)
                throw std::runtime_error("highway paint/map identity mismatch");
            for(const auto& row:markings.get_child("features")) {
                const auto& p=row.second;
                if(p.get<std::string>("feature_type","")!="lane_marking"||
                   p.get<std::string>("attributes.selection_role","")!="primary")continue;
                MarkingData m;m.id=p.get<std::string>("id");
                m.points=Points(p.get_child("geometry.coordinates"));
                m.width_m=p.get<double>("attributes.line_width_m");
                const auto pattern=p.get<std::string>("attributes.pattern","");
                m.pattern=pattern=="solid"?PaintPattern::SOLID:pattern=="broken"?PaintPattern::BROKEN:PaintPattern::UNKNOWN;
                const auto role=p.get<std::string>("attributes.functional_role","");
                m.stop_line=role=="stop_line";m.virtual_line=role=="virtual_line";
                data.markings.push_back(std::move(m));
            }
        }
        auto out=std::make_shared<HighwayMergeRoute>();
        out->map=HdLaneMap::create(std::move(data),error);
        if (!out->map) throw std::runtime_error("highway HD-map: "+error);
        out->reference=ref;out->config=config;out->vehicle=vehicle;
        out->approach_decel_mps2=approach_decel;
        const FootprintShape body{vehicle.length-vehicle.rear_overhang,vehicle.rear_overhang,vehicle.width*.5};
        if (!ValidFootprint(body)) throw std::runtime_error("invalid rear-axle footprint");
        double previous_end=-1;
        for (const auto& zone:config.zones) {
            if(zone.start_s<=previous_end||zone.completion_s>=ref.length||
               zone.checkpoints.empty()||zone.source_link.empty()||zone.target_lanes.empty()||
               !out->map->lane("lane:"+zone.source_link))
                throw std::runtime_error("invalid/missing highway zone identities: "+zone.name);
            HighwayMergeRoute::Zone prepared;prepared.config=zone;
            prepared.target_region=out->map->corridor(zone.target_lanes,error);
            if(!prepared.target_region)throw std::runtime_error(zone.name+": "+error);
            std::vector<double> x,y;
            const LaneData* previous=nullptr;
            for (const auto& id:zone.target_lanes) {
                const auto* lane=out->map->lane(id);
                if(!lane||!lane->traversable)throw std::runtime_error("non-traversable target lane: "+id);
                if(previous) {
                    const auto& a=previous->centerline.back();const auto& b=lane->centerline.front();
                    if(std::find(previous->successors.begin(),previous->successors.end(),id)==previous->successors.end()||
                       std::hypot(a.x-b.x,a.y-b.y)>.05)
                        throw std::runtime_error("disconnected target sequence: "+id);
                }
                for(const auto& p:lane->centerline) {
                    if(!x.empty()&&std::hypot(p.x-x.back(),p.y-y.back())<1e-6)continue;
                    x.push_back(p.x);y.push_back(p.y);
                }
                previous=lane;
            }
            prepared.target_reference=BuildRefLine(x,y,reference_config,max_curvature);
            if(prepared.target_reference.points.size()<3||prepared.target_reference.closed_loop)
                throw std::runtime_error("invalid target reference: "+zone.name);
            const auto completion=Interpolate(ref,zone.completion_s+vehicle.rear_overhang);
            if(!prepared.target_region->coversFootprint(completion.x,completion.y,completion.theta,body,0)||
               !out->map->road()->coversFootprint(completion.x,completion.y,completion.theta,body,0))
                throw std::runtime_error("completion footprint outside target/road: "+zone.name);
            prepared.entry_s=zone.checkpoints.front().conflict_s-body.front_m;
            for(double s=zone.start_s;s<=zone.checkpoints.front().clear_s;s+=.25) {
                const auto p=Interpolate(ref,s);
                const CartesianState pose{p.x,p.y,p.theta,p.kappa,0,0};
                if(prepared.target_region->intersectsPolygon(MakeEgoFootprint(pose,vehicle,0))) {
                    prepared.entry_s=std::min(prepared.entry_s,s-.25);break;
                }
            }
            // Trace only the SOURCE lane and its approach predecessors. Never
            // include a shared successor: travelling into it already requires
            // an accepted merge. At forks use the previously adopted lane chain.
            std::vector<std::string> source_ids{ "lane:"+zone.source_link };
            double source_length=0;
            const double needed=zone.completion_s-zone.start_s+config.approach_distance+
                config.speed_cap_mps*config.speed_cap_mps/(2*approach_decel)+vehicle.length;
            for(std::size_t depth=0;depth<64;++depth) {
                const auto* lane=out->map->lane(source_ids.front());
                for(std::size_t i=1;i<lane->centerline.size();++i)
                    source_length+=std::hypot(lane->centerline[i].x-lane->centerline[i-1].x,
                                             lane->centerline[i].y-lane->centerline[i-1].y);
                if(source_length>=needed)break;
                std::vector<std::string> previous=lane->predecessors;
                if(previous.size()>1&&!out->zones.empty()) {
                    const auto& adopted=out->zones.back().config.target_lanes;
                    previous.erase(std::remove_if(previous.begin(),previous.end(),[&](const auto& id){
                        return std::find(adopted.begin(),adopted.end(),id)==adopted.end();
                    }),previous.end());
                }
                if(previous.size()!=1||std::find(source_ids.begin(),source_ids.end(),previous.front())!=source_ids.end()||
                   std::find(zone.target_lanes.begin(),zone.target_lanes.end(),previous.front())!=zone.target_lanes.end())break;
                if(!out->map->lane(previous.front()))
                    throw std::runtime_error("missing source predecessor: "+previous.front());
                source_ids.insert(source_ids.begin(),previous.front());
            }
            prepared.source_region=out->map->corridor(source_ids,error);
            if(!prepared.source_region)throw std::runtime_error(zone.name+" source corridor: "+error);
            x.clear();y.clear();previous=nullptr;
            for(const auto& id:source_ids) {
                const auto* lane=out->map->lane(id);
                if(!lane||!lane->traversable||lane->centerline.size()<2)
                    throw std::runtime_error("invalid source lane: "+id);
                if(previous&&(std::find(previous->successors.begin(),previous->successors.end(),id)==previous->successors.end()||
                    std::hypot(previous->centerline.back().x-lane->centerline.front().x,
                               previous->centerline.back().y-lane->centerline.front().y)>.05))
                    throw std::runtime_error("disconnected source lane sequence: "+id);
                for(const auto& p:lane->centerline) {
                    if(!x.empty()&&std::hypot(p.x-x.back(),p.y-y.back())<1e-6)continue;
                    x.push_back(p.x);y.push_back(p.y);
                }
                previous=lane;
            }
            prepared.source_reference=BuildRefLine(x,y,reference_config,max_curvature);
            const auto begin=Interpolate(ref,zone.start_s);
            const double source_begin=FindClosestS(prepared.source_reference,begin.x,begin.y,nullptr,100);
            // The approach can begin before this source link (and across a
            // known microscopic polygon seam). Locate the FINAL frontier on
            // the source link itself, beyond its rear-footprint seam.
            const auto& source_start=out->map->lane("lane:"+zone.source_link)->centerline.front();
            const double scan_begin=std::max(source_begin,
                FindClosestS(prepared.source_reference,source_start.x,source_start.y,nullptr,100)+vehicle.length);
            double frontier=scan_begin;
            std::string boundary_detail;
            for(double s=scan_begin;s<=prepared.source_reference.length;s+=.25) {
                const auto p=Interpolate(prepared.source_reference,s);
                const CartesianState pose{p.x,p.y,p.theta,p.kappa,0,0};
                if(!prepared.source_region->coversFootprint(p.x,p.y,p.theta,body,0)||
                   !out->map->road()->coversFootprint(p.x,p.y,p.theta,body,0)||
                   out->map->touchesSolid(p.x,p.y,p.theta,body,0)) {
                    boundary_detail=" source="+std::to_string(prepared.source_region->coversFootprint(p.x,p.y,p.theta,body,0))+
                        " road="+std::to_string(out->map->road()->coversFootprint(p.x,p.y,p.theta,body,0))+
                        " paint="+std::to_string(out->map->touchesSolid(p.x,p.y,p.theta,body,0));
                    break;
                }
                frontier=s;
            }
            prepared.source_stop_local_s=frontier-config.stop_buffer;
            const auto stop=Interpolate(prepared.source_reference,prepared.source_stop_local_s);
            prepared.source_stop_s=FindClosestS(ref,stop.x,stop.y,&zone.start_s,100);
            if(prepared.source_stop_local_s<=source_begin||prepared.source_stop_s<=zone.start_s)
                throw std::runtime_error("no source-only stopping corridor: "+zone.name+
                    " begin="+std::to_string(source_begin)+" frontier="+std::to_string(frontier)+boundary_detail);
            // Catch a wrong station/origin before an on-road request can occur.
            for(const auto& checkpoint:zone.checkpoints) {
                const auto p=Interpolate(ref,checkpoint.clear_s);
                const double target_s=FindClosestS(prepared.target_reference,p.x,p.y,nullptr,100);
                const auto q=Interpolate(prepared.target_reference,target_s);
                if(std::hypot(p.x-q.x,p.y-q.y)>config.target_corridor_half_width||
                   std::cos(p.theta-q.theta)<config.min_heading_alignment)
                    throw std::runtime_error("checkpoint disagrees with HD-map target: "+zone.name);
            }
            out->zones.push_back(std::move(prepared));previous_end=zone.completion_s;
        }
        return out;
    } catch(const std::exception& e) {error=e.what();return {};}
}

HighwayMergeApproach HighwayApproachAt(const HighwayMergeRoute& route,
    const CartesianState& ego,const double* hint,double max_lateral_distance) {
    HighwayMergeApproach result;
    if(!std::isfinite(ego.x)||!std::isfinite(ego.y)||!std::isfinite(ego.yaw)||
       !std::isfinite(ego.v)||ego.v<0)return result;
    const double s=FindClosestS(route.reference,ego.x,ego.y,hint,50);
    const auto p=Interpolate(route.reference,s);
    if(!std::isfinite(max_lateral_distance)||max_lateral_distance<=0||
       std::hypot(ego.x-p.x,ego.y-p.y)>max_lateral_distance||std::cos(ego.yaw-p.theta)<.5)return result;
    for(std::size_t i=0;i<route.zones.size();++i) {
        const auto& z=route.zones[i].config;
        const double activation=std::min(z.start_s-route.config.approach_distance,
            route.zones[i].source_stop_s-ego.v*ego.v/(2*route.approach_decel_mps2)-route.config.approach_distance);
        if(s<activation||s>z.completion_s+route.config.speed_cap_mps*(route.config.completion_settle_s+route.config.source_max_age_s+route.config.post_merge_horizon_s)+route.vehicle.length)continue;
        result.valid=true;result.approach=s<z.completion_s;result.zone_index=i;
        result.route_s=s;result.speed_cap_mps=route.config.speed_cap_mps;
        result.source_link=z.source_link;
        result.target_link=route.map->lane(z.target_lanes.back())->link_id;
        return result;
    }
    return result;
}

HighwayMergeApproach UpdateHighwayApproach(const HighwayMergeRoute& route,
    const CartesianState& ego,HighwayMergeApproach& previous,double max_lateral_distance) {
    if(previous.valid&&previous.zone_index<route.zones.size()) {
        const double s=FindClosestS(route.reference,ego.x,ego.y,&previous.route_s,50);
        const auto p=Interpolate(route.reference,s);
        const auto& zone=route.zones[previous.zone_index].config;
        if(std::isfinite(s)&&s>=previous.route_s-1&&s<=zone.completion_s+route.config.speed_cap_mps*(route.config.completion_settle_s+route.config.source_max_age_s+route.config.post_merge_horizon_s)+route.vehicle.length&&
           std::hypot(ego.x-p.x,ego.y-p.y)<=route.config.target_corridor_half_width&&std::cos(ego.yaw-p.theta)>=.5) {
            previous.route_s=s;previous.approach=s<zone.completion_s;return previous;
        }
    }
    previous=HighwayApproachAt(route,ego,nullptr,max_lateral_distance);return previous;
}
