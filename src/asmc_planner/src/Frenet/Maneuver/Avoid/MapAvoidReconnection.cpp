#include "Frenet/frenet_planner.hpp"
#include "Frenet/Math/Polynomial.hpp"
#include "Frenet/Math/Utils.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include "Frenet/Evaluation/KinematicFilter.hpp"
#include "Frenet/Generator/StaticAvoidPath.hpp"
#include "Frenet/Geometry/AvoidanceGeometry.hpp"
#include <algorithm>
#include <cmath>

// Shared blocked-path search for bypass and return. Lifecycle determines
// completion/side ownership, not the admissible shape of the intermediate path.
bool FrenetPlanner::PlanAvoidReconnection(const CartesianState& ego,double s,double d,
    double speed,double initial_a,const std::vector<TrackedObject>& obstacles,
    bool committed,CartesianPath& output,AvoidPrepareResult& result,
    std::chrono::steady_clock::time_point deadline,const StaticAvoidPolicy* policy) {
    const double length=ref_.points.back().s;
    const double gap=ReferenceArcDistance(ref_, s, v2_avoidance_.obstacle_s);
    const double pass=gap+v2_avoidance_.obstacle_longitudinal_half_extent+
        vehicle_shape_.rear_overhang+avoid_cfg_.pass_clearance;
    const bool target_ahead=pass>0.0;
    const bool left=v2_avoidance_.target_d>=v2_avoidance_.obstacle_d;
    const auto initial_ref=Interpolate(ref_,s);
    const double delta=std::remainder(ego.yaw-initial_ref.theta,2*M_PI);
    const double slope=std::tan(delta)*(1-initial_ref.kappa*d);
    const double cosine=std::cos(delta);
    if(std::fabs(cosine)<.2||!std::isfinite(slope))return false;
    auto second_for_curvature=[&](double curvature){
        const double metric=1-initial_ref.kappa*d;
        return (curvature-initial_ref.kappa*cosine/metric)*metric*metric/
            (cosine*cosine*cosine)-(initial_ref.d_kappa*d+initial_ref.kappa*slope)*std::tan(delta);
    };
    std::vector<double> curvatures{ego.kappa};
    // Wheel angle is unavailable at rest. Keep the observed curvature instead
    // of inventing an initial steering state to make a candidate executable.
    std::vector<double> offsets{0.0,d,v2_avoidance_.target_d};
    std::vector<double> first_lengths;
    std::vector<TrackedObject> fixed_objects;
    for(const auto& object:obstacles) {
        if(!object.geometry_valid||object.motion_state!=ObjectMotionState::LOW_THREAT)continue;
        if(!object.velocity_unavailable)fixed_objects.push_back(object);
        double os=0,od=0,ov=0;
        ProjectObjectToFrenet(ref_,object,os,od,ov,ref_config_.ego_projection_search_window);
        const double ahead=ReferenceArcDistance(ref_, s, os);
        if(ahead<0||ahead>40)continue;
        const auto bounds=ProjectAvoidFootprint(object.corners,object.x,object.y,Interpolate(ref_,os).theta);
        if(!bounds.valid)continue;
        const bool reduced=policy&&policy->mode!=StaticAvoidPolicy::NORMAL&&
            std::find(policy->object_ids.begin(),policy->object_ids.end(),object.id)!=policy->object_ids.end();
        const double safety=reduced?std::min(collision_cfg_.safety_margin,policy->margin):collision_cfg_.safety_margin;
        const double margin=.5*vehicle_shape_.width+safety+.05;
        offsets.push_back(od+bounds.max_d+margin);offsets.push_back(od+bounds.min_d-margin);
        if(od+bounds.min_d<=d+margin&&od+bounds.max_d>=d-margin)
            first_lengths.push_back(ahead+bounds.far_s+vehicle_shape_.rear_overhang+avoid_cfg_.pass_clearance);
    }
    for(double value=-4;value<=4.0001;value+=.25)offsets.push_back(value);
    std::sort(offsets.begin(),offsets.end());
    offsets.erase(std::unique(offsets.begin(),offsets.end(),[](double a,double b){return std::fabs(a-b)<1e-6;}),offsets.end());
    std::stable_sort(offsets.begin(),offsets.end(),[&](double a,double b){return std::fabs(a-d)<std::fabs(b-d);});
    const double crawl=std::min(speed,avoid_cfg_.bypass_speed_cap);
    if(crawl<=.05)return false;
    std::size_t ordinal=0;
    const std::size_t resume=v2_avoidance_.map_reconnection_cursor;
    bool has_best=false;AvoidPrepareResult best_result;
    const auto accept_best=[&]() {
        if(!has_best)return false;
        const auto generated=result.generated_candidate_count,valid=result.valid_candidate_count;
        result=best_result;result.generated_candidate_count=generated;result.valid_candidate_count=valid;
        output=result.candidate_path;v2_avoidance_.map_reconnection_cursor=0;return true;
    };
    const double budget=std::max(0.0,6.0-v2_avoidance_.map_recovery_travel_m);
    VehicleShape map_shape=vehicle_shape_;
    map_shape.width=std::max(map_shape.width,map_shape.road_outer_width);
    first_lengths.insert(first_lengths.end(),{3.0,4.5,6.0,8.0,10.0,12.0,16.0,20.0,24.0,32.0});
    for(double first_length:first_lengths){
        if(first_length<pass)continue;
        for(double via:offsets){
            if(std::fabs(via)>4.0)continue;
            if(v2_avoidance_.has_requested_target_d&&std::fabs(via-v2_avoidance_.requested_target_d)>1e-6)continue;
            for(double via_slope:{0.0,.25,-.25,.5,-.5})for(double curvature:curvatures)
            for(double last_length:{8.0,12.0,16.0}){
                const auto index=ordinal++;
                if(index<resume)continue;
                if(std::chrono::steady_clock::now()>=deadline){
                    if(accept_best())return true;
                    v2_avoidance_.map_reconnection_cursor=index;return false;
                }
                ++result.generated_candidate_count;
                const auto first=MakeQuintic(d,slope,second_for_curvature(curvature),via,via_slope,0,first_length);
                const auto back=MakeQuintic(via,via_slope,0,0,0,0,last_length);
                if(target_ahead) {
                    // Direction belongs to the actual primary passage, not to
                    // a waypoint reached after its rear edge. This permits a
                    // checked S return while retaining the committed side.
                    const double at_primary=EvalPos(first,std::clamp(gap,0.0,first_length));
                    const auto direction=execution_->avoid.requested_direction;
                    if((direction==LateralDirection::LEFT&&at_primary<=v2_avoidance_.obstacle_d)||
                       (direction==LateralDirection::RIGHT&&at_primary>=v2_avoidance_.obstacle_d))continue;
                    if(committed&&(v2_avoidance_.map_lateral_started||v2_avoidance_.map_side_reselected||std::fabs(slope)>.1)&&
                       ((at_primary>=v2_avoidance_.obstacle_d)!=left))continue;
                }
                const double total=first_length+last_length;
                auto sample=[&](double station){
                    const double z=station<first_length?station:station-first_length;
                    const auto& polynomial=station<first_length?first:back;
                    const auto rp=Interpolate(ref_,s+station);
                    return FrenetToCartesian(rp,s+station,1,0,EvalPos(polynomial,z),EvalVel(polynomial,z),EvalAcc(polynomial,z));
                };
                bool geometry_valid=true;double peak_curvature=0;
                CartesianPath geometry;
                // Spatial precheck prevents time sampling from jumping over a
                // short curvature peak. Collision/road are rechecked in time.
                for(double station=0;station<=total+.0001;station+=.1){
                    const auto point=sample(std::min(station,total));
                    if(!std::isfinite(point.kappa)||std::fabs(point.kappa)>limits_.max_curvature+1e-6){geometry_valid=false;break;}
                    peak_curvature=std::max(peak_curvature,std::fabs(point.kappa));
                    geometry.x.push_back(point.x);geometry.y.push_back(point.y);
                    geometry.yaw.push_back(point.yaw);geometry.kappa.push_back(point.kappa);
                }
                if(!geometry_valid){result.rejection_stage="reconnection.curvature";continue;}
                const double cap=std::min(crawl,std::sqrt(limits_.max_lateral_accel/std::max(1e-6,peak_curvature)));
                CartesianPath candidate;double station=0,v=std::max(0.0,ego.v),a=initial_a;
                bool complete=false;
                if(!v2_avoidance_.dynamic_target) {
                    // Reconnect geometry can require a lower speed for slew
                    // than for lateral acceleration. Retiming creates that
                    // candidate instead of merely rejecting the fast version.
                    const double tail=std::max(8.0,crawl*crawl/(limits_.max_longitudinal_accel*.8)+crawl);
                    for(double extra=.1;extra<=tail+.1;extra+=.1) {
                        const auto rp=Interpolate(ref_,s+total+std::min(extra,tail));
                        geometry.x.push_back(rp.x);geometry.y.push_back(rp.y);
                        geometry.yaw.push_back(rp.theta);geometry.kappa.push_back(rp.kappa);
                    }
                    int blocker=-1;
                    if(!StaticAvoidTrajectoryClear(geometry,fixed_objects,vehicle_shape_,collision_cfg_,
                            0,policy,&blocker,nullptr,&v2_avoidance_.margin_recovery_ids)) {
                        result.blocking_object_id=blocker;
                        result.rejection_stage="reconnection.spatial_collision";continue;
                    }
                    auto limits=limits_;
                    limits.max_forward_accel=std::min(limits.max_forward_accel,avoid_cfg_.bypass_accel_cap);
                    std::string rejected;
                    if(!RetimeStaticAvoidGeometry(geometry,ego.v,initial_a,cap,path_cfg_.dt,wheelbase_,limits,candidate,&rejected)) {
                        result.rejection_stage="reconnection."+rejected;continue;
                    }
                    complete=true;
                }
                for(int i=0;!complete&&i<1000;++i){
                    const auto point=sample(station);
                    if(v*v*std::fabs(point.kappa)>limits_.max_lateral_accel+1e-6)break;
                    candidate.x.push_back(point.x);candidate.y.push_back(point.y);candidate.yaw.push_back(point.yaw);
                    candidate.kappa.push_back(point.kappa);candidate.v.push_back(v);candidate.a.push_back(a);
                    if(station>=total-.05){complete=candidate.x.size()>1;break;}
                    const double desired=clip((cap-v)/.8,-limits_.max_longitudinal_accel*.8,limits_.max_longitudinal_accel);
                    const double next_a=clip(desired,a-limits_.max_longitudinal_jerk*path_cfg_.dt,a+limits_.max_longitudinal_jerk*path_cfg_.dt);
                    const double next_v=std::max(0.0,v+(a+next_a)*.5*path_cfg_.dt);
                    // sample(..., s_dot=1) reports ds-to-Cartesian arc metric.
                    station=std::min(total,station+(v+next_v)*.5*path_cfg_.dt/std::max(.1,point.v));
                    v=next_v;a=next_a;
                }
                if(!complete||!PathObeysSteeringRate(candidate,path_cfg_.dt,wheelbase_,limits_.steering))continue;
                const auto road=avoid_road_map_.EvaluateRecoveryPath(candidate,map_shape,path_cfg_.dt,
                    budget,v2_avoidance_.map_recovery_overlap_limit_m,v2_avoidance_.map_recovery_active,
                    v2_avoidance_.map_recovery_settled_travel_m,true,
                    solid_paint_contact_elapsed_s_);
                int blocking=-1;bool recovery=false;
                const bool clear=road.drivable&&(v2_avoidance_.dynamic_target?
                    CartesianAvoidTrajectoryClear(candidate,obstacles,vehicle_shape_,collision_cfg_,path_cfg_.dt,
                        &blocking,execution_->avoid.obstacle_id,&recovery):
                    StaticAvoidTrajectoryClear(candidate,obstacles,vehicle_shape_,collision_cfg_,path_cfg_.dt,
                        policy,&blocking,&recovery,&v2_avoidance_.margin_recovery_ids));
                last_candidate_paths_.push_back(candidate);last_candidate_validity_.push_back(clear);
                if(!clear){result.rejection_stage=road.drivable?"reconnection.collision":"reconnection.road";if(blocking>=0)result.blocking_object_id=blocking;continue;}
                ++result.valid_candidate_count;result.blocking_object_id=-1;
                result.candidate_valid=result.spatial_candidate_selected=true;result.escape_recovery_required=recovery;
                result.selected_target_d=via;result.selected_cost=total+AvoidRoadMap::PaintCost(road);
                result.map_entry_s=s;result.map_transition_distance=first_length;
                result.map_return_schedule_valid=true;
                result.map_return_start_s=std::fmod(s+first_length,length);
                result.map_return_end_s=std::fmod(s+total,length);
                result.spatial_geometry=geometry;
                result.candidate_path=candidate;output=candidate;
                result.rejection_stage.clear();result.search_pending=false;
                if(policy&&policy->mode!=StaticAvoidPolicy::NORMAL) {
                    const auto contact=MeasureStaticAvoidContact(geometry,obstacles,vehicle_shape_,*policy);
                    result.static_contact_cost=contact.cost;result.static_minimum_clearance=contact.minimum_clearance;
                    if(policy->mode==StaticAvoidPolicy::CONTACT_PROGRESS&&contact.physical_overlap) {
                        if(!has_best||contact.cost<best_result.static_contact_cost) {
                            best_result=result;has_best=true;
                        }
                        result.candidate_valid=false;continue;
                    }
                }
                v2_avoidance_.map_reconnection_cursor=0;return true;
            }
        }
    }
    if(accept_best())return true;
    v2_avoidance_.map_reconnection_cursor=0;return false;
}
