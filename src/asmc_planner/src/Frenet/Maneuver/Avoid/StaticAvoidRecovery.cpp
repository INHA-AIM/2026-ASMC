#include "Frenet/frenet_planner.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include "Frenet/Speed/AvoidSpeedMemory.hpp"
#include <algorithm>
#include <chrono>

bool FrenetPlanner::PlanStaticAvoidRecovery(const CartesianState& ego,double s,double d,
    double speed,double initial_a,const std::vector<TrackedObject>& objects,
    const PlannerCommand& command,bool committed,AvoidPrepareResult& result,
    std::chrono::steady_clock::time_point deadline) {
    auto& context=v2_avoidance_;
    if(!avoid_cfg_.static_margin_recovery_enabled||context.dynamic_target||
       result.direction_conflict||result.target_behind_ego||speed<=.05)return false;
    StaticAvoidPolicy policy;
    for(const auto& o:objects)if(IsStaticAvoidProgressObject(o))policy.object_ids.push_back(o.id);
    // A reclassified target cannot silently receive a static exception.
    if(std::find(policy.object_ids.begin(),policy.object_ids.end(),execution_->avoid.obstacle_id)==policy.object_ids.end())return false;
    const double margins[]={collision_cfg_.safety_margin,.30,.20,.10,0.0};
    std::size_t generated=result.generated_candidate_count,valid=result.valid_candidate_count;
    auto run=[&](int level,bool retained_only,AvoidPrepareResult& trial) {
        trial=result;trial.candidate_valid=false;trial.generated_candidate_count=trial.valid_candidate_count=0;
        trial.retained_geometry=false;trial.rejection_stage.clear();
        policy.mode=level==0?StaticAvoidPolicy::NORMAL:
            level==5?StaticAvoidPolicy::CONTACT_PROGRESS:StaticAvoidPolicy::REDUCED_MARGIN;
        policy.margin=level==5?0.0:margins[level];
        const double cap=level==0?speed:std::min(speed,level==5?
            avoid_cfg_.static_contact_speed_cap:avoid_cfg_.static_recovery_speed_cap);
        double seed=initial_a;
        if(level>0) {
            const double elapsed=std::max(replanning_period_s_,std::chrono::duration<double>(
                std::chrono::steady_clock::now()-context.map_speed_profile_at).count());
            const bool same_cap=context.static_progress_level==level&&context.map_speed_profile_valid;
            seed=AvoidAccelerationSeed(ego.a,ego.v,cap,same_cap,context.map_recovery_path.a,
                path_cfg_.dt,elapsed,limits_.max_longitudinal_accel*.8,limits_.max_longitudinal_accel,
                same_cap,same_cap);
        }
        const auto saved_corridor=context.map_corridor_cursor,saved_reconnect=context.map_reconnection_cursor;
        auto& slot=context.static_search_cursors[level];
        context.map_corridor_cursor=slot.corridor;context.map_reconnection_cursor=slot.reconnection;
        CartesianPath active_geometry;
        if(level==5) {
            active_geometry=std::move(context.map_spatial_geometry);
            context.map_spatial_geometry=context.last_nominal.spatial_geometry;
        }
        const auto now=std::chrono::steady_clock::now();
        const auto budget=retained_only?now:std::min(deadline,now+std::chrono::milliseconds(6));
        bool ok=PlanStaticAvoidCorridor(ego,s,d,cap,seed,objects,command,committed,trial,budget,&policy);
        if(level==5) {
            context.map_spatial_geometry=std::move(active_geometry);
            if(ok&&trial.retained_geometry) {
                const auto& old=context.last_nominal;
                trial.selected_target_d=old.selected_target_d;trial.map_entry_s=old.map_entry_s;
                trial.map_transition_distance=old.map_transition_distance;
                trial.map_return_schedule_valid=old.map_return_schedule_valid;
                trial.map_return_start_s=old.map_return_start_s;trial.map_return_end_s=old.map_return_end_s;
                trial.map_group_valid=old.map_group_valid;trial.map_group_end_s=old.map_group_end_s;
            }
        }
        if(!retained_only&&std::chrono::steady_clock::now()<budget&&(!ok||level==5)) {
            AvoidPrepareResult alternative=trial;
            alternative.candidate_valid=false;alternative.generated_candidate_count=alternative.valid_candidate_count=0;
            CartesianPath connected;
            if(PlanAvoidReconnection(ego,s,d,cap,seed,objects,committed,connected,alternative,budget,&policy)&&
               (!ok||alternative.static_contact_cost+.01<trial.static_contact_cost)) {
                generated+=trial.generated_candidate_count;valid+=trial.valid_candidate_count;
                trial=std::move(alternative);ok=true;
            } else {
                generated+=alternative.generated_candidate_count;valid+=alternative.valid_candidate_count;
            }
        }
        slot.corridor=context.map_corridor_cursor;slot.reconnection=context.map_reconnection_cursor;
        context.map_corridor_cursor=saved_corridor;context.map_reconnection_cursor=saved_reconnect;
        generated+=trial.generated_candidate_count;valid+=trial.valid_candidate_count;
        if(ok) {
            trial.static_progress_mode=policy.mode;trial.static_progress_level=level;
            trial.static_progress_margin=policy.margin;
            // A newly generated body-clear route is preferable to contact.
            if(level==5&&trial.static_contact_cost==0&&!trial.retained_geometry) {
                trial.static_progress_mode=StaticAvoidPolicy::REDUCED_MARGIN;trial.static_progress_level=4;
            }
        }
        return ok;
    };
    const auto accept=[&](AvoidPrepareResult& trial) {
        trial.generated_candidate_count=generated;trial.valid_candidate_count=valid;
        result=std::move(trial);return true;
    };
    AvoidPrepareResult trial;
    const int active=context.static_progress_level;
    if(active>0) {
        if(run(0,true,trial))++context.static_normal_clear_cycles;else context.static_normal_clear_cycles=0;
        if(context.static_normal_clear_cycles>=3)return accept(trial);
        // Keep the accepted reduced margin until a checked normal corridor is
        // stable. Contact progress continues searching for a body-clear route.
        if(active<5&&run(active,true,trial))return accept(trial);
    }
    for(int level=std::max(1,active<5?active:1);level<=4;++level) {
        if(margins[level]>=collision_cfg_.safety_margin)continue;
        if(run(level,true,trial))return accept(trial);
    }
    for(int attempt=0;attempt<4&&std::chrono::steady_clock::now()<deadline;++attempt) {
        int level=context.static_search_level;
        context.static_search_level=level==4?1:level+1;
        if(margins[level]>=collision_cfg_.safety_margin)continue;
        if(active>0&&active<5&&level<active)continue;
        if(run(level,false,trial))return accept(trial);
    }
    // Contact is an explicitly enabled simulator policy, following a previously
    // nominally checked route. It never exempts unknown/moving/new mobile actors.
    if(committed&&avoid_cfg_.static_contact_progress_enabled&&context.last_nominal.candidate_valid) {
        if(run(5,false,trial))return accept(trial);
    }
    result.generated_candidate_count=generated;result.valid_candidate_count=valid;
    return false;
}
