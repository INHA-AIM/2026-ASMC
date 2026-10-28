#include "Execution/HighwayReferenceExecution.hpp"
#include "Frenet/Generator/CartesianRetimer.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace asmc_planner::lane_reference;
namespace {
int checks=0,failures=0;
void expect(bool ok,const char* message){++checks;if(!ok){++failures;std::cerr<<"FAIL "<<message<<'\n';}}
CartesianState at(const RefLine& r,double s,double v,double d=0){const auto p=Interpolate(r,s);return {p.x-d*std::sin(p.theta),p.y+d*std::cos(p.theta),p.theta,p.kappa,v,0};}
TrackedObject box(const CartesianState& e,int id,double speed,double length=4.5,double width=1.8){
    TrackedObject o;o.id=id;o.type=int(asmc_planner::ObjectType::VEHICLE);o.x=e.x;o.y=e.y;o.heading=e.yaw;
    o.vx=speed*std::cos(e.yaw);o.vy=speed*std::sin(e.yaw);o.speed=speed;
    o.length=length;o.width=width;o.height=1.5;o.motion_state=speed>0?ObjectMotionState::DYNAMIC:ObjectMotionState::LOW_THREAT;
    o.geometry_valid=o.z_valid=true;
    const double x[]={length*.5,-length*.5,-length*.5,length*.5},y[]={width*.5,width*.5,-width*.5,-width*.5};
    for(int i=0;i<8;++i)o.corners[i]={e.x+std::cos(e.yaw)*x[i%4]-std::sin(e.yaw)*y[i%4],e.y+std::sin(e.yaw)*x[i%4]+std::cos(e.yaw)*y[i%4],i<4?0.:1.5};return o;
}
struct Trial {
    FrenetPlanner core;std::shared_ptr<const HighwayMergeRoute> route;
    std::shared_ptr<const LaneReference> source,target;HighwayMergeRoute::Zone zone;PlannerCommand command;
    CartesianState ego;CartesianPath path;FrenetPlanner::HighwayMergeResult result;double now=100;
    Trial(const HighwayReferenceStatus& network,std::shared_ptr<const HighwayMergeRoute> r,bool left):route(r) {
        source=network.network->lanes[left?0:1];target=network.network->lanes[left?1:0];
        if(!core.Init(FRENET_TEST_CONFIG)||!core.SetPreparedReference(source->line()))throw std::runtime_error("source init");
        core.ConfigurePreparedHighwayLane(route->config,route->map,source->lanes());core.ConfigureHighwayTrafficLanes(network.network->lanes);
        ego=at(source->line(),190,14);const auto* road=route->map->lane(source->lanes().front());
        double nearest=INFINITY;
        for(const auto& id:source->lanes())for(const auto& p:route->map->lane(id)->centerline) {
            const double distance=std::hypot(p.x-ego.x,p.y-ego.y);if(distance<nearest){nearest=distance;road=route->map->lane(id);}}
        command.mode=left?LANE_CHANGE_LEFT:LANE_CHANGE_RIGHT;command.has_target_speed=true;command.target_speed=route->config.speed_cap_mps;
        command.source_link_id=road->link_id;command.target_link_id=(left?road->left_adjacent:road->right_adjacent).substr(5);
        std::cerr<<(left?"LEFT ":"RIGHT ")<<command.source_link_id<<" -> "<<command.target_link_id<<'\n';
        zone.config.name="optional_test";zone.config.source_link=command.source_link_id;zone.config.target_lanes={"lane:"+command.target_link_id};
        zone.config.start_s=190;zone.config.completion_s=source->line().length-80;
        zone.config.checkpoints={{zone.config.start_s,zone.config.completion_s}};
        zone.source_reference=source->line();zone.target_reference=target->line();
        zone.source_stop_local_s=source->usableEnd()-10;
        std::string error;auto ids=source->lanes();ids.pop_back();zone.source_region=route->map->corridor(ids,error);
        ids=target->lanes();ids.pop_back();zone.target_region=route->map->corridor(ids,error);
        for(int i=0;i<12&&!result.ready;++i)step({});
        if(!result.ready)throw std::runtime_error("not ready: "+result.detail);
        core.ApplyHighwayMergeCommit(result);path=result.candidate_path;
    }
    bool step(const std::vector<TrackedObject>& objects){now+=core.sample_dt();return core.PlanHighwayTransition(*route,zone,true,ego,objects,command,now,now,path,result,now,target->line());}
    void advance(){if(path.x.size()<2)throw std::runtime_error("no path");ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};}
    void partial(double displacement){for(int i=0;i<100;++i){const auto p=ProjectOnLaneReference(*source,ego.x,ego.y,ego.yaw,ego.v);if(std::abs(p.d)>=displacement)return;advance();if(!step({})||!result.candidate_valid)throw std::runtime_error(result.detail);}throw std::runtime_error("no lateral progress");}
};
void exercise(const HighwayReferenceStatus& status,std::shared_ptr<const HighwayMergeRoute> route,bool left) {
    {
        Trial t(status,route,left);t.partial(1.2);
        const auto p=ProjectOnLaneReference(*t.target,t.ego.x,t.ego.y,t.ego.yaw,t.ego.v);
        double leader_s=p.s+28;
        auto leader=box(at(t.target->line(),leader_s,11),71,11);
        const bool ok=t.step({leader});
        std::cerr<<"following "<<ok<<" "<<t.result.detail<<" leader="<<t.result.following_object_id<<'\n';
        expect(ok&&t.result.candidate_valid&&!t.result.returning_to_source,"partial entry continues when target-front following is feasible");
        expect(t.result.following_object_id==71,"existing FOLLOW generator supplies the connector speed schedule");
        expect(!t.path.v.empty()&&t.path.v.back()>1,"a moving target leader does not become a zero-speed maintenance stop");
        expect(t.core.highway_merge_committed(),"following keeps the original committed transaction");
        // A still-UNKNOWN moving front vehicle must survive actual reference
        // adoption; otherwise FOLLOW works during crossing and disappears at
        // exactly the handover that should retain it.
        FrenetPlanner target_core;if(!target_core.Init(FRENET_TEST_CONFIG))throw std::runtime_error("target init");
        HandoverOptions alignment{route->config.completion_lateral_m,route->config.completion_heading_rad,
            .2,route->config.completion_settle_s,route->config.source_max_age_s,route->config.source_max_age_s,.21};
        HandoverContinuity continuity{1,.35,.05,.03,.5,1};
        LaneReferenceExecution handover(t.core,target_core,t.target,alignment,continuity,3,t.target->usableBegin());
        target_core.ConfigurePreparedHighwayLane(route->config,route->map,t.target->lanes());
        target_core.ConfigureHighwayTrafficLanes(status.network->lanes);
        handover.begin(1);bool adopted=false;
        for(int n=0;n<120&&!adopted;++n) {
            t.advance();leader_s+=11*t.core.sample_dt();
            leader=box(at(t.target->line(),leader_s,11),71,11);leader.type=int(asmc_planner::ObjectType::UNKNOWN);
            if(!t.step({leader})||!t.result.candidate_valid)break;
            ReferenceExecutionInput input;input.map=route->map->identity();input.sequence=n+1;
            input.observation={t.now,t.now,t.now,true,true,t.ego};input.objects={leader};std::string detail;
            if(handover.prepare(input,t.path,t.command.target_speed,detail)) {
                const auto* frame=handover.preparedFrame();
                expect(frame&&frame->leader_id==71&&frame->mode==FOLLOWING,
                    "target adoption retains a moving UNKNOWN front leader and FOLLOW mode");
                adopted=handover.commit(input,[]{return true;},detail);
            }
        }
        expect(adopted,"following connector reaches actual target reference adoption in both directions");

    }
    {
        Trial t(status,route,left);t.partial(.3);
        const auto p=ProjectOnLaneReference(*t.target,t.ego.x,t.ego.y,t.ego.yaw,t.ego.v);
        auto wall=box(at(t.target->line(),p.s+65,0),72,0,150,1.8);
        const bool ok=t.step({wall});
        std::cerr<<"return "<<ok<<" "<<t.result.detail<<'\n';
        expect(ok&&t.result.returning_to_source,"physically blocked target can select a checked source-lane return");
        expect(!t.result.source_return_completed,"issuing a return is not reporting that the actual vehicle returned");
        bool returned=false;
        if(t.result.returning_to_source)for(int i=0;i<120&&!returned;++i){t.advance();if(!t.step({}))break;returned=t.result.source_return_completed;
            expect(t.result.returning_to_source,"a safe return remains owned when the former target opens again");}
        expect(returned,"actual source alignment and settling complete the return");
        expect(!t.result.maneuver_completed,"source return does not claim successful target-lane adoption");
        expect(t.source->coversFootprint(t.ego.x,t.ego.y,t.ego.yaw),"returned body is entirely within the source lane");
    }
    {
        Trial t(status,route,left);t.partial(.3);
        const auto target=ProjectOnLaneReference(*t.target,t.ego.x,t.ego.y,t.ego.yaw,t.ego.v);
        const auto source=ProjectOnLaneReference(*t.source,t.ego.x,t.ego.y,t.ego.yaw,t.ego.v);
        t.zone.source_stop_local_s=source.s+75;
        const auto wall=box(at(t.target->line(),target.s+65,0),75,0,150,1.8);
        const bool ok=t.step({wall});
        expect(ok&&t.result.returning_to_source,
            "a bounded return uses the certified arrival/tail rather than an unissued distant horizon");
        if(ok&&t.result.returning_to_source)expect(FindClosestS(t.source->line(),t.path.x.back(),t.path.y.back(),nullptr,100)<=t.zone.source_stop_local_s,
            "every issued return sample ends before the source-only frontier");
    }
    {
        Trial t(status,route,left);t.ego=at(t.target->line(),FindClosestS(t.target->line(),t.ego.x,t.ego.y,nullptr,100)+50,12);
        auto obstacle=box(at(t.target->line(),FindClosestS(t.target->line(),t.ego.x,t.ego.y,nullptr,100)+12,0),73,0);
        t.step({obstacle});expect(!t.result.returning_to_source,"fully established target occupancy never triggers a late ordinary abort");
    }
    {
        Trial t(status,route,left);t.partial(.3);auto overlap=box(t.ego,74,0,6,3);
        expect(!t.step({overlap})&&!t.result.candidate_valid&&!t.result.returning_to_source,"current physical collision is not exempted by following or return");
        auto timeout=*route;timeout.config.planning_budget_ms=0;t.now+=.05;
        t.core.PlanHighwayTransition(timeout,t.zone,true,t.ego,{},t.command,t.now,t.now,t.path,t.result,t.now,t.target->line());
        expect(!t.result.returning_to_source,"computation timeout alone cannot authorize an abort");
    }
}
}
int main(){try{FrenetPlanner mission;std::string error;
    if(!mission.Init(FRENET_TEST_CONFIG)||!mission.EnableHighwayMerge(error))throw std::runtime_error(error);
    asmc_planner::execution::ManeuverState state;HighwayReferenceExecution refs(mission,FRENET_TEST_CONFIG,state);
    for(bool left:{true,false})exercise(refs.status(),mission.highway_route(),left);
}catch(const std::exception& e){++failures;std::cerr<<e.what()<<'\n';}
std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;}
