#include "Frenet/Maneuver/Merge/RoundaboutPrediction.hpp"
#include "Frenet/Maneuver/Merge/RoundaboutMerge.hpp"
#include "Frenet/frenet_planner.hpp"
#include <iostream>

namespace {
TrackedObject Box(int id,double x,double y,double vx,double vy,double length,double width,double height) {
    TrackedObject o{};o.id=id;o.x=x;o.y=y;o.vx=vx;o.vy=vy;o.length=length;o.width=width;o.height=height;
    o.geometry_valid=true;o.motion_state=ObjectMotionState::DYNAMIC;
    const double yaw=std::atan2(vy,vx),c=std::cos(yaw),s=std::sin(yaw);
    for(int i=0;i<8;++i) {
        const double a=(i%4==0||i%4==3)?-length/2:length/2,b=i%4<2?-width/2:width/2;
        o.corners[i]={x+a*c-b*s,y+a*s+b*c,height*(i/4)};
    }
    return o;
}
}
int main() {
    int failures=0;auto expect=[&](bool ok,const char* label){std::cout<<(ok?"[PASS] ":"[FAIL] ")<<label<<'\n';failures+=!ok;};
    const auto supported=[](double,double){return true;};
    RoundaboutPrediction history;TrackedObject o;
    for(int i=0;i<8;++i){o=Box(1,.6*i,0,4.3,0,4.5,1.9,1);history.Observe({o},100+.1*i,1,5);}
    expect(history.UncertainNear(o.x,o.y)==1,"raw position/velocity disagreement remains visible to the strict gate");
    auto bounded=history.DepartureOccupancies({o},{},6,3);
    expect(bounded.size()==1&&bounded[0].length>=4.5&&bounded[0].width>=1.9,
        "a same-direction speed disagreement has an explicit retained-body occupancy");
    auto intervals=history.WithObservedSpeedRanges(bounded);
    expect(intervals.size()==1&&intervals[0].prediction_min_speed<=4.3+1e-8&&intervals[0].prediction_max_speed>=6.5-1e-8,
        "both observed velocity and position-supported speed plus fit error remain in the final interval");
    CartesianState ego{};ego.x=o.x+12;auto ego_body=MakeEgoFootprint(ego,{.2,.2,-1},0);
    expect(!CheckObstacleOverlap(ego_body,{},o,2)&&CheckObstacleOverlap(ego_body,{},intervals[0],2),
        "speed-disagreement support cannot authorize a collision hidden by the slower measurement");
    expect(o.corners[0].x==Box(1,4.2,0,4.3,0,4.5,1.9,1).corners[0].x,
        "preparing departure evidence never edits the authoritative current corners");

    history.Reset();
    for(int i=0;i<8;++i){o=Box(2,.5*i,0,5,0,1,.5,.05);history.Observe({o},200+.1*i,1,5);}
    expect(history.UncertainNear(o.x,o.y)==2&&history.ConservativeOccupancies({o}).empty(),
        "a low strip is still not a fabricated whole-body observation");
    expect(history.DepartureOccupancies({o},{},6,3).empty(),"partial-only support needs mapped circulating-lane evidence");
    bounded=history.DepartureOccupancies({o},supported,6,3);
    expect(bounded.size()==1&&bounded[0].length>=12&&bounded[0].width>=6,
        "partial-only envelope includes the unknown fragment-to-body-centre offset");
    ego.x=o.x+5;ego_body=MakeEgoFootprint(ego,{.2,.2,-1},0);
    expect(!CheckObstacleOverlap(ego_body,{},o,0)&&CheckObstacleOverlap(ego_body,{},bounded[0],0),
        "the enlarged partial occupancy protects space beyond the visible fragment");
    history.Observe({},200.8,1,5);
    expect(history.DepartureOccupancies({o},supported,6,3).empty(),"a missing observation cannot inherit current departure evidence");
    history.Reset();
    for(int i=0;i<8;++i){o=Box(3,0,0,5,0,1,.5,.05);history.Observe({o},300+.1*i,1,5);}
    expect(history.DepartureOccupancies({o},supported,6,3).empty(),
        "a stationary fixture with a false velocity cannot gain a partial-vehicle certificate");
    history.Reset();
    for(int i=0;i<8;++i){o=Box(7,.1*i,0,5,0,1,.5,.05);history.Observe({o},350+.1*i,1,5);}
    auto slower=history.WithObservedSpeedRanges(history.DepartureOccupancies({o},supported,6,3));
    expect(slower.size()==1&&slower[0].prediction_min_speed<=.5+1e-8&&slower[0].prediction_max_speed>=5,
        "a slower position-supported partial actor cannot be incorrectly swept away at the faster sensor speed");
    history.Reset();
    for(int i=0;i<8;++i){o=Box(4,.6*i,0,0,5,4.5,1.9,1);history.Observe({o},400+.1*i,1,5);}
    expect(history.DepartureOccupancies({o},supported,6,3).empty(),
        "contradictory motion directions cannot be repaired by merely enlarging a speed interval");
    history.Reset();o=Box(5,0,0,5,0,1,.5,.05);history.Observe({o},500,1,5);
    expect(history.DepartureOccupancies({o},supported,6,3).empty(),"a new ID must establish fresh motion evidence");
    history.Reset();
    for(int i=0;i<8;++i){o=Box(6,.5*i,0,5,0,1,.5,.05);history.Observe({o},600+.1*i,1,5);}
    o=Box(6,40,0,5,0,1,.5,.05);history.Observe({o},600.8,1,5);
    expect(history.DepartureOccupancies({o},supported,6,3).empty(),"same-ID discontinuity clears the departure envelope");

    RoundaboutMergeZone zone;zone.valid=true;zone.circulating_areas={{{0,0},{10,0},{10,10},{0,10}}};
    expect(zone.SupportsCirculatingObservation(5,5)&&zone.SupportsCirculatingObservation(11.9,5),
        "circulating-lane support includes its bounded two-metre observation guard");
    expect(!zone.SupportsCirculatingObservation(12.1,5)&&!zone.SupportsCirculatingObservation(50,50),
        "a distant point is not accepted through a degenerate polygon distance query");
    FrenetPlanner planner;const bool initialized=planner.Init(
        std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/frenet/params.yaml");
    expect(initialized,"shared v6 map prepares the real roundabout for the rolling-entry contract");
    if(initialized) {
        PlannerCommand command;command.mode=MERGE;command.merge_type=MergeType::ROUNDABOUT;
        command.source_link_id="A2256W000148";command.target_link_id="A2256W000146";
        command.has_target_speed=true;command.target_speed=25.0/3.6;
        CartesianPath path;FrenetPlanner::MergePrepareResult result;bool ok=true;
        for(int i=0;i<3;++i) {
            const auto p=Interpolate(planner.reference(),planner.roundabout_zone().stop_s-2+.1*i);
            ego={p.x,p.y,p.theta,p.kappa,1,0};path={};
            ok&=planner.PlanRoundaboutMerge(ego,{},command,1000+.1*i,1000+.1*i,path,result);
        }
        expect(ok&&ego.v>0&&result.ready&&planner.CanCommitRoundaboutMerge(result),
            "fresh safe observations can authorize entry while ego is still moving");
        if(result.ready) {
            planner.ApplyRoundaboutMergeCommit(result);path={};
            ok=planner.PlanRoundaboutMerge(ego,{},command,1000.3,1000.3,path,result);
            expect(ok&&result.candidate_valid&&!path.v.empty()&&path.v.back()>.1,
                "committed rolling entry outputs the checked traversal instead of a waiting stop");
        }
    }
    return failures?1:0;
}
