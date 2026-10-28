#include "Frenet/frenet_planner.hpp"
#include "Frenet/Maneuver/Merge/HighwayMerge.hpp"
#include "Frenet/Speed/TargetSpeedResolver.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

int main() {
    int failures=0;
    const auto expect=[&](bool ok,const char* text){std::cout<<(ok?"PASS ":"FAIL ")<<text<<'\n';failures+=!ok;};
    FrenetPlanner mission;std::string error;
    if(!mission.Init(FRENET_TEST_CONFIG)||!mission.EnableHighwayMerge(error))return 2;
    const auto route=mission.highway_route();
    CurveSpeedConfig ordinary{};ordinary.target_vel=58/3.6;ordinary.curve_lookahead_m=27;
    ordinary.lateral_accel_safety_factor=.95;ordinary.comfortable_decel_factor=.65;ordinary.min_usable_decel=.5;
    PlannerCommand command;command.mode=LANE_KEEPING;command.source_link_id=route->map->lane(route->zones.back().config.target_lanes.back())->link_id;
    command.has_target_speed=true;command.target_speed=route->config.speed_cap_mps;
    const bool cruise=IsHighwayCruiseCommand(route->config,route->map.get(),command);
    const auto highway=ResolveHighwayCurveSpeed(ordinary,route->config,true,command,cruise);
    expect(cruise&&std::abs(highway.target_vel-route->config.speed_cap_mps)<1e-9,
        "common downstream highway lane retains highway cap on the mission reference");
    expect(!IsHighwayCruiseCommand(route->config,nullptr,command),"missing map cannot authorize highway speed");
    auto city=command;city.source_link_id="not_a_configured_highway_link";
    expect(!IsHighwayCruiseCommand(route->config,route->map.get(),city),"leaving configured highway restores ordinary policy");
    for(auto mode:{STOP,AVOID,EMERGENCY}) {
        auto other=command;other.mode=mode;
        const auto policy=ResolveHighwayCurveSpeed(ordinary,route->config,true,other,true);
        expect(policy.target_vel==ordinary.target_vel,"STOP/AVOID/emergency keep their existing ceiling");
    }
    RefLine straight;straight.length=500;
    for(int i=0;i<=500;++i)straight.points.push_back({double(i),0,double(i),0,0,0});
    auto limits=mission.kinematic_limits();
    TargetSpeedInput input{100,58/3.6,true,route->config.speed_cap_mps,false,0};
    expect(std::abs(ResolveTargetSpeed(straight,input,highway,limits)-route->config.speed_cap_mps)<1e-9,
        "actual target resolver no longer clips a 70 km/h highway request to 58");
    input.has_external_max_speed=true;input.external_max_speed=8;
    expect(ResolveTargetSpeed(straight,input,highway,limits)<=8,"lower external safety caps remain authoritative");
    input.has_external_max_speed=false;input.external_target_speed=7;
    expect(ResolveTargetSpeed(straight,input,highway,limits)<=7,"low-speed requests are not raised by highway ownership");
    const double s=FindClosestS(mission.reference(),70.3,-285,nullptr,100);
    const auto point=Interpolate(mission.reference(),s);
    CartesianState ego{point.x,point.y,point.theta,point.kappa,58/3.6,0};CartesianPath path;
    const bool ok=mission.Plan(ego,{},command,path);
    expect(ok&&mission.last_base_target_speed()>58/3.6+1e-3,
        "mission core applies highway policy to real planning after reference return");
    if(ok)expect(*std::max_element(path.v.begin(),path.v.end())>58/3.6,
        "KEEP speed envelope permits actual acceleration above 58, not just a diagnostic cap");
    std::cout<<"failures="<<failures<<'\n';return failures?1:0;
}
