#include "Frenet/frenet_planner.hpp"
#include "Frenet/Math/Utils.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
    int failures=0;
    auto check=[&](bool ok,const char* text) {
        std::cout<<(ok?"[PASS] ":"[FAIL] ")<<text<<'\n';failures+=!ok;
    };
    const std::filesystem::path root=ASMC_FUTURE_SOURCE;
    const auto dir=std::filesystem::temp_directory_path()/"asmc_keep_future_acceleration";
    std::filesystem::create_directories(dir);
    {std::ofstream out(dir/"waypoints.txt");for(int x=0;x<=500;++x)out<<x<<" 0 0\n";}
    auto cfg=YAML::LoadFile((root/"config/frenet/params.yaml").string());
    cfg["waypoint_file"]=(dir/"waypoints.txt").string();
    cfg["avoid_hd_map"]["enabled"]=false;cfg["roundabout_merge"]["enabled"]=false;
    cfg["planner"]["visualization"]["enabled"]=false;
    {std::ofstream out(dir/"params.yaml");out<<cfg;}
    auto box=[](int id,double x,double width,double length,double vx) {
        TrackedObject o;o.id=id;o.x=x;o.width=width;o.length=length;o.height=1;
        o.vx=vx;o.speed=std::fabs(vx);o.motion_state=ObjectMotionState::DYNAMIC;
        for(int i=0;i<8;++i) {
            const double xs[4]={-.5,.5,.5,-.5},ys[4]={-.5,-.5,.5,.5};
            o.corners[i]={x+length*xs[i%4],width*ys[i%4],double(i/4)};
        }
        o.geometry_valid=o.z_valid=true;return o;
    };
    CartesianState ego{};ego.x=100;ego.v=4;
    PlannerCommand keep;keep.mode=LANE_KEEPING;keep.has_target_speed=true;keep.target_speed=12;
    FrenetPlanner planner;check(planner.Init((dir/"params.yaml").string()),"straight test road initializes");
    planner.SetObjectObservationTime(100,100);
    const auto rear=box(1,88,1.9,4.5,7);
    CartesianPath path;const bool ok=planner.Plan(ego,{rear},keep,path);
    check(ok&&path.x.size()>2,"KEEP can choose physically feasible acceleration away from a rear approach");
    if(ok) {
        check(EvaluateCartesianTrajectorySafety(path,{rear},planner.vehicle_shape(),
            planner.collision_config(),planner.sample_dt()).clear,
            "accelerating path passes the unchanged complete collision horizon");
        check(path.v.back()>ego.v+.5,"future path represents real acceleration, not a frozen next-cycle setpoint");
        check(std::fabs(path.v.front()-ego.v)<1e-9&&path.v[1]-ego.v<.04,
            "current measured speed and near-term jerk continuity are retained");
    }
    FrenetPlanner nominal;
    check(nominal.Init((dir/"params.yaml").string()),"nominal road initializes independently");
    nominal.SetObjectObservationTime(100,100);
    CartesianPath nominal_path;
    check(nominal.Plan(ego,{},keep,nominal_path)&&!nominal_path.v.empty()&&
        nominal_path.v.back()<ego.v+.05,
        "unblocked nominal KEEP retains the original next-cycle speed profile");
    planner.SetObjectObservationTime(101,101);path={};keep.target_speed=4.5;
    check(planner.Plan(ego,{},keep,path),"an explicit lower speed cap remains feasible");
    check(!path.v.empty()&&*std::max_element(path.v.begin(),path.v.end())<=4.5+1e-4,
        "future acceleration cannot exceed the environmental speed cap");
    planner.SetObjectObservationTime(101.5,101.5);path={};
    const bool capped=planner.Plan(ego,{rear},keep,path);
    check(!capped||(!path.v.empty()&&*std::max_element(path.v.begin(),path.v.end())<=4.5+1e-4&&
        EvaluateCartesianTrajectorySafety(path,{rear},planner.vehicle_shape(),
            planner.collision_config(),planner.sample_dt()).clear),
        "a rear approach cannot authorize exceeding a lower environmental cap");
    planner.SetObjectObservationTime(102,102);path={};keep.target_speed=12;
    auto wall=box(2,110,10,1,0);wall.motion_state=ObjectMotionState::LOW_THREAT;
    const bool stopped=planner.Plan(ego,{wall},keep,path);
    check(!stopped||(!path.v.empty()&&path.v.back()<ego.v&&
        EvaluateCartesianTrajectorySafety(path,{wall},planner.vehicle_shape(),
            planner.collision_config(),planner.sample_dt()).clear),
        "front blockage still produces checked braking or a failure");
    FrenetPlanner departure;
    check(departure.Init((dir/"params.yaml").string()),"stopped-departure planner initializes");
    CartesianState parked=ego;parked.v=parked.a=0;
    departure.SetObjectObservationTime(103,103);path={};
    const auto distant_rear=box(3,90,1.9,4.5,4);
    const bool departs=departure.Plan(parked,{distant_rear},keep,path);
    std::cout<<"[DEPARTURE] valid="<<departs<<" samples="<<path.x.size()
        <<" terminal_speed="<<(path.v.empty()?-1:path.v.back())<<'\n';
    check(departs&&!path.v.empty()&&path.v.back()>2.5,
        "blocked crawl can consider a physically feasible departure without a second observation update");
    if(departs)check(EvaluateCartesianTrajectorySafety(path,{distant_rear},
        departure.vehicle_shape(),departure.collision_config(),departure.sample_dt()).clear,
        "stopped-departure alternative passes the unchanged complete collision check");
    departure.SetObjectObservationTime(104,104);parked.yaw=.2;path={};
    const bool angled_departure=departure.Plan(parked,{distant_rear},keep,path);
    check(!angled_departure||(!path.yaw.empty()&&
        std::fabs(normalizeAngle(path.yaw.front()-parked.yaw))<1e-6),
        "a stopped departure cannot discard the observed vehicle heading");
    check(angled_departure||(path.x.empty()&&departure.last_failure_reason()==
        FrenetPlanner::PlanFailureReason::COLLISION_BLOCKED),
        "an unsupported departure preserves the original collision failure and empty output");
    FrenetPlanner offset;
    check(offset.Init((dir/"params.yaml").string()),"in-lane departure planner initializes");
    CartesianState aligned=ego;aligned.y=.15;aligned.v=aligned.a=aligned.yaw=0;
    offset.SetObjectObservationTime(200,200);path={};
    check(offset.Plan(aligned,{},keep,path)&&!path.y.empty()&&
        std::fabs(path.y.back()-aligned.y)<.02,
        "a stopped in-lane vehicle does not request an unnecessary centering S turn");
    aligned.v=6;offset.SetObjectObservationTime(201,201);path={};
    check(offset.Plan(aligned,{},keep,path)&&!path.y.empty()&&path.y.back()<aligned.y-.01,
        "normal-speed KEEP retains its original centering behavior");
    aligned.y=2;aligned.v=0;offset.SetObjectObservationTime(202,202);path={};
    check(offset.Plan(aligned,{},keep,path)&&!path.y.empty()&&path.y.back()<aligned.y-.01,
        "an exterior crossing the lane boundary is not granted the departure offset hold");
    return failures?1:0;
}
