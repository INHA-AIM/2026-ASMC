#include "Frenet/frenet_planner.hpp"
#include "Frenet/Map/AvoidRoadMap.hpp"
#include "Frenet/Math/FrenetConverter.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cmath>

int main() {
    int failures=0;
    auto expect=[&](bool ok,const char* name){
        std::cout<<(ok?"[PASS] ":"[FAIL] ")<<name<<'\n'; failures+=!ok;
    };
    const std::string source=FRENET_PACKAGE_SRC_DIR;
    const auto original=YAML::LoadFile(source+"/../config/frenet/params.yaml");
    const auto dir=std::filesystem::absolute("stop_road_braking_fixture");
    std::filesystem::create_directories(dir);
    const auto map_dir=std::filesystem::absolute(source+"/../config/hd_map");
    auto config=YAML::Clone(original);
    config["waypoint_file"]=std::string(FRENET_TEST_WAYPOINT_FILE);
    config["avoid_hd_map"]["directory"]=map_dir.string();
    config["roundabout_merge"]["enabled"]=false;
    config["planner"]["visualization"]["enabled"]=false;
    const auto config_path=dir/"params.yaml";
    {std::ofstream file(config_path);file<<config;}
    RefLine ref;
    expect(LoadReferenceLine(FRENET_TEST_WAYPOINT_FILE,ref,{.05,10,.5,300,.3,30},.23),"real reference loads");
    AvoidRoadMap road; std::string error;
    expect(road.Load((map_dir/"drivable.json").string(),(map_dir/"lines.json").string(),error),"real whole-body road boundary loads");
    const VehicleShape shape{1.9,4.635,.79};
    // Regression poses from curved STOP: temporal centering bulged out of the
    // corridor. Distance-based alternatives must keep the same stop target.
    const CartesianState poses[] = {
        {-62.367,-183.833,1.376,.00135676,10.686,-1.691},
        {-61.818,-181.383,1.376,-.000644776,10.157,-1.896}
    };
    for (const auto& ego : poses) {
        FrenetPlanner planner; expect(planner.Init(config_path.string()),"curved STOP planner initializes");
        double s=0;
        expect(planner.ProjectEgoToS(ego,s),"curved STOP ego projects");
        PlannerCommand command{};command.mode=STOP;command.has_target_speed=true;command.target_speed=13.8888888889;
        expect(planner.ProjectPointToS(-60.0713284274,-160.3874670883,s,command.stop_position),"STOP target projects");
        CartesianPath path;
        const bool ok=planner.Plan(ego,{},command,path);
        expect(ok&&!path.x.empty(),"curved STOP retains a feasible candidate");
        expect(ok&&road.EvaluateRecoveryPath(path,shape,.1).drivable,"selected braking path keeps full body inside the road");
        double end=s;
        if(ok) end=FindClosestS(ref,path.x.back(),path.y.back(),&s,100);
        const double stop_buffer=config["planner"]["stop"]["stop_buffer"].as<double>();
        expect(ok&&std::fabs(end-(command.stop_position-stop_buffer))<.1,
               "spatial STOP reaches the configured buffered endpoint");
        bool limits_ok=ok;
        for(std::size_t i=0;i<path.v.size();++i)
            limits_ok &= std::isfinite(path.v[i]) && path.v[i]>=-1e-6 &&
                std::isfinite(path.a[i]) && std::fabs(path.a[i])<=5.01 &&
                std::fabs(path.kappa[i])<=.23+1e-6;
        expect(limits_ok&&path.v.back()<1e-6,"braking remains finite forward-only and ends stopped");
        expect(!planner.stop_holding(),"planned zero terminal speed does not latch measured HOLD early");
        // Consume one next sample and replan, rather than checking only the
        // first complete path. This is ideal tracking, not a MORAI/MPC model.
        CartesianState moving=ego;
        bool reached=false,continuous=true;
        for(int tick=0;tick<150;++tick) {
            if(!planner.Plan(moving,{},command,path) || path.x.empty()) {
                continuous=false;break;
            }
            continuous &= road.EvaluateRecoveryPath(path,shape,.1).drivable;
            if(planner.stop_holding()) {reached=true;break;}
            const std::size_t next=std::min(std::size_t(1),path.x.size()-1);
            moving={path.x[next],path.y[next],path.yaw[next],path.kappa[next],
                    path.v[next],path.a[next]};
        }
        double measured_s=s;
        if(reached)measured_s=FindClosestS(ref,moving.x,moving.y,&s,100);
        expect(continuous&&reached,"curved STOP repeated replanning reaches measured HOLD safely");
        expect(reached&&measured_s<=command.stop_position+.02 &&
               measured_s>=command.stop_position-stop_buffer-.3,
               "ideal curved tracking stops before the fixed external target");
    }
    // A valid old KEEP path must not be reused when STOP is unsafe.
    FrenetPlanner blocked;expect(blocked.Init(config_path.string()),"blocked STOP planner initializes");
    const auto rp=Interpolate(ref,150);
    CartesianState ego{rp.x,rp.y,rp.theta,rp.kappa,4,0};
    PlannerCommand keep{};keep.mode=LANE_KEEPING;
    CartesianPath previous;
    expect(blocked.Plan(ego,{},keep,previous),"valid KEEP precedes blocked STOP");
    TrackedObject object{};object.id=1;object.x=ego.x;object.y=ego.y;
    object.width=6;object.length=6;object.height=2;object.geometry_valid=object.z_valid=true;
    object.motion_state=ObjectMotionState::LOW_THREAT;
    for(int i=0;i<8;++i){const double xs[4]={-3,3,3,-3},ys[4]={-3,-3,3,3};object.corners[i]={ego.x+xs[i%4],ego.y+ys[i%4],double(i/4)*2};}
    PlannerCommand stop{};stop.mode=STOP;stop.stop_position=155;
    CartesianPath path=previous;
    expect(!blocked.Plan(ego,{object},stop,path),"occupied current footprint cannot be bypassed by STOP fallback");
    expect(path.x.empty()&&blocked.last_failure_reason()==FrenetPlanner::PlanFailureReason::COLLISION_BLOCKED,"unsafe STOP clears old output and explicitly blocks reuse");
    // A distant stopped ego must resume toward the target rather than falsely
    // declaring HOLD simply because a fallback ended at zero speed.
    ego.v=0;ego.a=0;
    FrenetPlanner approach;expect(approach.Init(config_path.string()),"stopped approach planner initializes");
    stop.stop_position=170;
    const bool advancing=approach.Plan(ego,{},stop,path);
    double vmax=0;for(double v:path.v)vmax=std::max(vmax,v);
    expect(advancing&&vmax>.1&&!approach.stop_holding(),"early stopped ego can safely continue to its original target");

    FrenetPlanner decelerating;expect(decelerating.Init(config_path.string()),"low-speed braking planner initializes");
    ego.v=.30;ego.a=-5.45;stop.stop_position=153;
    const bool braking=decelerating.Plan(ego,{},stop,path);
    expect(braking&&!path.x.empty()&&path.v.back()<1e-6,
           "strong measured deceleration at low speed does not demand reverse motion");
    bool forward=braking;for(double v:path.v)forward &= v>=-1e-6;
    expect(forward&&!decelerating.stop_holding(),"bounded brake boundary remains forward and preserves the original target lifecycle");

    FrenetPlanner strong;expect(strong.Init(config_path.string()),"service braking planner initializes");
    ego.v=6;ego.a=-3.5;stop.stop_position=157;
    expect(strong.Plan(ego,{},stop,path)&&!path.a.empty()&&path.a.front()< -3.3,
           "STOP planning preserves measured service braking beyond the KEEP boundary cap");
    ego.v=.30;ego.a=-5.45;

    FrenetPlanner approaching_object;expect(approaching_object.Init(config_path.string()),"future collision planner initializes");
    object.x=ego.x+5*std::cos(ego.yaw);object.y=ego.y+5*std::sin(ego.yaw);
    object.width=2;object.length=2;object.motion_state=ObjectMotionState::DYNAMIC;
    object.vx=-2*std::cos(ego.yaw);object.vy=-2*std::sin(ego.yaw);object.speed=2;
    for(int i=0;i<8;++i){const double xs[4]={-1,1,1,-1},ys[4]={-1,-1,1,1};
        object.corners[i]={object.x+xs[i%4]*std::cos(ego.yaw)-ys[i%4]*std::sin(ego.yaw),
            object.y+xs[i%4]*std::sin(ego.yaw)+ys[i%4]*std::cos(ego.yaw),double(i/4)*2};}
    stop.stop_position=160;
    expect(!approaching_object.Plan(ego,{object},stop,path)&&path.x.empty(),
           "stopped footprint is checked against an approaching object after braking ends");
    return failures?1:0;
}
