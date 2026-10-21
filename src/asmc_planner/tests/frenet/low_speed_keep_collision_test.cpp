#include "Frenet/frenet_planner.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
    int failures=0;
    auto check=[&](bool ok,const char* name) {
        std::cout<<(ok?"[PASS] ":"[FAIL] ")<<name<<'\n'; failures+=!ok;
    };
    const std::filesystem::path root=ASMC_LOW_KEEP_SOURCE;
    auto cfg=YAML::LoadFile((root/"config/frenet/params.yaml").string());
    cfg["waypoint_file"]=(root/"../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt").lexically_normal().string();
    cfg["avoid_hd_map"]["directory"]=(root/"config/hd_map").string();
    cfg["planner"]["visualization"]["enabled"]=false;
    const auto config=std::filesystem::temp_directory_path()/"asmc_low_keep_collision.yaml";
    {std::ofstream out(config);out<<cfg;}
    const auto frame=YAML::LoadFile((root/"tests/fixtures/low_speed_merge_conflict.json").string());
    const auto e=frame["ego"];
    CartesianState ego{e["x"].as<double>(),e["y"].as<double>(),e["yaw"].as<double>(),
        0,e["speed"].as<double>(),e["acceleration"].as<double>()};
    std::vector<TrackedObject> objects;
    for(const auto& item:frame["objects"]) {
        TrackedObject object;
        object.id=item["id"].as<int>();object.type=item["type"].as<int>();
        object.motion_state=static_cast<ObjectMotionState>(item["motion_state"].as<int>());
        object.x=item["x"].as<double>();object.y=item["y"].as<double>();object.z=item["z"].as<double>();
        object.heading=item["yaw"].as<double>();object.vx=item["vx"].as<double>();object.vy=item["vy"].as<double>();
        object.speed=std::hypot(object.vx,object.vy);object.width=item["width"].as<double>();
        object.length=item["length"].as<double>();object.height=item["height"].as<double>();
        for(int i=0;i<8;++i)object.corners[i]={item["corners"][i][0].as<double>(),
            item["corners"][i][1].as<double>(),item["corners"][i][2].as<double>()};
        object.geometry_valid=geometry::ValidateCorners8(object.corners).valid;
        object.z_valid=object.geometry_valid;
        objects.push_back(object);
    }
    FrenetPlanner planner;check(planner.Init(config.string()),"production planner initializes");
    const double object_stamp=frame["object_stamp"].as<double>(),ego_stamp=frame["ego_stamp"].as<double>();
    auto aligned=objects;
    for(auto& object:aligned)object=AdvanceObjectObservation(object,std::max(0.0,ego_stamp-object_stamp));
    CartesianPath recorded;
    for(const auto& p:frame["path"]) {
        recorded.x.push_back(p["x"].as<double>());recorded.y.push_back(p["y"].as<double>());
        recorded.yaw.push_back(p["yaw"].as<double>());recorded.v.push_back(p["velocity"].as<double>());
        recorded.a.push_back(p["acceleration"].as<double>());recorded.kappa.push_back(p["curvature"].as<double>());
    }
    check(!EvaluateCartesianTrajectorySafety(recorded,aligned,planner.vehicle_shape(),
        planner.collision_config(),planner.sample_dt()).clear,
        "recorded low-speed path conflicts with final observed objects");
    PlannerCommand command;command.mode=LANE_KEEPING;command.has_target_speed=true;command.target_speed=57.5/3.6;
    planner.SetObjectObservationTime(object_stamp,ego_stamp);
    CartesianPath output;
    const bool ok=planner.Plan(ego,objects,command,output);
    check(!ok,"production low-speed KEEP rejects the recorded unsafe crawl");
    check(!ok&&output.x.empty()&&planner.last_failure_reason()==FrenetPlanner::PlanFailureReason::COLLISION_BLOCKED,
        "blocked crawl clears output and reports collision, preventing transient cache reuse");

    planner.SetObjectObservationTime(ego_stamp+1,ego_stamp+1);
    output={};
    check(planner.Plan(ego,{},command,output)&&output.x.size()>1,
        "clear low-speed road still permits departure");
    // Synthetic authoritative corners: put a physical box on that departure.
    TrackedObject barrier;barrier.id=900;barrier.x=output.x[1];barrier.y=output.y[1];
    barrier.width=barrier.length=barrier.height=1;barrier.geometry_valid=barrier.z_valid=true;
    barrier.motion_state=ObjectMotionState::UNKNOWN;
    for(int i=0;i<8;++i) {
        const double xs[4]={-.5,.5,.5,-.5},ys[4]={-.5,-.5,.5,.5};
        barrier.corners[i]={barrier.x+xs[i%4],barrier.y+ys[i%4],double(i/4)};
    }
    output={};
    check(!planner.Plan(ego,{barrier},command,output),
        "unknown stationary box also blocks low-speed departure");
    for(double speed:{.49,.50,.51,1.0,1.49}) {
        ego.v=speed;ego.a=-2.4;output={};
        check(!planner.Plan(ego,{barrier},command,output),
            "the low/near-low-speed boundary never bypasses a physical barrier");
    }
    return failures?1:0;
}
