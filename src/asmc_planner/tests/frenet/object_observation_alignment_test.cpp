#include "Frenet/frenet_planner.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
    int failures=0;
    auto check=[&](bool pass,const char* name) {
        std::cout<<(pass?"[PASS] ":"[FAIL] ")<<name<<'\n'; failures+=!pass;
    };
    const std::filesystem::path root=ASMC_ALIGNMENT_SOURCE;
    const auto fixture=YAML::LoadFile((root/"tests/fixtures/merge_observation_latency.json").string());
    const auto f=fixture["object"];
    TrackedObject object;
    object.id=f["id"].as<int>();object.type=f["type"].as<int>();
    object.motion_state=static_cast<ObjectMotionState>(f["motion_state"].as<int>());
    object.x=f["center"][0].as<double>();object.y=f["center"][1].as<double>();
    object.heading=f["yaw"].as<double>();object.yaw_rate=f["yaw_rate"].as<double>();
    object.vx=f["velocity"][0].as<double>();object.vy=f["velocity"][1].as<double>();
    object.width=f["width"].as<double>();object.length=f["length"].as<double>();
    for(int i=0;i<8;++i) object.corners[i]={f["corners"][i][0].as<double>(),
        f["corners"][i][1].as<double>(),f["corners"][i][2].as<double>()};
    object.geometry_valid=geometry::ValidateCorners8(object.corners).valid;
    CartesianPath path;
    for(auto p:fixture["path"]) {
        path.x.push_back(p["x"].as<double>());path.y.push_back(p["y"].as<double>());
        path.yaw.push_back(p["yaw"].as<double>());path.v.push_back(p["v"].as<double>());
        path.a.push_back(0);path.kappa.push_back(0);
    }
    const double age=fixture["observation_age_s"].as<double>();
    const VehicleShape shape{1.9,4.635,.79};
    const CollisionCheckConfig collision{.45,.1,0};
    check(EvaluateCartesianTrajectorySafety(path,{object},shape,collision,.1).clear,
        "captured collision reproduces false clearance with mixed source times");
    const auto aligned=AdvanceObjectObservation(object,age);
    const auto result=EvaluateCartesianTrajectorySafety(path,{aligned},shape,collision,.1);
    check(!result.clear&&result.blocking_object_id==object.id,
        "same recorded path rejects the approaching rear vehicle after source-time alignment");
    check(aligned.corners[0].z==object.corners[0].z &&
        std::fabs(aligned.corners[0].y-object.corners[0].y-object.vy*age)<1e-8,
        "alignment translates authoritative corners without geometry reconstruction");
    check(object.y==f["center"][1].as<double>(),"alignment does not overwrite the sensor observation");
    auto turning=object;turning.vx=4;turning.vy=1;turning.yaw_rate=.4;
    turning.turn_prediction_duration=.3;turning.continuation_vx=3;turning.continuation_vy=2;
    for(double elapsed:{0.,.1,.3,.5}) {
        const auto advanced=AdvanceObjectObservation(turning,elapsed);
        const auto split=PredictTrackedObjectPose(advanced,.6);
        const auto direct=PredictTrackedObjectPose(turning,elapsed+.6);
        check(std::hypot(split.x-direct.x,split.y-direct.y)<1e-8,
            "advancing a bounded turn once preserves the shared future motion model");
    }
    auto stationary=object;stationary.motion_state=ObjectMotionState::LOW_THREAT;
    const auto held=AdvanceObjectObservation(stationary,age);
    check(held.x==stationary.x&&held.y==stationary.y&&held.vx==0&&held.vy==0,
        "confirmed stationary observation remains stationary despite noisy velocity");
    // Exercise production core wiring, not just the pure geometry helper.
    auto cfg=YAML::LoadFile((root/"config/frenet/params.yaml").string());
    cfg["waypoint_file"]=(root/"../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt").lexically_normal().string();
    cfg["avoid_hd_map"]["directory"]=(root/"config/hd_map").string();
    cfg["planner"]["visualization"]["enabled"]=false;
    const auto file=std::filesystem::temp_directory_path()/"asmc_observation_alignment_test.yaml";
    {std::ofstream out(file);out<<cfg;}
    FrenetPlanner planner;check(planner.Init(file.string()),"production planner initializes");
    planner.SetObjectObservationTime(fixture["object_stamp"].as<double>(),fixture["ego_stamp"].as<double>());
    CartesianState ego{path.x[0],path.y[0],path.yaw[0],0,path.v[0],0};
    PlannerCommand command;command.mode=LANE_KEEPING;command.has_target_speed=true;command.target_speed=57.5/3.6;
    CartesianPath output;
    const bool ok=planner.Plan(ego,{object},command,output);
    check(!ok||EvaluateCartesianTrajectorySafety(output,{aligned},shape,collision,.1).clear,
        "production KEEP cannot return the recorded unsafe acceleration path");
    return failures?1:0;
}
