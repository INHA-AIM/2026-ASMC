#include "Frenet/frenet_planner.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {
int failures=0;
void Expect(bool ok,const char* name){std::cout<<(ok?"[PASS] ":"[FAIL] ")<<name<<'\n';failures+=!ok;}
CartesianState Ego(const YAML::Node& f) {
    const auto e=f["ego"];
    return {e["x"].as<double>(),e["y"].as<double>(),e["yaw"].as<double>(),
        f["ego_curvature"].as<double>(),e["v"].as<double>(),e["a"].as<double>()};
}
std::vector<TrackedObject> Objects(const YAML::Node& f) {
    std::vector<TrackedObject> objects;
    for(const auto& n:f["objects"]) {
        TrackedObject o{};o.id=n["id"].as<int>();o.type=n["type"].as<int>();
        o.motion_state=static_cast<ObjectMotionState>(n["motion_state"].as<int>());
        o.classification=n["classification"].as<std::string>();o.camera_class=n["camera_class"].as<std::string>();
        o.x=n["x"].as<double>();o.y=n["y"].as<double>();o.z=n["z"].as<double>();
        o.heading=n["yaw"].as<double>();o.vx=n["vx"].as<double>();o.vy=n["vy"].as<double>();
        o.width=n["width"].as<double>();o.length=n["length"].as<double>();o.height=n["height"].as<double>();
        o.velocity_unavailable=n["velocity_unavailable"].as<bool>();o.geometry_valid=o.z_valid=true;
        for(int i=0;i<8;++i)o.corners[i]={n["corners"][i][0].as<double>(),n["corners"][i][1].as<double>(),n["corners"][i][2].as<double>()};
        objects.push_back(o);
    }
    return objects;
}
PlannerCommand Command(const YAML::Node& f) {
    PlannerCommand c{};c.mode=AVOID;c.target_object_id=f["command"]["primary"].as<int>();
    c.has_target_speed=true;c.target_speed=f["command"]["speed"].as<double>();return c;
}
bool Prepare(FrenetPlanner& p,const YAML::Node& f,CartesianPath& path,FrenetPlanner::AvoidPrepareResult& r) {
    for(int i=0;i<100;++i) {
        p.SetObjectObservationTime(f["object_stamp"].as<double>(),f["ego"]["stamp"].as<double>());
        p.PrepareAvoid(Ego(f),Objects(f),Command(f),path,r);
        if(r.candidate_valid)return true;
    }
    return false;
}
void SmoothReturn(const std::string& config,const YAML::Node& f) {
    FrenetPlanner p;Expect(p.Init(config),"recorded smooth-return map initializes");p.ConfigureStaticAvoidRecovery(true,true);
    CartesianPath path;FrenetPlanner::AvoidPrepareResult r;const bool ready=Prepare(p,f,path,r);
    Expect(ready&&r.static_progress_mode==StaticAvoidPolicy::NORMAL,"a complete nominal route is available at the recorded moving pose");
    if(!ready)return;
    auto objects=Objects(f);const double age=std::max(0.0,f["ego"]["stamp"].as<double>()-f["object_stamp"].as<double>());
    for(auto& object:objects)object=AdvanceObjectObservation(object,age);
    Expect(CartesianAvoidTrajectoryClear(r.candidate_path,objects,p.vehicle_shape(),p.collision_config(),p.sample_dt()),
        "the smoother return keeps 0.45 m clearance against every recorded hypothesis");
    double arc=0,peak=0;
    for(std::size_t i=1;i<r.spatial_geometry.x.size();++i) {
        arc+=std::hypot(r.spatial_geometry.x[i]-r.spatial_geometry.x[i-1],r.spatial_geometry.y[i]-r.spatial_geometry.y[i-1]);
        if(arc>=10)peak=std::max(peak,std::fabs(r.spatial_geometry.kappa[i]));
    }
    const auto sample=std::min(r.candidate_path.v.size()-1,std::size_t(std::lround(4/p.sample_dt())));
    std::cout<<"RETURN peak="<<peak<<" speed4="<<r.candidate_path.v[sample]<<" length="<<r.static_return_distance<<'\n';
    Expect(peak<.09&&r.candidate_path.v[sample]>3.5,
        "a checked gentle return wins over an unnecessary tight curve and crawl");
}
void PrefixAdoption(const std::string& config,const YAML::Node& sequence) {
    FrenetPlanner p;Expect(p.Init(config),"recorded prefix adoption initializes");p.ConfigureStaticAvoidRecovery(true,true);
    bool saw_prefix=false,saw_normal=false,consistent=true;
    for(const auto& f:sequence) {
        p.SetObjectObservationTime(f["object_stamp"].as<double>(),f["ego"]["stamp"].as<double>());
        CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
        if(p.avoid_committed())p.PlanCommittedAvoid(Ego(f),Objects(f),Command(f),path,r);
        else {
            p.PrepareAvoid(Ego(f),Objects(f),Command(f),path,r);
            if(r.candidate_valid&&f["command"]["request"].as<int>()==1)p.CommitAvoid(r);
        }
        if(!p.avoid_committed()||!r.candidate_valid)continue;
        if(r.static_progress_fresh) {
            saw_prefix=true;
            consistent&=std::string(p.avoid_phase_name())!="RETURN"&&!p.avoid_completed()&&r.maneuver_progress<.75;
        } else if(r.static_progress_mode==StaticAvoidPolicy::NORMAL) {
            saw_normal=true;
            consistent&=std::fabs(p.avoid_locked_target_d()-r.selected_target_d)<1e-4&&
                p.avoid_locked_target_d()< -1&&std::string(p.avoid_phase_name())!="RETURN";
        }
    }
    Expect(saw_prefix&&saw_normal&&consistent,
        "a rolling prefix never reports RETURN and its complete bypass atomically updates the locked target and entry");
}
void StableReference(const std::string& config,const YAML::Node& f) {
    FrenetPlanner p;Expect(p.Init(config),"recorded stable-reference map initializes");p.ConfigureStaticAvoidRecovery(true,true);
    CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
    if(!Prepare(p,f,path,r)||!p.CommitAvoid(r)){Expect(false,"checked reference commits");return;}
    auto ego=Ego(f);ego.x+=.12;ego.yaw+=.025;
    const auto objects=Objects(f);CartesianPath first;bool stable=true;
    for(int i=0;i<5;++i) {
        p.SetObjectObservationTime(f["object_stamp"].as<double>(),f["ego"]["stamp"].as<double>());
        if(!p.PlanCommittedAvoid(ego,objects,Command(f),path,r)||!r.retained_geometry){stable=false;break;}
        if(!i)first=r.spatial_geometry;
        else {
            stable&=first.x.size()==r.spatial_geometry.x.size();
            for(std::size_t j=0;stable&&j<first.x.size();++j)
                stable&=std::hypot(first.x[j]-r.spatial_geometry.x[j],first.y[j]-r.spatial_geometry.y[j])<1e-8;
        }
    }
    Expect(stable,"repeated measured-pose reconnections do not recursively deform the accepted spatial reference");
}
}
int main() {
    const auto root=std::filesystem::absolute("static_avoid_smoothness_fixture");std::filesystem::create_directories(root);
    auto cfg=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/frenet/params.yaml");
    cfg["waypoint_file"]=std::string(FRENET_PACKAGE_SRC_DIR)+"/../../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt";
    cfg["avoid_hd_map"]["directory"]=std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/frenet/hd_map";
    cfg["planner"]["avoid"]["map_search_budget_s"]=.035;
    const auto config=(root/"params.yaml").string();{std::ofstream out(config);out<<cfg;}
    const auto fixture=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../tests/fixtures/static_avoid_smoothness_bag.json");
    SmoothReturn(config,fixture["return_snapshot"]);PrefixAdoption(config,fixture["approach_sequence"]);
    StableReference(config,fixture["normal_approach"]);
    std::filesystem::remove_all(root);return failures?1:0;
}
