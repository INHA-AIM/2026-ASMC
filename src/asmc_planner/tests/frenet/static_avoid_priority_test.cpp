#include "Frenet/frenet_planner.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {
int failures=0;
void Expect(bool ok,const char* name){std::cout<<(ok?"[PASS] ":"[FAIL] ")<<name<<'\n';failures+=!ok;}
TrackedObject Box(int id,double x,double y,double width,double length){
    TrackedObject o{};o.id=id;o.type=0;o.motion_state=ObjectMotionState::LOW_THREAT;
    o.classification="static_obstacle";o.x=x;o.y=y;o.width=width;o.length=length;o.height=1;
    o.geometry_valid=o.z_valid=true;
    const double dx[]={-.5,.5,.5,-.5},dy[]={-.5,-.5,.5,.5};
    for(int i=0;i<8;++i)o.corners[i]={x+length*dx[i%4],y+width*dy[i%4],double(i/4)};
    return o;
}
PlannerCommand Command(){PlannerCommand c{};c.mode=AVOID;c.target_object_id=2;c.has_target_speed=true;c.target_speed=4.2;return c;}
bool Prepare(FrenetPlanner& p,const CartesianState& ego,const std::vector<TrackedObject>& objects,
    CartesianPath& path,FrenetPlanner::AvoidPrepareResult& r,bool& nominal_pending){
    const auto c=Command();
    for(int i=0;i<180;++i){
        p.SetObjectObservationTime(1+i*.05,1+i*.05);p.PrepareAvoid(ego,objects,c,path,r);
        nominal_pending|=r.search_pending&&r.static_search_pending_level==0;
        if(r.candidate_valid)return true;
    }
    std::cout<<"unfinished level="<<r.static_search_pending_level<<" stage="<<r.rejection_stage<<'\n';return false;
}
std::vector<TrackedObject> Scene(bool wide){
    return {Box(2,80,-1.034,2.552,.22),Box(5,84,-1.653,2.61,1.8),
        Box(3,80,wide?4.484:3.684,1.93,.32),Box(4,84,wide?4.826:4.026,2.51,1.8)};
}
std::string Config(YAML::Node cfg,const std::filesystem::path& root,const char* name){
    const auto path=root/name;std::ofstream f(path);f<<cfg;return path.string();
}
void Synthetic(const std::string& config){
    for(bool wide:{true,false}) {
        FrenetPlanner p;Expect(p.Init(config),"priority map initializes");p.ConfigureStaticAvoidRecovery(true,true);
        CartesianState ego{};ego.x=50;auto objects=Scene(wide);CartesianPath path;FrenetPlanner::AvoidPrepareResult r;bool pending=false;
        const bool ready=Prepare(p,ego,objects,path,r,pending);
        Expect(ready&&r.static_progress_mode==StaticAvoidPolicy::NORMAL,"nominal clearance wins before margin/contact recovery");
        if(!ready)continue;
        Expect(CartesianAvoidTrajectoryClear(r.candidate_path,objects,p.vehicle_shape(),p.collision_config(),.1),
            "every object, including the opposite side, keeps 0.45 m clearance");
        if(wide)Expect(r.static_corridor_kind=="center"&&r.selected_target_d>0&&r.selected_target_d<2.2,
            "a wide center interval wins over the newly allowed outside right detour");
        else Expect(r.selected_target_d<-4,"a narrow center is rejected and the widened right corridor is selected");
        Expect(p.CommitAvoid(r),"priority-selected geometry commits");
        const double target=r.selected_target_d;
        bool retained=true;
        for(int i=0;i<8;++i){p.SetObjectObservationTime(20+i*.05,20+i*.05);retained&=p.PlanCommittedAvoid(ego,objects,Command(),path,r)&&r.retained_geometry&&std::fabs(r.selected_target_d-target)<1e-6;}
        Expect(retained,"a still-valid incumbent is rechecked and retained instead of opportunistic replanning");
    }
}
void Recorded(YAML::Node cfg,const std::filesystem::path& root){
    cfg["waypoint_file"]=std::string(FRENET_PACKAGE_SRC_DIR)+"/../../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt";
    cfg["avoid_hd_map"]["directory"]=std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/frenet/hd_map";
    const auto config=Config(cfg,root,"recorded.yaml");
    const auto f=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../tests/fixtures/static_corridor_priority_bag.json");
    const auto e=f["ego"];CartesianState ego{e["x"].as<double>(),e["y"].as<double>(),e["yaw"].as<double>(),e["kappa"].as<double>(),e["v"].as<double>(),e["a"].as<double>()};
    std::vector<TrackedObject> objects;
    for(const auto& n:f["objects"]){
        auto o=Box(n["id"].as<int>(),n["x"].as<double>(),n["y"].as<double>(),n["width"].as<double>(),n["length"].as<double>());
        o.z=n["z"].as<double>();o.heading=n["heading"].as<double>();o.height=n["height"].as<double>();o.vx=n["vx"].as<double>();o.vy=n["vy"].as<double>();
        o.type=n["type"].as<int>();o.motion_state=static_cast<ObjectMotionState>(n["motion_state"].as<int>());o.classification=n["classification"].as<std::string>();o.camera_class=n["camera_class"].as<std::string>();o.velocity_unavailable=n["velocity_unavailable"].as<bool>();
        for(int i=0;i<8;++i)o.corners[i]={n["corners"][i][0].as<double>(),n["corners"][i][1].as<double>(),n["corners"][i][2].as<double>()};objects.push_back(o);
    }
    FrenetPlanner p;Expect(p.Init(config),"recorded priority map initializes");p.ConfigureStaticAvoidRecovery(true,true);
    CartesianPath path;FrenetPlanner::AvoidPrepareResult r;bool pending=false;
    const bool ready=Prepare(p,ego,objects,path,r,pending);
    std::cout<<"RECORDED PRIORITY ready="<<ready<<" policy="<<int(r.static_progress_mode)<<" target="<<r.selected_target_d<<" kind="<<r.static_corridor_kind<<'\n';
    Expect(ready&&r.static_progress_mode==StaticAvoidPolicy::NORMAL&&r.selected_target_d<-3,
        "recorded narrow middle waits for a complete normal right route instead of accepting reduced/contact first");
    if(ready)Expect(CartesianAvoidTrajectoryClear(r.candidate_path,objects,p.vehicle_shape(),p.collision_config(),.1),
        "recorded right candidate passes nominal timed checks against all recorded objects");
    cfg["planner"]["avoid"]["map_search_budget_s"]=1e-9;
    FrenetPlanner tiny;Expect(tiny.Init(Config(cfg,root,"tiny.yaml")),"tiny-budget map initializes");tiny.ConfigureStaticAvoidRecovery(true,true);
    bool waiting=true;
    for(int i=0;i<5;++i){tiny.PrepareAvoid(ego,objects,Command(),path,r);waiting&=!r.candidate_valid&&r.search_pending&&r.static_search_pending_level==0;}
    Expect(waiting,"budget exhaustion remains nominal search pending and never authorizes a lower clearance tier");
}
void RecordedMovingApproach(YAML::Node cfg,const std::filesystem::path& root){
    cfg["waypoint_file"]=std::string(FRENET_PACKAGE_SRC_DIR)+"/../../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt";
    cfg["avoid_hd_map"]["directory"]=std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/frenet/hd_map";
    cfg["planner"]["avoid"]["map_search_budget_s"]=.035;
    const auto config=Config(cfg,root,"recorded-moving.yaml");
    const auto fixture=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../tests/fixtures/static_side_reselection_bag.json");
    for(const auto& sample:fixture["snapshots"]){
        const auto e=sample["ego"];
        CartesianState ego{e["x"].as<double>(),e["y"].as<double>(),e["yaw"].as<double>(),
            e["kappa"].as<double>(),e["v"].as<double>(),e["a"].as<double>()};
        std::vector<TrackedObject> objects;
        for(const auto& n:sample["objects"]){
            auto o=Box(n["id"].as<int>(),n["x"].as<double>(),n["y"].as<double>(),n["width"].as<double>(),n["length"].as<double>());
            o.z=n["z"].as<double>();o.heading=n["heading"].as<double>();o.height=n["height"].as<double>();
            o.vx=n["vx"].as<double>();o.vy=n["vy"].as<double>();o.type=n["type"].as<int>();
            o.motion_state=static_cast<ObjectMotionState>(n["motion_state"].as<int>());
            o.classification=n["classification"].as<std::string>();o.camera_class=n["camera_class"].as<std::string>();
            o.velocity_unavailable=n["velocity_unavailable"].as<bool>();
            for(int i=0;i<8;++i)o.corners[i]={n["corners"][i][0].as<double>(),n["corners"][i][1].as<double>(),n["corners"][i][2].as<double>()};
            objects.push_back(o);
        }
        FrenetPlanner p;Expect(p.Init(config),"recorded moving approach initializes");p.ConfigureStaticAvoidRecovery(true,true);
        auto command=Command();command.target_object_id=sample["primary_object_id"].as<int>();command.target_speed=sample["target_speed"].as<double>();
        CartesianPath path;FrenetPlanner::AvoidPrepareResult result;
        for(int cycle=0;cycle<80;++cycle){
            p.SetObjectObservationTime(sample["observation_time"].as<double>(),sample["evaluation_time"].as<double>());
            p.PrepareAvoid(ego,objects,command,path,result);if(result.candidate_valid)break;
        }
        Expect(ego.v>1&&result.candidate_valid&&result.selected_target_d< -1&&
               result.static_progress_mode==StaticAvoidPolicy::NORMAL,
               "recorded moving pose has a complete normal right route without waiting for standstill");
        // Match the trajectory's ego time. Raw sensor corners precede it by
        // 14 ms in the first sample; keep all motion hypotheses after aligning.
        auto aligned=objects;
        const double age=std::max(0.0,sample["evaluation_time"].as<double>()-sample["observation_time"].as<double>());
        for(auto& object:aligned)object=AdvanceObjectObservation(object,age);
        Expect(result.candidate_valid&&CartesianAvoidTrajectoryClear(result.candidate_path,aligned,p.vehicle_shape(),p.collision_config(),.1),
               "recorded right route retains 0.45 m clearance and every uncertain-object hypothesis");
        const double target=result.selected_target_d;
        bool retained=result.candidate_valid&&p.CommitAvoid(result);
        for(int cycle=0;cycle<3&&retained;++cycle){
            retained=p.PlanCommittedAvoid(ego,objects,command,path,result)&&result.candidate_valid&&
                std::fabs(result.selected_target_d-target)<1e-6;
        }
        Expect(retained,"a checked recorded right route remains selected under repeated moving-pose updates");
    }
}
void RetainedRecovery(const std::string& config){
    FrenetPlanner p;Expect(p.Init(config),"narrow-road priority map initializes");p.ConfigureStaticAvoidRecovery(true,true);
    CartesianState ego{};ego.x=50;auto objects=Scene(false);CartesianPath path;FrenetPlanner::AvoidPrepareResult r;bool pending=false;
    const bool ready=Prepare(p,ego,objects,path,r,pending);
    Expect(ready&&r.static_progress_mode==StaticAvoidPolicy::REDUCED_MARGIN&&r.static_progress_margin<=.2,
        "a narrow road exhausts normal and 0.30 m alternatives before selecting a complete 0.20 m corridor");
    if(!ready)return;
    Expect(p.CommitAvoid(r),"body-clear recovery commits after higher-tier search");
    bool valid=true,background=false;
    for(int i=0;i<12;++i){p.SetObjectObservationTime(30+i*.05,30+i*.05);valid&=p.PlanCommittedAvoid(ego,objects,Command(),path,r);background|=r.static_search_pending_level>=0;}
    Expect(valid&&background,"a freshly rechecked recovery incumbent keeps executing while normal alternatives are searched");
}
void ContactOptOut(const std::string& config){
    FrenetPlanner p;Expect(p.Init(config),"contact opt-out map initializes");p.ConfigureStaticAvoidRecovery(true,true);
    CartesianState ego{};ego.x=50;const auto objects=std::vector<TrackedObject>{Box(2,58,0,20,1)};
    CartesianPath path;FrenetPlanner::AvoidPrepareResult r;bool pending=false;
    const bool ready=Prepare(p,ego,objects,path,r,pending);
    Expect(ready&&r.static_progress_mode==StaticAvoidPolicy::CONTACT_PROGRESS&&p.CommitAvoid(r),
        "contact requires exhausted body-clear tiers and explicit opt-in");
    if(!ready)return;
    p.ConfigureStaticAvoidRecovery(true,false);bool blocked=true;
    for(int i=0;i<5;++i)blocked&=!p.PlanCommittedAvoid(ego,objects,Command(),path,r);
    Expect(blocked,"revoking contact permission also forbids retention of a previously valid contact prefix");
}
void TightNominalCenter(const std::string& config){
    FrenetPlanner p;Expect(p.Init(config),"tight nominal-center map initializes");p.ConfigureStaticAvoidRecovery(true,true);
    CartesianState ego{};ego.x=50;auto objects=Scene(false);
    for(auto& o:objects)if(o.id==3||o.id==4){o.y+=.325;for(auto& c:o.corners)c.y+=.325;}
    CartesianPath path;FrenetPlanner::AvoidPrepareResult r;bool pending=false;
    const bool ready=Prepare(p,ego,objects,path,r,pending);
    Expect(ready&&r.static_progress_mode==StaticAvoidPolicy::NORMAL&&r.static_corridor_kind=="center"&&
        CartesianAvoidTrajectoryClear(r.candidate_path,objects,p.vehicle_shape(),p.collision_config(),.1),
        "a center with just over 0.45 m on both sides is not rejected by an extra hidden interval-width margin");
}
void CenterOpensDuringSearch(YAML::Node cfg,const std::filesystem::path& root){
    cfg["planner"]["avoid"]["map_search_budget_s"]=.002;
    FrenetPlanner p;Expect(p.Init(Config(cfg,root,"opening.yaml")),"changing-center map initializes");p.ConfigureStaticAvoidRecovery(true,true);
    CartesianState ego{};ego.x=50;auto objects=Scene(false);CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
    bool lower_pending=false;
    for(int i=0;i<180;++i){p.SetObjectObservationTime(50+i*.05,50+i*.05);p.PrepareAvoid(ego,objects,Command(),path,r);if(!r.candidate_valid&&r.static_search_pending_level>0){lower_pending=true;break;}}
    Expect(lower_pending,"the narrow scene reaches a pending lower-tier sweep after nominal alternatives are exhausted");
    if(!lower_pending)return;
    objects=Scene(true);bool normal=false;
    for(int i=0;i<8;++i){p.SetObjectObservationTime(70+i*.05,70+i*.05);p.PrepareAvoid(ego,objects,Command(),path,r);if(r.candidate_valid){normal=r.static_progress_mode==StaticAvoidPolicy::NORMAL&&r.static_corridor_kind=="center";break;}}
    Expect(normal,"a newly opened nominal center preempts a pending margin sweep without restarting every outside combination");
}
}
int main(){
    const auto root=std::filesystem::absolute("static_avoid_priority_fixture");std::filesystem::create_directories(root);
    auto cfg=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/frenet/params.yaml");Recorded(YAML::Clone(cfg),root);RecordedMovingApproach(YAML::Clone(cfg),root);
    {std::ofstream f(root/"waypoints.txt");for(int x=0;x<=260;++x)f<<x<<" 0\n";}
    {std::ofstream f(root/"drivable.json");f<<R"({"frame_id":"map","map_id":"waize_katri_2025-10-23","grid":{"origin_x":0,"origin_y":-8},"drivable_areas":[{"validity":"validated","polygon":[[0,-6],[260,-6],[260,6],[0,6]]}]})";}
    {std::ofstream f(root/"lines.json");f<<R"({"map_id":"kcity2025_competition_route_curation_candidate_v14","frame":{"local_origin_in_global":[302595,4124145]},"features":[{"feature_type":"lane_marking","attributes":{"pattern":"solid","functional_role":"lane_boundary_paint"},"geometry":{"coordinates":[[0,7],[260,7]]}}]})";}
    {std::ofstream f(root/"drivable_grid.rle");f<<"3085 7232 0.2 0 -8\n";for(int row=10;row<70;++row)f<<row<<" 0 1300\n";}
    cfg["waypoint_file"]=(root/"waypoints.txt").string();cfg["avoid_hd_map"]["directory"]=root.string();cfg["roundabout_merge"]["enabled"]=false;
    const auto synthetic=Config(cfg,root,"synthetic.yaml");Synthetic(synthetic);ContactOptOut(synthetic);TightNominalCenter(synthetic);
    // Tighten only the drivable raster, leaving the same recorded-style gap.
    {std::ofstream f(root/"drivable_grid.rle");f<<"3085 7232 0.2 0 -8\n";for(int row=20;row<60;++row)f<<row<<" 0 1300\n";}
    RetainedRecovery(Config(cfg,root,"narrow.yaml"));
    CenterOpensDuringSearch(YAML::Clone(cfg),root);
    std::filesystem::remove_all(root);return failures?1:0;
}
