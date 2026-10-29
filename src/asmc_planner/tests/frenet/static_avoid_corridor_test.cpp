#include "Frenet/frenet_planner.hpp"
#include "Frenet/Generator/StaticAvoidPath.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {
int failures=0;
void Expect(bool ok,const std::string& name) {
    std::cout<<(ok?"[PASS] ":"[FAIL] ")<<name<<'\n';if(!ok)++failures;
}
TrackedObject Box(int id,double x,double y,double width=1,double longitudinal=1) {
    TrackedObject o{};o.id=id;o.type=3;o.x=x;o.y=y;o.width=width;o.length=longitudinal;
    o.height=1;o.geometry_valid=o.z_valid=true;o.motion_state=ObjectMotionState::LOW_THREAT;
    for(int i=0;i<8;++i) {
        const double dx[4]={-.5,.5,.5,-.5},dy[4]={-.5,-.5,.5,.5};
        o.corners[i]={x+longitudinal*dx[i%4],y+width*dy[i%4],double(i/4)};
    }
    return o;
}
struct Fixture {
    std::filesystem::path dir=std::filesystem::absolute("static_avoid_corridor_fixture");
    std::string config;
    Fixture() {
        std::filesystem::create_directories(dir);
        {std::ofstream f(dir/"waypoints.txt");for(int x=0;x<=260;++x)f<<x<<" 0\n";}
        {std::ofstream f(dir/"drivable.json");f<<R"({"frame_id":"map","map_id":"waize_katri_2025-10-23","grid":{"origin_x":0,"origin_y":-6},"drivable_areas":[{"validity":"validated","polygon":[[0,-4],[260,-4],[260,4],[0,4]]}]})";}
        {std::ofstream f(dir/"lines.json");f<<R"({"map_id":"kcity2025_competition_route_curation_candidate_v14","frame":{"local_origin_in_global":[302595,4124145]},"features":[{"feature_type":"lane_marking","attributes":{"pattern":"solid","functional_role":"lane_boundary_paint"},"geometry":{"coordinates":[[0,5],[260,5]]}}]})";}
        {std::ofstream f(dir/"drivable_grid.rle");f<<"3085 7232 0.2 0 -6\n";for(int row=10;row<50;++row)f<<row<<" 0 1300\n";}
        auto cfg=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/frenet/params.yaml");
        cfg["roundabout_merge"]["enabled"]=false;
        cfg["waypoint_file"]=(dir/"waypoints.txt").string();cfg["avoid_hd_map"]["directory"]=dir.string();
        cfg["planner"]["collision_check"]["margin_growth_rate"]=0;
        config=(dir/"params.yaml").string();std::ofstream f(config);f<<cfg;
    }
    ~Fixture(){std::filesystem::remove_all(dir);}
};
PlannerCommand Command(int id=1) {
    PlannerCommand c{};c.mode=AVOID;c.target_object_id=id;c.has_target_speed=true;c.target_speed=4;return c;
}
bool Prepare(FrenetPlanner& p,const CartesianState& ego,const std::vector<TrackedObject>& objects,
    const PlannerCommand& command,FrenetPlanner::AvoidPrepareResult& r,CartesianPath& path) {
    for(int i=0;i<100;++i) {
        p.PrepareAvoid(ego,objects,command,path,r);
        if(r.candidate_valid){std::cout<<"  ready spatial="<<r.spatial_geometry.x.size()<<" stage="<<r.rejection_stage<<" group="<<r.map_group_valid<<'\n';return true;}
    }
    std::cout<<"  no candidate: stage="<<r.rejection_stage<<" blocker="<<r.blocking_object_id<<" pending="<<r.search_pending<<'\n';
    return false;
}
void MultiMarginRecovery() {
    CartesianPath path;path.x={0,.5,1,2,3};path.y={0,0,0,0,0};path.yaw=path.y;
    path.kappa=path.y;path.v={1,1,1,1,1};path.a=path.y;
    std::vector<TrackedObject> objects={Box(1,-.2,1.4,.6),Box(2,-.2,-1.4,.6)};
    std::vector<int> ids={1,2};
    Expect(!EvaluateCartesianTrajectorySafety(path,objects,{1.9,4.635,.79},{.45,0,8},.1,1).clear,
        "single recovery ID cannot exempt the other initial side margin");
    Expect(EvaluateCartesianTrajectorySafety(path,objects,{1.9,4.635,.79},{.45,0,8},.1,-1,&ids).clear,
        "both initial static margins independently escape without body overlap");
    objects[1].motion_state=ObjectMotionState::UNKNOWN;
    Expect(!EvaluateCartesianTrajectorySafety(path,objects,{1.9,4.635,.79},{.45,0,8},.1,-1,&ids).clear,
        "additional recovery never relabels an uncertain object as static");
    objects[1]=Box(2,-.2,-1.0,.6);
    Expect(!EvaluateCartesianTrajectorySafety(path,objects,{1.9,4.635,.79},{.45,0,8},.1,-1,&ids).clear,
        "multiple recovery cannot permit physical body overlap");
}
}

int main() {
    MultiMarginRecovery();Fixture fixture;
    for(double initial_speed:{0.0,.49,.51,4.0}) {
        FrenetPlanner planner;Expect(planner.Init(fixture.config),"static fixture initializes");
        CartesianState ego{};ego.x=50;ego.v=initial_speed;
        const auto objects=std::vector<TrackedObject>{Box(1,80,0)};
        CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
        const bool ready=Prepare(planner,ego,objects,Command(),r,path);
        Expect(ready,"candidate exists from speed "+std::to_string(initial_speed));
        if(!ready)continue;
        Expect(!r.spatial_geometry.x.empty(),"candidate preserves explicit spatial geometry");
        Expect(PathObeysSteeringRate(r.candidate_path,.1,3,{.844,1,5,.1}),"candidate obeys Controller slew");
        Expect(std::fabs(r.candidate_path.v.front()-initial_speed)<1e-9,"candidate preserves observed initial speed");
        Expect(*std::max_element(r.candidate_path.v.begin(),r.candidate_path.v.end())>1,"standstill candidate actually moves");
        Expect(r.candidate_path.v.back()<.02,"accepted geometry includes a stop-capable tail");
        const double target=r.selected_target_d;
        Expect(planner.CommitAvoid(r),"candidate commits");
        auto changed=objects;changed[0].width=20; // Same authoritative corners.
        const bool ok=planner.PlanCommittedAvoid(ego,changed,Command(),path,r);
        Expect(ok&&r.retained_geometry,"valid incumbent survives scalar-fit noise");
        Expect(ok&&std::fabs(r.selected_target_d-target)<1e-9,"observation does not widen committed target");
        bool steady=ok;
        for(int i=0;i<12&&steady;++i) {
            ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};
            auto noisy=objects;
            const double jitter=i%2?.008:-.008;
            noisy[0].x+=jitter;
            for(auto& corner:noisy[0].corners)corner.x+=jitter;
            steady=planner.PlanCommittedAvoid(ego,noisy,Command(),path,r)&&r.retained_geometry;
        }
        Expect(steady,"small observation jitter keeps the accepted corridor while moving");
        auto blocked=changed;blocked.push_back(Box(9,ego.x+3,0,12,1));
        Expect(!planner.PlanCommittedAvoid(ego,blocked,Command(),path,r),"new physical wall invalidates the incumbent");
        // Resume from a stopped, known feasible pose on the same transaction.
        ego.v=ego.a=0;
        bool recovered=false;
        for(int i=0;i<100&&!recovered;++i)recovered=planner.PlanCommittedAvoid(ego,objects,Command(),path,r);
        Expect(recovered&&r.candidate_valid,"blocked transaction recovers after the wall disappears");
    }
    {
        FrenetPlanner planner;Expect(planner.Init(fixture.config),"S corridor initializes");
        CartesianState ego{};ego.x=50;ego.v=2;
        std::vector<TrackedObject> objects={Box(1,80,1.25,2.5),Box(2,104,-1.25,2.5),Box(3,128,1.25,2.5)};
        CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
        bool ready=Prepare(planner,ego,objects,Command(),r,path);
        Expect(ready,"three alternating obstacles have a gentle S candidate");
        if(ready) {
            Expect(r.map_group_valid,"alternating blockers form one finite group");
            Expect(*std::min_element(r.candidate_path.y.begin(),r.candidate_path.y.end())<-1&&
                   *std::max_element(r.candidate_path.y.begin(),r.candidate_path.y.end())>1,
                   "S candidate contains both lateral directions");
            Expect(CartesianAvoidTrajectoryClear(r.candidate_path,objects,{1.9,4.635,.79},{.45,0,8},.1),"S path checks every obstacle");
            Expect(planner.CommitAvoid(r),"S candidate commits");
            bool complete=false,valid=true,stopped_for_test=false;int cycles=0;
            int unintended_stops=0;bool premature_return=false;
            for(;cycles<1000;++cycles) {
                if(!planner.PlanCommittedAvoid(ego,objects,Command(),path,r)||path.x.size()<2){valid=false;break;}
                if(planner.avoid_completed()){complete=true;break;}
                ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};
                if(ego.x>85&&ego.x<125)premature_return|=std::string(planner.avoid_phase_name())=="RETURN";
                if(ego.v<.02)++unintended_stops;
                if(!stopped_for_test&&ego.x>92) {
                    // A stop during the first-to-second transition, with no
                    // measured standstill wheel angle, must reconnect.
                    ego.v=ego.a=ego.kappa=0;stopped_for_test=true;
                }
            }
            std::cout<<"  S tracking cycles="<<cycles<<" x="<<ego.x<<" d="<<planner.last_d()<<" stage="<<r.rejection_stage<<'\n';
            Expect(valid&&complete,"gentle S reaches completion under ideal tracking");
            Expect(!premature_return,"primary pass does not start the group return");
            Expect(unintended_stops==0,"fixed S corridor has no repeated unrequested stops");
        }
    }
    {
        const std::string root=FRENET_PACKAGE_SRC_DIR;
        const auto planner=YAML::LoadFile(root+"/../config/frenet/params.yaml");
        const auto behavior=YAML::LoadFile(root+"/../config/behavior/behavior_params.yaml");
        const auto controller=YAML::LoadFile(root+"/../../asmc_controller/config/controller.yaml");
        Expect(behavior["vehicle"]["width"].as<double>()==planner["planner"]["vehicle_shape"]["width"].as<double>(),"production Behavior and collision body width agree");
        Expect(behavior["avoid"]["path_margin_m"].as<double>()==planner["planner"]["collision_check"]["safety_margin"].as<double>(),"production acquisition and collision margins agree");
        Expect(planner["planner"]["avoid"]["bypass_accel_cap"].as<double>()<=controller["constraints"]["acceleration"]["max_forward"].as<double>(),"bypass positive acceleration fits Controller capability");
    }
    {
        FrenetPlanner planner;Expect(planner.Init(fixture.config),"overlapping blocker fixture initializes");
        CartesianState ego{};ego.x=50;ego.v=2;
        const std::vector<TrackedObject> objects={Box(1,80,0,1),Box(2,83,0,2)};
        CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
        Expect(Prepare(planner,ego,objects,Command(),r,path)&&r.map_group_valid,
            "close wider blocker reserves the wider corridor before the first object");
    }
    {
        FrenetPlanner planner;Expect(planner.Init(fixture.config),"side recovery initializes");
        CartesianState ego{};ego.x=50;
        std::vector<TrackedObject> objects={Box(1,80,0),Box(2,49.8,1.65,.6)};
        CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
        Expect(Prepare(planner,ego,objects,Command(),r,path),"non-primary initial side margin permits checked forward escape");
    }
    {
        FrenetPlanner planner;Expect(planner.Init(fixture.config),"alongside primary initializes");
        CartesianState ego{};ego.x=50;
        std::vector<TrackedObject> objects={Box(1,49.8,1.65,.6)};
        CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
        const bool ready=Prepare(planner,ego,objects,Command(),r,path);
        Expect(ready,"a primary centered behind the axle remains actionable while its footprint is alongside");
        if(ready) {
            Expect(planner.CommitAvoid(r),"alongside primary commits a checked escape");
            objects.front().id=11;
            Expect(planner.PlanCommittedAvoid(ego,objects,Command(),path,r)&&
                planner.active_avoid_target_id()==11,"alongside primary survives a unique spatial re-identification");
        }
    }
    {
        std::ofstream f(fixture.dir/"lines.json");
        f<<R"({"map_id":"kcity2025_competition_route_curation_candidate_v14","frame":{"local_origin_in_global":[302595,4124145]},"features":[{"feature_type":"lane_marking","attributes":{"pattern":"solid","functional_role":"lane_boundary_paint","line_width_m":0.15},"geometry":{"coordinates":[[0,-2],[260,-2]]}}]})";
        f.close();
        FrenetPlanner planner;Expect(planner.Init(fixture.config),"paint preference fixture initializes");
        CartesianState ego{};ego.x=50;ego.v=2;
        CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
        const bool ready=Prepare(planner,ego,{Box(1,80,0)},Command(),r,path);
        Expect(ready&&r.selected_target_d>0,"paint-free side beats preferred painted side before commit");
    }
    return failures?1:0;
}
