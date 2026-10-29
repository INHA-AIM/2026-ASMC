#include "Frenet/frenet_planner.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include "Util/StaticAvoidProgress.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {
int failures=0;
void Expect(bool ok,const char* name){std::cout<<(ok?"[PASS] ":"[FAIL] ")<<name<<'\n';failures+=!ok;}
CartesianState State(const YAML::Node& n) {
    return {n["x"].as<double>(),n["y"].as<double>(),n["yaw"].as<double>(),n["kappa"].as<double>(),n["v"].as<double>(),n["a"].as<double>()};
}
std::vector<TrackedObject> Objects(const YAML::Node& nodes) {
    std::vector<TrackedObject> objects;
    for(const auto& n:nodes) {
        TrackedObject o{};o.id=n["id"].as<int>();o.type=n["type"].as<int>();
        o.x=n["x"].as<double>();o.y=n["y"].as<double>();o.z=n["z"].as<double>();
        o.heading=n["heading"].as<double>();o.width=n["width"].as<double>();
        o.length=n["length"].as<double>();o.height=n["height"].as<double>();
        o.vx=n["vx"].as<double>();o.vy=n["vy"].as<double>();o.speed=std::hypot(o.vx,o.vy);
        o.motion_state=static_cast<ObjectMotionState>(n["motion_state"].as<int>());
        o.classification=n["classification"].as<std::string>();o.camera_class=n["camera_class"].as<std::string>();
        o.velocity_unavailable=n["velocity_unavailable"].as<bool>();o.geometry_valid=o.z_valid=true;
        for(int i=0;i<8;++i)o.corners[i]={n["corners"][i][0].as<double>(),n["corners"][i][1].as<double>(),n["corners"][i][2].as<double>()};
        objects.push_back(o);
    }
    return objects;
}
TrackedObject Box(int id,double x,double y,double width=1) {
    TrackedObject o{};o.id=id;o.type=3;o.x=x;o.y=y;o.width=width;o.length=1;o.height=1;
    o.geometry_valid=o.z_valid=true;o.motion_state=ObjectMotionState::LOW_THREAT;
    for(int i=0;i<8;++i){const double xs[]={-.5,.5,.5,-.5},ys[]={-.5,-.5,.5,.5};o.corners[i]={x+xs[i%4],y+width*ys[i%4],double(i/4)};}
    return o;
}
PlannerCommand Command(int id=1){PlannerCommand c{};c.mode=AVOID;c.target_object_id=id;c.has_target_speed=true;c.target_speed=4;return c;}
bool ConsumerAllows(const CartesianPath& path,StaticAvoidPolicy::Mode mode) {
    asmc_msgs::Trajectory wire;wire.valid=true;wire.dt=.1;wire.execution_policy=mode;
    wire.points.resize(path.v.size());
    for(std::size_t i=0;i<path.v.size();++i) {
        wire.points[i].velocity=path.v[i];wire.points[i].acceleration=path.a[i];
    }
    return StaticAvoidProgressProfileAllowed(wire,true,1,.5);
}
bool Prepare(FrenetPlanner& p,const CartesianState& ego,const std::vector<TrackedObject>& objects,
    const PlannerCommand& c,CartesianPath& path,FrenetPlanner::AvoidPrepareResult& r) {
    for(int i=0;i<150;++i){p.PrepareAvoid(ego,objects,c,path,r);if(r.candidate_valid)return true;}
    std::cout<<"blocked: "<<r.rejection_stage<<" id="<<r.blocking_object_id<<'\n';return false;
}
void Recorded(const std::string& config) {
    const auto data=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../tests/fixtures/static_obb_growth_stall.json");
    FrenetPlanner p;Expect(p.Init(config),"recorded map initializes");p.ConfigureStaticAvoidRecovery(true,false);
    auto ego=State(data["initial"]["ego"]);auto objects=Objects(data["initial"]["objects"]);
    CartesianPath path;FrenetPlanner::AvoidPrepareResult r;auto c=Command(2);c.target_speed=data["target_speed"].as<double>();
    const bool ready=Prepare(p,ego,objects,c,path,r);Expect(ready,"recorded moving approach has a candidate");
    if(!ready)return;
    Expect(p.CommitAvoid(r),"recorded transaction commits before OBB growth");
    ego=State(data["stopped"]["ego"]);objects=Objects(data["stopped"]["objects"]);
    bool recovered=false;
    for(int i=0;i<150&&!recovered;++i)recovered=p.PlanCommittedAvoid(ego,objects,c,path,r);
    std::cout<<"recorded recovery mode="<<r.static_progress_mode<<" margin="<<r.static_progress_margin<<" clearance="<<r.static_minimum_clearance<<" stage="<<r.rejection_stage<<'\n';
    Expect(recovered&&r.static_progress_mode==StaticAvoidPolicy::REDUCED_MARGIN,"existing COMMIT recovers with a reduced margin without contact permission");
    Expect(recovered&&r.static_progress_margin<=.2&&r.static_minimum_clearance>0,"recovery keeps the physical body separated");
    bool completed=false,valid=recovered,consumer=ConsumerAllows(path,r.static_progress_mode);
    for(int i=0;i<800&&valid;++i) {
        valid=p.PlanCommittedAvoid(ego,objects,c,path,r)&&path.x.size()>1;
        if(!valid)break;
        if(p.avoid_completed()){completed=true;break;}
        consumer&=ConsumerAllows(path,r.static_progress_mode);
        ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};
    }
    Expect(valid&&completed,"reduced-margin recorded transaction completes under ideal tracking");
    Expect(consumer,"recorded recovery profiles are executable by the Controller policy gate");
}
void Contact(const std::string& config) {
    FrenetPlanner p;Expect(p.Init(config),"contact map initializes");p.ConfigureStaticAvoidRecovery(true,true);
    CartesianState ego{};ego.x=50;ego.v=1;
    std::vector<TrackedObject> objects={Box(1,80,0)};auto c=Command();CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
    const bool ready=Prepare(p,ego,objects,c,path,r);Expect(ready&&p.CommitAvoid(r),"contact policy begins with a nominally checked transaction");
    if(!ready)return;
    const auto wall=Box(9,65,0,20);objects.push_back(wall);
    p.ConfigureStaticAvoidRecovery(true,false);
    bool blocked_without_contact=true;
    for(int i=0;i<6;++i)blocked_without_contact&=!p.PlanCommittedAvoid(ego,objects,c,path,r);
    Expect(blocked_without_contact,"margin-only recovery cannot authorize progress through a physical wall");
    p.ConfigureStaticAvoidRecovery(true,true);
    bool moving=false,contact_seen=false,normal_resumed=false,completed=false,valid=true,slew=true,consumer=true;
    for(int i=0;i<1100&&valid;++i) {
        valid=p.PlanCommittedAvoid(ego,objects,c,path,r)&&path.x.size()>1;
        if(!valid){std::cout<<"contact stalled x="<<ego.x<<" mode="<<r.static_progress_mode<<" stage="<<r.rejection_stage<<'\n';break;}
        if(p.avoid_completed()){completed=true;break;}
        moving|=r.static_progress_mode==StaticAvoidPolicy::CONTACT_PROGRESS;
        contact_seen|=r.static_contact_cost>0;
        normal_resumed|=moving&&r.static_progress_mode==StaticAvoidPolicy::NORMAL;
        consumer&=ConsumerAllows(path,r.static_progress_mode);
        slew&=PathObeysSteeringRate(path,.1,3,{.844,1,5,.1});
        ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};
    }
    Expect(valid&&moving&&contact_seen&&completed,"explicit contact policy follows the stored reference through fixed occupancy and completes");
    Expect(slew,"contact prefixes remain actuator-feasible");
    Expect(normal_resumed,"ordinary execution resumes after the contact corridor clears");
    Expect(consumer,"contact recovery profiles are executable by the Controller policy gate");
    // No globally weakened collision checker, even after contact was used.
    CartesianPath straight;straight.x={0,1,2};straight.y={0,0,0};straight.yaw=straight.y;
    std::vector<TrackedObject> actor={Box(99,2,0,1)};
    StaticAvoidPolicy policy;policy.mode=StaticAvoidPolicy::CONTACT_PROGRESS;policy.margin=0;policy.object_ids={99};
    Expect(StaticAvoidTrajectoryClear(straight,actor,{1.9,4.635,.79},{.45,.1,8},.1,&policy),"opted-in fixed occupancy is a contact cost");
    for(auto motion:{ObjectMotionState::UNKNOWN,ObjectMotionState::DYNAMIC_PENDING,ObjectMotionState::DYNAMIC}) {
        actor[0].motion_state=motion;
        Expect(!StaticAvoidTrajectoryClear(straight,actor,{1.9,4.635,.79},{.45,.1,8},.1,&policy),"motion uncertainty cannot inherit a static contact exception");
    }
    actor[0].motion_state=ObjectMotionState::LOW_THREAT;
    for(int type:{1,2}){actor[0].type=type;Expect(!StaticAvoidTrajectoryClear(straight,actor,{1.9,4.635,.79},{.45,.1,8},.1,&policy),"vehicles and pedestrians remain blocking");}
    actor[0].type=0;actor[0].camera_class="person";
    Expect(!StaticAvoidTrajectoryClear(straight,actor,{1.9,4.635,.79},{.45,.1,8},.1,&policy),"mobile semantics remain blocking even with UNKNOWN transport type");
    actor[0].camera_class.clear();actor[0].geometry_valid=false;
    Expect(!StaticAvoidTrajectoryClear(straight,actor,{1.9,4.635,.79},{.45,.1,8},.1,&policy),"invalid geometry never gains contact permission");
}
}

int main() {
    const auto root=std::filesystem::absolute("static_avoid_progress_fixture");std::filesystem::create_directories(root);
    auto cfg=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/frenet/params.yaml");
    cfg["waypoint_file"]=std::string(FRENET_PACKAGE_SRC_DIR)+"/../../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt";
    cfg["avoid_hd_map"]["directory"]=std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/frenet/hd_map";
    {std::ofstream f(root/"recorded.yaml");f<<cfg;}Recorded((root/"recorded.yaml").string());
    {std::ofstream f(root/"waypoints.txt");for(int x=0;x<=260;++x)f<<x<<" 0\n";}
    {std::ofstream f(root/"drivable.json");f<<R"({"frame_id":"map","map_id":"waize_katri_2025-10-23","grid":{"origin_x":0,"origin_y":-6},"drivable_areas":[{"validity":"validated","polygon":[[0,-4],[260,-4],[260,4],[0,4]]}]})";}
    {std::ofstream f(root/"lines.json");f<<R"({"map_id":"kcity2025_competition_route_curation_candidate_v14","frame":{"local_origin_in_global":[302595,4124145]},"features":[{"feature_type":"lane_marking","attributes":{"pattern":"solid","functional_role":"lane_boundary_paint"},"geometry":{"coordinates":[[0,5],[260,5]]}}]})";}
    {std::ofstream f(root/"drivable_grid.rle");f<<"3085 7232 0.2 0 -6\n";for(int row=10;row<50;++row)f<<row<<" 0 1300\n";}
    cfg["waypoint_file"]=(root/"waypoints.txt").string();cfg["avoid_hd_map"]["directory"]=root.string();cfg["roundabout_merge"]["enabled"]=false;
    {std::ofstream f(root/"synthetic.yaml");f<<cfg;}Contact((root/"synthetic.yaml").string());
    std::filesystem::remove_all(root);return failures?1:0;
}
