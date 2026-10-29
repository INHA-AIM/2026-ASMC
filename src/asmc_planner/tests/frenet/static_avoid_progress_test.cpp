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
    bool moving=false,contact_seen=false,slow=true,completed=false,valid=true,slew=true,consumer=true;
    for(int i=0;i<1100&&valid;++i) {
        valid=p.PlanCommittedAvoid(ego,objects,c,path,r)&&path.x.size()>1;
        if(!valid){std::cout<<"contact stalled x="<<ego.x<<" mode="<<r.static_progress_mode<<" stage="<<r.rejection_stage<<'\n';break;}
        if(p.avoid_completed()){completed=true;break;}
        moving|=r.static_progress_mode==StaticAvoidPolicy::CONTACT_PROGRESS;
        contact_seen|=r.static_contact_cost>0;
        const double cap=r.static_progress_mode==StaticAvoidPolicy::CONTACT_PROGRESS?.5:
            r.static_progress_mode==StaticAvoidPolicy::REDUCED_MARGIN?1:2;
        if(moving)slow&=*std::max_element(path.v.begin(),path.v.end())<=
            std::max(cap,ego.v+std::pow(std::max(0.0,path.a.front()),2)/12)+.11;
        consumer&=ConsumerAllows(path,r.static_progress_mode);
        slew&=PathObeysSteeringRate(path,.1,3,{.844,1,5,.1});
        ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};
    }
    Expect(valid&&moving&&contact_seen&&completed,"explicit contact policy follows the stored reference through fixed occupancy and completes");
    Expect(slew,"contact prefixes remain actuator-feasible");
    Expect(slow,"each recovery candidate obeys its current policy speed cap");
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

void LostId(const std::string& config) {
    const auto data=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../tests/fixtures/static_id_loss_stall.json");
    FrenetPlanner p;Expect(p.Init(config),"ID-loss recording initializes");
    p.ConfigureStaticAvoidRecovery(true,false);
    auto c=Command(data["primary"].as<int>());c.target_speed=data["target_speed"].as<double>();
    const auto frames=data["frames"];
    auto ego=State(frames[0]["ego"]);auto objects=Objects(frames[0]["objects"]);
    CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
    const bool ready=Prepare(p,ego,objects,c,path,r);
    Expect(ready&&p.CommitAvoid(r),"ID-loss scene begins with the original nominal transaction");
    if(!ready)return;
    p.ConfigureStaticAvoidRecovery(true,true);
    for(std::size_t i=1;i<frames.size();++i) {
        ego=State(frames[i]["ego"]);objects=Objects(frames[i]["objects"]);
        p.PlanCommittedAvoid(ego,objects,c,path,r);
    }
    bool valid=true,phase=true,consumer=true,slow=true;
    for(int i=0;i<90;++i) {
        const bool ok=p.PlanCommittedAvoid(ego,objects,c,path,r);
        if(!ok){std::cout<<"ID-loss failure "<<r.rejection_stage<<" recovery="<<r.static_recovery_rejection<<'\n';valid=false;break;}
        phase&=std::string(p.avoid_phase_name())!="TRACKING";
        consumer&=ConsumerAllows(path,r.static_progress_mode);
        slow&=*std::max_element(path.v.begin(),path.v.end())<=1.1;
    }
    Expect(valid&&phase,"stationary ID-loss scene keeps producing recovery beyond the dropout grace without TRACKING regression");
    Expect(consumer&&slow,"ID-loss recovery retains a Controller-executable crawl policy");
    bool completed=false;
    for(int i=0;i<1300&&valid;++i) {
        valid=p.PlanCommittedAvoid(ego,objects,c,path,r)&&path.x.size()>1;
        if(!valid){std::cout<<"ID-loss tracking stalled "<<r.rejection_stage<<'\n';break;}
        if(p.avoid_completed()){completed=true;break;}
        consumer&=ConsumerAllows(path,r.static_progress_mode);
        ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};
    }
    Expect(valid&&completed&&consumer,"recorded ID-loss transaction completes under ideal tracking");
}

void ColdStart(const std::string& config,int request_kind) {
    FrenetPlanner p;Expect(p.Init(config),"cold-start recovery map initializes");
    p.ConfigureStaticAvoidRecovery(true,true);
    CartesianState ego{};ego.x=50;
    auto wall=Box(7,58,0,20);wall.type=0;wall.classification="static_obstacle";wall.camera_class="plastic_barrier";
    std::vector<TrackedObject> objects{wall};
    if(request_kind==2)objects.push_back(Box(9,140,0));
    auto c=Command(request_kind==1?999:request_kind==2?9:7);
    CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
    const bool ready=Prepare(p,ego,objects,c,path,r);
    Expect(ready&&r.static_progress_mode==StaticAvoidPolicy::CONTACT_PROGRESS,
        request_kind==1?"stale command ID can prepare current-group contact progress without nominal history":
                      "first-ever candidate can be contact progress without nominal history");
    if(!ready)return;
    Expect(p.CommitAvoid(r),"cold-start recovery still requires COMMIT");
    bool valid=true,completed=false,contact=false,consumer=true,phase=true,slow=true,road=true;
    for(int i=0;i<850&&valid;++i) {
        // Simulate tracker churn without changing physical occupancy.
        objects[0].id=100+i/12;
        valid=p.PlanCommittedAvoid(ego,objects,c,path,r)&&path.x.size()>1;
        if(!valid){std::cout<<"cold start stalled x="<<ego.x<<" stage="<<r.rejection_stage<<" recovery="<<r.static_recovery_rejection<<'\n';break;}
        if(p.avoid_completed()){completed=true;break;}
        contact|=r.static_contact_cost>0;
        consumer&=ConsumerAllows(path,r.static_progress_mode);
        phase&=std::string(p.avoid_phase_name())!="TRACKING";
        const double cap=r.static_progress_mode==StaticAvoidPolicy::CONTACT_PROGRESS?.5:1;
        slow&=*std::max_element(path.v.begin(),path.v.end())<=cap+.1;
        auto shape=p.vehicle_shape();shape.width=std::max(shape.width,shape.road_outer_width);
        road&=p.road_map().PathClear(path,shape);
        ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};
    }
    Expect(valid&&completed&&contact,"cold-start recovery progresses through fixed occupancy and finishes despite repeated ID churn");
    Expect(consumer&&phase&&slow&&road,"cold-start progress keeps its stop tail, policy cap, road containment and execution phase until handoff");
    if(request_kind==2)Expect(completed&&ego.x<100,"nearby recovery group hands off before a separated remote primary");
}

void ColdStartGuards(const std::string& config) {
    for(int variant=0;variant<9;++variant) {
        FrenetPlanner p;const bool initialized=p.Init(config);p.ConfigureStaticAvoidRecovery(true,variant!=0);
        CartesianState ego{};ego.x=50;
        auto wall=Box(7,58,0,20);
        if(variant==1)wall.motion_state=ObjectMotionState::UNKNOWN;
        if(variant==2)wall.motion_state=ObjectMotionState::DYNAMIC_PENDING;
        if(variant==3)wall.motion_state=ObjectMotionState::DYNAMIC;
        if(variant==4)wall.type=1;
        if(variant==5)wall.type=2;
        if(variant==6){wall.type=0;wall.camera_class="person";}
        CartesianPath path;FrenetPlanner::AvoidPrepareResult r;bool blocked=initialized;
        auto command=Command(7);
        if(variant>=7) {
            p.PrepareAvoid(ego,{wall},command,path,r);
            double station=0;blocked&=p.ProjectEgoToS(ego,station);
            command.has_avoid_target_d=true;command.avoid_target_s=station+.5;
            command.avoid_target_d=variant==7?3:-3;
            if(variant==8)command.requested_direction=LateralDirection::LEFT;
        }
        for(int i=0;i<8;++i){p.PrepareAvoid(ego,{wall},command,path,r);blocked&=!r.candidate_valid;}
        Expect(blocked,"cold-start contact cannot bypass opt-out, motion/mobile guards or explicit pose/direction constraints");
    }
}

void EmptyObservationRecovery(const std::string& config) {
    FrenetPlanner p;Expect(p.Init(config),"complete-dropout recovery map initializes");
    p.ConfigureStaticAvoidRecovery(true,true);
    CartesianState ego{};ego.x=50;
    auto wall=Box(7,58,0,20);wall.type=0;
    CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
    const auto c=Command(7);
    const bool ready=Prepare(p,ego,{wall},c,path,r);
    Expect(ready&&p.CommitAvoid(r),"complete-dropout scene commits without nominal history");
    if(!ready)return;
    bool nominal=false,valid=true,slow=true;
    // After finite ghost expiry, an independently checked full nominal path
    // can use ordinary control even if Behavior now proposes KEEP. Crawl
    // speed is still a separate, per-maneuver planning constraint.
    for(int i=0;i<100;++i) {
        valid&=p.PlanCommittedAvoid(ego,{},c,path,r)&&path.v.size()>1;
        if(!valid)break;
        nominal|=r.static_progress_mode==StaticAvoidPolicy::NORMAL;
        slow&=*std::max_element(path.v.begin(),path.v.end())<=1.1;
    }
    Expect(valid&&nominal&&slow,"complete observation dropout can regain a normal path without exceeding the body-clear cap before progress");
}
}

int main() {
    const auto root=std::filesystem::absolute("static_avoid_progress_fixture");std::filesystem::create_directories(root);
    auto cfg=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/frenet/params.yaml");
    cfg["waypoint_file"]=std::string(FRENET_PACKAGE_SRC_DIR)+"/../../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt";
    cfg["avoid_hd_map"]["directory"]=std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/frenet/hd_map";
    {std::ofstream f(root/"recorded.yaml");f<<cfg;}Recorded((root/"recorded.yaml").string());LostId((root/"recorded.yaml").string());
    {std::ofstream f(root/"waypoints.txt");for(int x=0;x<=260;++x)f<<x<<" 0\n";}
    {std::ofstream f(root/"drivable.json");f<<R"({"frame_id":"map","map_id":"waize_katri_2025-10-23","grid":{"origin_x":0,"origin_y":-6},"drivable_areas":[{"validity":"validated","polygon":[[0,-4],[260,-4],[260,4],[0,4]]}]})";}
    {std::ofstream f(root/"lines.json");f<<R"({"map_id":"kcity2025_competition_route_curation_candidate_v14","frame":{"local_origin_in_global":[302595,4124145]},"features":[{"feature_type":"lane_marking","attributes":{"pattern":"solid","functional_role":"lane_boundary_paint"},"geometry":{"coordinates":[[0,5],[260,5]]}}]})";}
    {std::ofstream f(root/"drivable_grid.rle");f<<"3085 7232 0.2 0 -6\n";for(int row=10;row<50;++row)f<<row<<" 0 1300\n";}
    cfg["waypoint_file"]=(root/"waypoints.txt").string();cfg["avoid_hd_map"]["directory"]=root.string();cfg["roundabout_merge"]["enabled"]=false;
    {std::ofstream f(root/"synthetic.yaml");f<<cfg;}Contact((root/"synthetic.yaml").string());
    for(int kind=0;kind<3;++kind)ColdStart((root/"synthetic.yaml").string(),kind);
    ColdStartGuards((root/"synthetic.yaml").string());
    EmptyObservationRecovery((root/"synthetic.yaml").string());
    std::filesystem::remove_all(root);return failures?1:0;
}
