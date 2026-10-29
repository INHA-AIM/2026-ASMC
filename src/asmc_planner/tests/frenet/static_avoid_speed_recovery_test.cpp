#include "Frenet/frenet_planner.hpp"
#include "Frenet/Generator/StaticAvoidPath.hpp"
#include "Frenet/Geometry/PolygonUtils.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include "Util/StaticAvoidProgress.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {
int failures=0;
void Expect(bool ok,const char* text){std::cout<<(ok?"[PASS] ":"[FAIL] ")<<text<<'\n';failures+=!ok;}
void SpeedProof() {
    StaticAvoidSpeedRecovery proof;proof.Enter(.5);
    proof.ObserveNominal(10,0,2,1,.5,.5);
    for(int i=0;i<100;++i)proof.ObserveNominal(10,10,2,1,.5,.5);
    Expect(proof.cap==.5&&proof.frames==1,"one repeated sensor frame cannot accumulate speed permission");
    for(int i=1;i<=30;++i)proof.ObserveNominal(10+i*.1,0,2,1,.5,.5);
    Expect(proof.cap==.5,"fresh frames without actual forward progress cannot raise speed");
    for(int i=0;i<60;++i)proof.ObserveNominal(20+i*.1,i*.1,2,1,.5,.5);
    Expect(proof.cap==2,"sustained fresh nominal frames and progress raise the cap in bounded steps");
    proof.Enter(.5);
    for(int i=0;i<8;++i)proof.ObserveNominal(30+i*.01,i*.1,2,1,.5,.5);
    Expect(proof.cap==.5,"forward travel alone cannot bypass the confirmation interval");
    proof.ObserveNominal(32,2,2,1,.5,.5);
    Expect(proof.frames==1&&proof.cap==.5,"a sensor gap discards previous nominal evidence");
    proof.ObserveNominal(31,3,2,1,.5,.5);
    Expect(proof.frames==0,"backward sensor time invalidates the confirmation interval");
    proof.ObserveNominal(33,0,2,1,.5,.5);
    proof.ObserveNominal(33.1,-.2,2,1,.5,.5);
    Expect(proof.frames==1&&proof.cap==.5,"backward vehicle progress cannot count as forward clearance");
}

std::vector<TrackedObject> Objects(const YAML::Node& nodes) {
    std::vector<TrackedObject> objects;
    for(const auto& n:nodes) {
        TrackedObject o{};o.id=n["id"].as<int>();o.type=n["type"].as<int>();
        o.x=n["x"].as<double>();o.y=n["y"].as<double>();o.z=n["z"].as<double>();
        o.heading=n["heading"].as<double>();o.width=n["width"].as<double>();o.length=n["length"].as<double>();o.height=n["height"].as<double>();
        o.vx=n["vx"].as<double>();o.vy=n["vy"].as<double>();o.motion_state=static_cast<ObjectMotionState>(n["motion_state"].as<int>());
        o.classification=n["classification"].as<std::string>();o.camera_class=n["camera_class"].as<std::string>();
        o.velocity_unavailable=n["velocity_unavailable"].as<bool>();
        for(int i=0;i<8;++i)o.corners[i]={n["corners"][i][0].as<double>(),n["corners"][i][1].as<double>(),n["corners"][i][2].as<double>()};
        o.geometry_valid=geometry::ValidateCorners8(o.corners).valid;o.z_valid=true;objects.push_back(o);
    }
    return objects;
}
bool ConsumerAllows(const CartesianPath& path,StaticAvoidPolicy::Mode mode) {
    asmc_msgs::Trajectory wire;wire.valid=true;wire.dt=.1;wire.execution_policy=mode;wire.points.resize(path.v.size());
    for(std::size_t i=0;i<path.v.size();++i){wire.points[i].velocity=path.v[i];wire.points[i].acceleration=path.a[i];}
    return StaticAvoidProgressProfileAllowed(wire,true,1,.5);
}
void BagGeometry(const std::string& config) {
    FrenetPlanner p;Expect(p.Init(config),"bag geometry audit initializes the production map and limits");
    auto limits=p.kinematic_limits();limits.max_forward_accel=2;
    auto road_shape=p.vehicle_shape();road_shape.width=std::max(road_shape.width,road_shape.road_outer_width);
    const auto data=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../tests/fixtures/static_speed_recovery_bag.json");
    int normal=0,raised=0,resumed=0,margin=0,instant_cruise_rejected=0;
    for(const auto& frame:data["frames"]) {
        CartesianPath geometry;
        for(const auto& q:frame["path"]) {
            geometry.x.push_back(q[0].as<double>());geometry.y.push_back(q[1].as<double>());
            geometry.yaw.push_back(q[2].as<double>());geometry.kappa.push_back(q[3].as<double>());
        }
        const auto objects=Objects(frame["objects"]);
        if(frame["policy"].as<int>()==0) {
            ++normal;
            for(double cap:{1.0,1.5,2.0,4.2}) {
                CartesianPath candidate;
                if(RetimeStaticAvoidGeometry(geometry,frame["ego"]["v"].as<double>(),frame["ego"]["a"].as<double>(),cap,.1,3,limits,candidate)&&
                   p.road_map().PathClear(candidate,road_shape)&&
                   CartesianAvoidTrajectoryClear(geometry,objects,p.vehicle_shape(),p.collision_config(),0)&&
                   CartesianAvoidTrajectoryClear(candidate,objects,p.vehicle_shape(),p.collision_config(),.1)) {
                    if(cap>2)++resumed;else ++raised;
                }
            }
            CartesianPath instantaneous;
            if(!RetimeStaticAvoidGeometry(geometry,4.2,0,4.2,.1,3,limits,instantaneous))++instant_cruise_rejected;
        } else {
            StaticAvoidPolicy policy;policy.mode=StaticAvoidPolicy::REDUCED_MARGIN;policy.margin=frame["margin"].as<double>();
            for(const auto& o:objects)if(IsStaticAvoidProgressObject(o))policy.object_ids.push_back(o.id);
            CartesianPath candidate;
            if(RetimeStaticAvoidGeometry(geometry,frame["ego"]["v"].as<double>(),frame["ego"]["a"].as<double>(),1,.1,3,limits,candidate)&&
               p.road_map().PathClear(candidate,road_shape)&&
               StaticAvoidTrajectoryClear(candidate,objects,p.vehicle_shape(),p.collision_config(),.1,&policy)&&
               ConsumerAllows(candidate,policy.mode))++margin;
        }
    }
    std::cout<<"BAG SPEED AUDIT nominal="<<normal<<" raised="<<raised<<" margin="<<margin<<" instant_cruise_rejected="<<instant_cruise_rejected<<'\n';
    Expect(normal==10&&raised==30,"all ten nominal bag paths pass 1.0/1.5/2.0 m/s retiming, road and collision checks");
    Expect(resumed==10,"all ten nominal bag paths can rebuild a checked acceleration profile toward 4.2 m/s");
    Expect(margin==2,"body-clear reduced-margin bag paths pass at 1.0 m/s independently of past contact");
    Expect(instant_cruise_rejected>0,"a collision-clear geometry alone cannot authorize an instantaneous 4.2 m/s entry");
}

TrackedObject Wall(double y,double width) {
    TrackedObject o{};o.id=7;o.type=0;o.motion_state=ObjectMotionState::LOW_THREAT;
    o.x=80;o.y=y;o.width=width;o.length=1;o.height=1;o.geometry_valid=o.z_valid=true;
    for(int i=0;i<8;++i){const double x[]={-.5,.5,.5,-.5},dy[]={-.5,-.5,.5,.5};o.corners[i]={o.x+x[i%4],y+width*dy[i%4],double(i/4)};}
    return o;
}
struct Run {bool valid=true,complete=false,consumer=true,road=true,raised=false,demoted=false,retried=false;double maximum_cap=0;int ticks=0;};
Run NominalRecovery(const std::string& config,bool inject_blocker,bool crossing=false) {
    FrenetPlanner p;Run run;run.valid=p.Init(config);p.ConfigureStaticAvoidRecovery(true,true);
    CartesianState ego{};ego.x=50;
    PlannerCommand c{};c.mode=AVOID;c.target_object_id=7;c.has_target_speed=true;c.target_speed=4.2;
    CartesianPath path;FrenetPlanner::AvoidPrepareResult r;
    p.SetObjectObservationTime(1,1);
    for(int i=0;i<120&&!r.candidate_valid;++i)p.PrepareAvoid(ego,{Wall(0,20)},c,path,r);
    run.valid&=r.candidate_valid&&p.CommitAvoid(r);
    bool injected=false,restored=false;TrackedObject new_blocker;
    for(int i=0;i<1600&&run.valid;++i) {
        const double stamp=2+i*.1;p.SetObjectObservationTime(stamp,stamp);
        std::vector<TrackedObject> objects{i<12?Wall(0,20):Wall(3.2,1)};
        if(injected&&!restored)objects.push_back(new_blocker);
        bool crossing_now=false;
        if(crossing&&!injected&&r.static_next_speed_cap>r.static_execution_speed_cap+1e-6&&
           r.static_next_speed_cap>4) {
            new_blocker=Wall(-6.9,1);new_blocker.id=19;new_blocker.type=1;
            new_blocker.motion_state=ObjectMotionState::DYNAMIC;new_blocker.vy=3;
            const double dx=ego.x+8-new_blocker.x;new_blocker.x+=dx;
            for(auto& corner:new_blocker.corners)corner.x+=dx;
            objects.push_back(new_blocker);injected=restored=crossing_now=true;
        }
        run.valid=p.PlanCommittedAvoid(ego,objects,c,path,r)&&path.v.size()>1;
        if(!run.valid){std::cout<<"SPEED RUN failure tick="<<i<<" stage="<<r.rejection_stage<<" recovery="<<r.static_recovery_rejection<<'\n';break;}
        if(p.avoid_completed()){run.complete=true;run.ticks=i;break;}
        run.consumer&=ConsumerAllows(path,r.static_progress_mode);
        auto shape=p.vehicle_shape();shape.width=std::max(shape.width,shape.road_outer_width);
        run.road&=p.road_map().PathClear(path,shape);
        run.raised|=r.static_progress_mode==StaticAvoidPolicy::NORMAL&&r.static_execution_speed_cap>=1.5;
        run.maximum_cap=std::max(run.maximum_cap,r.static_execution_speed_cap);
        if(crossing_now)run.retried=r.static_speed_retry&&r.static_progress_mode==StaticAvoidPolicy::NORMAL&&
            r.static_execution_speed_cap<=1;
        if(inject_blocker&&run.raised&&!injected){
            injected=true;new_blocker=Wall(0,20);new_blocker.id=9;
            const double dx=ego.x+5-new_blocker.x;new_blocker.x+=dx;
            for(auto& corner:new_blocker.corners)corner.x+=dx;
        }
        else if(injected&&!restored) {
            run.demoted=r.static_progress_mode==StaticAvoidPolicy::CONTACT_PROGRESS&&r.static_execution_speed_cap<=.5;
            restored=true;
        }
        ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};
    }
    std::cout<<"SPEED RUN ticks="<<run.ticks<<" raised="<<run.raised<<" demoted="<<run.demoted<<" complete="<<run.complete<<'\n';
    return run;
}
}

int main() {
    SpeedProof();
    const auto root=std::filesystem::absolute("static_avoid_speed_fixture");std::filesystem::create_directories(root);
    auto cfg=YAML::LoadFile(std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/frenet/params.yaml");
    cfg["waypoint_file"]=std::string(FRENET_PACKAGE_SRC_DIR)+"/../../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt";
    cfg["avoid_hd_map"]["directory"]=std::string(FRENET_PACKAGE_SRC_DIR)+"/../config/frenet/hd_map";
    {std::ofstream f(root/"production.yaml");f<<cfg;}BagGeometry((root/"production.yaml").string());
    {std::ofstream f(root/"waypoints.txt");for(int x=0;x<=260;++x)f<<x<<" 0\n";}
    {std::ofstream f(root/"drivable.json");f<<R"({"frame_id":"map","map_id":"waize_katri_2025-10-23","grid":{"origin_x":0,"origin_y":-6},"drivable_areas":[{"validity":"validated","polygon":[[0,-4],[260,-4],[260,4],[0,4]]}]})";}
    {std::ofstream f(root/"lines.json");f<<R"({"map_id":"kcity2025_competition_route_curation_candidate_v14","frame":{"local_origin_in_global":[302595,4124145]},"features":[{"feature_type":"lane_marking","attributes":{"pattern":"solid","functional_role":"lane_boundary_paint"},"geometry":{"coordinates":[[0,5],[260,5]]}}]})";}
    {std::ofstream f(root/"drivable_grid.rle");f<<"3085 7232 0.2 0 -6\n";for(int row=10;row<50;++row)f<<row<<" 0 1300\n";}
    cfg["waypoint_file"]=(root/"waypoints.txt").string();cfg["avoid_hd_map"]["directory"]=root.string();cfg["roundabout_merge"]["enabled"]=false;
    cfg["planner"]["avoid"]["static_normal_resume_speed_cap"]=.5;
    {std::ofstream f(root/"baseline.yaml");f<<cfg;}
    cfg["planner"]["avoid"]["static_normal_resume_speed_cap"]=2.0;
    {std::ofstream f(root/"raised.yaml");f<<cfg;}
    const auto baseline=NominalRecovery((root/"baseline.yaml").string(),false);
    const auto raised=NominalRecovery((root/"raised.yaml").string(),false);
    Expect(baseline.valid&&baseline.complete&&raised.valid&&raised.complete&&raised.raised,
        "nominal speed returns after fresh observations and actual progress without losing completion");
    Expect(raised.ticks<baseline.ticks*.85,"certified speed recovery shortens the ideal-tracking episode");
    const auto blocked=NominalRecovery((root/"raised.yaml").string(),true);
    Expect(blocked.valid&&blocked.complete&&blocked.raised&&blocked.demoted,
        "a renewed blockage lowers the cap before the newly checked contact path executes");
    Expect(raised.consumer&&raised.road&&blocked.consumer&&blocked.road,
        "speed recovery and fallback preserve Controller policy gates and road containment");
    cfg["planner"]["avoid"]["static_normal_resume_speed_cap"]=4.2;
    cfg["planner"]["avoid"]["static_speed_step"]=4.2;
    {std::ofstream f(root/"resumed.yaml");f<<cfg;}
    const auto resumed=NominalRecovery((root/"resumed.yaml").string(),false);
    Expect(resumed.valid&&resumed.complete&&resumed.consumer&&resumed.road&&resumed.maximum_cap>4,
        "stable nominal recovery restores the ordinary bypass target with checked acceleration");
    const auto crossing=NominalRecovery((root/"resumed.yaml").string(),false,true);
    Expect(crossing.valid&&crossing.complete&&crossing.consumer&&crossing.road&&crossing.retried,
        "a crossing actor that rejects an upgrade retains the freshly rechecked lower-speed corridor");
    std::filesystem::remove_all(root);return failures?1:0;
}
