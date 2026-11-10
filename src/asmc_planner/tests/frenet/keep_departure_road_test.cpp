#include "Frenet/frenet_planner.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
    int failures=0;
    auto check=[&](bool ok,const char* text) {
        std::cout<<(ok?"[PASS] ":"[FAIL] ")<<text<<'\n';failures+=!ok;
    };
    const std::filesystem::path root=ASMC_DEPARTURE_ROAD_SOURCE;
    const auto control=YAML::LoadFile((root/"../asmc_controller/config/controller.yaml").string());
    struct { double wheelbase,steering_curvature_gain,steering_rate_max,
        steering_rate_min,steering_rate_reference_speed_mps; } mpc;
    mpc.wheelbase=control["mpc"]["wheelbase"].as<double>();
    mpc.steering_curvature_gain=control["mpc"]["steering_curvature_gain"].as<double>();
    mpc.steering_rate_max=control["constraints"]["steering"]["max_rate"].as<double>();
    mpc.steering_rate_min=control["constraints"]["steering"]["min_rate"].as<double>();
    mpc.steering_rate_reference_speed_mps=control["constraints"]["steering"]["rate_reference_speed_mps"].as<double>();
    auto cfg=YAML::LoadFile((root/"config/frenet/params.yaml").string());
    const auto steering=cfg["planner"]["steering_profile"];
    check(cfg["planner"]["wheelbase"].as<double>()==mpc.wheelbase &&
        steering["curvature_gain"].as<double>()==mpc.steering_curvature_gain &&
        steering["max_rate"].as<double>()==mpc.steering_rate_max &&
        steering["min_rate"].as<double>()==mpc.steering_rate_min &&
        steering["reference_speed"].as<double>()==mpc.steering_rate_reference_speed_mps,
        "Planner actuator calibration and limits match the current read-only Controller configuration");
    SteeringProfileLimits steering_limits;
    steering_limits.curvature_gain=mpc.steering_curvature_gain;
    steering_limits.max_rate=mpc.steering_rate_max;
    steering_limits.min_rate=mpc.steering_rate_min;
    steering_limits.reference_speed=mpc.steering_rate_reference_speed_mps;
    check(SteeringProfileRateLimit(0,steering_limits)==mpc.steering_rate_max &&
        SteeringProfileRateLimit(2*steering_limits.reference_speed,steering_limits)==mpc.steering_rate_max/4 &&
        SteeringProfileRateLimit(100,steering_limits)==mpc.steering_rate_min,
        "low-speed limit, inverse-square envelope, and high-speed floor agree with the Controller contract");
    CartesianPath rates;rates.kappa={0,.2};rates.v={.5,.5};
    check(!PathObeysSteeringRate(rates,.1,mpc.wheelbase,steering_limits),
        "curvature within the angle limit can still demand impossible steering slew");
    rates.kappa={0,.01};
    check(PathObeysSteeringRate(rates,.1,mpc.wheelbase,steering_limits),
        "gradual low-speed steering remains feasible");
    rates.v={15,15};
    check(!PathObeysSteeringRate(rates,.1,mpc.wheelbase,steering_limits),
        "future speed tightening of steering slew is enforced");
    rates.kappa={-.1,.1};rates.v={.5,.5};
    check(!PathObeysSteeringRate(rates,.7,mpc.wheelbase,steering_limits),
        "linear curvature interpolation cannot hide an intermediate steering-rate peak");
    cfg["waypoint_file"]=(root/"../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt").lexically_normal().string();
    cfg["avoid_hd_map"]["directory"]=(root/"config/hd_map").string();
    cfg["planner"]["visualization"]["enabled"]=false;
    const auto config=std::filesystem::temp_directory_path()/"asmc_keep_departure_road.yaml";
    {std::ofstream out(config);out<<cfg;}
    const auto fixture=YAML::LoadFile((root/"tests/fixtures/low_speed_road_departure.json").string());
    AvoidRoadMap road;std::string error;
    check(road.Load((root/"config/hd_map/drivable.json").string(),
        (root/"config/hd_map/lines.json").string(),error),"original road map loads");
    int moving=0;
    for(const auto& frame:fixture["frames"]) {
        const auto e=frame["ego"];
        CartesianState ego{e["x"].as<double>(),e["y"].as<double>(),e["yaw"].as<double>(),
            0,e["speed"].as<double>(),e["acceleration"].as<double>()};
        std::vector<TrackedObject> objects;
        for(const auto& item:frame["objects"]) {
            TrackedObject o;o.id=item["id"].as<int>();o.type=item["type"].as<int>();
            o.motion_state=static_cast<ObjectMotionState>(item["motion_state"].as<int>());
            o.x=item["x"].as<double>();o.y=item["y"].as<double>();o.z=item["z"].as<double>();
            o.heading=item["yaw"].as<double>();o.vx=item["vx"].as<double>();o.vy=item["vy"].as<double>();
            o.speed=std::hypot(o.vx,o.vy);o.width=item["width"].as<double>();
            o.length=item["length"].as<double>();o.height=item["height"].as<double>();
            for(int i=0;i<8;++i)o.corners[i]={item["corners"][i][0].as<double>(),
                item["corners"][i][1].as<double>(),item["corners"][i][2].as<double>()};
            o.geometry_valid=geometry::ValidateCorners8(o.corners).valid;o.z_valid=o.geometry_valid;
            objects.push_back(o);
        }
        FrenetPlanner planner;check(planner.Init(config.string()),"production planner initializes for a recorded frame");
        auto exterior=planner.vehicle_shape();exterior.width=fixture["road_outer_width"].as<double>();
        PlannerCommand keep;keep.mode=LANE_KEEPING;keep.has_target_speed=true;keep.target_speed=57.5/3.6;
        planner.SetObjectObservationTime(frame["object_stamp"].as<double>(),frame["ego_stamp"].as<double>());
        CartesianPath path;const bool ok=planner.Plan(ego,objects,keep,path);
        const bool initial_inside=road.FootprintInside(ego.x,ego.y,ego.yaw,exterior);
        const auto evaluation=initial_inside?road.EvaluatePath(path,exterior,planner.sample_dt()):
            road.EvaluateDepartureRecoveryPath(path,exterior,planner.sample_dt());
        std::cout<<"[FRAME] t="<<frame["elapsed_s"].as<double>()<<" v="<<ego.v
            <<" ok="<<ok<<" samples="<<path.x.size()<<" road="<<evaluation.drivable
            <<" overlap="<<evaluation.maximum_overlap_m<<'\n';
        check(!ok||evaluation.drivable,"accepted low-speed KEEP is inside or completes exterior recovery");
        bool steerable=true;double peak=0;
        for(std::size_t i=1;i<path.kappa.size();++i) {
            const double before=std::atan(mpc.wheelbase*path.kappa[i-1]/mpc.steering_curvature_gain);
            const double after=std::atan(mpc.wheelbase*path.kappa[i]/mpc.steering_curvature_gain);
            const double rate=std::fabs(after-before)/planner.sample_dt();peak=std::max(peak,rate);
            // These recorded departures remain below the actual Controller's
            // 5m/s knee, where its configured maximum rate applies directly.
            steerable &= std::max(std::fabs(path.v[i-1]),std::fabs(path.v[i]))<=
                mpc.steering_rate_reference_speed_mps && rate<=mpc.steering_rate_max+1e-9;
        }
        std::cout<<"[STEERING] peak_rate="<<peak<<'\n';
        check(!ok||steerable,"accepted departure respects the actual Controller steering-rate contract");
        if(ok&&path.x.size()>1)++moving;
        check(ok||(path.x.empty()&&planner.last_failure_reason()!=
            FrenetPlanner::PlanFailureReason::TRANSIENT_GENERATION_FAILURE),
            "a rejected departure cannot reuse an older moving trajectory");
    }
    check(moving>=2,"recorded departures retain moving alternatives instead of unconditionally holding");
    {
        // Recorded stationary KEEP failure: 244 identical-position cycles,
        // zero objects, exterior overlap about 0.0182 m at route s=253.23 m.
        CartesianState ego{-71.99312931883362,-215.5578657260853,
            1.2294611194687775,0,0,0};
        FrenetPlanner planner;check(planner.Init(config.string()),"initial-overlap regression initializes");
        auto exterior=planner.vehicle_shape();exterior.width=std::max(exterior.width,exterior.road_outer_width);
        const double overlap=road.FootprintOverlapDepth(ego.x,ego.y,ego.yaw,exterior);
        check(!road.FootprintInside(ego.x,ego.y,ego.yaw,exterior)&&overlap>0&&overlap<.035,
            "recorded exterior really has the small initial overlap that formerly prevented generation");
        PlannerCommand keep;keep.mode=LANE_KEEPING;keep.has_target_speed=true;keep.target_speed=57.5/3.6;
        planner.SetObjectObservationTime(400,400);CartesianPath path;
        const bool ok=planner.Plan(ego,{},keep,path);
        const auto recovery=road.EvaluateRecoveryPath(path,exterior,planner.sample_dt());
        check(ok&&path.x.size()>1&&recovery.drivable&&recovery.initial_overlap_m>0&&
              road.FootprintInside(path.x.back(),path.y.back(),path.yaw.back(),exterior),
            "original stopped pose generates a complete bounded return without moving the fixture");
        check(ok&&!path.v.empty()&&std::fabs(path.v.front())<1e-6&&path.v.back()>0&&
              PathObeysSteeringRate(path,planner.sample_dt(),mpc.wheelbase,steering_limits),
            "recovery preserves standstill and the existing Controller steering limits");
        auto tracking=ego;double settled=0;bool resumed=ok;
        auto settled_shape=exterior;settled_shape.width+=.10;
        settled_shape.length+=.10;settled_shape.rear_overhang+=.05;
        for(int step=0;step<60&&resumed&&settled<.5;++step){
            planner.SetObjectObservationTime(401+step*.1,401+step*.1);
            CartesianPath next;resumed=planner.Plan(tracking,{},keep,next)&&next.x.size()>1;
            if(!resumed){std::cout<<"[RECOVERY] step="<<step<<" speed="<<tracking.v
                <<" inside="<<road.FootprintInside(tracking.x,tracking.y,tracking.yaw,exterior)
                <<" overlap="<<road.FootprintOverlapDepth(tracking.x,tracking.y,tracking.yaw,exterior)
                <<" reason="<<int(planner.last_failure_reason())<<'\n';break;}
            const double movement=std::hypot(next.x[1]-tracking.x,next.y[1]-tracking.y);
            tracking={next.x[1],next.y[1],next.yaw[1],next.kappa[1],next.v[1],next.a[1]};
            settled=road.FootprintInside(tracking.x,tracking.y,tracking.yaw,settled_shape)?settled+movement:0;
        }
        check(resumed&&settled>=.5,"ideal prefix tracking replans through complete recovery without a STOP gap");
        planner.SetObjectObservationTime(410,410);CartesianPath settled_path;
        check(resumed&&planner.Plan(tracking,{},keep,settled_path)&&
              road.EvaluatePath(settled_path,exterior,planner.sample_dt()).drivable,
            "settled departure hands back to strict road containment");
        TrackedObject barrier{};barrier.id=909;barrier.type=3;
        barrier.motion_state=ObjectMotionState::LOW_THREAT;
        barrier.x=ego.x;barrier.y=ego.y;barrier.width=barrier.length=1;barrier.height=1;
        barrier.geometry_valid=barrier.z_valid=true;
        for(int i=0;i<8;++i){const double x[4]={-.5,.5,.5,-.5},y[4]={-.5,-.5,.5,.5};
            barrier.corners[i]={ego.x+x[i%4],ego.y+y[i%4],double(i/4)};}
        FrenetPlanner obstructed;check(obstructed.Init(config.string()),"obstructed departure initializes independently");
        obstructed.SetObjectObservationTime(400,400);
        path={};const bool blocked=obstructed.Plan(ego,{barrier},keep,path);
        check(!blocked&&path.x.empty()&&obstructed.last_failure_reason()==FrenetPlanner::PlanFailureReason::COLLISION_BLOCKED,
            "initial-boundary recovery cannot waive a physical obstacle collision");
    }
    {
        // The three new reports: two larger KEEP departures and the ordinary
        // STOP whose old first sample rotated the body by about 37 degrees.
        const CartesianState recorded[]={
            {-60.95011186418934,-174.3739479970567,1.2727066809258276,0,0,0},
            {-73.60611624491803,-220.69647325733857,1.1909298815080442,0,0,0},
            {-60.340885646314675,-172.60027045877567,1.2727412820788093,0,
                .09764309311456462,.6513196390120068}};
        for(int index=0;index<3;++index) {
            FrenetPlanner planner;check(planner.Init(config.string()),"reported partial departure initializes");
            auto exterior=planner.vehicle_shape();exterior.width=std::max(exterior.width,exterior.road_outer_width);
            const auto ego=recorded[index];
            check(!road.FootprintInside(ego.x,ego.y,ego.yaw,exterior)&&
                road.IntersectsDrivable(MakeEgoFootprint(ego,exterior,0),true),
                "reported pose is partially outside, not fully separated");
            PlannerCommand command;command.mode=index==2?STOP:LANE_KEEPING;
            command.has_target_speed=true;command.target_speed=index==0||index==2?25./3.6:57.5/3.6;
            if(index==2)command.stop_position=FindClosestS(planner.reference(),
                -60.0713284274163,-160.3874670883239,nullptr,100);
            planner.SetObjectObservationTime(500,500);CartesianPath path;
            const bool ok=planner.Plan(ego,{},command,path);
            check(ok&&path.x.size()>1&&road.EvaluateDepartureRecoveryPath(
                path,exterior,planner.sample_dt(),true).drivable,
                "reported pose produces a complete departure without the old overlap cutoffs");
            check(ok&&std::hypot(path.x.front()-ego.x,path.y.front()-ego.y)<1e-6&&
                std::fabs(std::remainder(path.yaw.front()-ego.yaw,2*std::acos(-1.0)))<1e-6&&
                PathObeysSteeringRate(path,planner.sample_dt(),mpc.wheelbase,steering_limits),
                "departure starts at the measured pose and obeys Controller steering slew");
            if(index==2) {
                double hint=FindClosestS(planner.reference(),ego.x,ego.y,nullptr,100);bool before=true;
                for(std::size_t i=0;i<path.x.size();++i){hint=FindClosestS(planner.reference(),
                    path.x[i],path.y[i],&hint,5);before&=hint<=command.stop_position+1e-6;}
                check(ok&&before&&std::fabs(path.v.back())<1e-6,
                    "partial STOP retains its original frontier and reaches zero speed");
            }
            auto tracking=ego;auto settled_shape=exterior;settled_shape.width+=.10;
            settled_shape.length+=.10;settled_shape.rear_overhang+=.05;
            double settled=0;bool progress=ok;
            for(int step=0;step<180&&progress&&settled<.5;++step) {
                planner.SetObjectObservationTime(501+step*.1,501+step*.1);
                CartesianPath next;progress=planner.Plan(tracking,{},command,next)&&next.x.size()>1;
                if(!progress)break;
                const double movement=std::hypot(next.x[1]-tracking.x,next.y[1]-tracking.y);
                tracking={next.x[1],next.y[1],next.yaw[1],next.kappa[1],next.v[1],next.a[1]};
                settled=road.FootprintInside(tracking.x,tracking.y,tracking.yaw,settled_shape)
                    ? settled+movement:0;
            }
            check(progress&&settled>=.5,"reported departure replans until settled without a STOP gap");
            if(index==1) {
                FrenetPlanner reversed;check(reversed.Init(config.string()),"opposite-heading departure initializes");
                auto backward=ego;backward.yaw+=std::acos(-1.0);
                reversed.SetObjectObservationTime(600,600);CartesianPath rejected;
                check(!reversed.Plan(backward,{},command,rejected)&&rejected.x.empty()&&
                    reversed.last_failure_reason()==FrenetPlanner::PlanFailureReason::ACTUATION_BLOCKED,
                    "recovery cannot fabricate a forward-facing body for an opposite heading or reuse an old path");
            }
        }
    }
    {
        const auto e=fixture["frames"][0]["ego"];
        CartesianState crossed{e["x"].as<double>()-10000,e["y"].as<double>(),
            e["yaw"].as<double>()+std::acos(-1.0)/2,0,0,0};
        FrenetPlanner planner;
        check(planner.Init(config.string()),"road-rejection planner initializes");
        auto exterior=planner.vehicle_shape();exterior.width=fixture["road_outer_width"].as<double>();
        check(!road.FootprintInside(crossed.x,crossed.y,crossed.yaw,exterior),
            "the fully outside exterior is not covered by departure recovery");
        PlannerCommand keep;keep.mode=LANE_KEEPING;keep.has_target_speed=true;keep.target_speed=10;
        planner.SetObjectObservationTime(200,200);
        CartesianPath path;const bool ok=planner.Plan(crossed,{},keep,path);
        check(!ok&&path.x.empty()&&planner.last_failure_reason()==
            FrenetPlanner::PlanFailureReason::ROAD_BOUNDARY_BLOCKED,
            "road rejection is explicit, empty, and ineligible for transient path reuse");
    }
    {
        auto limited=YAML::Clone(cfg);
        limited["planner"]["steering_profile"]["max_rate"]=1e-9;
        limited["planner"]["steering_profile"]["min_rate"]=1e-9;
        const auto file=std::filesystem::temp_directory_path()/"asmc_departure_actuation_blocked.yaml";
        {std::ofstream out(file);out<<limited;}
        FrenetPlanner planner;check(planner.Init(file.string()),"bounded-actuator test planner initializes");
        const auto e=fixture["frames"][0]["ego"];
        CartesianState ego{e["x"].as<double>(),e["y"].as<double>(),e["yaw"].as<double>(),0,0,0};
        PlannerCommand keep;keep.mode=LANE_KEEPING;keep.has_target_speed=true;keep.target_speed=10;
        planner.SetObjectObservationTime(300,300);CartesianPath path;
        const bool ok=planner.Plan(ego,{},keep,path);
        check(!ok&&path.x.empty()&&planner.last_failure_reason()==
            FrenetPlanner::PlanFailureReason::ACTUATION_BLOCKED,
            "actuator-infeasible departure cannot reuse an older moving trajectory");
    }
    return failures?1:0;
}
