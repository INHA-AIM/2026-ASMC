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
        const auto evaluation=road.EvaluatePath(path,exterior,planner.sample_dt());
        std::cout<<"[FRAME] t="<<frame["elapsed_s"].as<double>()<<" v="<<ego.v
            <<" ok="<<ok<<" samples="<<path.x.size()<<" road="<<evaluation.drivable
            <<" overlap="<<evaluation.maximum_overlap_m<<'\n';
        check(!ok||evaluation.drivable,"accepted low-speed KEEP contains the entire calibrated exterior footprint");
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
        const auto e=fixture["frames"][0]["ego"];
        CartesianState crossed{e["x"].as<double>(),e["y"].as<double>(),
            e["yaw"].as<double>()+std::acos(-1.0)/2,0,0,0};
        FrenetPlanner planner;
        check(planner.Init(config.string()),"road-rejection planner initializes");
        auto exterior=planner.vehicle_shape();exterior.width=fixture["road_outer_width"].as<double>();
        check(!road.FootprintInside(crossed.x,crossed.y,crossed.yaw,exterior),
            "the rotated exterior actually crosses the road boundary");
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
