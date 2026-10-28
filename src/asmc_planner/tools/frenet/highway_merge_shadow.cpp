// Recorded ego motion is fixed. This checks actual PREPARE candidates and
// reasons against recorded objects; it never publishes to a vehicle.
// Optional core-only commitment exercises recovery on fixed recorded ego motion.
#include "Frenet/frenet_planner.hpp"
#include "Execution/HighwayReferenceExecution.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <ctime>

namespace pt=boost::property_tree;
int main(int argc,char** argv) {
    bool commit_on_ready=false,reference_transition=false;
    for(int i=3;i<argc;++i) {
        const std::string option=argv[i];
        if(option=="--commit-on-ready")commit_on_ready=true;
        else if(option=="--reference-handover")reference_transition=true;
        else {std::cerr<<"unknown option: "<<option<<'\n';return 2;}
    }
    if(argc<3){std::cerr<<"usage: highway_merge_shadow FRAMES_JSON PLANNER_YAML [--commit-on-ready] [--reference-handover]\n";return 2;}
    try {
        FrenetPlanner planner;std::string error;
        if(!planner.Init(argv[2])||!planner.EnableHighwayMerge(error))throw std::runtime_error(error);
        asmc_planner::execution::ManeuverState state;
        std::unique_ptr<asmc_planner::lane_reference::HighwayReferenceExecution> references;
        if(reference_transition)references=std::make_unique<asmc_planner::lane_reference::HighwayReferenceExecution>(planner,argv[2],state);
        pt::ptree frames;pt::read_json(argv[1],frames);
        int previous_zone=-1;HighwayMergeApproach approach_memory;
        for(const auto& row:frames) {
            const auto& f=row.second;const auto& e=f.get_child("ego");
            const double speed=e.get<double>("speed");
            CartesianState ego{e.get<double>("x"),e.get<double>("y"),e.get<double>("yaw"),
                speed>.5?e.get<double>("yaw_rate")/speed:0,speed,e.get<double>("acceleration")};
            const auto route=planner.highway_route();const auto approach=UpdateHighwayApproach(*route,ego,approach_memory);
            if(!approach.valid||!approach.approach)continue;
            if(previous_zone!=static_cast<int>(approach.zone_index))planner.ResetHighwayMerge();
            previous_zone=approach.zone_index;
            std::vector<TrackedObject> objects;
            for(const auto& item:f.get_child("objects")) {
                const auto& p=item.second;TrackedObject obj;
                obj.id=p.get<int>("id");obj.type=p.get<int>("type");
                obj.motion_state=static_cast<ObjectMotionState>(p.get<int>("motion_state"));
                obj.x=p.get<double>("x");obj.y=p.get<double>("y");obj.z=p.get<double>("z");
                obj.heading=p.get<double>("yaw");obj.vx=p.get<double>("vx");obj.vy=p.get<double>("vy");
                obj.speed=std::hypot(obj.vx,obj.vy);obj.yaw_rate=p.get<double>("yaw_rate");
                obj.width=p.get<double>("width");obj.length=p.get<double>("length");obj.height=p.get<double>("height");
                std::size_t k=0;
                for(const auto& corner:p.get_child("corners")) {
                    if(k>=8)throw std::runtime_error("too many OBB corners");
                    auto v=corner.second.begin();const double x=v++->second.get_value<double>();
                    const double y=v++->second.get_value<double>();obj.corners[k++]={x,y,v->second.get_value<double>()};
                }
                obj.geometry_valid=k==8&&geometry::ValidateCorners8(obj.corners).valid;obj.z_valid=true;
                objects.push_back(obj);
            }
            PlannerCommand command;command.mode=MERGE;command.merge_type=MergeType::HIGHWAY;
            command.source_link_id=approach.source_link;command.target_link_id=approach.target_link;
            command.has_target_speed=true;command.target_speed=approach.speed_cap_mps;
            CartesianPath path;FrenetPlanner::HighwayMergeResult result;
            const auto start=std::chrono::steady_clock::now();
            const auto cpu_start=std::clock();
            bool ok=false;
            if(f.get<std::string>("object_frame")=="map"&&
               f.get<double>("object_receipt_age_s")<=route->config.source_max_age_s)
                ok=planner.PlanHighwayMerge(ego,objects,command,f.get<double>("t"),
                    f.get<double>("object_stamp"),path,result,f.get<double>("ego_stamp"),
                    references?&references->target(approach.zone_index).line():nullptr,
                    references?&planner.reference():nullptr);
            else {result.detail="invalid_or_stale_recorded_object_input";planner.RevokeHighwayMergeReadiness();}
            if(commit_on_ready&&ok&&result.ready&&planner.CanCommitHighwayMerge(result))
                planner.ApplyHighwayMergeCommit(result);
            const double milliseconds=std::chrono::duration<double,std::milli>(
                std::chrono::steady_clock::now()-start).count();
            const double cpu_milliseconds=1000.0*(std::clock()-cpu_start)/CLOCKS_PER_SEC;
            std::cout<<std::setprecision(12)<<"HIGHWAY {\"t\":"<<f.get<double>("t")
                <<",\"s\":"<<approach.route_s<<",\"zone\":"<<approach.zone_index
                <<",\"maintenance\":"<<ok<<",\"candidate\":"<<result.candidate_valid
                <<",\"ready\":"<<result.ready<<",\"generated\":"<<result.generated
                <<",\"committed\":"<<planner.highway_merge_committed()
                <<",\"reference_transition\":"<<reference_transition
                <<",\"entry_time_s\":"<<result.check.entry_time_s
                <<",\"clear_time_s\":"<<result.check.clear_time_s
                <<",\"failure_reason\":"<<static_cast<int>(planner.last_failure_reason())
                <<",\"blocker\":"<<result.check.blocking_object_id
                <<",\"milliseconds\":"<<milliseconds<<",\"cpu_milliseconds\":"<<cpu_milliseconds
                <<",\"reason\":\""<<result.detail<<"\"}\n";
        }
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
    return 0;
}
