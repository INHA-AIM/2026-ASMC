#include "Frenet/frenet_planner.hpp"
#include "Execution/HighwayReferenceExecution.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>

int main(int argc,char** argv) {
    if(argc<2||argc>3) {
        std::cerr<<"usage: highway_merge_preflight PLANNER_YAML [REFERENCE_JSON]\n";return 2;
    }
    FrenetPlanner planner;std::string error;
    if(!planner.Init(argv[1])||!planner.EnableHighwayMerge(error)) {
        std::cerr<<"PREFLIGHT FAILED: "<<error<<'\n';return 1;
    }
    const auto route=planner.highway_route();
    try {
        asmc_planner::execution::ManeuverState state;
        asmc_planner::lane_reference::HighwayReferenceExecution references(planner,argv[1],state);
        for(std::size_t i=0;i<route->zones.size();++i)
            std::cout<<"HANDOVER PREFLIGHT "<<route->zones[i].config.name
                <<" target_length_m="<<references.target(i).line().length
                <<" max_curvature="<<references.target(i).maximumCurvature()<<'\n';
        const auto network=references.status().network;
        if(network) {
            std::cout<<"LANE_CHANGE PREFLIGHT paths="<<network->lanes.size()<<" enabled="<<network->policy.enabled<<'\n';
            for(const auto& lane:network->lanes)
                std::cout<<"  "<<lane->lanes().front()<<" -> "<<lane->lanes().back()
                    <<" length_m="<<lane->line().length<<" max_curvature="<<lane->maximumCurvature()<<'\n';
        }
    } catch(const std::exception& e) {
        std::cerr<<"PREFLIGHT FAILED reference handover: "<<e.what()<<'\n';return 1;
    }
    std::cout<<"PREFLIGHT OK map="<<route->map->identity().build_id
        <<" geometry="<<route->map->identity().geometry_sha256
        <<" zones="<<route->zones.size()<<" reference_length_m="<<route->reference.length<<'\n';
    for(const auto& z:route->zones)
        std::cout<<z.config.name<<" source_stop_rear_axle_s="<<z.source_stop_s
            <<" entry_s="<<z.entry_s<<" completion_rear_clear_s="
            <<z.config.completion_s+route->vehicle.rear_overhang<<'\n';
    if(argc==3) {
        std::ofstream f(argv[2]);if(!f){std::cerr<<"cannot write reference export\n";return 1;}
        f<<std::setprecision(17)<<"{\"points\":[";bool first=true;
        for(const auto& p:route->reference.points) {
            if(!first)f<<',';first=false;
            f<<'['<<p.x<<','<<p.y<<','<<p.s<<','<<p.theta<<','<<p.kappa<<']';
        }
        f<<"],\"zones\":[";first=true;
        for(const auto& z:route->zones) {
            if(!first)f<<',';first=false;
            f<<"{\"start_s\":"<<z.config.start_s<<",\"entry_s\":"<<z.entry_s
                <<",\"source_stop_s\":"<<z.source_stop_s<<",\"completion_s\":"<<z.config.completion_s<<'}';
        }
        f<<"]}\n";
    }
    return 0;
}
