// Real ProductionRuntime, timers, ROS transport and frozen command/feedback.
// An ideal trajectory follower supplies poses; this is not an MPC/MORAI test.
#include "Planning/ProductionRuntime.hpp"
#include <yaml-cpp/yaml.h>
#include <ros/master.h>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>

int RunHighwayRuntimeTest() {
    if(!ros::master::check()){std::cerr<<"ROS master required\n";return 2;}
    const auto temporary=std::filesystem::temp_directory_path()/
        ("asmc_highway_runtime_"+std::to_string(ros::WallTime::now().toNSec()));
    std::filesystem::create_directories(temporary);
    try {
        const auto original=YAML::LoadFile(FRENET_TEST_CONFIG);
        FrenetPlanner probe;std::string error;
        if(!probe.Init(FRENET_TEST_CONFIG)||!probe.EnableHighwayMerge(error))throw std::runtime_error(error);
        const auto route=probe.highway_route();
        // A controlled Behavior map subset uses the exact production HD-map
        // centerlines. Its route is deliberately unsignalized and empty.
        const std::vector<std::string> links={"A2256W000411","A2256W000420","A2256W000408","A2256W000445","A2256W000153",
            "A2256W000409","A2256W000434","A2256W000422"};
        YAML::Node map,ids;
        for(std::size_t i=0;i<links.size();++i) {
            const auto* lane=route->map->lane("lane:"+links[i]);
            if(!lane)throw std::runtime_error("missing controlled map lane");
            YAML::Node value;value["idx"]=links[i];value["from_node_idx"]=std::to_string(i);
            value["to_node_idx"]=std::to_string(i+1);value["max_speed"]=50;
            for(const auto& p:lane->centerline) {
                YAML::Node xy;xy.push_back(p.x);xy.push_back(p.y);value["points"].push_back(xy);
            }
            map.push_back(value);if(i<5)ids["route"]["links"].push_back(links[i]);
        }
        {std::ofstream f(temporary/"links.yaml");f<<map;}
        {std::ofstream f(temporary/"nodes.yaml");f<<"[]\n";}
        {std::ofstream f(temporary/"route.yaml");f<<ids;}
        ros::NodeHandle nh;
        nh.setParam("/frenet_planner/planner/config_path",std::string(FRENET_TEST_CONFIG));
        nh.setParam("/frenet_planner/highway_merge_enabled",true);
        nh.setParam("/behavior_planner/interface_version","v2");
        nh.setParam("/behavior_planner/map/link_set_path",(temporary/"links.yaml").string());
        nh.setParam("/behavior_planner/map/node_set_path",(temporary/"nodes.yaml").string());
        nh.setParam("/behavior_planner/route/route_links_path",(temporary/"route.yaml").string());
        nh.setParam("/behavior_planner/objects/input_type","detected");
        nh.setParam("/behavior_planner/topics/objects","/perception/objects");
        std::mutex mutex;asmc_msgs::Trajectory trajectory;
        bool saw_prepare=false,saw_ready=false,saw_execute=false,saw_commit=false,saw_complete=false,saw_keep=false;
        bool wrong_merge_type=false;unsigned valid_paths=0;
        const auto commands=nh.subscribe<asmc_msgs::BehaviorCommand>("/behavior/command_v2",10,
            [&](const asmc_msgs::BehaviorCommand::ConstPtr& c){
                std::lock_guard<std::mutex> lock(mutex);
                if(c->maneuver_type==asmc_msgs::BehaviorCommand::MERGE) {
                    wrong_merge_type=wrong_merge_type||c->merge_type!=asmc_msgs::BehaviorCommand::MERGE_HIGHWAY;
                    saw_prepare=saw_prepare||c->execution_request==asmc_msgs::BehaviorCommand::PREPARE;
                    saw_execute=saw_execute||c->execution_request==asmc_msgs::BehaviorCommand::EXECUTE;
                }
                if(saw_complete&&c->maneuver_type==asmc_msgs::BehaviorCommand::KEEP)saw_keep=true;
            });
        const auto feedback=nh.subscribe<asmc_msgs::FrenetFeedback>("/planning/frenet_feedback_v2",20,
            [&](const asmc_msgs::FrenetFeedback::ConstPtr& f){
                std::lock_guard<std::mutex> lock(mutex);
                saw_ready=saw_ready||f->execution_state==asmc_msgs::FrenetFeedback::READY;
                saw_commit=saw_commit||f->execution_state==asmc_msgs::FrenetFeedback::COMMITTED||
                    f->execution_state==asmc_msgs::FrenetFeedback::EXECUTING;
                saw_complete=saw_complete||f->execution_state==asmc_msgs::FrenetFeedback::COMPLETED;
            });
        const auto paths=nh.subscribe<asmc_msgs::Trajectory>("/planning/trajectory",2,
            [&](const asmc_msgs::Trajectory::ConstPtr& p){
                if(!p->valid||p->points.size()<2)return;
                std::lock_guard<std::mutex> lock(mutex);trajectory=*p;++valid_paths;
            });
        auto state_pub=nh.advertise<asmc_msgs::VehicleState>("/vehicle/state",2);
        auto objects_pub=nh.advertise<planning_inputs::DetectedObjectArray>("/perception/objects",2);
        asmc_planner::ProductionRuntime runtime;runtime.start();
        ros::AsyncSpinner transport(1);transport.start();
        const auto p=Interpolate(route->reference,route->zones[0].entry_s-10);
        asmc_msgs::VehicleState vehicle;vehicle.header.frame_id="map";
        vehicle.valid=true;vehicle.source=asmc_msgs::VehicleState::SOURCE_GPS_IMU;
        vehicle.x=p.x;vehicle.y=p.y;vehicle.yaw=p.theta;vehicle.speed=12;
        bool success=false;
        ros::WallRate rate(20);
        for(int tick=0;tick<500&&ros::ok();++tick) {
            const auto now=ros::Time::now();
            {
                std::lock_guard<std::mutex> lock(mutex);
                if(trajectory.valid&&trajectory.dt>0&&trajectory.points.size()>1) {
                    const double t=std::max(0.0,(now-trajectory.header.stamp).toSec());
                    const double position=t/trajectory.dt;
                    const auto lo=std::min(trajectory.points.size()-2,static_cast<std::size_t>(position));
                    const double w=std::clamp(position-lo,0.0,1.0);
                    const auto& a=trajectory.points[lo];const auto& b=trajectory.points[lo+1];
                    vehicle.x=a.x+w*(b.x-a.x);vehicle.y=a.y+w*(b.y-a.y);
                    vehicle.yaw=a.yaw+w*std::remainder(b.yaw-a.yaw,2*M_PI);
                    vehicle.speed=a.velocity+w*(b.velocity-a.velocity);
                    vehicle.acceleration=a.acceleration+w*(b.acceleration-a.acceleration);
                    vehicle.yaw_rate=vehicle.speed*(a.curvature+w*(b.curvature-a.curvature));
                }
                success=saw_prepare&&saw_ready&&saw_execute&&saw_commit&&saw_complete&&saw_keep&&!wrong_merge_type;
            }
            if(success)break;
            vehicle.header.stamp=now;state_pub.publish(vehicle);
            planning_inputs::DetectedObjectArray objects;objects.header=vehicle.header;objects_pub.publish(objects);
            rate.sleep();
        }
        transport.stop();
        std::cout<<"prepare="<<saw_prepare<<" ready="<<saw_ready<<" execute="<<saw_execute
            <<" committed="<<saw_commit<<" complete="<<saw_complete<<" keep="<<saw_keep
            <<" wrong_type="<<wrong_merge_type<<" valid_paths="<<valid_paths<<'\n';
        std::filesystem::remove_all(temporary);
        return success?0:1;
    } catch(const std::exception& e) {
        std::cerr<<e.what()<<'\n';std::filesystem::remove_all(temporary);return 1;
    }
}

TEST(HighwayRuntime, PrepareExecuteCompleteAndKeep) { EXPECT_EQ(0,RunHighwayRuntimeTest()); }
int main(int argc,char** argv) {
    testing::InitGoogleTest(&argc,argv);
    ros::init(argc,argv,"highway_runtime_test");
    return RUN_ALL_TESTS();
}
