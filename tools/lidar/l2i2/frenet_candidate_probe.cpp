// Isolated consumer acceptance probe: link the real Frenet converter without
// remapping /perception/objects or launching the planner's control outputs.
#include <asmc_msgs/DetectedObjectArray.h>
#include <ros/ros.h>
#include <Runtime/message_converter.hpp>
#include <iostream>
#include <map>
#include <string>
#include <vector>

int main(int argc,char** argv){
  ros::init(argc,argv,"asmc_lidar_frenet_candidate_probe");
  ros::NodeHandle nh;
  std::map<std::string,int> statuses;
  int messages=0,objects=0;
  auto subscriber=nh.subscribe<asmc_msgs::DetectedObjectArray>(
      "/perception/objects_lidar_candidate",100,
      [&](const asmc_msgs::DetectedObjectArray::ConstPtr& input){
        std::vector<TrackedObject> output;
        const auto result=frenet_planner_ros::ConvertDetectedObjects(*input,output);
        ++statuses[frenet_planner_ros::ConversionStatusName(result)];
        ++messages;objects+=output.size();
      });
  const double seconds=argc>1?std::stod(argv[1]):20.;
  const auto end=ros::WallTime::now()+ros::WallDuration(seconds);
  ros::Rate rate(200);
  while(ros::ok()&&ros::WallTime::now()<end){ros::spinOnce();rate.sleep();}
  std::cout<<"frenet_actual_converter messages="<<messages<<" objects="<<objects;
  for(const auto& [status,count]:statuses)std::cout<<" "<<status<<"="<<count;
  std::cout<<'\n';
  return messages>0&&statuses.size()==1&&statuses.count("OK")?0:1;
}
