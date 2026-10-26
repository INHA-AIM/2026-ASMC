// Offline causal diagnosis of received vs accepted IMU. No point cloud needed.
#include <asmc_lidar/ros/runtime.hpp>
#include <asmc_lidar/ros/ros_inputs.hpp>
#include <asmc_lidar/ros/scan_trace.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
namespace pl=asmc::lidar::pipeline;
namespace rt=asmc::lidar::runtime;
int main(int argc,char** argv){try{
 if(argc!=4)throw std::runtime_error("imu_history_probe BAG CONFIG NEW.jsonl");
 if(std::ifstream(argv[3]).good())throw std::runtime_error("output exists");
 const std::string file=argv[2];auto config=rt::LoadRuntimeConfig(file,file.substr(0,file.rfind("/config/")));
 pl::PoseHistory history(config.pose);rosbag::Bag bag(argv[1],rosbag::bagmode::Read);
 rosbag::View view(bag,rosbag::TopicQuery({"/imu","/Ego_topic","/migration/lidar/trace"}));
 std::ofstream out(argv[3]);if(!out)throw std::runtime_error("output open failed");
 out<<std::setprecision(17);std::size_t rejected=0;std::uint64_t first=0;
 for(const auto& m:view){const auto receipt=m.getTime().toNSec();if(!first)first=receipt;
  const auto logical=receipt-first+1000000000ULL;
  if(m.getTopic()=="/imu"){
   auto input=rt::ImuInputFromMsg(*m.instantiate<sensor_msgs::Imu>());std::string why;
   const bool ok=history.AddImu(pl::SourceStampMs(input.stamp_ns,receipt),pl::ImuFromInput(input),&why,input.stamp_ns,logical);
   if(!ok)++rejected;
  }else if(m.getTopic()=="/Ego_topic"){
   auto input=rt::CanInputFromMsg(*m.instantiate<morai_msgs::EgoVehicleStatus>());
   history.AddCan(pl::SourceStampMs(input.stamp_ns,receipt),pl::CanFromInput(input),nullptr,input.stamp_ns,logical);
  }else{
   const auto trace=YAML::Load(m.instantiate<std_msgs::String>()->data);
   const auto scan=trace["stamp_ns"].as<std::uint64_t>();
   pl::LocalInputSelection s;pl::LocalState state;std::string why;
   const bool ok=history.WaitForLocalLookupNs(scan,0.,&state,&s,&why);
   out<<"{\"stamp_ns\":"<<scan<<",\"valid\":"<<(ok?"true":"false")<<",\"reason\":";
   rt::trace::Str(out,why);out<<",\"recorded_reason\":";rt::trace::Str(out,trace["reason"].as<std::string>());
   out<<",\"selected_imu_ns\":"<<s.imu_stamp_ns<<",\"latest_seen_ns\":"<<s.imu_latest_seen_ns
      <<",\"latest_invalid_ns\":"<<s.imu_invalid_ns<<",\"rejected\":"<<rejected<<",\"rejection_reason\":";
   rt::trace::Str(out,s.imu_rejection_reason);out<<"}\n";
  }
 }
 return out?0:1;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
