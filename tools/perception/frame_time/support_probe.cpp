// C0-only exact support sensitivity. Same Runtime inputs/order, no source edits.
#include <asmc_lidar/ros/runtime.hpp>
#include <asmc_lidar/ros/ros_inputs.hpp>
#include <asmc_lidar/ros/scan_trace.hpp>
#include <asmc_lidar/filtering/delivery_preview.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <morai_msgs/GPSMessage.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
using namespace asmc::lidar;
namespace rt=asmc::lidar::runtime;
int main(int argc,char**argv){try{
 if(argc!=5)throw std::runtime_error("support_probe BAG CONFIG PLANNING.jsonl NEW.jsonl");
 if(std::ifstream(argv[4]).good())throw std::runtime_error("output exists");
 ros::Time::init();std::srand(0);
 const std::string config_path=argv[2];const auto at=config_path.rfind("/config/");
 auto cfg=rt::LoadRuntimeConfig(config_path,config_path.substr(0,at));rt::FinalizeRuntimeIdentity(cfg);
 rt::LidarRuntime runtime(cfg,nullptr,nullptr);rt::TypedOutput before_delivery;
 rosbag::Bag bag(argv[1],rosbag::bagmode::Read);const auto first=rosbag::View(bag).getBeginTime().toNSec();
 rosbag::View view(bag,rosbag::TopicQuery({"/imu","/gps","/Ego_topic","/velodyne_points"}));
 std::ifstream states(argv[3]);std::ofstream file(argv[4]);if(!states||!file)throw std::runtime_error("file open");
 file<<std::setprecision(17);std::uint64_t seq=0;
 for(const auto& m:view){
  const auto arrival=m.getTime().toNSec();const Ns logical=arrival-first+1000000000ULL;ros::Time::setNow(m.getTime());
  if(m.getTopic()=="/imu")runtime.OnImu(rt::ImuInputFromMsg(*m.instantiate<sensor_msgs::Imu>()),logical,arrival);
  else if(m.getTopic()=="/gps")runtime.OnGps(rt::GpsInputFromMsg(*m.instantiate<morai_msgs::GPSMessage>()),logical,arrival);
  else if(m.getTopic()=="/Ego_topic")runtime.OnCan(rt::CanInputFromMsg(*m.instantiate<morai_msgs::EgoVehicleStatus>()),logical,arrival);
  else{
   std::string line;if(!std::getline(states,line))throw std::runtime_error("missing state");const auto state=YAML::Load(line);
   const auto msg=m.instantiate<sensor_msgs::PointCloud2>();const auto stamp=rt::StampNs(msg->header.stamp);++seq;
   if(state["seq"].as<std::uint64_t>()!=seq||state["scan_ns"].as<Ns>()!=stamp||state["arrival_ns"].as<std::uint64_t>()!=arrival)throw std::runtime_error("scan identity mismatch");
   pipeline::ScanInput input;input.points=rt::CloudFromMsg(*msg);input.scan_sequence=seq;input.generation=1;
   input.time={stamp,logical,logical,"ros",cfg.stamp_provenance,"steady"};std::string why;
   const bool accepted=runtime.AcceptScan(stamp,input.points->size(),&why);rt::ScanOutput out;
   if(accepted){rt::ScanTimes t;t.receipt_ns=t.processing_ns=t.decision_ns=t.steady_now_ns=logical;t.ros_now_ns=arrival;out=runtime.ProcessScan(input,0,t);}
   file<<"{\"seq\":"<<seq<<",\"typed_local\":";rt::trace::Array(file,out.arrays.accepted,out.arrays.local);
   file<<",\"typed_map\":";rt::trace::Array(file,out.arrays.accepted,out.arrays.map);file<<",\"objects\":[";
   if(out.result.processed&&out.result.context.map_pose.valid&&state["valid"].as<bool>()&&cfg.hd_map_audit){
    const auto& ctx=out.result.context;const auto base=base_from_input(ctx);if(!base)throw std::runtime_error("base transform");
    const Pose original=ctx.map_pose.map_from_base;Pose alternative=original;
    alternative.translation().x()=state["x"].as<double>();alternative.translation().y()=state["y"].as<double>();
    const double yaw=std::atan2(original.linear()(1,0),original.linear()(0,0));
    alternative.linear()=rpy(0,0,state["yaw"].as<double>()-yaw)*original.linear();
    const auto arrays=before_delivery.Observed(out.result,out.result.objects);bool comma=false;
    for(const auto& obj:arrays.local.objects){
     auto source=std::find_if(out.result.objects.begin(),out.result.objects.end(),[&](const auto&t){return t.tracking.s32_Id==obj.track_id;});
     if(source==out.result.objects.end())throw std::runtime_error("support source absent");
     if(comma)file<<',';comma=true;file<<"{\"track_id\":"<<obj.track_id<<",\"point_count\":"<<source->detection.pcl_ClusterPoints.size();
     for(const auto& candidate:{std::make_pair("original",original),std::make_pair("alternative",alternative)}){
      filtering::PointEvidence evidence;const Pose map_from_level=candidate.second* *base;
      for(const auto&p:source->detection.pcl_ClusterPoints){const Vec raw(p.x,p.y,p.z),map=map_from_level*raw;const double z=(*base*raw).z();cfg.hd_map_audit->Accumulate(evidence,{map.x(),map.y(),map.z()},z);}
      filtering::AuditFrame frame;frame.frame_id="map";frame.status=1;frame.map_id=cfg.hd_map_audit->map_id();
      for(int i=0;i<3;++i)frame.map_origin[i]=cfg.gps.map_origin[i];frame.pose_valid=true;frame.stamp_ns=frame.pose_stamp_ns=stamp;
      filtering::AuditObject obb;obb.track_id=obj.track_id;obb.observation_state="observed";
      double bottom=INFINITY;for(const auto&p:obj.obb_corners){Vec b(p.x,p.y,p.z);const Vec map=candidate.second*b;obb.corners.push_back({map.x(),map.y(),map.z()});bottom=std::min(bottom,p.z);}
      auto decision=cfg.hd_map_audit->Evaluate(frame,obb);
      // Isolate geometry: no invented camera protection or independent uncertainty bound.
      filtering::ApplyDeliveryPreview(decision,evidence,cfg.delivery_preview,true,"unknown",bottom);decision.comparison_ms=0;
      file<<",\""<<candidate.first<<"\":";filtering::WriteAuditDecision(file,decision);
     }file<<'}';
    }
   }file<<"]}\n";
  }
 }
 return file?0:1;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
