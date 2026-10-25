// Test-only JSONL adapter for the ROS-independent core; not installed as a node.
#include <asmc_lidar/filtering/hd_map_audit.hpp>
#include <yaml-cpp/yaml.h>
#include <iomanip>
#include <cmath>
#include <iostream>
#include <limits>
namespace f=asmc::lidar::filtering;
template<class T>T Get(const YAML::Node& n,const char* key,T fallback){try{return n[key].as<T>();}catch(const YAML::Exception&){return fallback;}}
int main(int argc,char**argv){
 try{
  if(argc!=3)throw std::runtime_error("manifest and expected SHA required");
  const f::HdMapAudit audit(argv[1],argv[2]);std::string line;
  std::cout<<std::setprecision(17);
  while(std::getline(std::cin,line)){
   const auto root=YAML::Load(line), frame=root["frame"],object=root["object"],pose=frame["pose"];
   f::AuditFrame in;in.frame_id=Get<std::string>(frame,"frame_id","");in.map_id=Get<std::string>(frame,"map_id","");
   in.status=Get<int>(frame,"status",0);in.stamp_ns=Get<std::int64_t>(frame,"stamp_ns",0);
   if(frame["map_origin"]&&frame["map_origin"].size()==3)for(int i=0;i<3;++i)in.map_origin[i]=frame["map_origin"][i].as<double>();
   else in.map_origin.fill(std::numeric_limits<double>::quiet_NaN());
   in.pose_valid=Get<bool>(pose,"valid",false);in.pose_stamp_ns=Get<std::int64_t>(pose,"at_stamp_ns",0);
   in.position_error_verified=Get<bool>(pose,"position_error_verified",false);
   in.position_error_bound_m=Get<double>(pose,"position_error_bound_m",NAN);
   in.source_age_ms=Get<double>(pose,"source_age_ms",NAN);in.mission_scope_verified=Get<bool>(frame,"mission_scope_verified",false);
   f::AuditObject obj;obj.track_id=Get<int>(object,"track_id",-1);obj.observation_state=Get<std::string>(object,"observation_state","unknown");
   if(object["corners"]&&object["corners"].IsSequence())for(const auto& c:object["corners"]){
    f::Point p{};if(c.IsSequence()&&c.size()==3){for(int i=0;i<3;++i)try{p[i]=c[i].as<double>();}catch(...){p[i]=NAN;}}
    else p.fill(NAN);obj.corners.push_back(p);
   }
   f::WriteAuditDecision(std::cout,audit.Evaluate(in,obj));std::cout<<'\n'<<std::flush;
  }
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
