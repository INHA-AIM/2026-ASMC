// JSONL in/out; actual coordinate core, not a second Python transformation implementation.
#include <asmc_lidar/core/frame_time.hpp>
#include <asmc_lidar/filtering/hd_map_audit.hpp>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <iomanip>
#include <iostream>
using namespace asmc::lidar;
Pose ReadPose(const YAML::Node& n) {
  if(n.size()!=16)throw std::runtime_error("pose needs 16 entries");
  Pose p=Pose::Identity();for(int r=0;r<4;++r)for(int c=0;c<4;++c)p.matrix()(r,c)=n[r*4+c].as<double>();
  if(!pose_valid(p))throw std::runtime_error("invalid pose");return p;
}
int main(int argc,char** argv){try {
 std::unique_ptr<filtering::HdMapAudit> audit; filtering::AuditFrame frame;
 if(argc==3){
  audit=std::make_unique<filtering::HdMapAudit>(argv[1],argv[2],.10);
  auto y=YAML::LoadFile(argv[1]);frame.frame_id="map";frame.map_id=y["map_id"].as<std::string>();
  for(int i=0;i<3;++i)frame.map_origin[i]=y["map_origin"][i].as<double>();
  frame.status=1;frame.pose_valid=true;frame.stamp_ns=frame.pose_stamp_ns=1;
 }else if(argc!=1)throw std::runtime_error("optional args: manifest SHA");
 std::string line;std::cout<<std::setprecision(17);
 while(std::getline(std::cin,line)) {
  const auto n=YAML::Load(line);const Pose original=ReadPose(n["map_from_base"]);
  Pose alternative=original;
  alternative.translation().x()=n["alternative"]["x"].as<double>();
  alternative.translation().y()=n["alternative"]["y"].as<double>();
  // Change only yaw/xy: preserve the original roll/pitch, z. Sensitivity, not a new pose producer.
  const double yaw=std::atan2(original.linear()(1,0),original.linear()(0,0));
  alternative.linear()=rpy(0,0,n["alternative"]["yaw"].as<double>()-yaw)*original.linear();
  std::vector<Point> points;for(const auto& p:n["points"])points.push_back({Vec(p[0].as<double>(),p[1].as<double>(),p[2].as<double>()),0});
  const auto a=transform_points(points,original),b=transform_points(points,alternative),back=transform_points(a,original.inverse());
  double err=0;for(size_t i=0;i<points.size();++i)err=std::max(err,(points[i].xyz-back[i].xyz).norm());
  std::cout<<"{\"roundtrip_m\":"<<err;
  for(const auto& entry:{std::make_pair("original",&a),std::make_pair("alternative",&b)}){
   std::cout<<",\""<<entry.first<<"\":[";for(size_t i=0;i<entry.second->size();++i){if(i)std::cout<<',';const auto& p=(*entry.second)[i].xyz;std::cout<<'['<<p.x()<<','<<p.y()<<','<<p.z()<<']';}std::cout<<']';}
  if(audit){
   for(const auto& entry:{std::make_pair("original_relation",&a),std::make_pair("alternative_relation",&b)}){
    filtering::AuditObject obj;obj.observation_state="observed";
    for(const auto& p:*entry.second)obj.corners.push_back({p.xyz.x(),p.xyz.y(),p.xyz.z()});
    std::cout<<",\""<<entry.first<<"\":";
    auto decision=audit->Evaluate(frame,obj);decision.comparison_ms=0;
    filtering::WriteAuditDecision(std::cout,decision);
   }
  }
  std::cout<<"}\n";
 }return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
