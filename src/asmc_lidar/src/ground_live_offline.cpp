#include <asmc_lidar/ground_live.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
namespace fs=std::filesystem;
template<class T>void Array(std::ostream& out,const std::vector<T>& a){
  out<<'[';for(std::size_t i=0;i<a.size();++i){if(i)out<<',';out<<int(a[i]);}out<<']';
}
asmc_lidar::Cloud Read(const fs::path& path){
  std::ifstream in(path,std::ios::binary|std::ios::ate);
  if(!in)throw std::runtime_error("missing cloud "+path.string());
  const auto bytes=in.tellg();if(bytes<0||bytes%16)throw std::runtime_error("invalid XYZI");
  in.seekg(0);asmc_lidar::Cloud cloud;float f[4];
  while(in.read(reinterpret_cast<char*>(f),sizeof(f))){
    pcl::PointXYZI p;p.x=f[0];p.y=f[1];p.z=f[2];p.intensity=f[3];cloud.push_back(p);
  }
  return cloud;
}
int main(int argc,char** argv)try{
  if(argc<4||argc>6)throw std::runtime_error("usage: ground_live_offline snapshots.jsonl calibration.yaml output.jsonl [guard25|fixed_height] [all|jaeho|patchwork|linefit]");
  const fs::path source=argv[1],output=argv[3];
  if(fs::exists(output))throw std::runtime_error("output already exists");
  std::ifstream in(source);if(!in)throw std::runtime_error("missing snapshots");
  const auto cal=asmc_lidar::LoadCalibration(argv[2]);
  asmc_lidar::GroundComparison model(argc>5?argv[5]:"all",argc>4?argv[4]:"guard25");
  std::ofstream out(output);if(!out)throw std::runtime_error("cannot create output");
  out<<std::setprecision(17);std::string text,phase;std::int64_t last=0;std::size_t count=0;
  while(std::getline(in,text)){
    if(text.empty())continue;const auto row=YAML::Load(text);
    const auto current=row["phase"].as<std::string>();
    if(current!=phase){model.Reset();phase=current;last=0;}
    const auto stamp=row["scan_effective_stamp_ns"].as<std::int64_t>();
    const auto roll=row["roll"].as<double>(),pitch=row["pitch"].as<double>();
    if(stamp<=last||!row["geometry_valid"].as<bool>()||!std::isfinite(roll)||!std::isfinite(pitch))
      throw std::runtime_error("invalid input clock/pose");
    last=stamp;const fs::path rel=row["cloud_path"].as<std::string>();
    if(rel.is_absolute()||rel.string().find("..")!=std::string::npos)throw std::runtime_error("invalid cloud path");
    const auto cloud=Read(source.parent_path()/rel);
    const auto result=model.Process(cloud,cal,float(pitch));
    const auto id=row["input_id"].as<std::string>();
    if(id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)
      throw std::runtime_error("unsafe input id");
    out<<"{\"input_id\":\""<<id<<"\",\"stamp_ns\":"<<stamp<<",\"prepare_ms\":"<<result.prepared.prepare_ms;
    out<<",\"raw_to_voxel\":";Array(out,result.prepared.raw_to_voxel);
    out<<",\"methods\":{";bool first=true;
    for(const auto& entry:result.methods){
      if(!first)out<<',';first=false;const auto& r=entry.second;
      out<<'"'<<entry.first<<"\":{\"mask\":";Array(out,r.labels);
      out<<",\"structural_removed\":";Array(out,r.structural_removed);
      out<<",\"copy_ms\":"<<r.copy_ms<<",\"segment_ms\":"<<r.segment_ms
         <<",\"guard_ms\":"<<r.guard_ms<<",\"restore_ms\":"<<r.restore_ms<<'}';
    }
    out<<"}}\n";if(++count%250==0)std::cerr<<"processed "<<count<<'\n';
  }
  if(!out||!count)throw std::runtime_error("empty or failed output");
  std::cerr<<"complete "<<count<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
