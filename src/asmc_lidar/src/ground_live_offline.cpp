#include <asmc_lidar/ground_live.hpp>
#include <asmc_lidar/review_io.hpp>
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
int main(int argc,char** argv)try{
  if(argc<4||argc>7)throw std::runtime_error("usage: ground_live_offline snapshots.jsonl calibration.yaml output.jsonl [guard25|fixed_height] [all|jaeho|patchwork|linefit] [comparison_config.yaml]");
  const fs::path source=argv[1],output=argv[3];
  if(fs::exists(output))throw std::runtime_error("output already exists");
  std::ifstream in(source);if(!in)throw std::runtime_error("missing snapshots");
  const auto cal=asmc_lidar::LoadCalibration(argv[2]);
  asmc_lidar::GroundComparison model(argc>5?argv[5]:"all",argc>4?argv[4]:"guard25",asmc_lidar::LoadGroundComparisonConfig(argc>6?argv[6]:""));
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
    const auto cloud=asmc_lidar::ReadReviewCloud(source.parent_path()/rel);
    const auto result=model.Process(cloud,cal,float(roll),float(pitch));
    const auto id=row["input_id"].as<std::string>();
    if(id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)
      throw std::runtime_error("unsafe input id");
    out<<"{\"input_id\":\""<<id<<"\",\"stamp_ns\":"<<stamp<<",\"prepare_ms\":"<<result.prepared.prepare_ms;
    out<<",\"config_id\":\""<<model.config().id<<"\",\"config_sha256\":\""<<model.config().sha256<<'"';
    if(model.config().leveling_mode!="pitch_only")
      out<<",\"leveling_mode\":\""<<model.config().leveling_mode<<'"';
    out<<",\"raw_to_voxel\":";Array(out,result.prepared.raw_to_voxel);
    out<<",\"methods\":{";bool first=true;
    for(const auto& entry:result.methods){
      if(!first)out<<',';first=false;const auto& r=entry.second;
      out<<'"'<<entry.first<<"\":{\"mask\":";Array(out,r.labels);
      out<<",\"structural_removed\":";Array(out,r.structural_removed);
      out<<",\"pre_guard_mask\":";Array(out,r.pre_guard_labels);
      out<<",\"reasons\":";Array(out,r.reasons);
      out<<",\"events\":{";bool first_event=true;
      for(const auto& e:r.events){if(!first_event)out<<',';first_event=false;out<<'"'<<e.first<<"\":"<<e.second;}
      out<<'}';
      if(entry.first=="jaeho"&&model.config().leveling_mode=="roll_pitch"){
        out<<",\"plane_attempts\":[";bool first_plane=true;
        for(const auto& plane:r.planes){
          if(!first_plane)out<<',';first_plane=false;
          out<<"{\"attempt\":"<<plane.attempt<<",\"candidate_points\":"<<plane.candidate_points
             <<",\"inlier_points\":"<<plane.inlier_points
             <<",\"valid\":"<<(plane.valid?"true":"false")
             <<",\"accepted_ground\":"<<(plane.accepted_ground?"true":"false")
             <<",\"height_valid\":"<<(plane.height_valid?"true":"false")
             <<",\"height_at_origin_m\":"<<plane.height_at_origin_m
             <<",\"normal\":["<<plane.normal[0]<<','<<plane.normal[1]<<','<<plane.normal[2]
             <<"],\"centroid\":["<<plane.centroid[0]<<','<<plane.centroid[1]<<','<<plane.centroid[2]<<"]}";
        }
        out<<']';
      }
      out<<",\"copy_ms\":"<<r.copy_ms<<",\"segment_ms\":"<<r.segment_ms
         <<",\"guard_ms\":"<<r.guard_ms<<",\"restore_ms\":"<<r.restore_ms<<'}';
    }
    out<<"}}\n";if(++count%250==0)std::cerr<<"processed "<<count<<'\n';
  }
  if(!out||!count)throw std::runtime_error("empty or failed output");
  std::cerr<<"complete "<<count<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
