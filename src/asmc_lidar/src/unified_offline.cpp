#include <asmc_lidar/unified.hpp>
#include <asmc_lidar/review_io.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
namespace fs=std::filesystem;
int main(int argc,char** argv)try{
  if(argc!=4)throw std::runtime_error(
    "usage: asmc_lidar_unified_offline snapshots.jsonl calibration.yaml output.jsonl");
  const fs::path source=argv[1],output=argv[3];
  if(fs::exists(output))throw std::runtime_error("output already exists");
  std::ifstream in(source);if(!in)throw std::runtime_error("missing snapshots");
  std::ofstream out(output);if(!out)throw std::runtime_error("cannot write output");
  const auto cal=asmc_lidar::LoadCalibration(argv[2]);
  const auto config=fs::path(argv[2]).parent_path()/"ground_v4/jaeho_support.yaml";
  asmc_lidar::GroundComparison model("jaeho","guard25",asmc_lidar::LoadGroundComparisonConfig(config.string()));
  std::string line,phase;std::int64_t last=0;std::size_t count=0;
  out<<std::setprecision(17);
  while(std::getline(in,line)){
    if(line.empty())continue;
    const auto row=YAML::Load(line);
    const auto current=row["phase"].as<std::string>();
    if(current!=phase){model.Reset();phase=current;last=0;}
    const auto stamp=row["scan_effective_stamp_ns"].as<std::int64_t>();
    const auto roll=row["roll"].as<double>(),pitch=row["pitch"].as<double>();
    const bool stationary_repeat=row["repetition_role"]&&
      row["repetition_role"].as<std::string>()=="stationary_snapshot_warmup_not_tracking";
    // Repeated, same-stamp Capture snapshots are an explicit offline warmup,
    // never accepted as independent live observations or tracker updates.
    if(stamp<last||(stamp==last&&!stationary_repeat)||
       !row["geometry_valid"].as<bool>()||!std::isfinite(roll)||!std::isfinite(pitch))
      throw std::runtime_error("invalid observation stamp or IMU");
    last=stamp;
    const fs::path rel=row["cloud_path"].as<std::string>();
    // Historical Capture manifests record the container's /data path instead
    // of a path relative to snapshots.jsonl. Accept only that pinned data root.
    if(rel.string().find("..")!=std::string::npos||
       (rel.is_absolute()&&rel.lexically_normal().string().rfind("/data/lidar_assessment/",0)!=0))
      throw std::runtime_error("unsafe cloud path");
    const auto cloud=asmc_lidar::ReadReviewCloud(rel.is_absolute()?rel:source.parent_path()/rel);
    const auto ground=model.Process(cloud,cal,float(roll),float(pitch));
    const auto result=asmc_lidar::BuildUnified(cloud,ground,cal,stamp);
    const auto id=row["input_id"].as<std::string>();
    if(id.empty()||id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)
      throw std::runtime_error("unsafe input id");
    out<<"{\"input_id\":\""<<id<<"\",\"stamp_ns\":"<<stamp
       <<",\"frame\":\"base_link\",\"stationary_repeat\":"<<(stationary_repeat?"true":"false")
       <<",\"config_sha256\":\""<<model.config().sha256
       <<"\",\"ground_raw_points\":"<<result.ground_raw_points
       <<",\"retained_raw_points\":"<<result.retained_raw_indices.size()
       <<",\"structure_candidate_raw_points\":"<<result.structure_candidate_raw_indices.size()
       <<",\"detections\":[";
    for(std::size_t k=0;k<result.detections.size();++k){
      if(k)out<<',';const auto& d=result.detections[k];
      out<<"{\"frame_index\":"<<d.detection_index<<",\"raw_points\":"<<d.raw_points
         <<",\"class\":\"unknown\",\"velocity\":null,\"center\":["
         <<d.center_base.x()<<','<<d.center_base.y()<<','<<d.center_base.z()
         <<"],\"yaw\":"<<d.yaw_base_rad<<",\"corners\":[";
      for(std::size_t j=0;j<8;++j){if(j)out<<',';const auto& p=d.corners_base[j];
        out<<'['<<p.x()<<','<<p.y()<<','<<p.z()<<']';}
      out<<"],\"source_indices\":[";
      for(std::size_t j=0;j<d.source_indices.size();++j){if(j)out<<',';out<<d.source_indices[j];}
      out<<"]}";
    }
    out<<"],\"structure_hypotheses\":"<<result.structure_evidence.size()
       <<",\"structure_auto_suppression\":false}\n";
    ++count;
  }
  if(!out||!count)throw std::runtime_error("empty/failed output");
  std::cerr<<"complete "<<count<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
