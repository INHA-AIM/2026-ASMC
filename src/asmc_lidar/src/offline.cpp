#include <asmc_lidar/core.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace fs=std::filesystem;
using asmc_lidar::Cloud;

Cloud::Ptr ReadFloat4(const fs::path& file) {
  std::ifstream in(file,std::ios::binary|std::ios::ate);
  if(!in)throw std::runtime_error("cannot read "+file.string());
  const auto bytes=in.tellg();
  if(bytes<0||bytes%16)throw std::runtime_error("bad float4 XYZI size");
  in.seekg(0);Cloud::Ptr cloud(new Cloud);cloud->reserve(std::size_t(bytes)/16);
  float values[4];
  while(in.read(reinterpret_cast<char*>(values),sizeof(values))) {
    pcl::PointXYZI p;p.x=values[0];p.y=values[1];p.z=values[2];p.intensity=values[3];
    cloud->push_back(p);
  }
  cloud->width=cloud->size();cloud->height=1;cloud->is_dense=false;
  return cloud;
}
void String(std::ostream& out,const std::string& value) {
  out<<'"';for(unsigned char c:value) {
    if(c=='"'||c=='\\')out<<'\\'<<c;
    else if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;
    else out<<c;
  }out<<'"';
}
template<class T> void Array(std::ostream& out,const std::vector<T>& values) {
  out<<'[';for(std::size_t i=0;i<values.size();++i) {
    if(i)out<<',';out<<int(values[i]);
  }out<<']';
}
int main(int argc,char** argv) try {
  if(argc!=6 && argc!=7)throw std::runtime_error(
      "usage: asmc_lidar_offline snapshots.jsonl calibration.yaml reference.yaml reference|ground_review output.jsonl");
  const fs::path input=argv[1],output=argv[5];
  if(fs::exists(output))throw std::runtime_error("output exists");
  const std::string mode=argv[4];
  const std::string profile=argc==7?argv[6]:"tunnel";
  if(profile!="tunnel"&&profile!="general")throw std::runtime_error("invalid profile");
  if(mode!="reference"&&mode!="ground_review")throw std::runtime_error("invalid mode");
  asmc_lidar::Processor processor(mode=="reference"?asmc_lidar::Mode::Reference:asmc_lidar::Mode::GroundReview,
      asmc_lidar::LoadCalibration(argv[2]),asmc_lidar::LoadReferenceParams(argv[3]));
  std::ifstream stream(input);if(!stream)throw std::runtime_error("cannot open snapshots");
  std::ofstream out(output);if(!out)throw std::runtime_error("cannot create output");
  std::string line,phase;
  while(std::getline(stream,line)) {
    if(line.empty())continue;
    const auto row=YAML::Load(line);
    std::string current=row["phase"]?row["phase"].as<std::string>():"one_sequence";
    if(current!=phase) {processor.Reset();phase=current;}
    asmc_lidar::FrameInput frame;
    frame.profile=profile=="tunnel"?asmc_lidar::ReferenceProfile::Tunnel:asmc_lidar::ReferenceProfile::General;
    const auto rel=row["cloud_path"].as<std::string>();
    if(fs::path(rel).is_absolute()||rel.find("..")!=std::string::npos)
      throw std::runtime_error("unsafe relative cloud path");
    frame.cloud=ReadFloat4(input.parent_path()/rel);
    frame.stamp_ns=row["scan_effective_stamp_ns"]?
        row["scan_effective_stamp_ns"].as<std::int64_t>():row["stamp_ns"].as<std::int64_t>();
    frame.receipt_ns=row["scan_arrival_ns"]?
        row["scan_arrival_ns"].as<std::int64_t>():frame.stamp_ns;
    frame.roll_rad=row["roll"]?row["roll"].as<double>():0;
    frame.pitch_rad=row["pitch"]?row["pitch"].as<double>():0;
    frame.imu_valid=row["geometry_valid"]?row["geometry_valid"].as<bool>():true;
    frame.already_preprocessed=row["already_preprocessed"]?
        row["already_preprocessed"].as<bool>():false;
    frame.camera_available=row["controlled_camera_state"] &&
        row["controlled_camera_state"].as<std::string>()=="empty";
    frame.allow_repeated_snapshot=row["repetition_role"] &&
        row["repetition_role"].as<std::string>()=="stationary_snapshot_warmup_not_tracking";
    const auto result=processor.Process(frame);
    out<<"{\"input_id\":";String(out,row["input_id"]?row["input_id"].as<std::string>():rel);
    out<<",\"phase\":";String(out,phase);
    out<<",\"stamp_ns\":"<<result.stamp_ns<<",\"receipt_ns\":"<<frame.receipt_ns;
    out<<",\"stamp_provenance\":";String(out,frame.stamp_provenance);
    out<<",\"clock\":";String(out,frame.clock);
    out<<",\"status\":";String(out,result.status);
    out<<",\"camera_state\":";String(out,result.camera_state);
    out<<",\"raw_points\":"<<result.raw_points<<",\"roi_points\":"<<result.roi_points;
    out<<",\"voxel_points\":"<<result.voxel_points<<",\"frozen_voxel_fnv1a64\":";
    String(out,result.voxel_fnv1a64);out<<",\"raw_to_voxel\":";
    Array(out,result.raw_to_voxel);
    out<<",\"reference_labels\":";Array(out,result.reference_labels);
    out<<",\"reference_objects\":[";
    for(std::size_t i=0;i<result.reference_objects_base.size();++i) {
      if(i)out<<',';const auto& b=result.reference_objects_base[i].obb;
      out<<"{\"center_base\":["<<b.cx<<','<<b.cy<<','<<b.cz<<"],\"corners_base\":[";
      for(std::size_t j=0;j<8;++j) {
        if(j)out<<',';out<<'['<<b.corners[j].x<<','<<b.corners[j].y<<','<<b.corners[j].z<<']';
      }out<<"]}";
    }
    out<<"],\"patchwork_roi_to_original\":";
    Array(out,result.patchwork.roi_to_original);
    out<<",\"patchwork_reason\":";String(out,result.patchwork.reason);
    out<<",\"patchwork_classes\":";
    std::vector<unsigned> classes;classes.reserve(result.patchwork.by_original.size());
    for(auto value:result.patchwork.by_original)classes.push_back(unsigned(value));
    Array(out,classes);
    out<<",\"timing_ms\":{\"preprocess\":"<<result.preprocessing_ms
       <<",\"reference\":"<<result.reference_ms
       <<",\"patchwork_copy\":"<<result.patchwork.input_copy_ms
       <<",\"patchwork_segment\":"<<result.patchwork.segmentation_ms
       <<",\"patchwork_restore\":"<<result.patchwork.restore_ms<<"}}\n";
  }
  if(!out)throw std::runtime_error("cannot finish JSONL");
  return 0;
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
