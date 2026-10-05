// Ground-method comparison only: no ROS, map, tracker, or operating output.
// The baseline voxel/leveling contract matches height_review/review.cpp.
#include <patchwork/patchworkpp.h>
#include <ground_segmentation/ground_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
namespace fs=std::filesystem;
using Cloud=pcl::PointCloud<pcl::PointXYZI>;
using Clock=std::chrono::steady_clock;

Cloud Read(const fs::path& path) {
  std::ifstream in(path,std::ios::binary|std::ios::ate);
  if(!in)throw std::runtime_error("missing cloud "+path.string());
  const auto bytes=in.tellg();
  if(bytes<=0||bytes%16)throw std::runtime_error("invalid XYZI cloud");
  in.seekg(0);Cloud cloud;cloud.reserve(std::size_t(bytes)/16);
  float f[4];while(in.read(reinterpret_cast<char*>(f),sizeof(f))) {
    pcl::PointXYZI p;p.x=f[0];p.y=f[1];p.z=f[2];p.intensity=f[3];cloud.push_back(p);
  }
  cloud.width=cloud.size();cloud.height=1;cloud.is_dense=false;return cloud;
}
void WriteMask(std::ostream& out,const std::vector<std::uint8_t>& mask) {
  out<<'[';for(std::size_t i=0;i<mask.size();++i){if(i)out<<',';out<<unsigned(mask[i]);}out<<']';
}
std::vector<std::uint8_t> Patch(Eigen::MatrixXf& matrix,patchwork::PatchWorkpp& pp) {
  pp.estimateGround(matrix);
  std::vector<std::uint8_t> labels(std::size_t(matrix.rows()),0);
  auto fill=[&](const Eigen::VectorXi& selected,std::uint8_t label) {
    for(Eigen::Index j=0;j<selected.size();++j) {
      int i=selected[j];
      if(i<0||std::size_t(i)>=labels.size()||labels[std::size_t(i)]!=0)
        throw std::runtime_error("Patchwork++ bad or duplicate index");
      labels[std::size_t(i)]=label;
    }
  };
  fill(pp.getGroundIndices(),1);fill(pp.getNongroundIndices(),2);
  return labels;
}
std::vector<std::uint8_t> Line(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                               GroundSegmentation& model) {
  std::vector<int> labels;model.segment(cloud,&labels);
  if(labels.size()!=cloud.size())throw std::runtime_error("linefit size changed");
  std::vector<std::uint8_t> result;result.reserve(labels.size());
  for(int l:labels){if(l!=0&&l!=1)throw std::runtime_error("linefit invalid label");result.push_back(l?1:2);}
  return result;
}
int main(int argc,char** argv)try {
  if(argc!=3)throw std::runtime_error("usage: ground_method_review snapshots.jsonl output.jsonl");
  const fs::path snapshots=argv[1],output=argv[2];
  if(fs::exists(output))throw std::runtime_error("output already exists");
  std::ifstream in(snapshots);std::ofstream out(output);
  if(!in||!out)throw std::runtime_error("cannot open input/output");
  patchwork::Params base;base.sensor_height=1.80;base.min_range=1;base.max_range=60;
  base.enable_RNR=false;base.height_policy=2;base.height_trace=false;
  std::map<std::string,patchwork::Params> pp_params;
  pp_params["pp_h180"]=base;
  pp_params["pp_h178"]=base;pp_params["pp_h178"].sensor_height=1.78;
  pp_params["pp_h180_d08"]=base;pp_params["pp_h180_d08"].th_dist=.08;
  pp_params["pp_h180_no_tgr"]=base;pp_params["pp_h180_no_tgr"].enable_TGR=false;
  std::map<std::string,std::unique_ptr<patchwork::PatchWorkpp>> pp;
  GroundSegmentationParams line_base;
  line_base.r_min_square=1.;line_base.r_max_square=60.*60.;
  line_base.n_bins=120;line_base.n_segments=360;line_base.n_threads=4;
  line_base.max_dist_to_line=.05;line_base.min_slope=0.;line_base.max_slope=.3;
  line_base.max_error_square=.05*.05;line_base.long_threshold=1.;
  line_base.max_long_height=.1;line_base.max_start_height=.2;
  line_base.line_search_angle=.1;line_base.sensor_height=1.78;
  auto line_wide=line_base;line_wide.max_dist_to_line=.10;
  GroundSegmentation line05(line_base),line10(line_wide);
  std::ofstream discard("/dev/null");auto* oldcout=std::cout.rdbuf(discard.rdbuf());
  std::string line,phase;std::int64_t last_stamp=0;int count=0;
  out<<std::setprecision(17);
  while(std::getline(in,line)) {
    if(line.empty())continue;
    auto row=YAML::Load(line);
    for(const char* k:{"input_id","phase","cloud_path","pitch","roll","geometry_valid",
                        "scan_effective_stamp_ns","scan_arrival_ns"})
      if(!row[k])throw std::runtime_error(std::string("missing ")+k);
    const std::string id=row["input_id"].as<std::string>();
    const auto current=row["phase"].as<std::string>();
    if(current!=phase) {
      pp.clear();for(const auto& kv:pp_params)pp.emplace(kv.first,new patchwork::PatchWorkpp(kv.second));
      phase=current;last_stamp=0;
    }
    const auto stamp=row["scan_effective_stamp_ns"].as<std::int64_t>();
    const auto arrival=row["scan_arrival_ns"].as<std::int64_t>();
    if(stamp<=last_stamp||arrival<stamp||!row["geometry_valid"].as<bool>())
      throw std::runtime_error("invalid stamp, arrival or geometry "+id);
    last_stamp=stamp;
    const double roll=row["roll"].as<double>(),pitch=row["pitch"].as<double>();
    if(!std::isfinite(roll)||!std::isfinite(pitch))throw std::runtime_error("invalid attitude");
    const auto rel=fs::path(row["cloud_path"].as<std::string>());
    if(rel.is_absolute()||rel.string().find("..")!=std::string::npos)throw std::runtime_error("unsafe cloud path");
    Cloud raw=Read(snapshots.parent_path()/rel),roi;
    std::vector<std::size_t> lineage;roi.reserve(raw.size());lineage.reserve(raw.size());
    for(std::size_t i=0;i<raw.size();++i){
      const auto& p=raw[i];
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||!std::isfinite(p.intensity))continue;
      const double radius=std::sqrt(double(p.x)*p.x+double(p.y)*p.y+double(p.z)*p.z);
      if(radius<.5||radius>100||p.x< -30||p.x>50||p.y< -20||p.y>20||p.z< -2||p.z>3)continue;
      const double bx=p.x+.80;
      if(bx>=-.700&&bx<=3.845&&p.y>=-.946&&p.y<=.946)continue;
      roi.push_back(p);lineage.push_back(i);
    }
    roi.width=roi.size();roi.height=1;roi.is_dense=false;
    pcl::VoxelGrid<pcl::PointXYZI> grid;grid.setInputCloud(roi.makeShared());
    grid.setLeafSize(.1f,.1f,.1f);grid.setSaveLeafLayout(true);
    Cloud voxel;grid.filter(voxel);
    if(voxel.empty())throw std::runtime_error("empty voxel "+id);
    std::vector<int> source_to_voxel;source_to_voxel.reserve(roi.size());
    for(const auto& p:roi)source_to_voxel.push_back(grid.getCentroidIndex(p));
    const float cp=std::cos(float(pitch)),sp=std::sin(float(pitch));
    Eigen::MatrixXf matrix(voxel.size(),4);pcl::PointCloud<pcl::PointXYZ> xyz;
    xyz.reserve(voxel.size());
    for(std::size_t i=0;i<voxel.size();++i){
      const auto& p=voxel[i];const float x=cp*p.x+sp*p.z,z=-sp*p.x+cp*p.z;
      matrix.row(Eigen::Index(i))<<x,p.y,z,p.intensity;
      xyz.emplace_back(x,p.y,z);
    }
    const auto begin=Clock::now();
    std::map<std::string,std::vector<std::uint8_t>> methods;
    std::map<std::string,double> times;
    for(auto& kv:pp){
      auto start=Clock::now();methods[kv.first]=Patch(matrix,*kv.second);
      times[kv.first]=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
    }
    for(const auto& entry:std::vector<std::pair<std::string,GroundSegmentation*>>{
          {"linefit_h178_d05",&line05},{"linefit_h178_d10",&line10}}){
      auto start=Clock::now();methods[entry.first]=Line(xyz,*entry.second);
      times[entry.first]=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
    }
    out<<"{\"input_id\":\""<<id<<"\",\"stamp_ns\":"<<stamp
       <<",\"roi_points\":"<<roi.size()<<",\"voxel_points\":"<<voxel.size()
       <<",\"methods\":{";
    bool first=true;
    for(const auto& kv:methods){
      if(!first)out<<',';first=false;out<<'"'<<kv.first<<"\":{";
      out<<"\"segment_ms\":"<<times.at(kv.first)<<",\"mask\":";
      std::vector<std::uint8_t> restored(raw.size(),0);
      for(std::size_t i=0;i<roi.size();++i){
        int j=source_to_voxel[i];
        if(j<0||std::size_t(j)>=kv.second.size())throw std::runtime_error("bad voxel index");
        restored[lineage[i]]=kv.second[std::size_t(j)];
      }
      WriteMask(out,restored);out<<'}';
    }
    out<<"}}\n";
    ++count;
    if(count%200==0)std::cerr<<"processed "<<count<<" last="<<id<<'\n';
    (void)begin;
  }
  std::cout.rdbuf(oldcout);
  if(count==0||!out)throw std::runtime_error("no rows or write failure");
  std::cerr<<"complete frames="<<count<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
