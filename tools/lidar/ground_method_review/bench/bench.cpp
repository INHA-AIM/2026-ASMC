// Ground-stage latency only, same ROI/voxel/pitch representation; no ROS or GT.
#include <patchwork/patchworkpp.h>
#include <ground_segmentation/ground_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
using Cloud=pcl::PointCloud<pcl::PointXYZI>;
double Millis(Clock::time_point a,Clock::time_point b){return std::chrono::duration<double,std::milli>(b-a).count();}
Cloud Read(const fs::path& path){
  std::ifstream in(path,std::ios::binary|std::ios::ate);
  if(!in)throw std::runtime_error("missing cloud");auto bytes=in.tellg();
  if(bytes<=0||bytes%16)throw std::runtime_error("invalid XYZI");
  in.seekg(0);Cloud out;out.reserve(std::size_t(bytes)/16);float f[4];
  while(in.read(reinterpret_cast<char*>(f),sizeof(f))){
    pcl::PointXYZI p;p.x=f[0];p.y=f[1];p.z=f[2];p.intensity=f[3];out.push_back(p);
  }
  out.width=out.size();out.height=1;out.is_dense=false;return out;
}
int main(int argc,char** argv)try{
  if(argc!=3)throw std::runtime_error("usage: ground_method_bench snapshots.jsonl timings.csv");
  fs::path source=argv[1],output=argv[2];if(fs::exists(output))throw std::runtime_error("output exists");
  std::ifstream in(source);std::ofstream out(output);
  if(!in||!out)throw std::runtime_error("cannot open input/output");
  patchwork::Params pp_param;pp_param.sensor_height=1.8;pp_param.min_range=1;
  pp_param.max_range=60;pp_param.enable_RNR=false;pp_param.height_policy=2;
  std::ofstream discard("/dev/null");auto* oldcout=std::cout.rdbuf(discard.rdbuf());
  patchwork::PatchWorkpp pp(pp_param);
  GroundSegmentationParams lp;lp.sensor_height=1.78;lp.r_min_square=1.;lp.r_max_square=3600.;
  lp.n_bins=60;lp.n_segments=90;lp.n_threads=4;lp.max_dist_to_line=.05;
  lp.min_slope=0.;lp.max_slope=.3;lp.max_error_square=.05*.05;
  lp.long_threshold=2.;lp.max_long_height=.1;lp.max_start_height=.2;lp.line_search_angle=.1;
  GroundSegmentation line(lp);
  out<<"frame,points,preprocess_ms,patchwork_segment_ms,linefit_segment_ms\n";
  std::string row;int ordinal=0;
  while(std::getline(in,row)){
    if(row.empty())continue;
    // A lightweight parser only for the fixed, audited snapshots.jsonl schema.
    auto take=[&](const std::string& key){
      const std::string marker='"'+key+'"';const auto k=row.find(marker);
      if(k==std::string::npos)throw std::runtime_error("missing "+key);
      const auto colon=row.find(':',k+marker.size());
      const auto first=row.find_first_not_of(" \t",colon+1);
      if(first==std::string::npos)throw std::runtime_error("missing value");
      if(row[first]=='"')return row.substr(first+1,row.find('"',first+1)-first-1);
      return row.substr(first,row.find_first_of(",}",first)-first);
    };
    const auto id=take("input_id");
    const auto rel=fs::path(take("cloud_path"));
    if(rel.is_absolute()||rel.string().find("..")!=std::string::npos)throw std::runtime_error("unsafe path");
    const float pitch=std::stof(take("pitch"));
    const auto begin=Clock::now();auto raw=Read(source.parent_path()/rel);
    Cloud roi;roi.reserve(raw.size());
    for(const auto& p:raw){
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||!std::isfinite(p.intensity))continue;
      const double radius=std::sqrt(double(p.x)*p.x+double(p.y)*p.y+double(p.z)*p.z);
      if(radius<.5||radius>100||p.x< -30||p.x>50||p.y< -20||p.y>20||p.z< -2||p.z>3)continue;
      const double bx=p.x+.8;if(bx>=-.700&&bx<=3.845&&p.y>=-.946&&p.y<=.946)continue;
      roi.push_back(p);
    }
    roi.width=roi.size();roi.height=1;roi.is_dense=false;
    pcl::VoxelGrid<pcl::PointXYZI> vg;vg.setInputCloud(roi.makeShared());vg.setLeafSize(.1f,.1f,.1f);
    Cloud voxel;vg.filter(voxel);
    if(voxel.empty())throw std::runtime_error("empty voxel");
    const float cp=std::cos(pitch),sp=std::sin(pitch);
    Eigen::MatrixXf matrix(voxel.size(),4);pcl::PointCloud<pcl::PointXYZ> xyz;xyz.reserve(voxel.size());
    for(std::size_t i=0;i<voxel.size();++i){const auto& p=voxel[i];
      const float x=cp*p.x+sp*p.z,z=-sp*p.x+cp*p.z;
      matrix.row(Eigen::Index(i))<<x,p.y,z,p.intensity;xyz.emplace_back(x,p.y,z);
    }
    const auto prepared=Clock::now();pp.estimateGround(matrix);const auto pp_done=Clock::now();
    std::vector<int> labels;line.segment(xyz,&labels);const auto line_done=Clock::now();
    if(labels.size()!=voxel.size())throw std::runtime_error("linefit output size mismatch");
    out<<id<<','<<voxel.size()<<','<<Millis(begin,prepared)<<','
       <<Millis(prepared,pp_done)<<','<<Millis(pp_done,line_done)<<'\n';
    ++ordinal;if(ordinal%200==0)std::cerr<<"bench "<<ordinal<<'\n';
  }
  std::cout.rdbuf(oldcout);if(!out||ordinal==0)throw std::runtime_error("write failure");
  std::cerr<<"complete "<<ordinal<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
