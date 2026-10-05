// Same frozen ROI/10 cm voxel/pitch input as the first method sweep.
// Only linefit angular/radial support and long-gap assumptions change.
#include <ground_segmentation/ground_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
namespace fs=std::filesystem;
using Cloud=pcl::PointCloud<pcl::PointXYZI>;

Cloud Read(const fs::path& path) {
  std::ifstream in(path,std::ios::binary|std::ios::ate);
  if(!in)throw std::runtime_error("missing cloud");
  const auto bytes=in.tellg();if(bytes<=0||bytes%16)throw std::runtime_error("bad XYZI");
  in.seekg(0);Cloud out;out.reserve(std::size_t(bytes)/16);
  float f[4];while(in.read(reinterpret_cast<char*>(f),sizeof(f))) {
    pcl::PointXYZI p;p.x=f[0];p.y=f[1];p.z=f[2];p.intensity=f[3];out.push_back(p);
  }
  out.width=out.size();out.height=1;out.is_dense=false;return out;
}
void Write(std::ostream& out,const std::vector<std::uint8_t>& mask){
  out<<'[';for(std::size_t i=0;i<mask.size();++i){if(i)out<<',';out<<unsigned(mask[i]);}out<<']';
}
int main(int argc,char** argv)try {
  if(argc!=3)throw std::runtime_error("usage: linefit_vlp16_review snapshots.jsonl output.jsonl");
  fs::path source=argv[1],destination=argv[2];
  if(fs::exists(destination))throw std::runtime_error("output exists");
  std::ifstream in(source);std::ofstream out(destination);
  if(!in||!out)throw std::runtime_error("cannot open input/output");
  GroundSegmentationParams base;
  base.sensor_height=1.78;base.r_min_square=1.;base.r_max_square=60.*60.;
  base.n_threads=4;base.max_dist_to_line=.05;base.min_slope=0.;base.max_slope=.3;
  base.max_error_square=.05*.05;base.max_long_height=.1;
  base.max_start_height=.2;base.line_search_angle=.1;
  std::map<std::string,GroundSegmentationParams> params;
  auto add=[&](const std::string& name,int sectors,int bins,double gap){
    auto p=base;p.n_segments=sectors;p.n_bins=bins;p.long_threshold=gap;params[name]=p;
  };
  add("lf_90x60_gap2",90,60,2.);
  add("lf_60x60_gap2",60,60,2.);
  add("lf_90x40_gap3",90,40,3.);
  add("lf_60x40_gap3",60,40,3.);
  std::map<std::string,std::unique_ptr<GroundSegmentation>> models;
  for(const auto& p:params)models.emplace(p.first,new GroundSegmentation(p.second));
  std::ofstream discard("/dev/null");auto* oldcout=std::cout.rdbuf(discard.rdbuf());
  std::string text;int count=0;std::int64_t last_stamp=0;
  while(std::getline(in,text)) {
    if(text.empty())continue;
    const auto row=YAML::Load(text);
    const auto id=row["input_id"].as<std::string>();
    const auto stamp=row["scan_effective_stamp_ns"].as<std::int64_t>();
    if(stamp<=last_stamp||row["scan_arrival_ns"].as<std::int64_t>()<stamp||
       !row["geometry_valid"].as<bool>())throw std::runtime_error("invalid time/geometry "+id);
    last_stamp=stamp;
    const auto rel=fs::path(row["cloud_path"].as<std::string>());
    if(rel.is_absolute()||rel.string().find("..")!=std::string::npos)throw std::runtime_error("unsafe cloud path");
    auto raw=Read(source.parent_path()/rel);
    Cloud roi;std::vector<std::size_t> lineage;
    roi.reserve(raw.size());lineage.reserve(raw.size());
    for(std::size_t i=0;i<raw.size();++i) {
      const auto& p=raw[i];
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||!std::isfinite(p.intensity))continue;
      const double radius=std::sqrt(double(p.x)*p.x+double(p.y)*p.y+double(p.z)*p.z);
      if(radius<.5||radius>100||p.x< -30||p.x>50||p.y< -20||p.y>20||p.z< -2||p.z>3)continue;
      const double bx=p.x+.8;
      if(bx>=-.700&&bx<=3.845&&p.y>=-.946&&p.y<=.946)continue;
      roi.push_back(p);lineage.push_back(i);
    }
    roi.width=roi.size();roi.height=1;roi.is_dense=false;
    pcl::VoxelGrid<pcl::PointXYZI> grid;grid.setInputCloud(roi.makeShared());
    grid.setLeafSize(.1f,.1f,.1f);grid.setSaveLeafLayout(true);
    Cloud voxel;grid.filter(voxel);
    std::vector<int> index;index.reserve(roi.size());
    for(const auto& p:roi)index.push_back(grid.getCentroidIndex(p));
    const float pitch=row["pitch"].as<float>();
    const float cp=std::cos(pitch),sp=std::sin(pitch);
    pcl::PointCloud<pcl::PointXYZ> points;points.reserve(voxel.size());
    for(const auto& p:voxel)points.emplace_back(cp*p.x+sp*p.z,p.y,-sp*p.x+cp*p.z);
    out<<"{\"input_id\":\""<<id<<"\",\"stamp_ns\":"<<stamp<<",\"methods\":{";
    bool first=true;
    for(auto& model:models){
      std::vector<int> result;model.second->segment(points,&result);
      if(result.size()!=voxel.size())throw std::runtime_error("linefit result size drift");
      std::vector<std::uint8_t> mask(raw.size(),0);
      for(std::size_t i=0;i<roi.size();++i){
        int j=index[i];if(j<0||std::size_t(j)>=result.size())throw std::runtime_error("bad voxel index");
        mask[lineage[i]]=result[std::size_t(j)]?1:2;
      }
      if(!first)out<<',';first=false;
      out<<'"'<<model.first<<"\":{\"segment_ms\":0,\"mask\":";
      Write(out,mask);out<<'}';
    }
    out<<"}}\n";
    ++count;if(count%200==0)std::cerr<<"processed "<<count<<'\n';
  }
  std::cout.rdbuf(oldcout);if(!out||count==0)throw std::runtime_error("write failure");
  std::cerr<<"complete "<<count<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
