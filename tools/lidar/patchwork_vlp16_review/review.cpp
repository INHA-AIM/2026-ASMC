// Isolated VLP16 parameter / far-surface comparison. Labels never enter inference.
#include <patchwork/patchworkpp.h>
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
namespace fs = std::filesystem;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using Clock = std::chrono::steady_clock;
using Mask = std::vector<std::uint8_t>;
double Ms(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double, std::milli>(b-a).count();
}
Cloud Read(const fs::path& path) {
  std::ifstream in(path, std::ios::binary|std::ios::ate);
  if (!in) throw std::runtime_error("missing cloud: " + path.string());
  const auto bytes=in.tellg();
  if (bytes<=0 || bytes%16) throw std::runtime_error("invalid XYZI cloud");
  in.seekg(0); Cloud cloud; cloud.reserve(std::size_t(bytes)/16);
  float p[4];
  while (in.read(reinterpret_cast<char*>(p), sizeof(p))) {
    pcl::PointXYZI x; x.x=p[0]; x.y=p[1]; x.z=p[2]; x.intensity=p[3]; cloud.push_back(x);
  }
  cloud.width=cloud.size(); cloud.height=1; cloud.is_dense=false;
  return cloud;
}
void WriteMask(std::ostream& out, const Mask& mask) {
  out << '[';
  for (std::size_t i=0;i<mask.size();++i) {
    if (i) out << ',';
    out << unsigned(mask[i]);
  }
  out << ']';
}
Mask Segment(Eigen::MatrixXf& input, patchwork::PatchWorkpp& model) {
  model.estimateGround(input);
  Mask mask(std::size_t(input.rows()),0);
  auto fill=[&](const Eigen::VectorXi& ids, std::uint8_t label) {
    for (Eigen::Index k=0;k<ids.size();++k) {
      const int j=ids[k];
      if (j<0 || std::size_t(j)>=mask.size() || mask[std::size_t(j)]!=0)
        throw std::runtime_error("Patchwork++ invalid or duplicate index");
      mask[std::size_t(j)]=label;
    }
  };
  fill(model.getGroundIndices(),1); fill(model.getNongroundIndices(),2);
  if (std::find(mask.begin(),mask.end(),0)!=mask.end())
    throw std::runtime_error("Patchwork++ unclassified point");
  return mask;
}
struct Variant {
  patchwork::Params params;
  std::unique_ptr<patchwork::PatchWorkpp> model;
  double guard_above_ground_m=-1;
  double guard_from_radius_m=0;
};
int main(int argc,char** argv) try {
  if (argc!=3) throw std::runtime_error("usage: patchwork_vlp16_review snapshots.jsonl output.jsonl");
  const fs::path snapshots=argv[1], output=argv[2];
  if (fs::exists(output)) throw std::runtime_error("output exists");
  std::ifstream in(snapshots);std::ofstream out(output);
  if (!in||!out) throw std::runtime_error("cannot open input/output");
  patchwork::Params base; base.sensor_height=1.80; base.min_range=1; base.max_range=60;
  base.enable_RNR=false; base.height_policy=2; base.height_trace=false;
  std::map<std::string,Variant> variants;
  variants["pp_h180"].params=base;
  auto coarse=base;
  coarse.num_sectors_each_zone={8,16,27,16};
  variants["pp_coarse_sector"].params=coarse;
  coarse.num_rings_each_zone={1,2,2,2};
  variants["pp_coarse_sector_ring"].params=coarse;
  auto min20=base; min20.num_min_pts=20;
  variants["pp_min_pts20"].params=min20;
  variants["pp_far_guard_25cm"].params=base;
  variants["pp_far_guard_25cm"].guard_above_ground_m=.25;
  variants["pp_far_guard_25cm"].guard_from_radius_m=12.;
  variants["pp_far_guard_15cm"].params=base;
  variants["pp_far_guard_15cm"].guard_above_ground_m=.15;
  variants["pp_far_guard_15cm"].guard_from_radius_m=12.;
  variants["pp_all_guard_25cm"].params=base;
  variants["pp_all_guard_25cm"].guard_above_ground_m=.25;
  variants["pp_all_guard_25cm"].guard_from_radius_m=0.;
  variants["pp_all_guard_15cm"].params=base;
  variants["pp_all_guard_15cm"].guard_above_ground_m=.15;
  variants["pp_all_guard_15cm"].guard_from_radius_m=0.;
  // Keep the original implementation quiet. The review record keeps all outputs.
  std::ofstream discard("/dev/null"); auto* saved=std::cout.rdbuf(discard.rdbuf());
  out << std::setprecision(17);
  std::string line,phase;std::int64_t last_stamp=0;int count=0;
  while (std::getline(in,line)) {
    if (line.empty()) continue;
    const auto row=YAML::Load(line);
    for (const char* key:{"input_id","phase","cloud_path","pitch","geometry_valid",
                          "scan_effective_stamp_ns","scan_arrival_ns"})
      if (!row[key]) throw std::runtime_error(std::string("missing ")+key);
    const auto id=row["input_id"].as<std::string>();
    const auto current=row["phase"].as<std::string>();
    if (current!=phase) {
      for (auto& [name,v]:variants) v.model.reset(new patchwork::PatchWorkpp(v.params));
      phase=current;last_stamp=0;
    }
    const auto stamp=row["scan_effective_stamp_ns"].as<std::int64_t>();
    const auto arrival=row["scan_arrival_ns"].as<std::int64_t>();
    if (stamp<=last_stamp || arrival<stamp || !row["geometry_valid"].as<bool>())
      throw std::runtime_error("bad stamp or geometry "+id);
    last_stamp=stamp;
    const float pitch=row["pitch"].as<float>();
    if (!std::isfinite(pitch)) throw std::runtime_error("invalid pitch "+id);
    const auto rel=fs::path(row["cloud_path"].as<std::string>());
    if (rel.is_absolute() || rel.string().find("..")!=std::string::npos)
      throw std::runtime_error("unsafe cloud path");
    const Cloud raw=Read(snapshots.parent_path()/rel);
    Cloud roi;std::vector<std::size_t> originals;
    roi.reserve(raw.size());originals.reserve(raw.size());
    for (std::size_t i=0;i<raw.size();++i) {
      const auto& p=raw[i];
      if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||!std::isfinite(p.intensity)) continue;
      const double r=std::sqrt(double(p.x)*p.x+double(p.y)*p.y+double(p.z)*p.z);
      if (r<.5||r>100||p.x< -30||p.x>50||p.y< -20||p.y>20||p.z< -2||p.z>3) continue;
      const double bx=p.x+.8;
      if (bx>=-.700&&bx<=3.845&&p.y>=-.946&&p.y<=.946) continue;
      roi.push_back(p); originals.push_back(i);
    }
    roi.width=roi.size();roi.height=1;roi.is_dense=false;
    pcl::VoxelGrid<pcl::PointXYZI> grid;
    grid.setInputCloud(roi.makeShared());grid.setLeafSize(.1f,.1f,.1f);
    grid.setSaveLeafLayout(true);
    Cloud voxel;grid.filter(voxel);
    if (voxel.empty()) throw std::runtime_error("empty voxel "+id);
    Eigen::MatrixXf matrix(voxel.size(),4);
    const float cp=std::cos(pitch),sp=std::sin(pitch);
    for (std::size_t i=0;i<voxel.size();++i) {
      const auto& p=voxel[i];
      matrix.row(Eigen::Index(i))<<cp*p.x+sp*p.z,p.y,-sp*p.x+cp*p.z,p.intensity;
    }
    out << "{\"input_id\":\""<<id<<"\",\"stamp_ns\":"<<stamp<<",\"methods\":{";
    bool first=true;
    for (auto& [name,v]:variants) {
      const auto start=Clock::now();
      Mask mask=Segment(matrix,*v.model);
      const auto segmented=Clock::now();
      if (v.guard_above_ground_m>=0) {
        const float cutoff=float(-v.params.sensor_height+v.guard_above_ground_m);
        for (std::size_t i=0;i<mask.size();++i) {
          if (mask[i]!=1) continue;
          const float x=matrix(Eigen::Index(i),0),y=matrix(Eigen::Index(i),1),z=matrix(Eigen::Index(i),2);
          if (std::hypot(x,y)>=v.guard_from_radius_m && z>cutoff) mask[i]=2;
        }
      }
      Mask original(raw.size(),0);
      for (std::size_t i=0;i<roi.size();++i) {
        const int voxel_id=grid.getCentroidIndex(roi[i]);
        if (voxel_id<0 || std::size_t(voxel_id)>=mask.size()) throw std::runtime_error("voxel lineage drift");
        original[originals[i]]=mask[std::size_t(voxel_id)];
      }
      if (!first) out<<',';
      first=false;
      out << '"'<<name<<"\":{\"segment_ms\":"<<Ms(start,segmented)<<",\"mask\":";
      WriteMask(out,original);out << '}';
    }
    out << "}}\n";
    ++count;
    if (count%200==0) std::cerr << "processed "<<count<<'\n';
  }
  std::cout.rdbuf(saved);
  if (!out || count==0) throw std::runtime_error("empty or failed output");
  std::cerr << "complete "<<count<<'\n';
  return 0;
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';return 1;
}
