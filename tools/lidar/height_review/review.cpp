// Isolated, read-only bag snapshot review. Does not alter operating LiDAR.
#include <patchwork/patchworkpp.h>
#include <asmc_lidar/core.hpp>
#include <asmc_lidar/general.hpp>
#include <cstring>
#include <Lidar/TunnelFilter/TunnelFilterCore.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
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
  if(bytes<=0||bytes%16)throw std::runtime_error("bad XYZI");
  in.seekg(0); Cloud out;out.reserve(std::size_t(bytes)/16);
  float a[4];while(in.read(reinterpret_cast<char*>(a),sizeof(a))) {
    pcl::PointXYZI p;p.x=a[0];p.y=a[1];p.z=a[2];p.intensity=a[3];out.push_back(p);
  }
  out.width=out.size();out.height=1;out.is_dense=false;return out;
}
void PrintMask(std::ostream& out,const std::vector<std::uint8_t>& labels) {
  out<<'[';for(std::size_t i=0;i<labels.size();++i){if(i)out<<',';out<<unsigned(labels[i]);}out<<']';
}
int main(int argc,char** argv)try {
  if(argc!=7)throw std::runtime_error("usage: review snapshots.jsonl raw|voxel pitch|full original|singleton|fixed|jaeho trace|quiet output.jsonl");
  const fs::path input=argv[1],output=argv[6];
  const std::string representation=argv[2],attitude=argv[3];
  const double sensor_height=1.80;
  const std::string policy=argv[4], trace=argv[5];
  if((representation!="raw"&&representation!="voxel") ||
     (attitude!="none"&&attitude!="pitch"&&attitude!="full")||
     (policy!="original"&&policy!="singleton"&&policy!="fixed"&&policy!="jaeho")||
     (trace!="trace"&&trace!="quiet")||(policy=="jaeho"&&attitude!="pitch")||fs::exists(output))
    throw std::runtime_error("invalid arguments or output exists");
  std::ifstream source(input);std::ofstream dest(output);
  if(!source||!dest)throw std::runtime_error("cannot open input/output");
  patchwork::Params params;params.sensor_height=sensor_height;params.min_range=1;
  params.max_range=60;params.enable_RNR=false;
  params.height_policy=policy=="fixed"?2:policy=="singleton"?1:0;params.height_trace=trace=="trace";
  std::unique_ptr<patchwork::PatchWorkpp> pp;
  std::string line,phase;int rows=0;std::int64_t last_stamp=0;
  dest<<std::setprecision(17);
  while(std::getline(source,line)) {
    if(line.empty())continue;
    YAML::Node row=YAML::Load(line);
    for(const char* key:{"input_id","phase","cloud_path","roll","pitch","geometry_valid",
       "scan_effective_stamp_ns","scan_arrival_ns","controlled_camera_state"})
      if(!row[key])throw std::runtime_error(std::string("missing ")+key);
    if(!row["geometry_valid"].as<bool>())throw std::runtime_error("invalid geometry");
    const auto stamp=row["scan_effective_stamp_ns"].as<std::int64_t>();
    const auto arrival=row["scan_arrival_ns"].as<std::int64_t>();
    if(stamp<=0||arrival<stamp)throw std::runtime_error("invalid timestamp");
    if(row["imu_stamp_ns"]){
      auto imu=row["imu_stamp_ns"].as<std::int64_t>();
      if(imu<=0||imu>stamp||stamp-imu>100000000||row["imu_arrival_ns"].as<std::int64_t>()>arrival)
        throw std::runtime_error("noncausal/stale IMU");
    }
    const auto camera=row["controlled_camera_state"].as<std::string>();
    if(camera!="missing"&&camera!="empty")throw std::runtime_error("unsupported camera");
    if(!std::isfinite(row["roll"].as<double>())||!std::isfinite(row["pitch"].as<double>()))
      throw std::runtime_error("NaN attitude");
    const auto current=row["phase"].as<std::string>();
    if(current!=phase) {pp.reset(new patchwork::PatchWorkpp(params));phase=current;last_stamp=0;}
    if(stamp<=last_stamp)throw std::runtime_error("duplicate/regressing stamp");
    last_stamp=stamp;
    const fs::path rel=row["cloud_path"].as<std::string>();
    if(rel.is_absolute()||rel.string().find("..")!=std::string::npos)throw std::runtime_error("unsafe path");
    const Cloud original=Read(input.parent_path()/rel);
    const auto total_begin=Clock::now();
    Cloud roi;std::vector<std::size_t> lineage;roi.reserve(original.size());
    for(std::size_t i=0;i<original.size();++i) {
      const auto& p=original[i];
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||!std::isfinite(p.intensity))continue;
      const double r=std::sqrt(double(p.x)*p.x+double(p.y)*p.y+double(p.z)*p.z);
      if(r<.5||r>100||p.x< -30||p.x>50||p.y< -20||p.y>20||p.z< -2||p.z>3)continue;
      const double bx=p.x+.80;
      if(bx>=-.700&&bx<=3.845&&p.y>=-.946&&p.y<=.946)continue;
      roi.push_back(p);lineage.push_back(i);
    }
    roi.width=roi.size();roi.height=1;roi.is_dense=false;
    Cloud voxel;std::vector<int> source_to_voxel;
    if(representation=="voxel") {
      pcl::VoxelGrid<pcl::PointXYZI> grid;grid.setInputCloud(roi.makeShared());
      grid.setLeafSize(.1f,.1f,.1f);grid.setSaveLeafLayout(true);grid.filter(voxel);
      source_to_voxel.reserve(roi.size());
      for(const auto& p:roi)source_to_voxel.push_back(grid.getCentroidIndex(p));
    }
    const Cloud& points=representation=="voxel"?voxel:roi;
    const auto prep_end=Clock::now();
    const auto roll=row["roll"].as<float>();const auto pitch=row["pitch"].as<float>();
    const auto matrix=tunnel_filter::LevelingRotation(roll,pitch,attitude=="full",attitude!="none");
    Cloud leveled;tunnel_filter::ApplyRotation(points,matrix,&leveled);
    const auto level_end=Clock::now();
    Eigen::MatrixXf xyzi(leveled.size(),4);
    for(std::size_t i=0;i<leveled.size();++i)
      xyzi.row(Eigen::Index(i))<<leveled[i].x,leveled[i].y,leveled[i].z,leveled[i].intensity;
    const auto copy_end=Clock::now();
    for(std::size_t i=0;i<points.size();++i)
      if(std::memcmp(&points[i].intensity,&leveled[i].intensity,4))throw std::runtime_error("intensity changed");
    const auto before=pp->getHeight();const auto begin=Clock::now();
    std::vector<std::uint8_t> ref_labels;
    if(policy=="jaeho"){
      const auto ref=asmc_lidar::ProcessGeneralReference(points,pitch,asmc_lidar::PinnedLShapeObb());
      for(auto v:ref.labels)ref_labels.push_back(v==tunnel_filter::kGround?1:v==tunnel_filter::kWall?3:2);
    }else if(!points.empty())pp->estimateGround(xyzi);
    const auto milliseconds=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();
    const auto after=pp->getHeight();const auto restore_begin=Clock::now();
    std::vector<std::uint8_t> labels(points.size(),0);
    auto fill=[&](const Eigen::VectorXi& ids,std::uint8_t type){
      for(Eigen::Index j=0;j<ids.size();++j){
        const int i=ids[j];if(i<0||std::size_t(i)>=labels.size()||labels[std::size_t(i)])
          throw std::runtime_error("index missing, duplicate, or out of range");
        labels[std::size_t(i)]=type;
      }
    };
    if(policy=="jaeho")labels=ref_labels;
    else {fill(pp->getGroundIndices(),1);fill(pp->getNongroundIndices(),2);}
    std::vector<std::uint8_t> restored(original.size(),0);
    for(std::size_t i=0;i<roi.size();++i){
      const int j=representation=="voxel"?source_to_voxel[i]:int(i);
      if(j<0||std::size_t(j)>=labels.size())throw std::runtime_error("voxel lineage invalid");
      restored[lineage[i]]=labels[std::size_t(j)];
    }
    const auto restored_end=Clock::now();
    auto ms=[](auto a,auto b){return std::chrono::duration<double,std::milli>(b-a).count();};
    dest<<"{\"input_id\":\""<<row["input_id"].as<std::string>()
        <<"\",\"phase\":\""<<phase<<"\",\"stamp_ns\":"<<row["scan_effective_stamp_ns"].as<std::int64_t>()
        <<",\"height_before\":"<<before<<",\"height_after\":"<<after
        <<",\"segment_ms\":"<<milliseconds<<",\"roi_points\":"<<roi.size()
        <<",\"processed_points\":"<<points.size()<<",\"mask\":";
    PrintMask(dest,restored);
    dest<<",\"arrival_ns\":"<<arrival<<",\"timing_ms\":{\"preprocess\":"<<ms(total_begin,prep_end)
      <<",\"level\":"<<ms(prep_end,level_end)<<",\"copy\":"<<ms(level_end,copy_end)
      <<",\"segment\":"<<milliseconds<<",\"restore\":"<<ms(restore_begin,restored_end)
      <<",\"total\":"<<ms(total_begin,restored_end)<<"},\"patches\":[";
    for(std::size_t j=0;j<pp->height_patches.size();++j){if(j)dest<<',';const auto d=pp->height_patches[j];
      dest<<"{\"zone\":"<<d.zone<<",\"ring\":"<<d.ring<<",\"sector\":"<<d.sector
        <<",\"points\":"<<d.count<<",\"plane_points\":"<<d.seed_count<<",\"z\":"<<d.z
        <<",\"normal\":["<<d.nx<<','<<d.ny<<','<<d.nz<<"],\"plane_variance_m2\":"<<d.variance
        <<",\"elevation_threshold\":"<<d.elevation_threshold<<",\"accepted\":"<<(d.accepted?"true":"false")<<'}';
    }
    dest<<"],\"updates\":[";
    for(std::size_t j=0;j<pp->height_updates.size();++j){if(j)dest<<',';const auto d=pp->height_updates[j];
      dest<<"{\"ring\":"<<d.ring<<",\"history_count\":"<<d.count<<",\"before\":"<<d.before
        <<",\"after\":"<<d.after<<",\"threshold_before\":"<<d.threshold_before
        <<",\"threshold_after\":"<<d.threshold_after<<",\"reason\":"<<std::quoted(d.reason)<<'}';
    }dest<<"]}\n";
    ++rows;
  }
  if(!dest||rows==0)throw std::runtime_error("write failure");
  std::cerr<<"rows="<<rows<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
