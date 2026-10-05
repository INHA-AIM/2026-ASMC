// L2-A2 isolated four-way ground comparison; no ROS or simulator control.
#include <asmc_lidar/core.hpp>
#include <asmc_lidar/general.hpp>
#include <patchwork/patchworkpp.h>
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>
namespace fs=std::filesystem;
using Cloud=asmc_lidar::Cloud;
using Clock=std::chrono::steady_clock;
double Ms(Clock::time_point t){return std::chrono::duration<double,std::milli>(Clock::now()-t).count();}
Cloud::Ptr Read(const fs::path& path){
  std::ifstream in(path,std::ios::binary|std::ios::ate);
  if(!in)throw std::runtime_error("missing cloud: "+path.string());
  auto size=in.tellg();if(size<=0||size%16)throw std::runtime_error("bad XYZI");
  in.seekg(0);Cloud::Ptr out(new Cloud);out->reserve(std::size_t(size)/16);
  float a[4];while(in.read(reinterpret_cast<char*>(a),sizeof(a))){
    pcl::PointXYZI p;p.x=a[0];p.y=a[1];p.z=a[2];p.intensity=a[3];out->push_back(p);
  }
  out->width=out->size();out->height=1;out->is_dense=false;return out;
}
std::string Q(const std::string& s){
  std::ostringstream o;o<<'"';for(unsigned char c:s){
    if(c=='"'||c=='\\')o<<'\\'<<c;
    else if(c<32)o<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;
    else o<<c;
  }o<<'"';return o.str();
}
void Mask(std::ostream& o,const std::vector<std::uint8_t>& x){
  o<<'[';for(std::size_t i=0;i<x.size();++i){if(i)o<<',';o<<unsigned(x[i]);}o<<']';
}
// 0 outside ROI/unassigned, 1 ground, 2 not ground, 3 common ceiling exclusion.
std::uint8_t RefClass(std::uint8_t v){
  if(v==tunnel_filter::kGround)return 1;
  if(v==tunnel_filter::kCeiling)return 3;
  return v==tunnel_filter::kInvalid?0:2;
}
struct Time{double level=0,copy=0,segment=0,restore=0,total=0;};
void TimeJson(std::ostream& o,const Time& t){
  o<<"{\"level\":"<<t.level<<",\"copy\":"<<t.copy
   <<",\"segment\":"<<t.segment<<",\"restore\":"<<t.restore
   <<",\"total\":"<<t.total<<'}';
}
std::vector<std::uint8_t> Patch(const Cloud& cloud,const std::array<float,9>& rotation,
                                bool ceiling,float ceiling_z,patchwork::PatchWorkpp* pp,Time* t){
  auto start=Clock::now();Cloud leveled;
  tunnel_filter::ApplyRotation(cloud,rotation,&leveled);t->level=Ms(start);
  auto copy_start=Clock::now();
  std::vector<std::uint8_t> labels(cloud.size(),0);std::vector<int> indices;
  for(std::size_t i=0;i<leveled.size();++i){
    if(ceiling&&leveled[i].z>ceiling_z)labels[i]=3;
    else indices.push_back(int(i));
  }
  Eigen::MatrixXf matrix(indices.size(),4);
  for(std::size_t i=0;i<indices.size();++i){
    const auto& p=leveled[std::size_t(indices[i])];
    matrix.row(Eigen::Index(i))<<p.x,p.y,p.z,p.intensity;
  }
  t->copy=Ms(copy_start);
  if(!indices.empty()){
    auto segment_start=Clock::now();pp->estimateGround(matrix);t->segment=Ms(segment_start);
    auto restore_start=Clock::now();
    auto fill=[&](const Eigen::VectorXi& selected,std::uint8_t label){
      for(Eigen::Index j=0;j<selected.size();++j){
        const int k=selected(j);
        if(k<0||std::size_t(k)>=indices.size())throw std::runtime_error("PP index range");
        const int original=indices[std::size_t(k)];
        if(labels[std::size_t(original)])throw std::runtime_error("PP duplicate index");
        labels[std::size_t(original)]=label;
      }
    };
    fill(pp->getGroundIndices(),1);fill(pp->getNongroundIndices(),2);
    t->restore=Ms(restore_start);
  }
  t->total=Ms(start);return labels;
}
std::string Fnv(const Cloud& cloud){
  std::uint64_t h=14695981039346656037ULL;
  for(const auto& p:cloud.points)for(float v:{p.x,p.y,p.z,p.intensity}){
    const auto* b=reinterpret_cast<const unsigned char*>(&v);
    for(std::size_t i=0;i<sizeof(float);++i){h^=b[i];h*=1099511628211ULL;}
  }
  std::ostringstream o;o<<std::hex<<std::setw(16)<<std::setfill('0')<<h;return o.str();
}
int main(int argc,char** argv)try{
  if(argc==4&&std::string(argv[1])=="--general-mask"){
    const fs::path output=argv[3];
    if(fs::exists(output))throw std::runtime_error("output exists");
    auto input=Read(argv[2]);
    auto result=asmc_lidar::ProcessGeneralReference(*input,0,asmc_lidar::PinnedLShapeObb());
    std::ofstream mask(output,std::ios::binary);
    if(!mask)throw std::runtime_error("cannot write mask");
    for(const auto value:result.labels){
      const auto ground=value==tunnel_filter::kGround?1:
                        value==tunnel_filter::kWall?3:2;
      mask.put(char(ground));
    }
    if(!mask)throw std::runtime_error("mask write failure");
    return 0;
  }
  if(argc!=6)throw std::runtime_error(
    "usage: asmc_lidar_ground_compare snapshots.jsonl calibration.yaml reference.yaml tunnel|general output.jsonl");
  fs::path input=argv[1],output=argv[5];
  if(fs::exists(output))throw std::runtime_error("output exists");
  const std::string profile=argv[4];
  if(profile!="tunnel"&&profile!="general")throw std::runtime_error("bad profile");
  auto cal=asmc_lidar::LoadCalibration(argv[2]);
  auto params=asmc_lidar::LoadReferenceParams(argv[3]);
  asmc_lidar::Processor reference(asmc_lidar::Mode::Reference,cal,params);
  tunnel_filter::TunnelPipeline reference_voxel(params,asmc_lidar::PinnedLShapeObb());
  tunnel_filter::TunnelPipeline reference_raw(params,asmc_lidar::PinnedLShapeObb());
  patchwork::Params pp_params;pp_params.sensor_height=1.80;pp_params.min_range=1;
  pp_params.max_range=60;pp_params.enable_RNR=false;
  std::unique_ptr<patchwork::PatchWorkpp> pp_voxel,pp_raw;
  std::ifstream source(input);if(!source)throw std::runtime_error("missing snapshots");
  std::ofstream dest(output);if(!dest)throw std::runtime_error("cannot write output");
  dest<<std::setprecision(12);std::string line,phase;int rows=0;
  while(std::getline(source,line)){
    if(line.empty())continue;
    const auto row=YAML::Load(line);
    for(const char* key:{"input_id","phase","capture_key","cloud_path","roll","pitch",
                         "geometry_valid","scan_effective_stamp_ns","scan_arrival_ns",
                         "controlled_camera_state","measurement"})
      if(!row[key])throw std::runtime_error(std::string("missing ")+key);
    auto current=row["phase"].as<std::string>();
    if(current!=phase){
      reference.Reset();reference_voxel.Reset();reference_raw.Reset();
      pp_voxel.reset(new patchwork::PatchWorkpp(pp_params));
      pp_raw.reset(new patchwork::PatchWorkpp(pp_params));phase=current;
    }
    auto rel=row["cloud_path"].as<std::string>();
    if(fs::path(rel).is_absolute()||rel.find("..")!=std::string::npos)
      throw std::runtime_error("unsafe cloud path");
    auto camera=row["controlled_camera_state"].as<std::string>();
    if(camera!="missing"&&camera!="empty")throw std::runtime_error("unsupported camera state");
    if(row["already_preprocessed"]&&row["already_preprocessed"].as<bool>())
      throw std::runtime_error("raw input required");
    asmc_lidar::FrameInput frame;frame.cloud=Read(input.parent_path()/rel);
    frame.stamp_ns=row["scan_effective_stamp_ns"].as<std::int64_t>();
    frame.receipt_ns=row["scan_arrival_ns"].as<std::int64_t>();
    frame.roll_rad=row["roll"].as<double>();frame.pitch_rad=row["pitch"].as<double>();
    frame.imu_valid=row["geometry_valid"].as<bool>();
    frame.camera_available=camera=="empty";
    frame.allow_repeated_snapshot=row["repetition_role"]&&
      row["repetition_role"].as<std::string>()=="stationary_snapshot_warmup_not_tracking";
    frame.profile=profile=="tunnel"?asmc_lidar::ReferenceProfile::Tunnel:
                                    asmc_lidar::ReferenceProfile::General;
    auto base=reference.Process(frame);
    if(base.status!="valid")throw std::runtime_error("reference "+base.status);
    Cloud roi;std::vector<int> originals;
    for(std::size_t i=0;i<frame.cloud->size();++i)if(base.raw_to_voxel[i]>=0){
      roi.push_back((*frame.cloud)[i]);originals.push_back(int(i));
    }
    roi.width=roi.size();roi.height=1;roi.is_dense=false;
    pcl::VoxelGrid<pcl::PointXYZI> vg;Cloud voxel;
    vg.setInputCloud(roi.makeShared());vg.setLeafSize(.1f,.1f,.1f);
    vg.setSaveLeafLayout(true);vg.filter(voxel);
    if(voxel.size()!=base.voxel_points||Fnv(voxel)!=base.voxel_fnv1a64)
      throw std::runtime_error("voxel fingerprint drift");
    for(std::size_t i=0;i<roi.size();++i)
      if(vg.getCentroidIndex(roi[i])!=base.raw_to_voxel[std::size_t(originals[i])])
        throw std::runtime_error("voxel lineage drift");
    tunnel_filter::FrameResult voxel_general,raw_general;
    const tunnel_filter::FrameResult *vref,*rref;
    std::vector<std::uint8_t> pv(voxel.size(),0),av(voxel.size(),0);
    std::vector<std::uint8_t> pr(roi.size(),0),ar(roi.size(),0);
    if(profile=="tunnel"){
      vref=&reference_voxel.Process(voxel,float(frame.roll_rad),float(frame.pitch_rad),
                                     &pv,&av,frame.camera_available,&frame.motion);
      rref=&reference_raw.Process(roi,float(frame.roll_rad),float(frame.pitch_rad),
                                  &pr,&ar,frame.camera_available,&frame.motion);
    }else{
      voxel_general=asmc_lidar::ProcessGeneralReference(voxel,float(frame.pitch_rad),
                                                         asmc_lidar::PinnedLShapeObb());
      raw_general=asmc_lidar::ProcessGeneralReference(roi,float(frame.pitch_rad),
                                                       asmc_lidar::PinnedLShapeObb());
      vref=&voxel_general;rref=&raw_general;
    }
    if(vref->labels.size()!=base.reference_labels.size())
      throw std::runtime_error("reference point count drift");
    for(std::size_t i=0;i<vref->labels.size();++i)
      if(RefClass(vref->labels[i])!=RefClass(base.reference_labels[i]))
        throw std::runtime_error("reference ground or ceiling drift");
    auto rotation=profile=="tunnel"?
      tunnel_filter::LevelingRotation(float(frame.roll_rad),float(frame.pitch_rad),
                                     params.level_roll,params.level_pitch):
      tunnel_filter::LevelingRotation(0,float(frame.pitch_rad),false,true);
    Time tv,tr;
    auto pvoxel=Patch(voxel,rotation,profile=="tunnel",params.ceiling_max_z_lidar_m,
                      pp_voxel.get(),&tv);
    auto praw=Patch(roi,rotation,profile=="tunnel",params.ceiling_max_z_lidar_m,
                    pp_raw.get(),&tr);
    std::vector<std::uint8_t> rv(frame.cloud->size(),0),rr(rv),cv(rv),cr(rv);
    for(std::size_t i=0;i<roi.size();++i){
      std::size_t raw=std::size_t(originals[i]);
      int vi=base.raw_to_voxel[raw];
      if(vi<0||std::size_t(vi)>=pvoxel.size())throw std::runtime_error("bad voxel index");
      rv[raw]=RefClass(vref->labels[std::size_t(vi)]);
      rr[raw]=RefClass(rref->labels[i]);
      cv[raw]=pvoxel[std::size_t(vi)];cr[raw]=praw[i];
    }
    dest<<"{\"input_id\":"<<Q(row["input_id"].as<std::string>())
        <<",\"capture_key\":"<<Q(row["capture_key"].as<std::string>())
        <<",\"phase\":"<<Q(phase)<<",\"profile\":"<<Q(profile)
        <<",\"camera\":"<<Q(camera)
        <<",\"measurement\":"<<(row["measurement"].as<bool>()?"true":"false")
        <<",\"stamp_ns\":"<<frame.stamp_ns
        <<",\"raw_points\":"<<frame.cloud->size()
        <<",\"roi_points\":"<<roi.size()
        <<",\"voxel_points\":"<<voxel.size()
        <<",\"voxel_fnv1a64\":"<<Q(base.voxel_fnv1a64)
        <<",\"ref_voxel_ground_valid\":"<<(vref->ground_fit.valid?"true":"false")
        <<",\"ref_voxel_fallback\":"<<(vref->ground_fallback_used?"true":"false")
        <<",\"ref_raw_ground_valid\":"<<(rref->ground_fit.valid?"true":"false")
        <<",\"ref_raw_fallback\":"<<(rref->ground_fallback_used?"true":"false")
        <<",\"ref_voxel\":";Mask(dest,rv);
    dest<<",\"ref_raw\":";Mask(dest,rr);
    dest<<",\"patch_voxel\":";Mask(dest,cv);
    dest<<",\"patch_raw\":";Mask(dest,cr);
    dest<<",\"timing_ms\":{\"preprocess\":"<<base.preprocessing_ms
        <<",\"ref_voxel_level\":"<<vref->timings.level_ms
        <<",\"ref_voxel_ground\":"<<vref->timings.ground_fit_ms+vref->timings.ground_remove_ms
        <<",\"ref_raw_level\":"<<rref->timings.level_ms
        <<",\"ref_raw_ground\":"<<rref->timings.ground_fit_ms+rref->timings.ground_remove_ms
        <<",\"patch_voxel\":";TimeJson(dest,tv);
    dest<<",\"patch_raw\":";TimeJson(dest,tr);dest<<"}}\n";
    ++rows;
  }
  if(!dest)throw std::runtime_error("write failed");
  std::cerr<<"ground comparison rows="<<rows<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
