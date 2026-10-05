// Exact PCL-voxel adaptation of pinned Patchwork++ masks. No GT enters inference.
#include <jsoncpp/json/json.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
namespace fs=std::filesystem;
using Cloud=pcl::PointCloud<pcl::PointXYZI>;

Json::Value Parse(const std::string& row) {
  Json::CharReaderBuilder builder;
  std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  Json::Value result;std::string error;
  if (!reader->parse(row.data(),row.data()+row.size(),&result,&error))
    throw std::runtime_error("invalid JSON: "+error);
  return result;
}
Cloud Read(const fs::path& path) {
  std::ifstream in(path,std::ios::binary|std::ios::ate);
  if(!in)throw std::runtime_error("missing cloud: "+path.string());
  auto bytes=in.tellg();if(bytes<=0||bytes%16)throw std::runtime_error("invalid XYZI");
  in.seekg(0);Cloud out;out.reserve(std::size_t(bytes)/16);float f[4];
  while(in.read(reinterpret_cast<char*>(f),sizeof(f))) {
    pcl::PointXYZI p;p.x=f[0];p.y=f[1];p.z=f[2];p.intensity=f[3];out.push_back(p);
  }
  out.width=out.size();out.height=1;out.is_dense=false;return out;
}
int main(int argc,char** argv) try {
  if(argc!=4)throw std::runtime_error("usage: patchwork_adaptive_exact snapshots.jsonl sweep.jsonl output.jsonl");
  const fs::path snapshots=argv[1],sweep=argv[2],output=argv[3];
  if(fs::exists(output))throw std::runtime_error("output exists");
  std::ifstream in(snapshots),results(sweep);std::ofstream out(output);
  if(!in||!results||!out)throw std::runtime_error("cannot open input/output");
  Json::StreamWriterBuilder writer;writer["indentation"]="";
  std::string source_line,result_line;int frames=0;
  while(std::getline(in,source_line)) {
    if(!std::getline(results,result_line))throw std::runtime_error("truncated result");
    Json::Value source=Parse(source_line),sweep_row=Parse(result_line);
    const auto id=source["input_id"].asString();
    if(id!=sweep_row["input_id"].asString()||
       source["scan_effective_stamp_ns"].asInt64()!=sweep_row["stamp_ns"].asInt64())
      throw std::runtime_error("identity/stamp mismatch: "+id);
    const auto rel=fs::path(source["cloud_path"].asString());
    if(rel.is_absolute()||rel.string().find("..")!=std::string::npos)
      throw std::runtime_error("unsafe cloud path");
    const Cloud raw=Read(snapshots.parent_path()/rel);
    const auto& base_json=sweep_row["methods"]["pp_h180"]["mask"];
    const auto& fixed_json=sweep_row["methods"]["pp_all_guard_25cm"]["mask"];
    if(base_json.size()!=raw.size()||fixed_json.size()!=raw.size())
      throw std::runtime_error("mask length mismatch: "+id);
    Cloud roi;roi.reserve(raw.size());std::vector<std::size_t> originals;
    originals.reserve(raw.size());
    for(std::size_t i=0;i<raw.size();++i) {
      const auto b=base_json[Json::ArrayIndex(i)].asInt();
      const auto f=fixed_json[Json::ArrayIndex(i)].asInt();
      if(b<0||b>2||f<0||f>2||((b==0)!=(f==0)))throw std::runtime_error("mask/ROI drift");
      if(b) {roi.push_back(raw[i]);originals.push_back(i);}
    }
    roi.width=roi.size();roi.height=1;roi.is_dense=false;
    pcl::VoxelGrid<pcl::PointXYZI> grid;grid.setInputCloud(roi.makeShared());
    grid.setLeafSize(.1f,.1f,.1f);grid.setSaveLeafLayout(true);
    Cloud voxel;grid.filter(voxel);
    if(voxel.empty())throw std::runtime_error("empty ROI voxel");
    std::vector<int> voxel_mask(voxel.size(),0),ids(roi.size(),-1);
    for(std::size_t i=0;i<roi.size();++i) {
      const int j=grid.getCentroidIndex(roi[i]);
      if(j<0||std::size_t(j)>=voxel.size())throw std::runtime_error("bad voxel index");
      const int label=base_json[Json::ArrayIndex(originals[i])].asInt();
      if(voxel_mask[std::size_t(j)]&&voxel_mask[std::size_t(j)]!=label)
        throw std::runtime_error("baseline voxel label drift: "+id);
      voxel_mask[std::size_t(j)]=label;ids[i]=j;
    }
    const float pitch=source["pitch"].asFloat(),cp=std::cos(pitch),sp=std::sin(pitch);
    std::vector<float> support;support.reserve(voxel.size());
    for(std::size_t i=0;i<voxel.size();++i) {
      const auto& p=voxel[i];const float x=cp*p.x+sp*p.z,z=-sp*p.x+cp*p.z;
      if(voxel_mask[i]==1&&x>=3&&x<=12&&std::abs(p.y)<=2.5&&z>=-2.2&&z<=-1.2)
        support.push_back(z);
    }
    const bool valid=support.size()>=30;
    float floor_z=0;
    if(valid){std::sort(support.begin(),support.end());floor_z=support[support.size()/4];}
    int changed_voxels=0;
    if(valid)for(std::size_t i=0;i<voxel.size();++i) {
      const auto& p=voxel[i];const float z=-sp*p.x+cp*p.z;
      if(voxel_mask[i]==1&&z>floor_z+.25f){voxel_mask[i]=2;++changed_voxels;}
    }
    Json::Value candidate(Json::objectValue);candidate["segment_ms"]=0;
    candidate["timing_note"]="post-mask research probe; zero is not measured latency";
    candidate["support_count"]=Json::UInt64(support.size());
    if(valid)candidate["estimated_floor_z"]=floor_z;
    else candidate["estimated_floor_z"]=Json::nullValue;
    candidate["changed_voxels"]=changed_voxels;
    Json::Value mask(Json::arrayValue);
    mask.resize(Json::ArrayIndex(raw.size()));
    for(std::size_t i=0;i<raw.size();++i)
      mask[Json::ArrayIndex(i)]=base_json[Json::ArrayIndex(i)];
    for(std::size_t i=0;i<roi.size();++i)
      mask[Json::ArrayIndex(originals[i])]=voxel_mask[std::size_t(ids[i])];
    candidate["mask"]=std::move(mask);
    Json::Value result(Json::objectValue);
    result["input_id"]=id;result["stamp_ns"]=sweep_row["stamp_ns"];
    result["methods"]["pp_h180"]=sweep_row["methods"]["pp_h180"];
    result["methods"]["pp_all_guard_25cm"]=sweep_row["methods"]["pp_all_guard_25cm"];
    result["methods"]["pp_adaptive_guard_25cm"]=std::move(candidate);
    out<<Json::writeString(writer,result)<<'\n';
    ++frames;if(frames%200==0)std::cerr<<"processed "<<frames<<'\n';
  }
  if(std::getline(results,result_line)||frames==0||!out)throw std::runtime_error("output or count mismatch");
  std::cerr<<"complete "<<frames<<'\n';return 0;
} catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
