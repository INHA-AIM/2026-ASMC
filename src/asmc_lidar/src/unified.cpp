#include <asmc_lidar/unified.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace asmc_lidar {
namespace {
using Clock=std::chrono::steady_clock;
double Ms(Clock::time_point start) {
  return std::chrono::duration<double,std::milli>(Clock::now()-start).count();
}
std::vector<pcl::PointIndices> Clusters(const Cloud::Ptr& cloud,double tolerance,
                                        int minimum,int maximum) {
  std::vector<pcl::PointIndices> clusters;
  if(cloud->size()<std::size_t(minimum))return clusters;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> extractor;
  extractor.setClusterTolerance(tolerance);
  extractor.setMinClusterSize(minimum);
  extractor.setMaxClusterSize(maximum);
  extractor.setSearchMethod(tree);
  extractor.setInputCloud(cloud);
  extractor.extract(clusters);
  return clusters;
}
Eigen::Vector3d Vec(const pcl::PointXYZ& p){return {p.x,p.y,p.z};}
Eigen::Matrix3d Rotation(const std::array<float,9>& a){
  Eigen::Matrix3d r;
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)r(i,j)=a[3*i+j];
  return r;
}
std::string ContextState(const OptionalContext* c,std::int64_t stamp){
  if(!c)return "missing";
  if(!c->valid||c->source.empty()||!std::isfinite(c->quality)||c->quality<=0||c->quality>1||
     c->observation_ns<=0)return "invalid";
  if(c->frame_id!="base_link")return "frame_mismatch";
  if(stamp<c->observation_ns)return "future";
  if(stamp-c->observation_ns>100000000)return "stale";
  return "available";
}
}  // namespace
bool ContextUsable(const OptionalContext& c,std::int64_t stamp,std::int64_t max_age_ns){
  return c.valid&&c.frame_id=="base_link"&&!c.source.empty()&&
         std::isfinite(c.quality)&&c.quality>0&&c.quality<=1&&
         c.observation_ns>0&&stamp>=c.observation_ns&&
         stamp-c.observation_ns<=max_age_ns;
}
UnifiedResult BuildUnified(const Cloud& raw,const GroundLiveResult& ground,
                           const Calibration& calibration,std::int64_t stamp,
                           const OptionalContext* map_context,
                           const OptionalContext* camera_context){
  if(stamp<=0)throw std::invalid_argument("unified zero observation stamp");
  if(!calibration.base_from_sensor.matrix().allFinite())
    throw std::invalid_argument("invalid sensor mount");
  const auto found=ground.methods.find("jaeho");
  if(found==ground.methods.end())throw std::invalid_argument("unified requires jaeho ground");
  const auto& mask=found->second;
  const auto& prepared=ground.prepared;
  if(mask.labels.size()!=raw.size()||mask.structural_removed.size()!=raw.size()||
     prepared.raw_to_voxel.size()!=raw.size())
    throw std::invalid_argument("ground/raw lineage mismatch");
  const auto voxel_count=prepared.leveled->size();
  if(prepared.voxel->size()!=voxel_count)throw std::invalid_argument("leveled voxel mismatch");
  UnifiedResult out;out.observation_ns=stamp;
  out.map_context_status=ContextState(map_context,stamp);
  out.camera_context_status=ContextState(camera_context,stamp);
  std::vector<std::uint8_t> voxel_label(voxel_count,0),structural(voxel_count,0);
  std::vector<std::vector<std::size_t>> raw_by_voxel(voxel_count);
  for(std::size_t i=0;i<raw.size();++i){
    const int j=prepared.raw_to_voxel[i];
    if(j<0){if(mask.labels[i]!=0)throw std::invalid_argument("nonzero label outside ROI");continue;}
    if(std::size_t(j)>=voxel_count)throw std::invalid_argument("voxel lineage outside range");
    const auto label=mask.labels[i];
    if(label<1||label>3)throw std::invalid_argument("invalid ground label");
    if(voxel_label[j]&&voxel_label[j]!=label)throw std::invalid_argument("mixed voxel labels");
    voxel_label[j]=label;
    structural[j]|=mask.structural_removed[i];
    raw_by_voxel[j].push_back(i);
    if(label==1)++out.ground_raw_points;
    else if(label==2)out.retained_raw_indices.push_back(i);
    else ++out.unknown_raw_points;
    if(mask.structural_removed[i])out.structure_candidate_raw_indices.push_back(i);
  }
  // A structural hypothesis NEVER suppresses an object point in this stage.
  // This is intentionally the same 1.5 m / 10..10000 voxel baseline as the
  // old normal-road path, not a claim that it is optimal for all missions.
  auto started=Clock::now();Cloud::Ptr kept(new Cloud),surface(new Cloud);
  std::vector<std::size_t> kept_ids,surface_ids;
  for(std::size_t j=0;j<voxel_count;++j){
    if(voxel_label[j]==2){kept->push_back((*prepared.leveled)[j]);kept_ids.push_back(j);}
    if(structural[j]){surface->push_back((*prepared.leveled)[j]);surface_ids.push_back(j);}
  }
  const auto clusters=Clusters(kept,1.5,10,10000);
  const auto surface_clusters=Clusters(surface,.8,20,10000);
  out.clustering_ms=Ms(started);
  const Eigen::Matrix3d inv_level=Rotation(prepared.leveling).transpose();
  const auto point_base=[&](const Eigen::Vector3d& p){return calibration.base_from_sensor*(inv_level*p);};
  started=Clock::now();std::vector<tunnel_filter::Obb> obbs;
  PinnedLShapeObb()(kept,clusters,&obbs);
  if(obbs.size()!=clusters.size())throw std::runtime_error("OBB count mismatch");
  for(std::size_t n=0;n<clusters.size();++n){
    const auto& box=obbs[n];FrameDetection d;d.detection_index=n;
    d.voxel_points=clusters[n].indices.size();
    d.center_base=point_base({box.cx,box.cy,box.cz});
    for(std::size_t k=0;k<8;++k)d.corners_base[k]=point_base(Vec(box.corners[k]));
    d.forward_base=calibration.base_from_sensor.linear()*
                   (inv_level*Eigen::Vector3d(std::cos(box.yaw),std::sin(box.yaw),0));
    d.yaw_base_rad=std::atan2(d.forward_base.y(),d.forward_base.x());
    for(int point_index:clusters[n].indices){
      const auto j=kept_ids.at(std::size_t(point_index));
      d.source_indices.insert(d.source_indices.end(),raw_by_voxel[j].begin(),raw_by_voxel[j].end());
    }
    std::sort(d.source_indices.begin(),d.source_indices.end());
    d.raw_points=d.source_indices.size();
    // Context availability may later refine relevance; no context or camera
    // hole is allowed to turn an observed object into a deletion decision.
    out.detections.push_back(std::move(d));
  }
  for(const auto& cluster:surface_clusters){
    StructureEvidence e;e.support_voxels=cluster.indices.size();
    e.min_base=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    e.max_base=Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
    for(int point_index:cluster.indices){
      const auto j=surface_ids.at(std::size_t(point_index));
      e.support_raw_points+=raw_by_voxel[j].size();
      const auto& p=(*prepared.leveled)[j];
      const auto b=point_base({p.x,p.y,p.z});
      e.min_base=e.min_base.cwiseMin(b);e.max_base=e.max_base.cwiseMax(b);
    }
    out.structure_evidence.push_back(std::move(e));
  }
  out.obb_ms=Ms(started);return out;
}
}  // namespace asmc_lidar
