#include <asmc_lidar/general.hpp>
#include <asmc_lidar/core.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace asmc_lidar {
tunnel_filter::FrameResult ProcessGeneralGroundLeveled(
    const Cloud& leveled,std::vector<GroundPlaneAttempt>* review_attempts,
    bool validate_sensor_height) {
  using Clock=std::chrono::steady_clock;
  const auto started=Clock::now();
  tunnel_filter::FrameResult result;
  result.input_points=leveled.size();
  result.leveled=leveled.makeShared();
  result.labels.assign(leveled.size(),tunnel_filter::kKept);
  result.cluster_id.assign(leveled.size(),-1);
  std::vector<int> remaining;
  remaining.reserve(leveled.size());
  for(std::size_t i=0;i<leveled.size();++i)remaining.push_back(int(i));
  bool ground_found=false;
  for(int attempt=0;attempt<3&&remaining.size()>=3;++attempt) {
    GroundPlaneAttempt diagnostic;diagnostic.attempt=attempt+1;diagnostic.candidate_points=remaining.size();
    Cloud::Ptr candidate(new Cloud);candidate->reserve(remaining.size());
    for(int idx:remaining)candidate->push_back(result.leveled->at(std::size_t(idx)));
    pcl::SACSegmentation<pcl::PointXYZI> segment;
    segment.setOptimizeCoefficients(true);
    segment.setModelType(pcl::SACMODEL_PLANE);segment.setMethodType(pcl::SAC_RANSAC);
    segment.setMaxIterations(20);segment.setDistanceThreshold(.3);
    segment.setInputCloud(candidate);
    pcl::PointIndices inliers;pcl::ModelCoefficients coefficients;
    segment.segment(inliers,coefficients);
    diagnostic.inlier_points=inliers.indices.size();
    if(inliers.indices.empty()||coefficients.values.size()<4) {
      if(review_attempts)review_attempts->push_back(diagnostic);
      break;
    }
    const auto& c=coefficients.values;
    const double norm=std::sqrt(double(c[0])*c[0]+double(c[1])*c[1]+double(c[2])*c[2]);
    const bool horizontal=norm>1e-6&&std::abs(c[2]/norm)>=.82;
    const double height=std::abs(c[2])>1e-6?-double(c[3])/c[2]:0.;
    const bool height_ok=std::isfinite(height)&&height>=-2.25&&height<=-1.35;
    const bool ground=horizontal&&(!validate_sensor_height||height_ok);
    diagnostic.valid=norm>1e-6&&std::isfinite(norm);
    diagnostic.accepted_ground=ground;
    if(diagnostic.valid) {
      const double sign=c[2]<0?-1.:1.;
      for(int k=0;k<3;++k)diagnostic.normal[k]=sign*c[k]/norm;
      diagnostic.height_valid=std::abs(c[2])>1e-6&&std::isfinite(c[3]);
      if(diagnostic.height_valid)diagnostic.height_at_origin_m=-double(c[3])/c[2];
    }
    std::vector<std::uint8_t> remove(remaining.size(),0);
    for(int index:inliers.indices) {
      const auto& p=result.leveled->at(std::size_t(remaining.at(std::size_t(index))));
      diagnostic.centroid[0]+=p.x;diagnostic.centroid[1]+=p.y;diagnostic.centroid[2]+=p.z;
      // Reject an implausible horizontal plane from the SEARCH, not from the
      // obstacle cloud. The pinned default path retains exactly its old labels.
      result.labels[std::size_t(remaining.at(std::size_t(index)))]=
          ground?tunnel_filter::kGround:
          (validate_sensor_height&&horizontal?tunnel_filter::kKept:tunnel_filter::kWall);
      remove[std::size_t(index)]=1;
    }
    for(double& v:diagnostic.centroid)v/=double(inliers.indices.size());
    if(review_attempts)review_attempts->push_back(diagnostic);
    std::vector<int> next;next.reserve(remaining.size());
    for(std::size_t i=0;i<remaining.size();++i)if(!remove[i])next.push_back(remaining[i]);
    remaining.swap(next);
    if(ground){ground_found=true;break;}
  }
  result.ground_removed=std::count(result.labels.begin(),result.labels.end(),tunnel_filter::kGround);
  result.wall_removed=std::count(result.labels.begin(),result.labels.end(),tunnel_filter::kWall);
  result.ground_fallback_used=!ground_found;
  if(validate_sensor_height) {
    for(std::size_t i=0;i<result.labels.size();++i)
      if(result.labels[i]==tunnel_filter::kKept)result.kept_indices.push_back(int(i));
  } else result.kept_indices=remaining;
  result.kept_points=result.kept_indices.size();
  result.timings.total_ms=std::chrono::duration<double,std::milli>(Clock::now()-started).count();
  return result;
}

tunnel_filter::FrameResult ProcessGeneralReference(const Cloud& voxel,float pitch,
    const tunnel_filter::ObbFunction& obb_function) {
  using Clock=std::chrono::steady_clock;
  const auto started=Clock::now();
  const auto rotation=tunnel_filter::LevelingRotation(0,pitch,false,true);
  Cloud leveled;
  tunnel_filter::ApplyRotation(voxel,rotation,&leveled);
  auto result=ProcessGeneralGroundLeveled(leveled);
  result.leveling=rotation;result.roll_rad=0;result.pitch_rad=pitch;
  const auto& remaining=result.kept_indices;
  Cloud::Ptr kept(new Cloud);kept->reserve(remaining.size());
  for(int idx:remaining)kept->push_back(result.leveled->at(std::size_t(idx)));
  if(kept->size()>=10) {
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(kept);
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> extraction;
    extraction.setClusterTolerance(1.5);extraction.setMinClusterSize(10);
    extraction.setMaxClusterSize(10000);extraction.setSearchMethod(tree);
    extraction.setInputCloud(kept);extraction.extract(result.clusters);
    std::vector<tunnel_filter::Obb> obbs;
    obb_function(kept,result.clusters,&obbs);
    for(std::size_t i=0;i<result.clusters.size();++i) {
      for(int index:result.clusters[i].indices)
        result.cluster_id[std::size_t(remaining.at(std::size_t(index)))]=int(i);
      tunnel_filter::TunnelObject object;object.obb=obbs.at(i);
      object.point_count=result.clusters[i].indices.size();
      result.objects.push_back(object);
    }
  }
  result.timings.total_ms=std::chrono::duration<double,std::milli>(Clock::now()-started).count();
  return result;
}
}  // namespace asmc_lidar
