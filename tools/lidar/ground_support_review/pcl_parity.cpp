// One-frame PCL parity probe for the isolated Python cluster/OBB replay.
// Input mask is a uint8 array indexed by original XYZI point; 2=retained.
#include <asmc_lidar/core.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

int main(int argc,char** argv)try {
  if(argc!=4)throw std::runtime_error("usage: pcl_parity cloud.bin mask.bin pitch_rad");
  std::ifstream input(argv[1],std::ios::binary|std::ios::ate);
  std::ifstream maskfile(argv[2],std::ios::binary|std::ios::ate);
  if(!input||!maskfile)throw std::runtime_error("missing input");
  const auto bytes=input.tellg(),maskbytes=maskfile.tellg();
  if(bytes<=0||bytes%16||maskbytes!=bytes/16)throw std::runtime_error("size mismatch");
  input.seekg(0);maskfile.seekg(0);
  std::vector<std::uint8_t> mask(std::size_t(maskbytes),0);
  maskfile.read(reinterpret_cast<char*>(mask.data()),mask.size());
  auto roi=asmc_lidar::Cloud::Ptr(new asmc_lidar::Cloud);
  std::vector<std::uint8_t> labels;
  float xyz[4];
  for(std::size_t i=0;i<mask.size();++i) {
    input.read(reinterpret_cast<char*>(xyz),sizeof(xyz));
    if(!mask[i])continue;
    pcl::PointXYZI p;p.x=xyz[0];p.y=xyz[1];p.z=xyz[2];p.intensity=xyz[3];
    roi->push_back(p);labels.push_back(mask[i]);
  }
  pcl::VoxelGrid<pcl::PointXYZI> vg;vg.setInputCloud(roi);
  vg.setLeafSize(.1f,.1f,.1f);vg.setSaveLeafLayout(true);
  asmc_lidar::Cloud voxel;vg.filter(voxel);
  std::vector<int> seen(voxel.size(),-1);
  for(std::size_t i=0;i<roi->size();++i) {
    const int k=vg.getCentroidIndex(roi->at(i));
    if(k<0||std::size_t(k)>=seen.size())throw std::runtime_error("bad lineage");
    if(seen[k]>=0&&seen[k]!=labels[i])throw std::runtime_error("mixed labels in voxel");
    seen[k]=labels[i];
  }
  asmc_lidar::Cloud kept;
  for(std::size_t i=0;i<voxel.size();++i)if(seen[i]==2)kept.push_back(voxel[i]);
  const auto rotation=tunnel_filter::LevelingRotation(0,std::stof(argv[3]),false,true);
  asmc_lidar::Cloud leveled;tunnel_filter::ApplyRotation(kept,rotation,&leveled);
  auto ptr=leveled.makeShared();
  auto tree=pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(ptr);
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> extract;
  extract.setClusterTolerance(1.5);extract.setMinClusterSize(10);extract.setMaxClusterSize(10000);
  extract.setSearchMethod(tree);extract.setInputCloud(ptr);
  std::vector<pcl::PointIndices> clusters;extract.extract(clusters);
  std::vector<tunnel_filter::Obb> obbs;asmc_lidar::PinnedLShapeObb()(ptr,clusters,&obbs);
  std::vector<int> sizes;for(const auto& c:clusters)sizes.push_back(c.indices.size());
  std::sort(sizes.begin(),sizes.end(),std::greater<int>());
  std::cout<<"{\"roi\":"<<roi->size()<<",\"voxel\":"<<voxel.size()
           <<",\"kept\":"<<kept.size()<<",\"clusters\":"<<clusters.size()<<",\"sizes\":[";
  for(std::size_t i=0;i<sizes.size();++i){if(i)std::cout<<',';std::cout<<sizes[i];}
  std::cout<<"]}\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
