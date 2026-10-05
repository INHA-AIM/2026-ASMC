#pragma once
#include <asmc_lidar/core.hpp>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace asmc_lidar {
inline Cloud ReadReviewCloud(const std::filesystem::path& path) {
  std::ifstream in(path,std::ios::binary|std::ios::ate);
  if(!in)throw std::runtime_error("missing cloud "+path.string());
  const auto bytes=in.tellg();if(bytes<0||bytes%16)throw std::runtime_error("invalid XYZI");
  in.seekg(0);Cloud cloud;float f[4];
  while(in.read(reinterpret_cast<char*>(f),sizeof(f))){
    pcl::PointXYZI p;p.x=f[0];p.y=f[1];p.z=f[2];p.intensity=f[3];cloud.push_back(p);
  }
  if(!in.eof())throw std::runtime_error("failed reading XYZI");
  return cloud;
}
}
