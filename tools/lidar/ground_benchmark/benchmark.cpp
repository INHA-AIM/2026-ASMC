// Offline research harness. Never publishes or commands the simulator.
// PCL reference follows the plane-selection parameters in Son Jaeho's
// Main/Lidar/ImuRansac/Ransac.cpp; it does not reproduce upstream ROI/voxel/IMU
// or downstream experimental tunnel filtering. Original source stays untouched.
#include <patchwork/patchworkpp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

using Mask = std::vector<std::uint8_t>;
// 0=unassigned, 1=ground removed, 2=nonground retained, 3=vertical plane removed.
Mask pcl_reference(const Eigen::MatrixXf& points) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<int> original;
  for (int i = 0; i < points.rows(); ++i) {
    pcl::PointXYZI p;
    p.x = points(i, 0); p.y = points(i, 1); p.z = points(i, 2); p.intensity = points(i, 3);
    cloud->push_back(p); original.push_back(i);
  }
  Mask result(points.rows(), 2);
  for (int attempt = 0; attempt < 3 && cloud->size() >= 3; ++attempt) {
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(20);
    seg.setDistanceThreshold(0.3);
    seg.setInputCloud(cloud);
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coefficients;
    seg.segment(inliers, coefficients);
    if (inliers.indices.empty() || coefficients.values.size() < 4) break;
    const auto& c = coefficients.values;
    const double norm = std::sqrt(c[0]*c[0] + c[1]*c[1] + c[2]*c[2]);
    const bool ground = norm > 1e-6 && std::abs(c[2] / norm) >= 0.82;
    std::vector<bool> remove(cloud->size(), false);
    for (int index : inliers.indices) {
      result.at(original.at(index)) = ground ? 1 : 3;
      remove.at(index) = true;
    }
    if (ground) break;
    pcl::PointCloud<pcl::PointXYZI>::Ptr next(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> next_original;
    for (std::size_t i = 0; i < cloud->size(); ++i) {
      if (!remove[i]) { next->push_back((*cloud)[i]); next_original.push_back(original[i]); }
    }
    cloud = next; original.swap(next_original);
  }
  return result;
}

Eigen::MatrixXf read_cloud(const std::string& path) {
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream) throw std::runtime_error("cannot open cloud: " + path);
  const std::streamoff size = stream.tellg();
  if (size <= 0 || size % 16 != 0) throw std::runtime_error("invalid float4 cloud");
  stream.seekg(0);
  std::vector<float> data(static_cast<std::size_t>(size) / 4);
  stream.read(reinterpret_cast<char*>(data.data()), size);
  if (!stream) throw std::runtime_error("truncated cloud");
  Eigen::MatrixXf cloud(data.size() / 4, 4);
  for (int i = 0; i < cloud.rows(); ++i)
    for (int j = 0; j < 4; ++j) cloud(i, j) = data[i*4+j];
  if (!cloud.allFinite()) throw std::runtime_error("nonfinite input cloud");
  return cloud;
}

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: ground_benchmark STAGE_DIRECTORY NEW_OUTPUT_DIRECTORY REPEATS\n";
    return 2;
  }
  try {
    Eigen::setNbThreads(1);
    const std::string stage = argv[1], output = argv[2];
    const int repeats = std::stoi(argv[3]);
    if (repeats < 1 || repeats > 20) throw std::runtime_error("invalid repetitions");
    std::ifstream index(stage + "/frames.txt");
    if (!index) throw std::runtime_error("missing frames.txt");
    std::ifstream existing(output + "/timings.csv");
    if (existing) throw std::runtime_error("output exists; select a new directory");
    std::ofstream timings(output + "/timings.csv");
    if (!timings) throw std::runtime_error("output directory must already exist");
    timings << "frame,algorithm,repeat,points,milliseconds\n" << std::setprecision(10);
    std::string frame;
    while (std::getline(index, frame)) {
      if (frame.empty() || frame.find('/') != std::string::npos || frame.find("..") != std::string::npos)
        throw std::runtime_error("invalid frame id");
      const auto cloud = read_cloud(stage + "/" + frame + ".bin");
      for (const std::string algorithm : {"pcl_plane_reference", "patchworkpp_sensor145", "patchworkpp_sensor180"}) {
        Mask first;
        for (int repeat = -1; repeat < repeats; ++repeat) {
          patchwork::Params parameters;
          parameters.sensor_height = algorithm == "patchworkpp_sensor180" ? 1.80 : 1.45;
          parameters.min_range = 1.0;
          parameters.max_range = 60.0;
          parameters.enable_RNR = false;  // simulator intensity is not calibrated real reflectance
          // New instance per independent capture frame/repetition; no temporal learning across pauses.
          std::unique_ptr<patchwork::PatchWorkpp> pp;
          if (algorithm != "pcl_plane_reference") pp.reset(new patchwork::PatchWorkpp(parameters));
          const auto start = std::chrono::steady_clock::now();
          Mask result;
          if (pp) {
            pp->estimateGround(cloud);
            result.assign(cloud.rows(), 0);
            const auto ground = pp->getGroundIndices();
            const auto nonground = pp->getNongroundIndices();
            for (int i = 0; i < ground.size(); ++i) result.at(ground(i)) = 1;
            for (int i = 0; i < nonground.size(); ++i) result.at(nonground(i)) = 2;
          } else result = pcl_reference(cloud);
          const auto stop = std::chrono::steady_clock::now();
          if (repeat < 0) continue;  // per-frame warm-up
          const double ms = std::chrono::duration<double, std::milli>(stop - start).count();
          timings << frame << ',' << algorithm << ',' << repeat << ',' << cloud.rows() << ',' << ms << '\n';
          if (repeat == 0) {
            first = result;
            std::ofstream mask(output + "/" + frame + "." + algorithm + ".mask", std::ios::binary);
            mask.write(reinterpret_cast<const char*>(result.data()), result.size());
            if (!mask) throw std::runtime_error("mask write failed");
          } else if (result != first) throw std::runtime_error("repeated mask changed");
        }
      }
    }
    if (!timings) throw std::runtime_error("timing write failed");
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
  return 0;
}
