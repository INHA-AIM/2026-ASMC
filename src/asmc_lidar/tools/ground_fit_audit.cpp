// Offline only. CSV is the existing replay observer's LEVELED point cloud.
// Keep removal thresholds, input ordering and points unchanged; vary only the
// plane-fit consensus band. Cluster IDs are from the original detector.
#include <asmc_lidar/preprocessing/ground.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
using namespace asmc::lidar::pipeline;
int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "ground_fit_audit stage.csv fit_band_m (0=baseline)\n";
    return 2;
  }
  CloudPtr cloud(new Cloud), out(new Cloud);
  std::ifstream file(argv[1]);
  if (!file)
    return 2;
  std::vector<int> labels;
  std::string line;
  std::getline(file, line);
  while (std::getline(file, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    pcl::PointXYZI p;
    int kept, id;
    row >> p.x >> p.y >> p.z >> kept >> id;
    if (!row)
      return 2;
    p.intensity = 0;
    cloud->push_back(p);
    labels.push_back(id);
  }
  GroundConfig c;
  c.fit_threshold_m = std::stod(argv[2]);
  if (!std::isfinite(c.fit_threshold_m) || c.fit_threshold_m < 0 ||
      c.fit_threshold_m > c.far_threshold_m)
    return 2;
  GroundFilter filter(c);
  std::vector<int> keep;
  auto start = std::chrono::steady_clock::now();
  filter.Filter(cloud, out, &keep);
  double ms = std::chrono::duration<double, std::milli>(
                  std::chrono::steady_clock::now() - start)
                  .count();
  const auto &plane = filter.CurrentPlane();
  std::map<int, std::pair<int, int>> count;
  for (int id : labels)
    count[id].first++;
  for (int i : keep)
    count[labels[i]].second++;
  std::cout << std::setprecision(17) << "{\"fit_band_m\":" << c.fit_threshold_m
            << ",\"valid\":" << (plane.valid ? "true" : "false")
            << ",\"coefficients\":[";
  for (int i = 0; i < 4; i++)
    std::cout << (i ? "," : "") << plane.coefficients[i];
  std::cout << "],\"elapsed_ms\":" << ms << ",\"clusters\":[";
  bool first = true;
  for (auto [id, n] : count) {
    if (!first)
      std::cout << ',';
    first = false;
    std::cout << "{\"id\":" << id << ",\"input\":" << n.first
              << ",\"kept\":" << n.second << '}';
  }
  std::cout << "]}\n";
}
