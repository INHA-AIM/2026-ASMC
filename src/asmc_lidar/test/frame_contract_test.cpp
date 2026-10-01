#include <asmc_lidar/core.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

namespace {
using asmc_lidar::Cloud;
using asmc_lidar::FrameInput;
using asmc_lidar::Mode;
using asmc_lidar::Processor;

Cloud::Ptr Scene() {
  Cloud::Ptr points(new Cloud);
  for (int x = 0; x < 24; ++x) for (int y = -12; y <= 12; ++y) {
    pcl::PointXYZI point;
    point.x = 2.0f + x * 0.25f;
    point.y = y * 0.20f;
    point.z = -1.80f + x * 0.003f;
    point.intensity = float((x + y + 100) % 17) / 17.0f;
    points->push_back(point);
  }
  for (int i = 0; i < 40; ++i) {
    pcl::PointXYZI point;
    point.x = 7.0f + float(i % 5) * 0.10f;
    point.y = 1.0f + float(i / 5) * 0.10f;
    point.z = -0.65f;
    point.intensity = 0.93f;
    points->push_back(point);
  }
  for (int i = 0; i < 18; ++i) {
    pcl::PointXYZI wall;
    wall.x = 9.0f; wall.y = 2.4f; wall.z = -1.2f + i * 0.13f;
    wall.intensity = 0.51f; points->push_back(wall);
  }
  pcl::PointXYZI near; near.x = -0.75f; near.y = 0.0f; near.z = -0.5f; near.intensity = 0.2f;
  points->push_back(near);
  pcl::PointXYZI invalid = near; invalid.x = std::numeric_limits<float>::quiet_NaN();
  points->push_back(invalid);
  points->width = points->size();
  points->height = 1;
  return points;
}
}  // namespace

int main() {
  asmc_lidar::Calibration calibration;
  calibration.source = "synthetic_nonzero_mount";
  calibration.base_from_sensor.linear() = asmc::lidar_eval::rpy(0.04, -0.07, 0.12);
  calibration.base_from_sensor.translation() = Eigen::Vector3d(0.8, -0.15, 1.45);
  const auto cloud = Scene();
  FrameInput input;
  input.cloud = cloud;
  input.stamp_ns = 1000000000;
  input.receipt_ns = 1000000010;
  input.imu_valid = true;
  input.roll_rad = 0.08;
  input.pitch_rad = -0.11;
  Processor reference(Mode::Reference, calibration, tunnel_filter::Params());
  Processor ground(Mode::GroundReview, calibration, tunnel_filter::Params());
  const auto a = reference.Process(input);
  const auto b = ground.Process(input);
  assert(a.status == "valid" && b.status == "valid");
  assert(a.stamp_ns == input.stamp_ns && b.stamp_ns == input.stamp_ns);
  assert(a.raw_points == cloud->size() && a.input_base->size() == cloud->size());
  assert(a.reference_labels == b.reference_labels);
  assert(a.voxel_fnv1a64 == b.voxel_fnv1a64);
  assert(b.patchwork.by_original.size() == cloud->size());
  assert(b.patchwork.ground->size() + b.patchwork.nonground->size() +
         b.patchwork.uncertain->size() == b.roi_points);
  assert(b.patchwork.reason == "indexed_complete");
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const auto& raw = cloud->at(i);
    if (!std::isfinite(raw.x)) {
      assert(a.raw_to_voxel[i] == -1);
      assert(std::isnan(a.input_base->at(i).x));
      continue;
    }
    const Eigen::Vector3d expected = calibration.base_from_sensor *
        Eigen::Vector3d(raw.x, raw.y, raw.z);
    const auto& transformed = a.input_base->at(i);
    assert((expected - Eigen::Vector3d(transformed.x, transformed.y,
                                     transformed.z)).norm() < 1e-6);
    assert(transformed.intensity == raw.intensity);
  }
  assert(reference.Process(input).status == "duplicate_stamp");
  input.stamp_ns -= 1;
  assert(reference.Process(input).status == "regressed_stamp");
  reference.Reset();
  input.stamp_ns = 1000000000;
  input.receipt_ns = input.stamp_ns - 1;
  assert(reference.Process(input).status == "receipt_before_stamp");
  input.receipt_ns = input.stamp_ns + 10;
  input.cloud.reset(new Cloud);
  assert(reference.Process(input).status == "observed_empty");
  std::cout << "asmc_lidar_frame_contract_test passed\n";
}
