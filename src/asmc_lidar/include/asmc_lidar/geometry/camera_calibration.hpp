#pragma once
#include <asmc_lidar/fusion/camera_observations.hpp>
#include <asmc_lidar/geometry/projection.hpp>

namespace asmc::lidar::camera {
// Reads the camera-owned YAML. Hashes the full per-view mapping plus source SHA
// with the same sorted ASCII JSON representation as ObservationIdentity.digest.
Calibrations load_calibrations(const std::string& yaml_path);
// Stable per-view identity; front-only retains its historical ID.
inline std::string fusion_calibration_identity(const Calibrations& cameras, bool all) {
  if (!all) {
    const auto front = cameras.find("camera_front");
    return front == cameras.end() ? "" : front->second.id;
  }
  std::string ids = "multi-view-calibration-v1\n";
  for (const auto& view : cameras) ids += view.first + "=" + view.second.id + "\n";
  return ids;
}
struct PixelResult {
  bool valid=false, time_compensated=false;
  std::string reason;
  double u=0, v=0, depth=0;
  Ns image_minus_scan_ns=0;
};
// An optional, validated image-time base <- scan-time base transform is supplied
// by the caller; no pose lookup, extrapolation or map dependency is hidden here.
PixelResult project(const Calibration&, const Context& scan, Ns image_stamp,
                    const Vec& scan_point, const std::optional<Pose>& image_base_from_scan_base={});
} // namespace asmc::lidar::camera
