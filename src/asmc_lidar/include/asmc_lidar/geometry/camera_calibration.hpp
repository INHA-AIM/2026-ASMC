#pragma once
#include <asmc_lidar/fusion/camera_observations.hpp>
#include <asmc_lidar/geometry/projection.hpp>

namespace asmc::lidar::camera {
// Reads the camera-owned YAML. Hashes the full per-view mapping plus source SHA
// with the same sorted ASCII JSON representation as ObservationIdentity.digest.
Calibrations load_calibrations(const std::string& yaml_path);
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
