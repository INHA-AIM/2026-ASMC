// L-shape OBB fitting and matched-track yaw stabilization.
// Ported from Main src/Lidar/Tracking/Tracking.cpp namespace obb_util at 8f93c22e.
// Operation order, double/float types and tie-breaking are kept exactly.
#include <asmc_lidar/detection/obb.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace asmc {
namespace lidar {
namespace pipeline {
namespace obb_util {

// Ported from Main Tracking.cpp:46-48 (8f93c22e)
double wrapToPi(double angle) {
  return std::remainder(angle, 2.0 * kPi);
}

// Ported from Main Tracking.cpp:51-55 (8f93c22e)
double normalizeAxisYaw(double yaw) {
  while (yaw >= kHalfPi) yaw -= kPi;
  while (yaw < -kHalfPi) yaw += kPi;
  return yaw;
}

// Ported from Main Tracking.cpp:57-66 (8f93c22e)
std::vector<P2> finiteXY(const Cloud& cloud) {
  std::vector<P2> points;
  points.reserve(cloud.size());
  for (const auto& p : cloud.points) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      points.push_back({p.x, p.y});
    }
  }
  return points;
}

// Ported from Main Tracking.cpp:70-138 (8f93c22e)
bool computeOBBAtYaw(const Cloud& cloud, double yaw, OBB2D& obb2d, double& zMin, double& zMax,
                     std::array<P3, 8>& corners3d) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  double uMin = std::numeric_limits<double>::infinity();
  double uMax = -std::numeric_limits<double>::infinity();
  double vMin = std::numeric_limits<double>::infinity();
  double vMax = -std::numeric_limits<double>::infinity();
  zMin = std::numeric_limits<double>::infinity();
  zMax = -std::numeric_limits<double>::infinity();
  std::size_t validCount = 0;

  for (const auto& p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const double u = p.x * c + p.y * s;
    const double v = -p.x * s + p.y * c;
    uMin = std::min(uMin, u);
    uMax = std::max(uMax, u);
    vMin = std::min(vMin, v);
    vMax = std::max(vMax, v);
    zMin = std::min(zMin, static_cast<double>(p.z));
    zMax = std::max(zMax, static_cast<double>(p.z));
    ++validCount;
  }

  if (validCount == 0) {
    obb2d = {};
    zMin = 0.0;
    zMax = 0.0;
    corners3d.fill({0.0, 0.0, 0.0});
    return false;
  }

  const double centerU = 0.5 * (uMin + uMax);
  const double centerV = 0.5 * (vMin + vMax);
  obb2d.cx = centerU * c - centerV * s;
  obb2d.cy = centerU * s + centerV * c;
  // A one-scan-line object keeps its measured centre with a non-degenerate
  // footprint at the PlannerObjectAdapter minimum.
  obb2d.w = std::max(uMax - uMin, kMinimumObbSide);
  obb2d.h = std::max(vMax - vMin, kMinimumObbSide);
  obb2d.yaw = normalizeAxisYaw(yaw);

  const double halfW = 0.5 * obb2d.w;
  const double halfH = 0.5 * obb2d.h;
  const P2 axisU{c, s};
  const P2 axisV{-s, c};
  const std::array<std::pair<double, double>, 4> signs{{{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}}};

  for (int i = 0; i < 4; ++i) {
    const double su = signs[i].first;
    const double sv = signs[i].second;
    obb2d.corners[i] = {obb2d.cx + su * halfW * axisU.x + sv * halfH * axisV.x,
                        obb2d.cy + su * halfW * axisU.y + sv * halfH * axisV.y};
    corners3d[i] = {obb2d.corners[i].x, obb2d.corners[i].y, zMin};
    corners3d[i + 4] = {obb2d.corners[i].x, obb2d.corners[i].y, zMax};
  }
  return true;
}

// Ported from Main Tracking.cpp:142-201 (8f93c22e)
void computeOBB3D_LShape(const Cloud& cloud, OBB2D& obb2d, double& zMin, double& zMax,
                         std::array<P3, 8>& corners3d) {
  const std::vector<P2> points = finiteXY(cloud);
  if (points.size() < kMinLShapePoints) {
    // Return values ignored, as in Main (all-zero box when nothing is finite).
    computeOBBAtYaw(cloud, 0.0, obb2d, zMin, zMax, corners3d);
    if (obb2d.w < obb2d.h) {
      computeOBBAtYaw(cloud, kHalfPi, obb2d, zMin, zMax, corners3d);
    }
    return;
  }

  double bestScore = -std::numeric_limits<double>::infinity();
  double bestYaw = 0.0;

  for (double theta = 0.0; theta < kHalfPi; theta += kLShapeAngleStep) {
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    double uMin = std::numeric_limits<double>::infinity();
    double uMax = -std::numeric_limits<double>::infinity();
    double vMin = std::numeric_limits<double>::infinity();
    double vMax = -std::numeric_limits<double>::infinity();

    for (const auto& p : points) {
      const double u = p.x * c + p.y * s;
      const double v = -p.x * s + p.y * c;
      uMin = std::min(uMin, u);
      uMax = std::max(uMax, u);
      vMin = std::min(vMin, v);
      vMax = std::max(vMax, v);
    }

    double score = 0.0;
    for (const auto& p : points) {
      const double u = p.x * c + p.y * s;
      const double v = -p.x * s + p.y * c;
      const double distanceToU = std::min(u - uMin, uMax - u);
      const double distanceToV = std::min(v - vMin, vMax - v);
      const double distanceToEdge = std::max(std::min(distanceToU, distanceToV), kClosenessMinDistance);
      score += 1.0 / distanceToEdge;
    }

    if (score > bestScore) {
      bestScore = score;
      bestYaw = theta;
    }
  }

  computeOBBAtYaw(cloud, bestYaw, obb2d, zMin, zMax, corners3d);

  // Long side always on the yaw axis (removes 90 deg axis flips).
  if (obb2d.w < obb2d.h) {
    bestYaw += kHalfPi;
    computeOBBAtYaw(cloud, bestYaw, obb2d, zMin, zMax, corners3d);
  }
}

// Ported from Main Tracking.cpp:203-223 (8f93c22e)
void writeOBBToObject(TrackedObject& object, const OBB2D& obb2d, double zMin, double zMax,
                      const std::array<P3, 8>& corners3d) {
  object.f32_OBB_CenterX = static_cast<float>(obb2d.cx);
  object.f32_OBB_CenterY = static_cast<float>(obb2d.cy);
  object.f32_OBB_CenterZ = static_cast<float>(0.5 * (zMin + zMax));
  object.f32_OBB_Width = static_cast<float>(obb2d.w);
  object.f32_OBB_Height = static_cast<float>(obb2d.h);
  object.f32_OBB_ZMin = static_cast<float>(zMin);
  object.f32_OBB_ZMax = static_cast<float>(zMax);
  object.f32_OBB_YawRad = static_cast<float>(obb2d.yaw);

  for (int i = 0; i < 8; ++i) {
    object.OBB_Corners_Body[i].x = static_cast<float>(corners3d[i].x);
    object.OBB_Corners_Body[i].y = static_cast<float>(corners3d[i].y);
    object.OBB_Corners_Body[i].z = static_cast<float>(corners3d[i].z);
  }
}

// Ported from Main Tracking.cpp:227-261 (8f93c22e)
void stabilizeMatchedOBB(const TrackedObject& previous, TrackedObject& current, double egoYawNowDeg) {
  // During a miss only the filter state follows the ego frame, so the first
  // frame after a re-detection keeps the raw L-shape.
  if (current.pcl_ClusterPoints.empty() || previous.s32_MissCount > 0) return;

  const double egoYawNow = egoYawNowDeg * kPi / 180.0;
  const double egoYawDelta = wrapToPi(egoYawNow - previous.f32_PrevYaw);
  const double predictedYaw = normalizeAxisYaw(static_cast<double>(previous.f32_OBB_YawRad) - egoYawDelta);
  const double rawYaw = current.f32_OBB_YawRad;
  // Preserved: float W/H division, promoted to double by the ternary.
  const double aspectRatio = current.f32_OBB_Height > 1e-6f ? current.f32_OBB_Width / current.f32_OBB_Height
                                                            : std::numeric_limits<double>::infinity();

  double filteredYaw = predictedYaw;
  if (aspectRatio >= kMinReliableAspectRatio) {
    const double yawError = std::remainder(rawYaw - predictedYaw, kPi);
    filteredYaw = predictedYaw + kYawFilterAlpha * yawError;
  }
  filteredYaw = normalizeAxisYaw(filteredYaw);

  OBB2D filteredObb;
  double zMin = 0.0;
  double zMax = 0.0;
  std::array<P3, 8> corners3d{};
  if (computeOBBAtYaw(current.pcl_ClusterPoints, filteredYaw, filteredObb, zMin, zMax, corners3d)) {
    writeOBBToObject(current, filteredObb, zMin, zMax, corners3d);
  }
}

}  // namespace obb_util
}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
