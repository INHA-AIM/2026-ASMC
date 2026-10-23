#pragma once
// L-shape OBB fitting and matched-track OBB yaw stabilization.
// Ported from Main Tracking.cpp namespace obb_util (8f93c22e). All fitting math
// is double; values are cast to float only when written to a TrackedObject.
//
// Corner convention (computeOBBAtYaw): (u, v) signs 0=(-,-) 1=(+,-) 2=(+,+)
// 3=(-,+), i.e. counter-clockwise seen from +z; corners 0-3 at zMin, 4-7 the
// same XY at zMax. u/v use the yaw passed in (not the normalized stored yaw),
// so the only invariant is "edge 0->1 is parallel to +-yaw".

#include <asmc_lidar/core/types.hpp>

#include <array>
#include <cstddef>
#include <vector>

namespace asmc {
namespace lidar {
namespace pipeline {
namespace obb_util {

// Main obb_util::P2/P3 are double (P3 shadows the float pipeline::P3 here, as
// it shadows ::P3 in Main).
struct P2 { double x, y; };
struct P3 { double x, y, z; };

struct OBB2D {
  double cx{0}, cy{0};       // centre (XY)
  double w{0}, h{0};         // w: extent along yaw, h: perpendicular extent
  double yaw{0};             // rad, CCW from +x, axis (pi-periodic)
  std::array<P2, 4> corners;  // XY only, computeOBBAtYaw order
};

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = 0.5 * kPi;
constexpr double kLShapeAngleStep = kPi / 180.0;  // 1 deg, accumulated (90 candidates)
constexpr double kClosenessMinDistance = 0.05;    // half the 0.1 m voxel leaf
constexpr double kMinimumObbSide = 0.05;          // planner OBB validity floor
constexpr std::size_t kMinLShapePoints = 10;
constexpr double kYawFilterAlpha = 0.25;
constexpr double kMinReliableAspectRatio = 1.20;

// std::remainder(angle, 2pi): [-pi, pi].
double wrapToPi(double angle);
// Axis yaw into [-pi/2, pi/2) (exactly +pi/2 becomes -pi/2).
double normalizeAxisYaw(double yaw);
// XY of points whose x, y and z are all finite.
std::vector<P2> finiteXY(const Cloud& cloud);

// Box containing every finite point at `yaw`. Sides are floored at
// kMinimumObbSide (centre kept). Stored yaw is normalizeAxisYaw(yaw); corners use
// the un-normalized yaw. Returns false (all zero) when no point is finite.
bool computeOBBAtYaw(const Cloud& cloud, double yaw, OBB2D& obb2d, double& zMin, double& zMax,
                     std::array<P3, 8>& corners3d);

// Closeness-criterion L-shape search over [0, pi/2) in accumulated 1 deg steps
// (strict '>' keeps the smallest best theta). Fewer than 10 finite points: AABB at
// yaw 0, or pi/2 when w < h. The long side is forced onto the yaw axis (w >= h).
void computeOBB3D_LShape(const Cloud& cloud, OBB2D& obb2d, double& zMin, double& zMax,
                         std::array<P3, 8>& corners3d);

// Double -> float copy into the object (CenterZ = 0.5 (zMin + zMax)).
void writeOBBToObject(TrackedObject& object, const OBB2D& obb2d, double zMin, double zMax,
                      const std::array<P3, 8>& corners3d);

// Matched-track yaw EMA. Skipped when the current cluster is empty or the
// previous track was missed (MissCount > 0). predicted = normalizeAxisYaw(prev yaw
// - wrapToPi(egoYawNowDeg*pi/180 - prev.f32_PrevYaw)); aspect W/H >= 1.2 blends
// 0.25 of the pi-periodic error, otherwise the prediction is kept. The whole box
// is then refit to the current points at the filtered yaw.
void stabilizeMatchedOBB(const TrackedObject& previous, TrackedObject& current, double egoYawNowDeg);

}  // namespace obb_util
}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
