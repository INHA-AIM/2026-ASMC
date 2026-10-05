#ifndef FRENET_POLYGON_UTILS_HPP
#define FRENET_POLYGON_UTILS_HPP

#include "frenet_planner/Geometry/GeometryTypes.hpp"

namespace geometry {

bool AreCornersFinite(const Corners8& corners);

Polygon2D ProjectCornersToXY(const Corners8& corners);

Polygon2D RemoveDuplicatePoints(const Polygon2D& points,
                                double epsilon = 1e-6);

// Returns a counter-clockwise convex hull without repeating the first point.
Polygon2D ComputeConvexHull(const Polygon2D& points,
                            double epsilon = 1e-9);

double PolygonSignedArea(const Polygon2D& polygon);

double PolygonArea(const Polygon2D& polygon);

void EnsureCounterClockwise(Polygon2D& polygon);

ZRange ComputeZRange(const Corners8& corners);

// Invalid ranges conservatively overlap by default so missing height data does
// not make an obstacle disappear from collision checking.
bool CheckZOverlap(const ZRange& a, const ZRange& b,
                   double margin = 0.0,
                   bool conservative_on_invalid = true);

// Boundary contact counts as overlap. Invalid or degenerate polygons are
// conservatively treated as overlapping.
bool CheckConvexPolygonOverlapSAT(const Polygon2D& a, const Polygon2D& b,
                                  double epsilon = 1e-9);

// 두 유효 볼록 다각형 경계 사이의 최단 XY 거리. 겹치거나 접하면 0이며,
// 유효하지 않은 입력도 안전 우선으로 0을 반환한다.
double ConvexPolygonDistance(const Polygon2D& a, const Polygon2D& b,
                             double epsilon = 1e-9);

// Applies constant-velocity XY translation. Z and relative corner geometry
// remain unchanged. Negative time is allowed as mathematical back-projection.
Corners8 TranslateCorners(const Corners8& corners, double vx, double vy,
                          double prediction_time);

CornerValidationResult ValidateCorners8(const Corners8& corners,
                                         double duplicate_epsilon = 1e-6,
                                         double min_xy_area = 1e-6);

}  // namespace geometry

#endif  // FRENET_POLYGON_UTILS_HPP
