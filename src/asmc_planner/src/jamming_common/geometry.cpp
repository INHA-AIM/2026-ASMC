#include "jamming_common/geometry.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include <algorithm>
#include <cmath>
namespace local_planning {
std::array<double, 3> Rotate(const std::array<double, 9> &r, double x, double y,
                             double z) {
  return {r[0] * x + r[1] * y + r[2] * z, r[3] * x + r[4] * y + r[5] * z,
          r[6] * x + r[7] * y + r[8] * z};
}
std::array<double, 9> AttitudeRotation(double roll, double pitch) {
  const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch),
               sp = std::sin(pitch);
  // Heading-free R_y(pitch) R_x(roll). Unlike a minimal normal rotation this
  // preserves the same heading convention as yaw-rate integration.
  return {cp, sp * sr, sp * cr, 0, cr, -sr, -sp, cp * sr, cp * cr};
}
std::array<double, 9> GroundRotation(double a, double b) {
  return AttitudeRotation(std::atan2(-b, 1.),
                          std::atan2(a, std::sqrt(1 + b * b)));
}
namespace {
// One affine transform per footprint check; no trigonometry per raster cell.
// The raster stays in its original camera plane, including unknown cells.
struct GridTransform {
  const Grid &g;
  double c, s;
  explicit GridTransform(const Grid &grid)
      : g(grid), c(std::cos(g.source_in_plan[2])),
        s(std::sin(g.source_in_plan[2])) {}
  std::array<double, 3> toPlan(double bx, double by) const {
    const auto &R = g.rotation;
    const double bz = (g.ground_z - R[6] * bx - R[7] * by) / R[8];
    const auto q = Rotate(R, bx, by, bz);
    return {g.source_in_plan[0] + c * q[0] - s * q[1],
            g.source_in_plan[1] + s * q[0] + c * q[1], q[2]};
  }
  std::array<double, 3> toGrid(double x, double y) const {
    const double dx = x - g.source_in_plan[0], dy = y - g.source_in_plan[1];
    const double sx = c * dx + s * dy, sy = -s * dx + c * dy;
    const auto &R = g.rotation;
    return {R[0] * sx + R[3] * sy + R[6] * g.ground_z,
            R[1] * sx + R[4] * sy + R[7] * g.ground_z,
            R[2] * sx + R[5] * sy + R[8] * g.ground_z};
  }
};
} // namespace
std::array<double, 3> GridToPlan(const Grid &g, double x, double y) {
  return GridTransform(g).toPlan(x, y);
}
std::array<double, 3> PlanToGrid(const Grid &g, double x, double y) {
  return GridTransform(g).toGrid(x, y);
}
namespace {
bool fail(Result &r, const char *reason, double x, double y, int id = -1) {
  ++r.rejected[reason];
  r.blocked_x = x;
  r.blocked_y = y;
  r.blocking_id = id;
  return false;
}
} // namespace
bool CheckFootprint(const Scene &s, const Config &c, const Pose &p, Result &r) {
  CartesianState state{p.x, p.y, p.yaw, p.k, 0, 0};
  VehicleShape shape{c.width, c.length, c.rear};
  const auto physical = MakeEgoFootprint(state, shape, 0.);
  const double clearance_margin =
      c.margin + std::min(.25, .015 * s.speed * s.speed);
  for (const auto &o : s.obstacles) {
    double distance = geometry::ConvexPolygonDistance(physical, o.polygon);
    r.minimum_clearance = std::min(r.minimum_clearance, distance);
    // Recovery also retains the configured physical clearance and source-age
    // uncertainty. A crawl label cannot mean zero centimetres of clearance.
    if (distance <= (c.recovery_geometry?c.margin:clearance_margin) + o.uncertainty+o.extra_clearance)
      return fail(r, "local_obb", p.x, p.y, o.id);
    if(!o.motion_sweep.empty() && geometry::ConvexPolygonDistance(physical,o.motion_sweep)<=clearance_margin+o.uncertainty+o.extra_clearance)
      return fail(r,"predicted_object_sweep",p.x,p.y,o.id);
  }
  const auto &g = s.grid;
  const GridTransform transform(g);
  if (g.rows <= 0 || g.cols <= 0 ||
      g.probability.size() != size_t(g.rows * g.cols) ||
      g.coverage.size() != g.probability.size())
    return fail(r, "grid_invalid", p.x, p.y);
  const double cs = std::cos(p.yaw), sn = std::sin(p.yaw);
  // Cell half-diagonal covers discretisation; the physical footprint is not
  // shrunk. A measured plane is undone only for querying its original raster.
  const double pad = clearance_margin + g.resolution * std::sqrt(.5);
  const double front = c.length - c.rear + pad, rear = c.rear + pad,
               half = c.width * .5 + pad;
  int row0 = g.rows, row1 = 0, col0 = g.cols, col1 = 0;
  for (double a : {-rear, front})
    for (double b : {-half, half}) {
      double x = p.x + cs * a - sn * b, y = p.y + sn * a + cs * b;
      const auto source = transform.toGrid(x, y);
      double bx = source[0], by = source[1];
      int row = std::lround((g.xmax - bx) / g.resolution),
          col = std::lround((g.ymax - by) / g.resolution);
      if (row < 0 || row >= g.rows || col < 0 || col >= g.cols)
        return fail(r, "grid_extent", x, y);
      row0 = std::min(row0, row);
      row1 = std::max(row1, row);
      col0 = std::min(col0, col);
      col1 = std::max(col1, col);
    }
  for (int row = row0; row <= row1; ++row) {
    std::vector<int> fallback;
    if (g.blocked_columns.empty()) {
      for (int col = col0; col <= col1; ++col)
        if (g.coverage[row * g.cols + col] != 255 ||
            g.probability[row * g.cols + col] < 153)
          fallback.push_back(col);
    }
    const auto &columns =
        g.blocked_columns.empty() ? fallback : g.blocked_columns[row];
    for (auto it = std::lower_bound(columns.begin(), columns.end(), col0);
         it != columns.end() && *it <= col1; ++it) {
      int col = *it;
      double bx = g.xmax - row * g.resolution, by = g.ymax - col * g.resolution;
      // The published grid's points lie on n.p=ground_z. Restore their Z before
      // changing coordinates, rather than applying camera projection twice.
      auto q = transform.toPlan(bx, by);
      double x = q[0], y = q[1];
      double dx = x - p.x, dy = y - p.y, a = cs * dx + sn * dy,
             b = -sn * dx + cs * dy;
      if (a < -rear || a > front || std::abs(b) > half)
        continue;
      const int index = row * g.cols + col;
      auto cover = g.coverage[index];
      // Current occupied ego footprint is a geometric fact, not future
      // free-space. Half a cell covers quantization at the occupied body edge.
      bool current_body = x >= -c.rear - .5 * g.resolution &&
                          x <= c.length - c.rear + .5 * g.resolution &&
                          std::abs(y) <= c.width * .5 + .5 * g.resolution;
      if (cover != 255) {
        if(c.recovery_geometry)continue; // Only the original bounded near-field recovery.
        if (current_body)
          continue;
        // Unknown clearance padding is not a measured collision. The physical
        // swept body still requires evidence; observed nonroad/OBB retain
        // margin.
        if (a < -c.rear || a > c.length - c.rear || std::abs(b) > c.width * .5)
          continue;
        if (g.recent_road_age.size() == g.coverage.size() &&
            g.recent_road_age[index] > 0 && g.recent_road_age[index] <= 5.) {
          ++r.remembered_cells;
          continue;
        }
        bool assist = c.blind_assist && bx >= -c.rear - .35 && bx <= 6.5 &&
                      ((cover == 128 && std::abs(by) <= 2.5) ||
                       (cover == 0 && std::abs(by) <= 1.6));
        if (assist) {
          r.assisted = true;
          continue;
        }
        return fail(r, cover == 128 ? "ego_occlusion" : "unobserved", x, y);
      }
      if (g.probability[index] < 153) {
        if(c.recovery_geometry) {
          if(g.exterior_nonroad.size()!=g.coverage.size())
            return fail(r,"recovery_boundary_unavailable",x,y);
          if(!g.exterior_nonroad[index]) {++r.soft_margin_cells;continue;}
        }
        // Keep physical body +10cm +cell quantization hard. Additional
        // speed-dependent semantic clearance is a preference, not collision.
        // Otherwise a single bonnet-edge pixel30cm outside the body can
        // truncate a clear15m path to40cm at speed. LiDAR veto is unchanged.
        const double hard = c.margin + g.resolution * std::sqrt(.5);
        if (a >= -c.rear - hard && a <= c.length - c.rear + hard &&
            std::abs(b) <= c.width * .5 + hard)
          return fail(r, c.recovery_geometry?"recovery_exterior_nonroad":"observed_nonroad", x, y);
        ++r.soft_margin_cells;
      }
    }
  }
  return true;
}
bool CheckSweep(const Scene &s, const Config &c, const Pose &a, const Pose &b,
                Result &r) {
  const double d = std::hypot(b.x - a.x, b.y - a.y), turn = b.yaw - a.yaw;
  const int steps = std::max(1, int(std::ceil((d + std::abs(turn) * c.length) /
                                              (.5 * s.grid.resolution))));
  for (int i = 0; i <= steps; ++i) {
    double q = double(i) / steps;
    Pose m{a.x + q * (b.x - a.x), a.y + q * (b.y - a.y), a.yaw + q * turn,
           a.k + q * (b.k - a.k), 0};
    if (!CheckFootprint(s, c, m, r))
      return false;
  }
  return true;
}
void BuildGridIndex(Grid &grid) {
  grid.blocked_columns.assign(grid.rows, {});
  grid.exterior_nonroad.assign(grid.rows*grid.cols,0);
  for (int row = 0; row < grid.rows; ++row) {
    int first=grid.cols,last=-1;
    for(int col=0;col<grid.cols;++col) {
      const int i=row*grid.cols+col;
      if(grid.coverage[i]==255 && grid.probability[i]>=153) {first=std::min(first,col);last=col;}
    }
    for (int col = 0; col < grid.cols; ++col) {
      int index = row * grid.cols + col;
      if (grid.coverage[index] != 255 || grid.probability[index] < 153)
        grid.blocked_columns[row].push_back(col);
      // No road mask is filled/changed. This current-row exterior protects
      // actual road sides when an internal occlusion hole is a recovery cost.
      if(grid.coverage[index]==255 && grid.probability[index]<153 && (col<first || col>last))
        grid.exterior_nonroad[index]=1;
    }
  }
}
} // namespace local_planning
