#pragma once
#include "local/planner.hpp"
#include <cmath>
#include <deque>

namespace local_planning {
// Causal, bounded road evidence for the calibrated near-ego blind region only.
// Never edits observed nonroad or extrapolates paint/wall width into free road.
// Optional speckle evidence is only metadata; newer nonroad vetoes old freedom.
// Fresh LiDAR hulls still veto every footprint. This is not object tracking.
class RoadMemory {
  struct Frame {
    double stamp;
    Pose source;
    Grid grid;
    std::vector<uint8_t> evidence; // 0 unknown,1 eroded free,2 observed nonroad
  };
  std::deque<Frame> frames;

public:
  void clear() { frames.clear(); }
  void apply(Grid &grid, const Pose &plan, double stamp, bool speckle=false) const {
    grid.recent_road_age.assign(grid.rows * grid.cols, 0.);
    struct Lookup {
      const Frame *frame;
      std::array<double, 3> a, x, y;
    };
    std::vector<Lookup> lookups;
    for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
      const double age = stamp - it->stamp;
      if (age <= 0 || age > 5. ||
          std::hypot(it->source.x - plan.x, it->source.y - plan.y) > 6. ||
          std::abs(it->source.yaw - plan.yaw) > .2)
        continue;
      Grid reference;
      reference.rotation = it->grid.rotation;
      reference.ground_z = it->grid.ground_z;
      double dx = it->source.x - plan.x, dy = it->source.y - plan.y;
      reference.source_in_plan = {
          std::cos(plan.yaw) * dx + std::sin(plan.yaw) * dy,
          -std::sin(plan.yaw) * dx + std::cos(plan.yaw) * dy,
          it->source.yaw - plan.yaw};
      auto a = PlanToGrid(reference, 0, 0), x = PlanToGrid(reference, 1, 0),
           y = PlanToGrid(reference, 0, 1);
      for (int k = 0; k < 3; ++k) {
        x[k] -= a[k];
        y[k] -= a[k];
      }
      lookups.push_back({&*it, a, x, y});
    }
    for (int r = 0; r < grid.rows; ++r)
      for (int c = 0; c < grid.cols; ++c) {
        const int i = r * grid.cols + c;
        const bool nonroad_candidate=speckle && grid.coverage[i]==255 && grid.probability[i]<153 &&
          grid.xmax-r*grid.resolution>=8. && grid.xmax-r*grid.resolution<=16.;
        if(grid.coverage[i]==255 && !nonroad_candidate)continue;
        const double bx = grid.xmax - r * grid.resolution,
                     by = grid.ymax - c * grid.resolution;
        if (!nonroad_candidate && (bx < -1.2 || bx > 6.5 || std::abs(by) > 2.5))
          continue;
        auto p = GridToPlan(grid, bx, by);
        for (const auto &l : lookups) {
          const auto &g = l.frame->grid;
          double x = l.a[0] + p[0] * l.x[0] + p[1] * l.y[0],
                 y = l.a[1] + p[0] * l.x[1] + p[1] * l.y[1];
          int rr = std::lround((g.xmax - x) / g.resolution),
              cc = std::lround((g.ymax - y) / g.resolution);
          if (rr < 0 || rr >= g.rows || cc < 0 || cc >= g.cols)
            continue;
          auto evidence = l.frame->evidence[rr * g.cols + cc];
          if (evidence == 2)
            break; // Newer observed obstruction wins.
          if (evidence == 1) {
            grid.recent_road_age[i] = stamp - l.frame->stamp;
            break;
          }
        }
      }
  }
  void observe(const Grid &g, const Pose &source, double stamp) {
    if (!frames.empty() && stamp <= frames.back().stamp)
      return;
    Frame f;
    f.stamp = stamp;
    f.source = source;
    f.grid.rows = g.rows;
    f.grid.cols = g.cols;
    f.grid.resolution = g.resolution;
    f.grid.xmax = g.xmax;
    f.grid.ymax = g.ymax;
    f.grid.rotation = g.rotation;
    f.grid.ground_z = g.ground_z;
    f.evidence.assign(g.rows * g.cols, 0);
    for (int r = 0; r < g.rows; ++r)
      for (int c = 0; c < g.cols; ++c) {
        int i = r * g.cols + c;
        if (g.coverage[i] != 255)
          continue;
        if (g.probability[i] < 153) {
          f.evidence[i] = 2;
          continue;
        }
        bool clear = r >= 2 && c >= 2 && r < g.rows - 2 && c < g.cols - 2;
        for (int dr = -2; dr <= 2 && clear; ++dr)
          for (int dc = -2; dc <= 2; ++dc) {
            int j = (r + dr) * g.cols + c + dc;
            if (g.coverage[j] != 255 || g.probability[j] < 204) {
              clear = false;
              break;
            }
          }
        if (clear)
          f.evidence[i] = 1;
      }
    frames.push_back(std::move(f));
    while (frames.size() > 50 || stamp - frames.front().stamp > 5.)
      frames.pop_front();
  }
};
} // namespace local_planning
