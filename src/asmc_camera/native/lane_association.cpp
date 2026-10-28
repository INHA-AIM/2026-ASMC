// Same-frame lane-node association. No ROS, history between frames, or GPU.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <tuple>
#include <vector>

namespace py = pybind11;
using Track = std::vector<std::size_t>;
using Candidate = std::tuple<double, double, std::size_t, std::size_t>;

struct Point { double x, y; };
struct Prediction { double slope, intercept; };

// Guard arithmetic-sensitive gates and near ties. The adapter evaluates such
// frames using the original NumPy implementation, including its BLAS rounding.
static bool near(double a, double b) {
  return std::abs(a - b) <= 1e-12 * std::max({1.0, std::abs(a), std::abs(b)});
}

static Prediction fit(const std::array<Point, 6>& points, std::size_t count,
                      double fallback_slope, bool& ambiguous) {
  double mx = 0.0, my = 0.0;
  for (std::size_t i = 0; i < count; ++i) { mx += points[i].x; my += points[i].y; }
  mx /= static_cast<double>(count);
  my /= static_cast<double>(count);
  double denominator = 0.0, numerator = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const double dx = points[i].x - mx;
    denominator += dx * dx;
    numerator += dx * (points[i].y - my);
  }
  ambiguous = ambiguous || near(denominator, 1e-9);
  const double slope = denominator <= 1e-9 ? fallback_slope : numerator / denominator;
  return {slope, my - slope * mx};
}

static py::tuple associate(
    py::array_t<double, py::array::c_style> xy,
    py::array_t<std::int64_t, py::array::c_style> classes,
    py::array_t<std::int64_t, py::array::c_style> bins,
    double max_gap, double max_residual, double max_heading,
    std::int64_t min_nodes, double min_span) {
  if (xy.ndim() != 2 || xy.shape(1) != 2 || classes.ndim() != 1 || bins.ndim() != 1 ||
      xy.shape(0) != classes.shape(0) || xy.shape(0) != bins.shape(0))
    throw py::value_error("association arrays must have shapes (N,2), (N,), (N,)");
  if (!(max_gap > 0.0) || !(max_residual > 0.0) || !(max_heading > 0.0 && max_heading < 90.0) ||
      min_nodes < 2 || !(min_span > 0.0))
    throw py::value_error("invalid association constraints");
  auto points = xy.unchecked<2>();
  auto labels = classes.unchecked<1>();
  auto xbins = bins.unchecked<1>();
  std::vector<Track> result;
  std::array<std::int64_t, 10> counts{};
  counts[0] = xy.shape(0);
  bool ambiguous = false;
  {
    py::gil_scoped_release release;
    std::vector<Point> values;
    values.reserve(xy.shape(0));
    std::map<std::int64_t, std::map<std::int64_t, Track>> groups;
    for (py::ssize_t i = 0; i < xy.shape(0); ++i) {
      values.push_back({points(i, 0), points(i, 1)});
      if (!std::isfinite(points(i, 0)) || !std::isfinite(points(i, 1))) {
        ambiguous = true;
        break;
      }
      groups[labels(i)][xbins(i)].push_back(i);
    }
    std::vector<Track> completed;
    if (!ambiguous) for (const auto& cls : groups) {
      std::vector<Track> active;
      // Prediction depends only on the last six nodes, not the queried bin.
      // Keep it with the track while scanning bins without an assignment.
      std::vector<Prediction> predictions;
      std::vector<bool> prediction_valid;
      for (const auto& bin : cls.second) {
        Track current = bin.second;
        std::stable_sort(current.begin(), current.end(), [&](std::size_t a, std::size_t b) {
          return values[a].y < values[b].y;
        });
        double x = 0.0;
        for (auto id : current) x += values[id].x;
        x /= static_cast<double>(current.size());
        std::vector<Track> retained;
        std::vector<Prediction> kept_predictions;
        std::vector<bool> kept_valid;
        for (std::size_t i = 0; i < active.size(); ++i) {
          const double gap = x - values[active[i].back()].x;
          ambiguous = ambiguous || near(gap, max_gap);
          if (gap <= max_gap) {
            retained.push_back(std::move(active[i]));
            kept_predictions.push_back(predictions[i]);
            kept_valid.push_back(prediction_valid[i]);
          } else completed.push_back(std::move(active[i]));
        }
        active = std::move(retained);
        predictions = std::move(kept_predictions);
        prediction_valid = std::move(kept_valid);
        std::vector<Candidate> candidates;
        for (std::size_t ti = 0; ti < active.size(); ++ti) {
          const auto& track = active[ti];
          if (!prediction_valid[ti]) {
            if (track.size() < 3) predictions[ti] = {0.0, values[track.back()].y};
            else {
              std::array<Point, 6> recent{};
              const auto start = track.size() > 6 ? track.size() - 6 : 0;
              for (std::size_t i = start; i < track.size(); ++i) recent[i - start] = values[track[i]];
              predictions[ti] = fit(recent, track.size() - start, 0.0, ambiguous);
            }
            prediction_valid[ti] = true;
          }
          const auto prediction = predictions[ti];
          const double predicted_y = prediction.slope * x + prediction.intercept;
          const double previous_angle = std::atan(prediction.slope);
          // Reuse the recent-five prefix across candidate nodes.
          std::array<Point, 6> recent{};
          const auto start = track.size() > 5 ? track.size() - 5 : 0;
          const auto prefix = track.size() - start;
          for (std::size_t i = start; i < track.size(); ++i) recent[i - start] = values[track[i]];
          for (std::size_t ni = 0; ni < current.size(); ++ni) {
            ++counts[1];
            const auto point = values[current[ni]];
            const double dx = point.x - values[track.back()].x;
            ambiguous = ambiguous || near(dx, 1e-6) || near(dx, max_gap);
            if (dx <= 1e-6 || dx > max_gap) { ++counts[2]; continue; }
            const double residual = std::abs(point.y - predicted_y);
            ambiguous = ambiguous || near(residual, max_residual);
            if (residual > max_residual) { ++counts[3]; continue; }
            recent[prefix] = point;
            const auto updated = fit(recent, prefix + 1, prediction.slope, ambiguous);
            constexpr double radians_to_degrees = 57.2957795130823208768;
            const double heading = std::abs((std::atan(updated.slope) - previous_angle) * radians_to_degrees);
            ambiguous = ambiguous || near(heading, max_heading);
            if (heading > max_heading) { ++counts[4]; continue; }
            candidates.emplace_back(residual, heading, ti, ni);
          }
        }
        std::sort(candidates.begin(), candidates.end());
        for (std::size_t i = 1; i < candidates.size(); ++i) {
          const auto& a = candidates[i - 1]; const auto& b = candidates[i];
          // Guard the whole tied group, including competitors separated by an
          // unrelated candidate in lexicographic order.
          if (near(std::get<0>(a), std::get<0>(b))) ambiguous = true;
        }
        std::vector<bool> assigned_tracks(active.size(), false), assigned_nodes(current.size(), false);
        for (const auto& candidate : candidates) {
          const auto ti = std::get<2>(candidate), ni = std::get<3>(candidate);
          if (assigned_tracks[ti] || assigned_nodes[ni]) { ++counts[5]; continue; }
          active[ti].push_back(current[ni]); prediction_valid[ti] = false;
          assigned_tracks[ti] = assigned_nodes[ni] = true;
        }
        for (std::size_t ni = 0; ni < current.size(); ++ni) if (!assigned_nodes[ni]) {
          active.push_back({current[ni]}); predictions.push_back({0.0, 0.0}); prediction_valid.push_back(false);
        }
      }
      for (auto& track : active) completed.push_back(std::move(track));
    }
    counts[6] = completed.size();
    for (auto& track : completed) {
      std::stable_sort(track.begin(), track.end(), [&](std::size_t a, std::size_t b) { return values[a].x < values[b].x; });
      const double span = values[track.back()].x - values[track.front()].x;
      if (static_cast<std::int64_t>(track.size()) < min_nodes) { ++counts[7]; continue; }
      if (span < min_span) { ++counts[8]; continue; }
      result.push_back(std::move(track));
    }
    std::stable_sort(result.begin(), result.end(), [&](const Track& a, const Track& b) {
      const auto ai = a.front(), bi = b.front();
      return std::make_tuple(labels(ai), values[ai].y, values[ai].x) <
             std::make_tuple(labels(bi), values[bi].y, values[bi].x);
    });
    counts[9] = result.size();
  }
  return py::make_tuple(result, counts, ambiguous);
}

PYBIND11_MODULE(_lane_association, module) {
  module.doc() = "Deterministic same-frame association; ambiguous arithmetic uses Python fallback.";
  module.def("associate", &associate, py::arg("xy").noconvert(), py::arg("classes").noconvert(),
             py::arg("bins").noconvert(), py::arg("max_gap"), py::arg("max_residual"),
             py::arg("max_heading"), py::arg("min_nodes"), py::arg("min_span"));
}
