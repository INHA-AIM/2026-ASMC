#include "Behavior/Map/route_manager.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace behavior_planner {
namespace {

SignalMovement ParseSignalMovement(const std::string& related_signal) {
  if (related_signal == "straight") return SignalMovement::STRAIGHT;
  if (related_signal == "left") return SignalMovement::LEFT;
  if (related_signal == "left_unprotected") {
    return SignalMovement::LEFT_UNPROTECTED;
  }
  if (related_signal == "right_unprotected") {
    return SignalMovement::RIGHT_UNPROTECTED;
  }
  if (related_signal == "uturn_normal" || related_signal == "uturn") {
    return SignalMovement::UTURN;
  }
  return SignalMovement::UNKNOWN;
}

}  // namespace

bool RouteManager::initialize(const MGeoLoader& map,
                              const std::vector<std::string>& ids) {
  map_ = &map;
  ids_ = ids;
  reference_.clear();
  cumulative_.clear();
  segment_link_.clear();
  link_spans_.clear();
  stops_.clear();
  std::unordered_set<std::string> route_end_nodes;
  double total = 0.0;

  for (const std::string& id : ids) {
    LinkInfo link;
    if (!map.getLink(id, &link)) continue;
    const double link_start_s = total;
    route_end_nodes.insert(link.to_node_id);
    for (const Point2D& point : link.centerline) {
      if (reference_.empty()) {
        reference_.push_back(point);
        cumulative_.push_back(0.0);
        continue;
      }
      const double distance =
          std::hypot(point.x - reference_.back().x,
                     point.y - reference_.back().y);
      if (distance < 1e-6) continue;
      total += distance;
      reference_.push_back(point);
      cumulative_.push_back(total);
      segment_link_.push_back(id);
    }
    link_spans_.push_back({id, link_start_s, total});
  }

  // Only accept a stop line whose node is the actual end node of a route link.
  // A distance-only search can select the stop line of an adjacent lane.
  for (StopLineInfo stop : map.stopLines()) {
    if (!route_end_nodes.count(stop.node_id)) continue;
    double route_s = 0.0;
    double route_d = 0.0;
    std::string route_link;
    if (projectEgo(stop.point.x, stop.point.y, &route_s, &route_d,
                   &route_link)) {
      stop.route_s = route_s;
      stop.valid = true;
      stops_.push_back(stop);
    }
  }
  std::sort(stops_.begin(), stops_.end(),
            [](const StopLineInfo& a, const StopLineInfo& b) {
              return a.route_s < b.route_s;
            });
  return reference_.size() > 1;
}

bool RouteManager::projectEgo(double x, double y, double* out_s, double* out_d,
                              std::string* out_id) const {
  if (reference_.size() < 2) return false;
  double best_distance_sq = 1e18;
  double best_s = 0.0;
  double best_d = 0.0;
  size_t best_segment = 0;
  for (size_t i = 1; i < reference_.size(); ++i) {
    const Point2D& a = reference_[i - 1];
    const Point2D& b = reference_[i];
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double length_sq = vx * vx + vy * vy;
    if (length_sq < 1e-9) continue;
    const double length = std::sqrt(length_sq);
    const double t = std::clamp(
        ((x - a.x) * vx + (y - a.y) * vy) / length_sq, 0.0, 1.0);
    const double dx = x - (a.x + t * vx);
    const double dy = y - (a.y + t * vy);
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_s = cumulative_[i - 1] + t * length;
      best_d = (vx * dy - vy * dx) / length;
      best_segment = i - 1;
    }
  }
  if (out_s) *out_s = best_s;
  if (out_d) *out_d = best_d;
  if (out_id) {
    *out_id = best_segment < segment_link_.size()
                  ? segment_link_[best_segment]
                  : ids_.back();
  }
  return true;
}

bool RouteManager::findStopLineByNode(const std::string& node_id, StopLineInfo* out) const {
  if (!out) return false;
  for (const auto& stop : stops_) if (stop.valid && stop.node_id == node_id) {
    *out = stop;
    return true;
  }
  return false;
}

bool RouteManager::findNextStopLine(double ego_s,
                                    StopLineInfo* out_stop_line) const {
  for (const StopLineInfo& stop : stops_) {
    if (stop.route_s >= ego_s) {
      if (out_stop_line) *out_stop_line = stop;
      return true;
    }
  }
  return false;
}

bool RouteManager::findNextSignalizedStopLine(
    double ego_s, double max_distance, StopLineInfo* out_stop_line,
    double* out_distance) const {
  if (!out_stop_line || !out_distance || !std::isfinite(ego_s) ||
      !std::isfinite(max_distance) || max_distance < 0.0) {
    return false;
  }
  // stops_ contains only stop-line nodes that are actual end nodes of the
  // configured route links. Requiring traffic_light_id excludes unrelated
  // stop marks without coupling map association to camera visibility.
  for (const StopLineInfo& stop : stops_) {
    if (!stop.valid || stop.traffic_light_id.empty() ||
        !std::isfinite(stop.route_s) || stop.route_s < ego_s) {
      continue;
    }
    const double distance = stop.route_s - ego_s;
    if (distance > max_distance) return false;
    *out_stop_line = stop;
    *out_distance = distance;
    return true;
  }
  return false;
}

bool RouteManager::findPreviousSignalizedStopLine(
    double ego_s, StopLineInfo* out_stop_line, double* out_distance) const {
  if (!out_stop_line || !out_distance || !std::isfinite(ego_s)) {
    return false;
  }

  // stops_ is ordered by route_s. Keep the nearest signalized stop line at
  // or behind ego so the speed cap can remain active for a short, explicit
  // post-intersection zone without changing ownership of the next stop line.
  const StopLineInfo* previous = nullptr;
  for (const StopLineInfo& stop : stops_) {
    if (!stop.valid || stop.traffic_light_id.empty() ||
        !std::isfinite(stop.route_s)) {
      continue;
    }
    if (stop.route_s > ego_s) break;
    previous = &stop;
  }
  if (!previous) return false;

  *out_stop_line = *previous;
  *out_distance = ego_s - previous->route_s;
  return true;
}

bool RouteManager::getCurrentLinkInfo(const std::string& id,
                                      LinkInfo* out) const {
  return map_ && map_->getLink(id, out);
}

double RouteManager::getSpeedLimit(const std::string& id) const {
  LinkInfo link;
  return getCurrentLinkInfo(id, &link) ? link.max_speed : 0.0;
}

bool RouteManager::getRoutePoint(double s, Point2D* out) const {
  if (!out || reference_.empty() || cumulative_.size() != reference_.size())
    return false;
  if (s <= 0.0) {
    *out = reference_.front();
    return true;
  }
  if (s >= cumulative_.back()) {
    *out = reference_.back();
    return true;
  }
  const auto it = std::lower_bound(cumulative_.begin(), cumulative_.end(), s);
  const size_t i = std::distance(cumulative_.begin(), it);
  if (i == 0) {
    *out = reference_.front();
    return true;
  }
  const double span = cumulative_[i] - cumulative_[i - 1];
  const double t = span > 1e-9 ? (s - cumulative_[i - 1]) / span : 0.0;
  out->x = reference_[i - 1].x +
           t * (reference_[i].x - reference_[i - 1].x);
  out->y = reference_[i - 1].y +
           t * (reference_[i].y - reference_[i - 1].y);
  return true;
}

bool RouteManager::getRoutePose(double s, Point2D* out,
                                double* out_yaw) const {
  if (!getRoutePoint(s, out) || !out_yaw || reference_.size() < 2) {
    return false;
  }
  const double clamped = std::clamp(s, 0.0, cumulative_.back());
  auto it = std::lower_bound(cumulative_.begin(), cumulative_.end(), clamped);
  size_t i = static_cast<size_t>(std::distance(cumulative_.begin(), it));
  if (i == 0) i = 1;
  if (i >= reference_.size()) i = reference_.size() - 1;
  const Point2D& a = reference_[i - 1];
  const Point2D& b = reference_[i];
  *out_yaw = std::atan2(b.y - a.y, b.x - a.x);
  return true;
}

bool RouteManager::getLinkTargetPose(const std::string& link_id,
                                     double ego_x, double ego_y,
                                     double lookahead_m, Point2D* out,
                                     double* out_yaw) const {
  if (!map_ || !out || !out_yaw) return false;
  LinkInfo link;
  if (!map_->getLink(link_id, &link) || link.centerline.size() < 2) {
    return false;
  }
  size_t nearest = 0;
  double best = 1e18;
  for (size_t i = 0; i < link.centerline.size(); ++i) {
    const double distance = std::hypot(link.centerline[i].x - ego_x,
                                       link.centerline[i].y - ego_y);
    if (distance < best) {
      best = distance;
      nearest = i;
    }
  }
  size_t selected = nearest;
  double accumulated = 0.0;
  while (selected + 1 < link.centerline.size() &&
         accumulated < std::max(0.0, lookahead_m)) {
    accumulated += std::hypot(
        link.centerline[selected + 1].x - link.centerline[selected].x,
        link.centerline[selected + 1].y - link.centerline[selected].y);
    ++selected;
  }
  const size_t heading_from = selected > 0 ? selected - 1 : selected;
  const size_t heading_to = selected + 1 < link.centerline.size()
                                ? selected + 1
                                : selected;
  if (heading_from == heading_to) return false;
  *out = link.centerline[selected];
  *out_yaw = std::atan2(link.centerline[heading_to].y -
                            link.centerline[heading_from].y,
                        link.centerline[heading_to].x -
                            link.centerline[heading_from].x);
  return true;
}

bool RouteManager::matchMapLink(double x, double y, double yaw,
                                LinkProjection* out,
                                double max_distance) const {
  return map_ && map_->findNearestLink(x, y, yaw, out, max_distance);
}

bool RouteManager::forwardDistance(const std::string& from_link_id,
                                   double from_link_s,
                                   const std::string& to_link_id,
                                   double to_link_s,
                                   double max_distance,
                                   double* out_distance) const {
  return map_ && map_->findForwardDistance(
                     from_link_id, from_link_s, to_link_id, to_link_s,
                     max_distance, out_distance);
}

bool RouteManager::findNextMapStopLine(const std::string& current_link_id,
                                       double current_link_s,
                                       double max_distance,
                                       StopLineInfo* out_stop_line,
                                       double* out_distance) const {
  return map_ && map_->findNextStopLine(current_link_id, current_link_s,
                                       max_distance, out_stop_line,
                                       out_distance);
}

bool RouteManager::findNextSignalMovement(
    double ego_route_s, double max_distance, RouteSignalMovement* out) const {
  if (!out || !map_ || !std::isfinite(ego_route_s) || max_distance < 0.0) {
    return false;
  }

  for (const RouteLinkSpan& span : link_spans_) {
    if (span.end_s + 1e-6 < ego_route_s) continue;
    LinkInfo link;
    if (!map_->getLink(span.link_id, &link) || link.related_signal.empty()) {
      continue;
    }
    const SignalMovement movement = ParseSignalMovement(link.related_signal);
    if (movement == SignalMovement::UNKNOWN) continue;

    const double distance = std::max(0.0, span.start_s - ego_route_s);
    if (distance > max_distance) return false;
    out->movement = movement;
    out->related_signal = link.related_signal;
    out->link_id = link.id;
    out->route_s = span.start_s;
    out->distance = distance;
    return true;
  }
  return false;
}

}  // namespace behavior_planner

