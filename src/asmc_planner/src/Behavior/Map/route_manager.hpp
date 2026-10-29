#pragma once

#include <string>
#include <cstdint>
#include <vector>

#include "Behavior/Context/behavior_types.hpp"
#include "Behavior/Map/mgeo_loader.hpp"

namespace behavior_planner {

struct RouteSignalMovement {
  SignalMovement movement = SignalMovement::UNKNOWN;
  std::string related_signal;
  std::string link_id;
  double route_s = 1e9;
  double distance = 1e9;
};

class RouteManager {
 public:
  bool initialize(const MGeoLoader& map,
                  const std::vector<std::string>& route_link_ids);
  bool projectEgo(double x, double y, double* out_s, double* out_d,
                  std::string* out_link_id) const;
  bool findStopLineByNode(const std::string& node_id, StopLineInfo* out) const;
  // false means the scope/stop cannot be established, rather than outside.
  bool stopInRouteSpan(const std::string& node_id, const std::string& first_link,
                       const std::string& last_link, bool* inside) const;
  bool routeSpan(const std::string& first_link,const std::string& last_link,
                 double* first_s,double* last_s) const;
  bool findNextStopLine(double ego_s, StopLineInfo* out_stop_line) const;
  bool findNextSignalizedStopLine(double ego_s, double max_distance,
                                  StopLineInfo* out_stop_line,
                                  double* out_distance) const;
  bool findPreviousSignalizedStopLine(double ego_s,
                                      StopLineInfo* out_stop_line,
                                      double* out_distance) const;
  bool getCurrentLinkInfo(const std::string& current_link_id,
                          LinkInfo* out) const;
  bool nextRouteLink(const std::string& id, LinkInfo* next, double* boundary, double* next_end) const;
  double getSpeedLimit(const std::string& current_link_id) const;
  bool getRoutePoint(double route_s, Point2D* out) const;
  bool getRoutePose(double route_s, Point2D* out, double* out_yaw) const;
  bool getLinkTargetPose(const std::string& link_id, double ego_x,
                         double ego_y, double lookahead_m, Point2D* out,
                         double* out_yaw) const;
  bool matchMapLink(double x, double y, double yaw,
                    LinkProjection* out,
                    double max_distance = 12.0) const;
  bool forwardDistance(const std::string& from_link_id,
                       double from_link_s,
                       const std::string& to_link_id,
                       double to_link_s,
                       double max_distance,
                       double* out_distance) const;
  bool findNextMapStopLine(const std::string& current_link_id,
                           double current_link_s,
                           double max_distance,
                           StopLineInfo* out_stop_line,
                           double* out_distance) const;
  bool findNextSignalMovement(double ego_route_s, double max_distance,
                              RouteSignalMovement* out) const;
  const std::vector<Point2D>& referenceLine() const { return reference_; }
  std::uint64_t generation() const { return generation_; }

 private:
  const MGeoLoader* map_ = nullptr;
  std::uint64_t generation_ = 0;
  std::vector<std::string> ids_;
  std::vector<Point2D> reference_;
  std::vector<double> cumulative_;
  std::vector<std::string> segment_link_;
  struct RouteLinkSpan {
    std::string link_id;
    double start_s = 0.0;
    double end_s = 0.0;
  };
  std::vector<RouteLinkSpan> link_spans_;
  std::vector<StopLineInfo> stops_;
};

}  // namespace behavior_planner
