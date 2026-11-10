#include "Parser/StaticMissionPolicy.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>

namespace asmc_planner {
namespace {
std::string Sha256(const std::string& bytes) {
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size(), digest);
  std::ostringstream out;
  for (const auto byte : digest) out << std::hex << std::setw(2) << std::setfill('0') << int(byte);
  return out.str();
}
}  // namespace

StaticMissionPolicy::StaticMissionPolicy(const std::string& metadata_file,
    const std::string& data_root, double lateral, double heading)
    : lateral_tolerance_(lateral), heading_tolerance_(heading) {
  if (!std::isfinite(lateral) || lateral <= 0 || !std::isfinite(heading) ||
      heading <= 0 || heading > 3.141592653589793)
    throw std::runtime_error("invalid static mission projection tolerances");
  const auto metadata = YAML::LoadFile(metadata_file);
  if (metadata["schema_version"].as<std::string>() != "asmc.competition_metadata/v1" ||
      metadata["frame_id"].as<std::string>() != "map")
    throw std::runtime_error("invalid static mission metadata schema/frame");
  unsigned matches = 0;
  for (const auto& zone : metadata["mission_zones"]) {
    if (zone["id"].as<std::string>() != "static_obstacle_candidate") continue;
    ++matches;
    if (zone["mission_type"].as<std::string>() != "static_obstacle" ||
        !zone["planning_enforcement_allowed"].as<bool>())
      throw std::runtime_error("static mission is not approved for planning");
    start_ = zone["route_station_start_m"].as<double>();
    end_ = zone["route_station_end_m"].as<double>();
  }
  if (matches != 1 || !std::isfinite(start_) || !std::isfinite(end_) ||
      start_ < 0 || end_ <= start_)
    throw std::runtime_error("missing, duplicate or invalid static mission interval");
  const auto source = metadata["sources"]["official_global_path"];
  std::string path = source["path"].as<std::string>();
  const std::string prefix = "$ASMC_DATA/";
  if (path.compare(0, prefix.size(), prefix) == 0)
    path = data_root + "/" + path.substr(prefix.size());
  else if (path.empty() || path.front() != '/')
    throw std::runtime_error("official route must use an absolute path or $ASMC_DATA");
  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("cannot open static mission official route: " + path);
  const std::string bytes{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
  if (Sha256(bytes) != source["sha256"].as<std::string>())
    throw std::runtime_error("static mission official route SHA256 mismatch");
  std::istringstream input(bytes);
  std::string line;
  double station = 0;
  std::vector<Point> route;
  while (std::getline(input, line)) {
    if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
    std::istringstream fields(line);
    double x, y, z;
    if (!(fields >> x >> y >> z) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      throw std::runtime_error("invalid static mission official route point");
    if (!route.empty()) station += std::hypot(x - route.back().x, y - route.back().y);
    route.push_back({x, y, station});
  }
  const double length = metadata["route"]["length_xy_m"].as<double>();
  if (route.size() < 2 || route.size() != metadata["route"]["point_count"].as<std::size_t>() ||
      !std::isfinite(length) || std::abs(station - length) > 1e-5 || end_ > station)
    throw std::runtime_error("static mission route length/count/interval mismatch");
  segments_.reserve(route.size()-1);
  for (std::size_t i = 1; i < route.size(); ++i) {
    const auto& a = route[i-1];
    const auto& b = route[i];
    const double dx = b.x-a.x, dy = b.y-a.y, squared = dx*dx+dy*dy;
    if (squared <= 1e-12) continue;
    // Conservative bounds cover rounding in the original x-a-t*dx distance
    // expression. Keep the exact projection arithmetic and hypot tie behavior.
    const double pad = 64*std::numeric_limits<double>::epsilon()*
        std::max({1., std::abs(a.x), std::abs(a.y), std::abs(b.x), std::abs(b.y)});
    segments_.push_back({a, dx, dy, squared, b.s-a.s, std::atan2(dy, dx),
        {std::min(a.x,b.x)-pad, std::min(a.y,b.y)-pad,
         std::max(a.x,b.x)+pad, std::max(a.y,b.y)+pad}, i});
  }
  nodes_.reserve(2*segments_.size());
  if (!segments_.empty()) buildIndex(0, segments_.size());
}

std::size_t StaticMissionPolicy::buildIndex(std::size_t begin, std::size_t end) {
  Bounds bounds = segments_[begin].bounds;
  for (std::size_t i = begin+1; i < end; ++i) {
    const auto& b = segments_[i].bounds;
    bounds.min_x = std::min(bounds.min_x,b.min_x); bounds.min_y = std::min(bounds.min_y,b.min_y);
    bounds.max_x = std::max(bounds.max_x,b.max_x); bounds.max_y = std::max(bounds.max_y,b.max_y);
  }
  const auto index = nodes_.size();
  nodes_.push_back({bounds, begin, end});
  if (end-begin > 8) {
    const bool split_x = bounds.max_x-bounds.min_x >= bounds.max_y-bounds.min_y;
    const auto mid = begin+(end-begin)/2;
    std::nth_element(segments_.begin()+begin, segments_.begin()+mid, segments_.begin()+end,
        [split_x](const Segment& a, const Segment& b) {
          const auto center = [split_x](const Bounds& box) {
            return split_x ? .5*box.min_x+.5*box.max_x : .5*box.min_y+.5*box.max_y;
          };
          const double ca = center(a.bounds), cb = center(b.bounds);
          return ca == cb ? a.order < b.order : ca < cb;
        });
    const auto left = buildIndex(begin,mid), right = buildIndex(mid,end);
    nodes_[index].left = left; nodes_[index].right = right;
  }
  return index;
}

void StaticMissionPolicy::nearest(std::size_t index, double x, double y, double& best,
    std::size_t& selected, double& station) const {
  const auto distanceTo = [x,y](const Bounds& b) {
    return std::hypot(std::max({b.min_x-x,0.,x-b.max_x}),
                      std::max({b.min_y-y,0.,y-b.max_y}));
  };
  const auto& node = nodes_[index];
  if (distanceTo(node.bounds) > std::nextafter(best, std::numeric_limits<double>::infinity())) return;
  if (node.end-node.begin <= 8) {
    for (auto i = node.begin; i < node.end; ++i) {
      const auto& s = segments_[i];
      const double t = std::clamp(((x-s.a.x)*s.dx+(y-s.a.y)*s.dy)/s.squared,0.,1.);
      const double distance = std::hypot(x-s.a.x-t*s.dx,y-s.a.y-t*s.dy);
      // The original linear search keeps the earliest route segment on ties.
      if (distance < best || (distance == best &&
          (selected == segments_.size() || s.order < segments_[selected].order))) {
        best = distance; selected = i; station = s.a.s+t*s.ds;
      }
    }
    return;
  }
  auto first = node.left, second = node.right;
  if (distanceTo(nodes_[second].bounds) < distanceTo(nodes_[first].bounds)) std::swap(first,second);
  nearest(first,x,y,best,selected,station);
  nearest(second,x,y,best,selected,station);
}

bool StaticMissionPolicy::project(double x, double y, double& station, double& yaw) const {
  if (!std::isfinite(x) || !std::isfinite(y) || nodes_.empty()) return false;
  // Index the whole official route, including segments outside the mission.
  // No candidate beyond the tolerance can make this query valid.
  double best = lateral_tolerance_;
  std::size_t selected = segments_.size();
  nearest(0,x,y,best,selected,station);
  if (selected == segments_.size()) return false;
  yaw = segments_[selected].yaw;
  return true;
}

bool StaticMissionPolicy::contains(double x, double y, double* station) const {
  double s = 0, yaw = 0;
  if (!project(x, y, s, yaw)) return false;
  if (station) *station = s;
  return s >= start_ && s <= end_;
}

bool StaticMissionPolicy::containsEgo(double x, double y, double yaw, bool valid) const {
  double s = 0, route_yaw = 0;
  if (!valid || !std::isfinite(yaw) || !project(x, y, s, route_yaw) || s < start_ || s > end_)
    return false;
  return std::abs(std::atan2(std::sin(yaw - route_yaw), std::cos(yaw - route_yaw))) <= heading_tolerance_;
}

std::size_t StaticMissionPolicy::apply(double x, double y, double yaw, bool valid,
    planning_inputs::DetectedObjectArray& objects) const {
  if (objects.header.frame_id != "map" || objects.header.stamp.isZero() ||
      !containsEgo(x, y, yaw, valid)) return 0;
  return remapObjects(objects);
}

std::size_t StaticMissionPolicy::remapObjects(planning_inputs::DetectedObjectArray& objects) const {
  if (objects.header.frame_id != "map" || objects.header.stamp.isZero()) return 0;
  std::size_t changed = 0;
  for (auto& object : objects.objects) {
    // Competition mission prior (2026-10-30 user-approved): every observed
    // footprint INSIDE this verified static mission is stationary for planning.
    // This is a private effective copy shared by Behavior/Frenet, not a change
    // to LiDAR measurements. Raw vx/vy, semantics and availability survive.
    if (object.motion_state > planning_inputs::DetectedObject::MOTION_DYNAMIC ||
        !std::isfinite(object.vx) || !std::isfinite(object.vy) ||
        (object.velocity_unavailable &&
         (object.motion_state != planning_inputs::DetectedObject::MOTION_UNKNOWN ||
          object.vx != 0.0 || object.vy != 0.0))) continue;
    if (contains(object.center.x, object.center.y)) {
      const auto motion = object.velocity_unavailable
          ? planning_inputs::DetectedObject::MOTION_UNKNOWN
          : planning_inputs::DetectedObject::MOTION_LOW_THREAT;
      if (object.type != 3 || object.motion_state != motion) ++changed;
      object.type = 3;  // STATIC_OBSTACLE policy, including semantic person/car props.
      object.motion_state = motion;
    }
  }
  return changed;
}

bool StaticMissionPolicy::Observation::update(const StaticMissionPolicy& policy,
    const planning_inputs::DetectedObjectArray::ConstPtr& source,
    double x, double y, double yaw, bool valid) {
  const bool in_scope = policy.containsEgo(x,y,yaw,valid);
  if (policy_ == &policy && source_ == source && ego_in_scope_ == in_scope) return false;
  policy_ = &policy; source_ = source; ego_in_scope_ = in_scope;
  effective_ = source ? *source : planning_inputs::DetectedObjectArray{};
  remapped_ = in_scope ? policy.remapObjects(effective_) : 0;
  return true;
}
}  // namespace asmc_planner
