#pragma once

#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <morai_msgs/GPSMessage.h>

#include "Planning/PlanningMode.hpp"

namespace asmc_planner {

// raw /gps 값과 GPS 기반 위치로 재밍을 판정한다. ROS 시간 대신 단조 시각[s]을 받아
// 단위 시험이 가능하다. 진입은 원인 하나만 있어도 즉시, 복귀는 모든 원인이
// exit_hold_s 동안 연속으로 사라졌을 때만 한다. 경계에서 모드가 번갈아 나오지 않는다.
struct JammingDetectorConfig {
  // MORAI /gps는 약 5Hz이며 간격이 0.37s까지 벌어진다. 정상 지터로 진입하지 않게 여유를 둔다.
  double gps_timeout_s = 1.0;  // 0 이하면 수신 끊김 규칙을 끈다.
  double exit_hold_s = 1.0;
  bool zone_enabled = true;
};

enum class JammingReason : std::uint8_t {
  NONE = 0,
  GPS_ZERO,     // 공식 규칙: raw /gps 위도·경도·고도가 모두 정확히 0
  GPS_NO_FIX,   // GGA fix quality 0 또는 유한하지 않거나 범위를 벗어난 좌표
  GPS_TIMEOUT,  // 수신 중이던 /gps가 끊김(blackout을 패킷 중단으로 주는 경우)
  SHADOW_ZONE,  // GPS 위치가 전역경로상의 음영 구간 안. blackout 직전부터 미리 전환한다.
};

inline const char* JammingReasonName(JammingReason reason) {
  switch (reason) {
    case JammingReason::GPS_ZERO: return "gps_zero";
    case JammingReason::GPS_NO_FIX: return "gps_no_fix";
    case JammingReason::GPS_TIMEOUT: return "gps_timeout";
    case JammingReason::SHADOW_ZONE: return "shadow_zone";
    default: return "none";
  }
}

class JammingDetector {
 public:
  explicit JammingDetector(JammingDetectorConfig config) : config_(config) {}

  void OnGps(const morai_msgs::GPSMessage& gps, double now) {
    has_gps_ = true;
    last_gps_s_ = now;
    gps_zero_ = DetectPlanningMode(gps) == PlanningMode::JAMMING;
    gps_no_fix_ = !gps_zero_ &&
        (gps.status <= 0 || !std::isfinite(gps.latitude) || !std::isfinite(gps.longitude) ||
         std::fabs(gps.latitude) > 90.0 || std::fabs(gps.longitude) > 180.0);
  }
  void OnZone(bool inside) { in_zone_ = inside; }

  // 모드가 바뀌면 true.
  bool Update(double now) {
    cause_ = Cause(now);
    if (cause_ != JammingReason::NONE) {
      clear_since_s_ = -1.0;
      entry_reason_ = cause_;
      if (mode_ == PlanningMode::JAMMING) return false;
      mode_ = PlanningMode::JAMMING;
      return true;
    }
    if (mode_ == PlanningMode::NORMAL) return false;
    if (clear_since_s_ < 0.0) clear_since_s_ = now;
    if (now - clear_since_s_ < config_.exit_hold_s) return false;
    mode_ = PlanningMode::NORMAL;
    entry_reason_ = JammingReason::NONE;
    return true;
  }

  PlanningMode mode() const { return mode_; }
  JammingReason cause() const { return cause_; }
  JammingReason entry_reason() const { return entry_reason_; }
  bool has_gps() const { return has_gps_; }
  bool gps_zero() const { return gps_zero_; }
  bool gps_no_fix() const { return gps_no_fix_; }
  bool in_zone() const { return in_zone_; }
  double gps_age(double now) const { return has_gps_ ? now - last_gps_s_ : -1.0; }
  double clear_elapsed(double now) const {
    return clear_since_s_ < 0.0 ? 0.0 : now - clear_since_s_;
  }

 private:
  JammingReason Cause(double now) const {
    if (has_gps_ && gps_zero_) return JammingReason::GPS_ZERO;
    if (has_gps_ && gps_no_fix_) return JammingReason::GPS_NO_FIX;
    // 첫 /gps 전에는 bridge가 아직 안 뜬 것이므로 끊김으로 보지 않는다.
    if (has_gps_ && config_.gps_timeout_s > 0.0 &&
        now - last_gps_s_ > config_.gps_timeout_s)
      return JammingReason::GPS_TIMEOUT;
    if (config_.zone_enabled && in_zone_) return JammingReason::SHADOW_ZONE;
    return JammingReason::NONE;
  }

  JammingDetectorConfig config_;
  PlanningMode mode_ = PlanningMode::NORMAL;
  JammingReason cause_ = JammingReason::NONE;
  JammingReason entry_reason_ = JammingReason::NONE;
  bool has_gps_ = false;
  bool gps_zero_ = false;
  bool gps_no_fix_ = false;
  bool in_zone_ = false;
  double last_gps_s_ = 0.0;
  double clear_since_s_ = -1.0;
};

// 전역경로 station 구간 [start, end]를 음영 구간으로 본다. 재밍 중 회피로 옆 차로까지
// 나가도 구간을 유지하도록 횡 허용폭을 넓게 두고, 다른 구간의 경로를 잘못 잡지
// 않도록 진행 방향을 함께 검사한다(같은 방향의 다른 경로는 30m 이상 떨어져 있다).
class ShadowZone {
 public:
  ShadowZone(const std::string& path_file, double station_start_m,
             double station_end_m, double lateral_tolerance_m,
             double heading_tolerance_rad)
      : lateral_tolerance_m_(lateral_tolerance_m),
        heading_tolerance_rad_(heading_tolerance_rad) {
    std::ifstream in(path_file);
    if (!in) throw std::runtime_error("cannot open shadow-zone path: " + path_file);
    std::vector<double> xs, ys;
    std::string line;
    while (std::getline(in, line)) {
      std::istringstream fields(line);
      double x, y;
      if (fields >> x >> y) { xs.push_back(x); ys.push_back(y); }
    }
    if (xs.size() < 2) throw std::runtime_error("shadow-zone path has < 2 points: " + path_file);
    double station = 0.0;
    for (std::size_t i = 0; i + 1 < xs.size(); ++i) {
      const double dx = xs[i + 1] - xs[i], dy = ys[i + 1] - ys[i];
      if (station >= station_start_m && station <= station_end_m)
        points_.push_back({xs[i], ys[i], std::atan2(dy, dx), station});
      station += std::hypot(dx, dy);
    }
    if (points_.empty())
      throw std::runtime_error("shadow-zone station range is outside the path");
  }

  // inside일 때 station_m에 가장 가까운 경로점의 station을 쓴다.
  bool Contains(double x, double y, double yaw, double* station_m = nullptr) const {
    const Point* nearest = nullptr;
    double best = lateral_tolerance_m_;
    for (const auto& p : points_) {
      const double d = std::hypot(p.x - x, p.y - y);
      if (d <= best) { best = d; nearest = &p; }
    }
    if (!nearest) return false;
    const double diff = std::atan2(std::sin(yaw - nearest->yaw), std::cos(yaw - nearest->yaw));
    if (std::fabs(diff) > heading_tolerance_rad_) return false;
    // 횡 허용폭이 넓어도 구간 시작 전·끝 뒤는 진행 방향 투영으로 잘라낸다.
    const double along = (x - nearest->x) * std::cos(nearest->yaw) +
                         (y - nearest->y) * std::sin(nearest->yaw);
    if (nearest == &points_.front() && along < 0.0) return false;
    if (nearest == &points_.back() && along > kPointSpacingM) return false;
    if (station_m) *station_m = nearest->station;
    return true;
  }

 private:
  static constexpr double kPointSpacingM = 0.5;  // 전역경로 점 간격
  struct Point { double x, y, yaw, station; };
  std::vector<Point> points_;
  double lateral_tolerance_m_;
  double heading_tolerance_rad_;
};

}  // namespace asmc_planner
