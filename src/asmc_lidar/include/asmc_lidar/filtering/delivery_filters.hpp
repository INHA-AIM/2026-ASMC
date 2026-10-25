#pragma once
#include <asmc_lidar/filtering/delivery_object.hpp>
#include <asmc_lidar/filtering/observation_quality.hpp>

namespace asmc::lidar::filtering {
struct DeliveryConfig {
  bool enabled=false;
  double minimum_obstacle_height_m=.10;
  int confirmation_frames=3, release_frames=5;
};

class DeliveryFilters {
 public:
  DeliveryFilters() = default;
  explicit DeliveryFilters(const DeliveryConfig& config) : enabled_(config.enabled) {
    if (!enabled_) return;
    quality_.Configure(config.minimum_obstacle_height_m,config.confirmation_frames,config.release_frames);
    initial_quality_ = quality_;
    status_ = "observation_quality_only";
  }
  void BeginFrame(bool map_valid) {
    if (map_valid) quality_.BeginFrame();
    else quality_ = initial_quality_;  // never join map history across GPS loss
  }
  std::string Exclusion(const DeliveryObject& object, double ego_x, double ego_y,
                         double speed_mps, double ego_length_m) {
    if (!enabled_) return {};
    const auto quality=quality_.Evaluate(object,true,ego_x,ego_y,speed_mps,ego_length_m);
    if (!quality.keep) return "observation_quality:"+quality.reason;
    return {};
  }
  bool enabled() const { return enabled_; }
  const std::string& status() const { return status_; }
 private:
  bool enabled_=false;
  std::string status_="off";
  ObservationQualityFilter quality_;
  ObservationQualityFilter initial_quality_;
};
}  // namespace asmc::lidar::filtering
