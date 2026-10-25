#pragma once
#include <asmc_lidar/filtering/delivery_object.hpp>
#include <asmc_lidar/filtering/known_static.hpp>
#include <asmc_lidar/filtering/structure_exclusion.hpp>
#include <asmc_lidar/filtering/observation_quality.hpp>

namespace asmc::lidar::filtering {
struct DeliveryConfig {
  bool enabled=false;
  std::string catalog, zones, catalog_map_id;
  std::array<double,3> origin{};
  std::vector<KnownStaticApplicationRegion> regions;
  double center_tolerance_m=2.5, structure_height_m=2.5, structure_bottom_m=2.;
  double minimum_obstacle_height_m=.10;
  int confirmation_frames=3, release_frames=5;
};

class DeliveryFilters {
 public:
  DeliveryFilters() = default;
  explicit DeliveryFilters(const DeliveryConfig& config) : enabled_(config.enabled) {
    if (!enabled_) return;
    const bool known=known_.ConfigureWithRegions(config.catalog,config.catalog_map_id,config.origin,
                                                config.center_tolerance_m,config.regions);
    const bool structure=structure_.Configure(config.zones,config.structure_height_m,config.structure_bottom_m);
    quality_.Configure(config.minimum_obstacle_height_m,config.confirmation_frames,config.release_frames);
    initial_quality_ = quality_;
    status_ = known && structure ? "ready" : "partial: " + known_.status_message() + "; " + structure_.status_message();
  }
  void BeginFrame(bool map_valid) {
    if (map_valid) quality_.BeginFrame();
    else quality_ = initial_quality_;  // never join map history across GPS loss
  }
  std::string Exclusion(const DeliveryObject& object, double ego_x, double ego_y,
                         double speed_mps, double ego_length_m) {
    if (!enabled_) return {};
    const auto known=known_.Evaluate(object);
    const auto structure=structure_.Evaluate(object);
    const auto quality=quality_.Evaluate(object,known.keep&&structure.keep,ego_x,ego_y,speed_mps,ego_length_m);
    if (!known.keep) return "known_static:"+known.reason;
    if (!structure.keep) return "structure:"+structure.reason;
    if (!quality.keep) return "observation_quality:"+quality.reason;
    return {};
  }
  bool enabled() const { return enabled_; }
  const std::string& status() const { return status_; }
 private:
  bool enabled_=false;
  std::string status_="off";
  KnownStaticObjectFilter known_;
  StaticStructureExclusionFilter structure_;
  ObservationQualityFilter quality_;
  ObservationQualityFilter initial_quality_;
};
}  // namespace asmc::lidar::filtering
