// Ported unchanged from Main PlannerInterface/KnownStaticObjectFilter.cpp at ab8db094.
// Namespace/types/includes only; retain acquisition/release and legacy policy for cutover.
#include <asmc_lidar/filtering/known_static.hpp>

#include <asmc_lidar/filtering/delivery_object.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>

namespace asmc::lidar::filtering {
namespace {

std::string NormalizeLabel(std::string label) {
    label.erase(label.begin(), std::find_if(
        label.begin(), label.end(), [](unsigned char value) {
            return !std::isspace(value);
        }));
    label.erase(std::find_if(
        label.rbegin(), label.rend(), [](unsigned char value) {
            return !std::isspace(value);
        }).base(), label.end());
    std::transform(label.begin(), label.end(), label.begin(),
                   [](unsigned char value) {
                       return static_cast<char>(std::tolower(value));
                   });
    return label;
}

bool HasConfirmedClassification(const DeliveryObject& object) {
    const std::string classification = NormalizeLabel(object.classification);
    if (!classification.empty() && classification != "unknown") return true;
    const std::string yolo = NormalizeLabel(object.yolo_class);
    return yolo == "car" || yolo == "vehicle" || yolo == "truck" ||
           yolo == "bus" || yolo == "person" || yolo == "pedestrian";
}

bool IsExplicitUnknown(const DeliveryObject& object) {
    return NormalizeLabel(object.classification) == "unknown";
}

}  // namespace

std::size_t KnownStaticObjectFilter::GridHash::operator()(
    const GridKey& key) const {
    std::size_t value = std::hash<std::int32_t>{}(key.x);
    value ^= std::hash<std::int32_t>{}(key.y) + 0x9e3779b9U +
             (value << 6U) + (value >> 2U);
    return value;
}

KnownStaticObjectFilter::GridKey KnownStaticObjectFilter::KeyFor(
    double x, double y) const {
    return {
        static_cast<std::int32_t>(std::floor(x / grid_resolution_m_)),
        static_cast<std::int32_t>(std::floor(y / grid_resolution_m_)),
    };
}

void KnownStaticObjectFilter::Reset(KnownStaticCatalogStatus status,
                                    const std::string& message) {
    references_.clear();
    grid_.clear();
    application_regions_.clear();
    application_region_enabled_ = false;
    application_min_x_ = 0.0;
    application_max_x_ = 0.0;
    application_min_y_ = 0.0;
    application_max_y_ = 0.0;
    loaded_map_id_.clear();
    status_ = status;
    status_message_ = message;
}

bool KnownStaticObjectFilter::IsInsideApplicationRegion(
    double x, double y) const {
    if (!application_region_enabled_) return true;
    return std::any_of(
        application_regions_.begin(), application_regions_.end(),
        [x, y](const KnownStaticApplicationRegion& region) {
            return x >= region.min_x && x <= region.max_x &&
                   y >= region.min_y && y <= region.max_y;
        });
}

bool KnownStaticObjectFilter::Configure(
    const std::string& catalog_path, const std::string& expected_map_id,
    const std::array<double, 3>& expected_origin,
    double center_tolerance_m, bool application_region_enabled,
    double application_min_x, double application_max_x,
    double application_min_y, double application_max_y) {
    std::vector<KnownStaticApplicationRegion> regions;
    if (application_region_enabled) {
        regions.push_back({application_min_x, application_max_x,
                           application_min_y, application_max_y});
    }
    return ConfigureWithRegions(
        catalog_path, expected_map_id, expected_origin,
        center_tolerance_m, regions);
}

bool KnownStaticObjectFilter::ConfigureWithRegions(
    const std::string& catalog_path, const std::string& expected_map_id,
    const std::array<double, 3>& expected_origin,
    double center_tolerance_m,
    const std::vector<KnownStaticApplicationRegion>& application_regions) {
    Reset(KnownStaticCatalogStatus::kNotConfigured, "configuration in progress");
    center_tolerance_m_ = std::max(0.1, center_tolerance_m);
    grid_resolution_m_ = std::max(1.0, 2.0 * center_tolerance_m_);
    neighbor_cells_ = std::max(
        1, static_cast<int>(std::ceil(
               center_tolerance_m_ / grid_resolution_m_)));
    application_regions_ = application_regions;
    application_region_enabled_ = !application_regions_.empty();
    if (application_region_enabled_) {
        application_min_x_ = std::numeric_limits<double>::infinity();
        application_max_x_ = -std::numeric_limits<double>::infinity();
        application_min_y_ = std::numeric_limits<double>::infinity();
        application_max_y_ = -std::numeric_limits<double>::infinity();
        for (const auto& region : application_regions_) {
            if (!std::isfinite(region.min_x) ||
                !std::isfinite(region.max_x) ||
                !std::isfinite(region.min_y) ||
                !std::isfinite(region.max_y) ||
                region.min_x > region.max_x ||
                region.min_y > region.max_y) {
                Reset(KnownStaticCatalogStatus::kInvalidMetadata,
                      "invalid known-static application region");
                return false;
            }
            application_min_x_ = std::min(application_min_x_, region.min_x);
            application_max_x_ = std::max(application_max_x_, region.max_x);
            application_min_y_ = std::min(application_min_y_, region.min_y);
            application_max_y_ = std::max(application_max_y_, region.max_y);
        }
    } else {
        application_min_x_ = 0.0;
        application_max_x_ = 0.0;
        application_min_y_ = 0.0;
        application_max_y_ = 0.0;
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(catalog_path);
    } catch (const std::exception& error) {
        Reset(KnownStaticCatalogStatus::kFileLoadFailed,
              "cannot load known-static catalog: " + catalog_path +
                  " (" + error.what() + ")");
        return false;
    }

    try {
        const std::string schema = root["schema"].as<std::string>();
        if (schema != "asmc_known_static_center_trace_v1") {
            throw std::runtime_error("unsupported schema: " + schema);
        }
        loaded_map_id_ = root["map_id"].as<std::string>();
        if (!expected_map_id.empty() && loaded_map_id_ != expected_map_id) {
            const std::string actual = loaded_map_id_;
            Reset(KnownStaticCatalogStatus::kMapIdMismatch,
                  "known-static map id mismatch: expected " +
                      expected_map_id + ", got " + actual);
            return false;
        }
        const YAML::Node origin = root["local_origin_in_global"];
        for (std::size_t i = 0; i < expected_origin.size(); ++i) {
            const double actual = origin[i].as<double>();
            if (!std::isfinite(actual) ||
                std::abs(actual - expected_origin[i]) > 1e-6) {
                Reset(KnownStaticCatalogStatus::kOriginMismatch,
                      "known-static local origin mismatch");
                return false;
            }
        }

        const YAML::Node observations = root["observations"];
        if (!observations || !observations.IsSequence()) {
            throw std::runtime_error("observations must be a sequence");
        }
        references_.reserve(observations.size());
        for (const auto& node : observations) {
            ReferenceObject reference;
            reference.track_id = node["track_id"].as<int>();
            reference.center_x = node["center"][0].as<double>();
            reference.center_y = node["center"][1].as<double>();
            if (!std::isfinite(reference.center_x) ||
                !std::isfinite(reference.center_y)) {
                throw std::runtime_error(
                    "catalog contains an invalid object observation");
            }
            if (!IsInsideApplicationRegion(
                    reference.center_x, reference.center_y)) {
                continue;
            }
            references_.push_back(reference);
        }
    } catch (const std::exception& error) {
        Reset(KnownStaticCatalogStatus::kInvalidMetadata,
              "invalid known-static catalog: " + std::string(error.what()));
        return false;
    }

    if (references_.empty()) {
        Reset(KnownStaticCatalogStatus::kInvalidMetadata,
              application_region_enabled_
                  ? "known-static application region has no observations"
                  : "known-static catalog has no observations");
        return false;
    }
    for (std::size_t i = 0; i < references_.size(); ++i) {
        const auto& reference = references_[i];
        grid_[KeyFor(reference.center_x, reference.center_y)].push_back(i);
    }
    status_ = KnownStaticCatalogStatus::kReady;
    status_message_ = "ready";
    return true;
}

KnownStaticDecision KnownStaticObjectFilter::Evaluate(
    const DeliveryObject& object) const {
    KnownStaticDecision decision;
    decision.filter_available = ready();
    if (!ready()) return decision;
    if (HasConfirmedClassification(object)) {
        decision.reason = "confirmed_class_keep";
        return decision;
    }
    if (!IsExplicitUnknown(object)) {
        decision.reason = "unrecognized_class_keep";
        return decision;
    }
    if (!std::isfinite(object.cluster_center_map.x) ||
        !std::isfinite(object.cluster_center_map.y)) {
        decision.reason = "invalid_geometry_keep";
        return decision;
    }
    if (!IsInsideApplicationRegion(
            object.cluster_center_map.x, object.cluster_center_map.y)) {
        decision.reason = "outside_application_region_keep";
        return decision;
    }
    decision.policy_applied = true;

    const GridKey center = KeyFor(
        object.cluster_center_map.x, object.cluster_center_map.y);
    double best_distance = std::numeric_limits<double>::infinity();
    for (int dx = -neighbor_cells_; dx <= neighbor_cells_; ++dx) {
        for (int dy = -neighbor_cells_; dy <= neighbor_cells_; ++dy) {
            const auto found = grid_.find({center.x + dx, center.y + dy});
            if (found == grid_.end()) continue;
            for (const std::size_t index : found->second) {
                const ReferenceObject& reference = references_[index];
                const double distance = std::hypot(
                    object.cluster_center_map.x - reference.center_x,
                    object.cluster_center_map.y - reference.center_y);
                if (distance > center_tolerance_m_) continue;
                if (distance >= best_distance) continue;
                best_distance = distance;
                decision.reference_track_id = reference.track_id;
                decision.center_distance_m = distance;
            }
        }
    }

    if (decision.reference_track_id < 0) {
        decision.reason = "no_static_match_keep";
        return decision;
    }
    decision.keep = false;
    decision.reason = "known_static_match_drop";
    return decision;
}

}  // namespace asmc::lidar::filtering
