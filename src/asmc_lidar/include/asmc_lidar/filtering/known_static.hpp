// Ported unchanged from Main PlannerInterface/KnownStaticObjectFilter.hpp at ab8db094.
// Namespace/types/includes only; retain acquisition/release and legacy policy for cutover.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace asmc::lidar::filtering {
struct DeliveryObject;

enum class KnownStaticCatalogStatus {
    kNotConfigured = 0,
    kReady,
    kFileLoadFailed,
    kMapIdMismatch,
    kOriginMismatch,
    kInvalidMetadata,
};

struct KnownStaticDecision {
    bool keep = true;
    bool filter_available = false;
    bool policy_applied = false;
    int reference_track_id = -1;
    double center_distance_m = -1.0;
    std::string reason = "filter_unavailable_keep";
};

struct KnownStaticApplicationRegion {
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
};

// Filters clean-lap background objects only at the final planner publication
// boundary. Clustering, tracking, motion classification, and camera fusion all
// run before this class is called. A missing/mismatched catalog is fail-safe:
// every object is preserved.
class KnownStaticObjectFilter {
public:
    bool Configure(const std::string& catalog_path,
                   const std::string& expected_map_id,
                   const std::array<double, 3>& expected_origin,
                   double center_tolerance_m,
                   bool application_region_enabled = false,
                   double application_min_x = 0.0,
                   double application_max_x = 0.0,
                   double application_min_y = 0.0,
                   double application_max_y = 0.0);

    // Multiple disjoint regions keep the catalog filter confined to verified
    // fixed structures without enabling it for the large rectangle between
    // those structures.
    bool ConfigureWithRegions(
        const std::string& catalog_path,
        const std::string& expected_map_id,
        const std::array<double, 3>& expected_origin,
        double center_tolerance_m,
        const std::vector<KnownStaticApplicationRegion>& application_regions);

    KnownStaticDecision Evaluate(const DeliveryObject& object) const;

    bool ready() const {
        return status_ == KnownStaticCatalogStatus::kReady;
    }
    KnownStaticCatalogStatus status() const { return status_; }
    const std::string& status_message() const { return status_message_; }
    const std::string& loaded_map_id() const { return loaded_map_id_; }
    std::size_t reference_count() const { return references_.size(); }
    double center_tolerance_m() const { return center_tolerance_m_; }
    bool application_region_enabled() const {
        return application_region_enabled_;
    }
    double application_min_x() const { return application_min_x_; }
    double application_max_x() const { return application_max_x_; }
    double application_min_y() const { return application_min_y_; }
    double application_max_y() const { return application_max_y_; }
    const std::vector<KnownStaticApplicationRegion>& application_regions()
        const {
        return application_regions_;
    }

private:
    struct ReferenceObject {
        int track_id = -1;
        double center_x = 0.0;
        double center_y = 0.0;
    };

    struct GridKey {
        std::int32_t x = 0;
        std::int32_t y = 0;
        bool operator==(const GridKey& other) const {
            return x == other.x && y == other.y;
        }
    };

    struct GridHash {
        std::size_t operator()(const GridKey& key) const;
    };

    GridKey KeyFor(double x, double y) const;
    bool IsInsideApplicationRegion(double x, double y) const;
    void Reset(KnownStaticCatalogStatus status,
               const std::string& message);

    std::vector<ReferenceObject> references_;
    std::unordered_map<GridKey, std::vector<std::size_t>, GridHash> grid_;
    double center_tolerance_m_ = 1.0;
    double grid_resolution_m_ = 2.0;
    int neighbor_cells_ = 1;
    bool application_region_enabled_ = false;
    double application_min_x_ = 0.0;
    double application_max_x_ = 0.0;
    double application_min_y_ = 0.0;
    double application_max_y_ = 0.0;
    std::vector<KnownStaticApplicationRegion> application_regions_;
    KnownStaticCatalogStatus status_ =
        KnownStaticCatalogStatus::kNotConfigured;
    std::string status_message_ = "not configured";
    std::string loaded_map_id_;
};

}  // namespace asmc::lidar::filtering
