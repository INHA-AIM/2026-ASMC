#include <PlannerInterface/StaticStructureExclusionFilter.hpp>

#include <PlannerInterface/PlannerObjectAdapter.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>

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

bool IsExplicitUnknown(const PlannerObject& object) {
    const std::string classification = NormalizeLabel(object.classification);
    const std::string yolo_class = NormalizeLabel(object.yolo_class);
    return classification == "unknown" &&
           (yolo_class.empty() || yolo_class == "unknown");
}

bool HasMotionEvidence(const PlannerObject& object) {
    // asmc motion-state contract: 2=DYNAMIC_PENDING, 3=DYNAMIC.
    return object.is_dynamic || object.motion_state == 2 ||
           object.motion_state == 3;
}

}  // namespace

void StaticStructureExclusionFilter::Reset(const std::string& message) {
    ready_ = false;
    zones_.clear();
    status_message_ = message;
}

bool StaticStructureExclusionFilter::Configure(
    const std::string& config_path, double minimum_height_m,
    double minimum_bottom_z_m) {
    Reset("configuration in progress");
    minimum_height_m_ = std::max(0.0, minimum_height_m);
    minimum_bottom_z_m_ = minimum_bottom_z_m;

    YAML::Node root;
    try {
        root = YAML::LoadFile(config_path);
        const YAML::Node zones = root["zones"];
        if (!zones || !zones.IsSequence()) {
            throw std::runtime_error("zones must be a sequence");
        }
        for (const auto& node : zones) {
            const YAML::Node bounds = node["bounds"];
            if (!bounds || !bounds.IsSequence() || bounds.size() != 4) {
                throw std::runtime_error(
                    "zone bounds must be [min_x,max_x,min_y,max_y]");
            }
            Zone zone;
            zone.name = node["name"].as<std::string>();
            zone.min_x = bounds[0].as<double>();
            zone.max_x = bounds[1].as<double>();
            zone.min_y = bounds[2].as<double>();
            zone.max_y = bounds[3].as<double>();
            if (node["require_infrastructure_shape"]) {
                zone.require_infrastructure_shape =
                    node["require_infrastructure_shape"].as<bool>();
            }
            if (node["require_elevated_pole_shape"]) {
                zone.require_elevated_pole_shape =
                    node["require_elevated_pole_shape"].as<bool>();
            }
            if (zone.name.empty() || !std::isfinite(zone.min_x) ||
                !std::isfinite(zone.max_x) ||
                !std::isfinite(zone.min_y) ||
                !std::isfinite(zone.max_y) ||
                zone.min_x > zone.max_x || zone.min_y > zone.max_y) {
                throw std::runtime_error("invalid static exclusion zone");
            }
            zones_.push_back(zone);
        }
    } catch (const std::exception& error) {
        Reset("cannot load static exclusion config: " +
              std::string(error.what()));
        return false;
    }

    if (zones_.empty()) {
        Reset("static exclusion config contains no zones");
        return false;
    }
    ready_ = true;
    status_message_ = "ready";
    return true;
}

StaticStructureExclusionDecision
StaticStructureExclusionFilter::Evaluate(
    const PlannerObject& object) const {
    StaticStructureExclusionDecision decision;
    decision.filter_available = ready_;
    if (!ready_) return decision;
    if (!IsExplicitUnknown(object)) {
        decision.reason = "classified_object_keep";
        return decision;
    }
    if (!std::isfinite(object.obb_center_map.x) ||
        !std::isfinite(object.obb_center_map.y) ||
        !std::isfinite(object.height)) {
        decision.reason = "invalid_geometry_keep";
        return decision;
    }

    double bottom_z = std::numeric_limits<double>::infinity();
    for (const auto& corner : object.obb_corners_map) {
        if (!std::isfinite(corner.z)) {
            decision.reason = "invalid_geometry_keep";
            return decision;
        }
        bottom_z = std::min(bottom_z, corner.z);
    }

    for (const auto& zone : zones_) {
        if (object.obb_center_map.x < zone.min_x ||
            object.obb_center_map.x > zone.max_x ||
            object.obb_center_map.y < zone.min_y ||
            object.obb_center_map.y > zone.max_y) {
            continue;
        }
        decision.policy_applied = true;
        decision.zone_name = zone.name;
        if (zone.require_elevated_pole_shape) {
            // The C091 sign is elevated above the road. Partial scans can
            // report only a thin slice of its full height, and the tracker
            // may call the sliding partial view PENDING. In this narrow
            // map zone require narrow geometry and measured ground clearance.
            const bool elevated_pole =
                std::isfinite(object.length) && object.length >= 0.15 &&
                object.length <= 1.5 && std::isfinite(object.width) &&
                object.width >= 0.05 && object.width <= 0.5 &&
                object.height >= 0.0 && object.height <= 3.5 &&
                object.ground_evidence.valid &&
                object.ground_evidence.sample_count >= 5 &&
                std::isfinite(object.ground_evidence.median_clearance_m) &&
                object.ground_evidence.median_clearance_m >= 1.8;
            if (!elevated_pole) {
                decision.reason = "elevated_pole_evidence_missing_keep";
                return decision;
            }
            decision.keep = false;
            decision.reason = "configured_elevated_pole_drop";
            return decision;
        }
        if (HasMotionEvidence(object)) {
            decision.reason = "motion_evidence_keep";
            return decision;
        }
        const bool infrastructure_shape =
            object.height >= minimum_height_m_ ||
            bottom_z >= minimum_bottom_z_m_;
        if (zone.require_infrastructure_shape && !infrastructure_shape) {
            decision.reason = "zone_shape_mismatch_keep";
            return decision;
        }
        decision.keep = false;
        decision.reason = "configured_static_structure_drop";
        return decision;
    }

    decision.reason = HasMotionEvidence(object)
        ? "motion_evidence_keep" : "outside_static_zone_keep";
    return decision;
}
