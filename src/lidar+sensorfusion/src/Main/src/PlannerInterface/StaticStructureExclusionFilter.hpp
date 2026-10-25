#pragma once

#include <string>
#include <vector>

struct PlannerObject;

struct StaticStructureExclusionDecision {
    bool keep = true;
    bool filter_available = false;
    bool policy_applied = false;
    std::string reason = "filter_unavailable_keep";
    std::string zone_name;
};

class StaticStructureExclusionFilter {
public:
    bool Configure(const std::string& config_path,
                   double minimum_height_m,
                   double minimum_bottom_z_m);

    StaticStructureExclusionDecision Evaluate(
        const PlannerObject& object) const;

    bool ready() const { return ready_; }
    const std::string& status_message() const { return status_message_; }
    std::size_t zone_count() const { return zones_.size(); }

private:
    struct Zone {
        std::string name;
        double min_x = 0.0;
        double max_x = 0.0;
        double min_y = 0.0;
        double max_y = 0.0;
        bool require_infrastructure_shape = true;
        bool require_elevated_pole_shape = false;
    };

    void Reset(const std::string& message);

    bool ready_ = false;
    std::string status_message_ = "not configured";
    double minimum_height_m_ = 2.5;
    double minimum_bottom_z_m_ = 2.0;
    std::vector<Zone> zones_;
};
