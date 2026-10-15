#include "Frenet/Maneuver/KeepCruise.hpp"

#include <iostream>
#include <limits>

int main() {
    int failures = 0;
    auto expect = [&](bool condition, const char* name) {
        std::cout << (condition ? "[PASS] " : "[FAIL] ") << name << '\n';
        if (!condition) ++failures;
    };
    const KeepCruiseConfig profile{true, 40.0 / 3.6, 3.8, 4.0, {"eligible_road"}};
    KinematicLimits base{};
    base.max_lateral_accel = 2.6;
    base.max_longitudinal_accel = 3.0;
    base.max_longitudinal_jerk = 6.0;
    base.max_curvature = 0.23;
    CostWeights costs{0.1, 1.0, 20.0, 35.0, 1.0, 1.0, 1.0, 1.0};
    PlannerCommand command;
    command.source_link_id = "eligible_road";
    command.has_target_speed = true;
    command.target_speed = 58.0 / 3.6;
    const auto tuned = ResolveKeepCruiseLimits(command, profile, base);
    expect(tuned.max_lateral_accel == 3.8 &&
               tuned.max_longitudinal_accel == base.max_longitudinal_accel &&
               tuned.max_longitudinal_jerk == base.max_longitudinal_jerk &&
               tuned.max_curvature == base.max_curvature,
           "high-speed KEEP changes lateral allowance only");
    expect(ResolveKeepCruiseCosts(command, profile, costs).ks_dot == 4.0,
           "high-speed KEEP uses configured speed tracking cost");
    for (auto mode : {STOP, FOLLOWING, AVOID, EMERGENCY,
                      LANE_CHANGE_LEFT, LANE_CHANGE_RIGHT, MERGE}) {
        command.mode = mode;
        expect(ResolveKeepCruiseLimits(command, profile, base).max_lateral_accel == 2.6 &&
                   ResolveKeepCruiseCosts(command, profile, costs).ks_dot == 1.0,
               "special command retains base limits/costs, including STOP approach");
    }
    command.mode = LANE_KEEPING;
    command.target_speed = 25.0 / 3.6;
    expect(!UsesKeepCruiseTuning(command, profile), "signal/obstacle low-speed KEEP excluded");
    command.target_speed = profile.min_requested_speed;
    expect(UsesKeepCruiseTuning(command, profile), "threshold is inclusive");
    command.has_max_speed = true;
    command.max_speed = 25.0 / 3.6;
    expect(!UsesKeepCruiseTuning(command, profile), "lower explicit cap excludes tuning");
    command.max_speed = std::numeric_limits<double>::quiet_NaN();
    expect(!UsesKeepCruiseTuning(command, profile), "invalid explicit cap excludes tuning");
    command.has_max_speed = false;
    command.has_target_speed = false;
    expect(!UsesKeepCruiseTuning(command, profile), "unspecified target excludes tuning");
    command.has_target_speed = true;
    command.merge_type = MergeType::ROUNDABOUT;
    expect(!UsesKeepCruiseTuning(command, profile), "merge-tagged KEEP excluded");
    command.merge_type = MergeType::NONE;
    command.source_link_id = "unverified_road";
    expect(!UsesKeepCruiseTuning(command, profile), "unlisted road retains base tuning");
    command.source_link_id.clear();
    expect(!UsesKeepCruiseTuning(command, profile), "missing road metadata retains base tuning");
    command.source_link_id = "eligible_road";
    auto disabled = profile;
    disabled.enabled = false;
    expect(!UsesKeepCruiseTuning(command, disabled), "disabled profile restores base tuning");
    expect(base.max_lateral_accel == 2.6 && costs.ks_dot == 1.0,
           "profile selection leaves authoritative base configuration unchanged");
    return failures == 0 ? 0 : 1;
}
