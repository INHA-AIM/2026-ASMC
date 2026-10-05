#include <Parser/GpsSanityFilter.hpp>

#include <algorithm>
#include <cmath>

GpsSanityFilter::GpsSanityFilter(const GpsSanityConfig& config)
    : config_(config) {}

GpsSanityDecision GpsSanityFilter::Evaluate(
    double map_x, double map_y, double source_stamp_s) {
    GpsSanityDecision decision;
    if (!std::isfinite(map_x) || !std::isfinite(map_y) ||
        !std::isfinite(source_stamp_s) || source_stamp_s <= 0.0) {
        decision.reason = "non-finite coordinate or stamp";
        return decision;
    }
    if (map_x < config_.min_x || map_x > config_.max_x ||
        map_y < config_.min_y || map_y > config_.max_y) {
        decision.reason = "outside configured local-map bounds";
        return decision;
    }
    if (!has_accepted_) {
        has_accepted_ = true;
        accepted_x_ = map_x;
        accepted_y_ = map_y;
        accepted_stamp_s_ = source_stamp_s;
        decision.accepted = true;
        decision.reason = "initial valid sample";
        return decision;
    }

    const double elapsed_s = source_stamp_s - accepted_stamp_s_;
    decision.jump_m = std::hypot(
        map_x - accepted_x_, map_y - accepted_y_);
    if (elapsed_s < -1e-3) {
        decision.reason = "source timestamp moved backwards";
        return decision;
    }
    if (elapsed_s >= config_.reanchor_after_s) {
        accepted_x_ = map_x;
        accepted_y_ = map_y;
        accepted_stamp_s_ = source_stamp_s;
        decision.accepted = true;
        decision.reason = "reanchored after input gap";
        return decision;
    }
    decision.allowed_jump_m = config_.base_jump_allowance_m +
        config_.maximum_speed_mps * std::max(0.0, elapsed_s);
    if (decision.jump_m > decision.allowed_jump_m) {
        decision.reason = "implausible position jump";
        return decision;
    }

    accepted_x_ = map_x;
    accepted_y_ = map_y;
    accepted_stamp_s_ = source_stamp_s;
    decision.accepted = true;
    decision.reason = "continuous sample";
    return decision;
}

void GpsSanityFilter::Reset() {
    has_accepted_ = false;
    accepted_x_ = 0.0;
    accepted_y_ = 0.0;
    accepted_stamp_s_ = 0.0;
}
