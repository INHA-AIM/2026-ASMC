#pragma once

#include <string>

struct GpsSanityConfig {
    double min_x = -454.5159;
    double min_y = -825.0458;
    double max_x = 162.4841;
    double max_y = 621.3542;
    double base_jump_allowance_m = 3.0;
    double maximum_speed_mps = 80.0;
    double reanchor_after_s = 2.0;
};

struct GpsSanityDecision {
    bool accepted = false;
    std::string reason;
    double jump_m = 0.0;
    double allowed_jump_m = 0.0;
};

// Rejects malformed/out-of-map samples and impossible source-timestamp jumps.
// A sustained new pose is accepted after reanchor_after_s so simulator
// respawns do not require restarting Main.
class GpsSanityFilter {
public:
    explicit GpsSanityFilter(const GpsSanityConfig& config = {});

    GpsSanityDecision Evaluate(double map_x, double map_y,
                               double source_stamp_s);
    void Reset();

private:
    GpsSanityConfig config_;
    bool has_accepted_ = false;
    double accepted_x_ = 0.0;
    double accepted_y_ = 0.0;
    double accepted_stamp_s_ = 0.0;
};
