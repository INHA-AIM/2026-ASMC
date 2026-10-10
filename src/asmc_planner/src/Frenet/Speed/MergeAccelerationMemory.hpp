#pragma once

#include <cmath>
#include <vector>

// Only normal, accepted traversal profiles may advance this feedforward state.
// Fresh measured speed/pose remain authoritative; braking never uses this seed.
struct MergeAccelerationMemory {
    std::vector<double> acceleration;
    double stamp = -1;
    double dt = 0;

    void Reset() { acceleration.clear(); stamp = -1; dt = 0; }
    double Seed(double measured, double now) const {
        const double elapsed = now - stamp;
        if (measured >= 0 || acceleration.size() < 2 || !std::isfinite(elapsed) ||
            elapsed <= 0 || elapsed > .5 || dt <= 0 || !std::isfinite(dt)) return measured;
        const double index = elapsed / dt;
        if (index >= acceleration.size() - 1) return measured;
        const auto lo = static_cast<std::size_t>(index);
        const double prior = acceleration[lo] +
            (acceleration[lo + 1] - acceleration[lo]) * (index - lo);
        return std::isfinite(prior) ? prior : measured;
    }
    void Accept(const std::vector<double>& profile, double now, double interval) {
        acceleration = profile; stamp = now; dt = interval;
    }
};
