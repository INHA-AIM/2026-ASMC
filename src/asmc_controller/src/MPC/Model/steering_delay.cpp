#include "MPC/Model/steering_delay.hpp"

#include <algorithm>
#include <cmath>

std::vector<MPCControl> ApplySteeringDelay(
    const std::vector<MPCControl>& commands,
    const std::vector<double>& history, double fallback_steering,
    double delay_s, double dt) {
    auto applied = commands;
    if (delay_s <= 0.0 || dt <= 0.0 || commands.empty()) return applied;
    const double delay_steps = delay_s / dt;
    const auto steering_at = [&](int index) {
        if (index >= 0) return commands[std::min(
            static_cast<std::size_t>(index), commands.size() - 1)].delta;
        if (history.empty()) return fallback_steering;
        const int past = std::max(0, static_cast<int>(history.size()) + index);
        return history[static_cast<std::size_t>(past)];
    };
    for (std::size_t i = 0; i < applied.size(); ++i) {
        const double index = static_cast<double>(i) - delay_steps;
        const int first = static_cast<int>(std::floor(index));
        const double fraction = index - first;
        applied[i].delta = steering_at(first) * (1.0 - fraction)
                         + steering_at(first + 1) * fraction;
    }
    return applied;
}
