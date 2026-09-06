#include "reference/path_resampler.hpp"

#include <algorithm>
#include <cstddef>

#include "global/utils.hpp"

ReferencePath ToReferencePath(const PlannedPath& path, double destination_dt,
                              int steps) {
    ReferencePath reference;
    const std::size_t size = path.size();
    if (size == 0 || path.dt <= 0.0 || destination_dt <= 0.0 || steps <= 0) {
        return reference;
    }

    reference.x_ref.reserve(steps);
    reference.y_ref.reserve(steps);
    reference.yaw_ref.reserve(steps);
    reference.v_ref.reserve(steps);
    reference.k_ref.reserve(steps);

    for (int i = 0; i < steps; ++i) {
        const double time = i * destination_dt;
        double index = time / path.dt;
        index = clip(index, 0.0, static_cast<double>(size - 1));
        const std::size_t first = static_cast<std::size_t>(index);
        const std::size_t second = std::min(first + 1, size - 1);
        const double fraction = index - static_cast<double>(first);

        reference.x_ref.push_back(
            path.x[first] + fraction * (path.x[second] - path.x[first]));
        reference.y_ref.push_back(
            path.y[first] + fraction * (path.y[second] - path.y[first]));
        reference.yaw_ref.push_back(
            path.yaw[first] +
            fraction * angleDiff(path.yaw[second], path.yaw[first]));
        reference.v_ref.push_back(
            path.v[first] + fraction * (path.v[second] - path.v[first]));
        reference.k_ref.push_back(
            path.kappa[first] +
            fraction * (path.kappa[second] - path.kappa[first]));
    }
    return reference;
}
