#include "MPC/Reference/path_resampler.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "MPC/Math/utils.hpp"

ReferencePath ToReferencePath(const PlannedPath& path, double destination_dt,
                              int steps, double start_time_offset) {
    ReferencePath reference;
    const std::size_t size = path.size();
    if (size == 0 || path.dt <= 0.0 || destination_dt <= 0.0 || steps <= 0 ||
        !std::isfinite(start_time_offset) || path.y.size() != size ||
        path.yaw.size() != size || path.v.size() != size ||
        path.kappa.size() != size) {
        return reference;
    }
    const double offset = std::max(0.0, start_time_offset);

    reference.x_ref.reserve(steps);
    reference.y_ref.reserve(steps);
    reference.yaw_ref.reserve(steps);
    reference.v_ref.reserve(steps);
    reference.a_ref.reserve(steps);
    reference.k_ref.reserve(steps);

    for (int i = 0; i < steps; ++i) {
        const double time = offset + i * destination_dt;
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

    // MPC acceleration feed-forward is defined in the same control space as
    // the longitudinal model v[k+1] = v[k] + accel[k] * destination_dt.
    // Therefore it is the time derivative of the FINAL, offset-applied and
    // destination-grid-resampled v_ref above.  It is deliberately not the
    // Frenet CartesianPath.a field: that diagnostic Cartesian acceleration
    // also contains lateral/reference-curvature terms and is not an actuator
    // acceleration reference.  path.dt (Frenet source sample period) and the
    // Frenet replanning period are likewise not the derivative interval here.
    reference.a_ref.assign(reference.v_ref.size(), 0.0);
    const std::size_t reference_size = reference.v_ref.size();
    if (reference_size >= 2) {
        reference.a_ref.front() =
            (reference.v_ref[1] - reference.v_ref[0]) / destination_dt;
        for (std::size_t i = 1; i + 1 < reference_size; ++i) {
            reference.a_ref[i] =
                (reference.v_ref[i + 1] - reference.v_ref[i - 1]) /
                (2.0 * destination_dt);
        }
        reference.a_ref.back() =
            (reference.v_ref[reference_size - 1] -
             reference.v_ref[reference_size - 2]) /
            destination_dt;
    }
    return reference;
}
