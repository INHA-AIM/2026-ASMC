#include "MPC/Reference/braking_reference.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include "MPC/Math/utils.hpp"
#include "MPC/Reference/reference_validator.hpp"

double BrakePedalDeceleration(double pedal, const MPCParams& params) {
    pedal = std::clamp(pedal, 0.0, 1.0);
    const auto& x = params.brake_pedal_lut;
    const auto& y = params.brake_response_lut;
    if (!params.use_calibrated_pedal_mapping || x.size() < 2 || y.size() != x.size())
        return pedal * params.brake_full_decel;
    std::size_t i = 1;
    while (i + 1 < x.size() && pedal > x[i]) ++i;
    return std::max(0.0, y[i-1] + (pedal-x[i-1]) *
        (y[i]-y[i-1]) / (x[i]-x[i-1]));
}

ReferencePath MakeBrakingReference(const ReferencePath& geometry,
    const MPCState& ego, double deceleration, const MPCParams& params) {
    ReferencePath out;
    if (geometry.size() < 2 || !ValidateReference(geometry, params).ok() ||
        !std::isfinite(deceleration) || deceleration <= 0.0 ||
        !std::isfinite(ego.x) || !std::isfinite(ego.y) ||
        !std::isfinite(ego.vx) || ego.vx < 0.0) return out;
    std::vector<double> arc(geometry.size(), 0.0);
    double best_distance = std::numeric_limits<double>::infinity();
    double start = 0.0;
    for (std::size_t i = 1; i < geometry.size(); ++i) {
        const double dx = geometry.x_ref[i] - geometry.x_ref[i-1];
        const double dy = geometry.y_ref[i] - geometry.y_ref[i-1];
        const double length = std::hypot(dx, dy);
        arc[i] = arc[i-1] + length;
        if (length < 1e-6) continue;
        const double fraction = std::clamp(((ego.x-geometry.x_ref[i-1])*dx +
            (ego.y-geometry.y_ref[i-1])*dy)/(length*length), 0.0, 1.0);
        const double distance = std::hypot(ego.x-geometry.x_ref[i-1]-fraction*dx,
            ego.y-geometry.y_ref[i-1]-fraction*dy);
        if (distance < best_distance) {
            best_distance = distance;
            start = arc[i-1] + fraction*length;
        }
    }
    if (arc.back() < 1e-6) return out;
    // When braking has passed the last sample, project onto its continuation.
    // No new road geometry is asserted: this is a terminal-curvature fallback.
    const double yaw_end = geometry.yaw_ref.back();
    const double k_end = geometry.k_ref.back();
    const double ex = ego.x-geometry.x_ref.back();
    const double ey = ego.y-geometry.y_ref.back();
    if (start >= arc.back()-1e-6) {
        const double along = ex*std::cos(yaw_end)+ey*std::sin(yaw_end);
        const double across = -ex*std::sin(yaw_end)+ey*std::cos(yaw_end);
        const double extension = std::abs(k_end)<1e-6 ? along :
            std::atan2(k_end*along, 1.0-k_end*across)/k_end;
        start += std::max(0.0, extension);
    }
    double travel = 0.0;
    double speed = ego.vx;
    for (int tick = 0; tick < params.horizon; ++tick) {
        const double s = start + travel;
        double x, y, yaw, curvature;
        if (s >= arc.back()) {
            const double d = s-arc.back();
            yaw = yaw_end+k_end*d;
            x = geometry.x_ref.back() + (std::abs(k_end)<1e-6 ?
                d*std::cos(yaw_end) : (std::sin(yaw)-std::sin(yaw_end))/k_end);
            y = geometry.y_ref.back() + (std::abs(k_end)<1e-6 ?
                d*std::sin(yaw_end) : (std::cos(yaw_end)-std::cos(yaw))/k_end);
            curvature = k_end;
        } else {
            const auto upper = std::upper_bound(arc.begin(), arc.end(), s);
            const auto i = static_cast<std::size_t>(upper-arc.begin());
            const double f = (s-arc[i-1])/(arc[i]-arc[i-1]);
            x = geometry.x_ref[i-1]+f*(geometry.x_ref[i]-geometry.x_ref[i-1]);
            y = geometry.y_ref[i-1]+f*(geometry.y_ref[i]-geometry.y_ref[i-1]);
            yaw = geometry.yaw_ref[i-1]+f*normalizeAngle(geometry.yaw_ref[i]-geometry.yaw_ref[i-1]);
            curvature = geometry.k_ref[i-1]+f*(geometry.k_ref[i]-geometry.k_ref[i-1]);
        }
        out.x_ref.push_back(x); out.y_ref.push_back(y);
        out.yaw_ref.push_back(normalizeAngle(yaw)); out.k_ref.push_back(curvature);
        out.v_ref.push_back(speed); out.a_ref.push_back(-deceleration);
        // Match the solver's forward Euler distance/speed convention.
        travel += speed*params.dt;
        speed = std::max(0.0, speed-deceleration*params.dt);
    }
    return out;
}
