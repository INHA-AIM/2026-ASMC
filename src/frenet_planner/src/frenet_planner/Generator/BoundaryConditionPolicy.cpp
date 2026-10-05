#include "frenet_planner/Generator/BoundaryConditionPolicy.hpp"

#include "frenet_planner/Math/Utils.hpp"

#include <cmath>

FrenetState ClampInitialBoundaryState(double s, double s_dot, double raw_s_ddot,
                                       double d, double raw_d_dot,
                                       double raw_d_ddot,
                                       const KinematicLimits& limits) {
    // 투영 오차가 후보의 초기 가속도 경계조건을 비현실적으로 키우지 않도록
    // 다항식 중간구간의 overshoot 여유를 포함해 한계의 일정 비율로 제한한다.
    const double boundary_margin = limits.boundary_condition_accel_fraction;
    const double s_ddot = clip(raw_s_ddot,
                                -limits.max_longitudinal_accel * boundary_margin,
                                limits.max_longitudinal_accel * boundary_margin);
    const double d_ddot = clip(raw_d_ddot,
                                -limits.max_lateral_accel * boundary_margin,
                                limits.max_lateral_accel * boundary_margin);

    // 큰 heading 오차로 과대 추정될 수 있는 초기 횡속도도 제한한다.
    const double d_dot = clip(raw_d_dot, -limits.max_initial_d_dot,
                               limits.max_initial_d_dot);

    return FrenetState{s, s_dot, s_ddot, d, d_dot, d_ddot};
}

double ResolveVelocityKeepingBoundaryAcceleration(double raw_s_ddot,
                                                   double current_s_dot,
                                                   double resolved_target_speed,
                                                   double deadband) {
    const bool accelerating_intent =
        resolved_target_speed > current_s_dot + deadband;
    if (accelerating_intent && raw_s_ddot < 0.0) {
        return 0.0;
    }
    const bool decelerating_intent =
        resolved_target_speed < current_s_dot - deadband;
    if (decelerating_intent && raw_s_ddot > 0.0) {
        return 0.0;
    }
    return raw_s_ddot;
}

double ResolveEnvelopeBoundaryAcceleration(
    double raw_s_ddot, double current_s_dot, double raw_speed_cap,
    const std::vector<SpeedEnvelopePoint>& envelope, double deadband) {
    if (envelope.empty()) return raw_s_ddot;
    if (envelope.size() > 1 && raw_s_ddot < 0.0 &&
        envelope[1].speed_squared < envelope[0].speed_squared) {
        return raw_s_ddot;
    }
    const double spatial_cap = std::sqrt(std::max(0.0, envelope.front().speed_squared));
    const double intent_cap = spatial_cap < current_s_dot - deadband
        ? spatial_cap : raw_speed_cap;
    return ResolveVelocityKeepingBoundaryAcceleration(raw_s_ddot, current_s_dot,
        intent_cap, deadband);
}
