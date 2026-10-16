#include "MPC/Cost/cost_function.hpp"

#include <cmath>
#include <algorithm>

#include "MPC/Math/utils.hpp"
#include "MPC/Model/vehicle_model.hpp"

double ContourTrackingWeight(double reference_curvature, const MPCParams& p) {
    if (!std::isfinite(reference_curvature) ||
        !std::isfinite(p.curve_contour_boost) || p.curve_contour_boost <= 0.0 ||
        !std::isfinite(p.contour_boost_start_curvature) ||
        !std::isfinite(p.contour_boost_full_curvature) ||
        p.contour_boost_start_curvature < 0.0 ||
        p.contour_boost_full_curvature <= p.contour_boost_start_curvature) {
        return p.weight_contour_error;
    }
    const double blend = std::clamp(
        (std::fabs(reference_curvature) - p.contour_boost_start_curvature) /
        (p.contour_boost_full_curvature - p.contour_boost_start_curvature),
        0.0, 1.0);
    return p.weight_contour_error + p.curve_contour_boost * blend;
}

// ========================================
// Reference-frame tracking geometry.
//
//   tangent = [ cos(psi_r), sin(psi_r)]
//   normal  = [-sin(psi_r), cos(psi_r)]  (positive to reference left)
//
// lag is the position error along the tangent and contour is the position
// error along the left normal.  Keeping this transform separate matters only
// when the two components receive different weights: with equal weights the
// rotation preserves dx^2 + dy^2 exactly.
// ========================================
TrackingError computeTrackingError(const MPCState& state,
                                   const ReferencePath& ref,
                                   size_t ref_idx)
{
    TrackingError error;
    if (ref.empty()) return error;
    if (ref_idx >= ref.size()) ref_idx = ref.size() - 1;
    const double dx = state.x - ref.x_ref[ref_idx];
    const double dy = state.y - ref.y_ref[ref_idx];
    const double cos_yaw = std::cos(ref.yaw_ref[ref_idx]);
    const double sin_yaw = std::sin(ref.yaw_ref[ref_idx]);
    error.lag = cos_yaw * dx + sin_yaw * dy;
    error.contour = -sin_yaw * dx + cos_yaw * dy;
    return error;
}

double computeContourErrorCost(const TrackingError& error, double weight)
{
    return weight * error.contour * error.contour;
}

double computeLagErrorCost(const TrackingError& error, double weight)
{
    return weight * error.lag * error.lag;
}

// ========================================
// 헤딩 오차
// ========================================
double computeHeadingErrorCost(const MPCState& state, const ReferencePath& ref,
                               size_t ref_idx, double weight)
{
    if (ref.empty()) return 0.0;
    if (ref_idx >= ref.size()) ref_idx = ref.size() - 1;
    double e = angleDiff(state.yaw, ref.yaw_ref[ref_idx]);
    return weight * e * e;
}

// ========================================
// 속도 오차
// ========================================
double computeSpeedErrorCost(const MPCState& state, double v_ref, double weight)
{
    double e = state.vx - v_ref;
    return weight * e * e;
}

// ========================================
// 제어 입력 크기
//   steer/accel 을 분리해서 weight 적용.
//   steer는 delta_ff(curvature feed-forward), acceleration은 Frenet의
//   accel_ref 대비 잔차를 벌한다. 두 reference가 0이면 기존 절대 입력
//   제곱 semantics와 동일하다.
// ========================================
double computeControlEffortCost(const MPCControl& u, double w_steer,
                                double w_accel, double delta_ff,
                                double accel_ref)
{
    const double steer_residual = u.delta - delta_ff;
    const double accel_residual = u.accel - accel_ref;
    return w_steer * steer_residual * steer_residual +
           w_accel * accel_residual * accel_residual;
}

// ========================================
// 제어 변화율
//   delta(rad)와 accel(m/s²)은 단위가 달라서 weight 분리
//   같은 weight로 묶으면 gradient 왜곡 → 흔들림/급정거 원인
// ========================================
double computeControlRateCost(const MPCControl& u_prev, const MPCControl& u_cur,
                              double weight_steer, double weight_accel)
{
    double dd = u_cur.delta - u_prev.delta;
    double da = u_cur.accel - u_prev.accel;
    // steer와 acceleration 변화율은 단위와 튜닝 목적이 달라 독립 가중치를 쓴다.
    return weight_steer * dd*dd + weight_accel * da*da;
}

// ========================================
// trajectory 전체 cost
//
// dt 정규화:
//   controller.yaml의 모든 stage 가중치는 kTunedDt(=0.1, 최초 튜닝 시점의 dt)를
//   기준으로 맞춰져 있다. horizon/dt를 바꿔도 물리적으로 같은 궤적이면 cost 균형이
//   유지되도록, "적분류" 항(contour/lag/heading/speed/effort)은 dt에 비례, "미분류" 항
//   (control_rate, Δu/Δt의 이산 근사)은 dt에 반비례로 스케일한다.
//   (dt=kTunedDt인 경우 두 스케일 모두 1.0이라 기존 튜닝 값과 완전히 동일하게 동작)
// ========================================
namespace {
constexpr double kTunedDt = 0.1;
}

CostBreakdown computeCostBreakdown(
    const std::vector<MPCState>&   states,
    const std::vector<MPCControl>& controls,
    const ReferencePath&           ref,
    const MPCControl&              prev_control,
    const MPCParams&               params)
{
    CostBreakdown bd;
    if (states.empty()) return bd;

    size_t N = controls.size();
    size_t S = states.size();
    size_t R = ref.size();

    const double dt_scale   = params.dt / kTunedDt;   // 적분류 stage cost
    const double rate_scale = kTunedDt / params.dt;   // 미분류 rate cost

    // ref는 main.cpp에서 MPC dt로 시간 정렬되며 ref[i]는 i*dt 시점의 목표다.

    // Stage costs (i = 0 .. N-1)
    for (size_t i = 0; i < N; ++i) {
        const MPCState& st = states[i];
        const size_t ref_idx = (R > 0) ? std::min(i, R - 1) : 0;

        // R==0 분기는 solveMPC가 ref.empty()를 미리 걸러서 실질적으로 도달하지 않음.
        double v_target = (R > 0) ? ref.v_ref[ref_idx] : 0.0;
        const TrackingError tracking =
            computeTrackingError(st, ref, ref_idx);
        bd.contour += dt_scale * computeContourErrorCost(
            tracking, ContourTrackingWeight(
                (R > 0) ? ref.k_ref[ref_idx] : 0.0, params));
        bd.lag += dt_scale * computeLagErrorCost(
            tracking, params.weight_lag_error);
        bd.heading += dt_scale * computeHeadingErrorCost(st, ref, ref_idx, params.weight_heading_error);
        bd.speed   += dt_scale * computeSpeedErrorCost  (st, v_target,     params.weight_speed_error);

        const MPCControl& u_cur = controls[i];
        // steer effort: weight_control / accel effort: weight_control * 0.5
        // accel은 속도 추종을 위해 어느 정도 자유롭게 두어야 급정거 방지
        //
        // steer effort는 절대 delta가 아니라 이 stage의 참조 curvature가
        // 요구하는 delta_ff 대비 잔차를 벌한다 - 코너에서 필요한 조향
        // 자체를 벌하지 않기 위함(Commit C). 직선(k_ref=0)에서는
        // delta_ff=0이라 기존 delta^2와 동일하게 동작한다.
        const double k_ref_i = (R > 0) ? ref.k_ref[ref_idx] : 0.0;
        const double raw_a_ref_i =
            (R > 0 && ref.a_ref.size() > ref_idx) ? ref.a_ref[ref_idx] : 0.0;
        const double a_ref_i = clip(
            raw_a_ref_i, params.accel_min, params.accel_max);
        const double delta_ff = SteeringFeedforward(
            k_ref_i, params.wheelbase, params.steering_max,
            params.steering_curvature_gain);
        const double accel_effort_weight = params.weight_control * 0.5;
        bd.control += dt_scale * computeControlEffortCost(
            u_cur, params.weight_control, accel_effort_weight, delta_ff,
            a_ref_i);
        const double accel_residual = u_cur.accel - a_ref_i;
        bd.acceleration_control += dt_scale * accel_effort_weight *
            accel_residual * accel_residual;

        const MPCControl& u_prev = (i == 0) ? prev_control : controls[i-1];
        bd.control_rate += rate_scale * computeControlRateCost(u_prev, u_cur,
                                          params.weight_control_rate,
                                          params.weight_control_rate_accel);
        const double accel_delta = u_cur.accel - u_prev.accel;
        bd.acceleration_rate += rate_scale *
            params.weight_control_rate_accel * accel_delta * accel_delta;
    }

    // Terminal cost (마지막 stage와 동일한 시간 인덱스: N-1).
    // Preserve the stage contour/lag ratio instead of reintroducing a 1:1
    // Euclidean position penalty at the end of the horizon.  The historical
    // terminal weight remains the contour coefficient; heading and speed keep
    // their existing 0.5/0.2 multipliers.
    if (S > 0 && R > 0) {
        const MPCState& st = states.back();
        const size_t ref_idx = std::min(N > 0 ? N - 1 : 0, R - 1);

        double v_target = ref.v_ref[ref_idx];
        const TrackingError tracking =
            computeTrackingError(st, ref, ref_idx);
        const double lag_ratio = params.weight_contour_error > 0.0
            ? params.weight_lag_error / params.weight_contour_error
            : 0.0;
        bd.terminal += computeContourErrorCost(
            tracking, params.weight_terminal);
        bd.terminal += computeLagErrorCost(
            tracking, params.weight_terminal * lag_ratio);
        bd.terminal += computeHeadingErrorCost(st, ref, ref_idx, params.weight_terminal * 0.5);
        bd.terminal += computeSpeedErrorCost  (st, v_target,     params.weight_terminal * 0.2);
    }

    return bd;
}

double computeTotalCost(
    const std::vector<MPCState>&   states,
    const std::vector<MPCControl>& controls,
    const ReferencePath&           ref,
    const MPCControl&              prev_control,
    const MPCParams&               params)
{
    return computeCostBreakdown(states, controls, ref, prev_control, params).total();
}
