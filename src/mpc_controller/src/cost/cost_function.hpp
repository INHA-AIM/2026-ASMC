#ifndef MPC_COST_FUNCTION_HPP
#define MPC_COST_FUNCTION_HPP

#include <cstddef>
#include <vector>

#include "common/mpc_control.hpp"
#include "common/mpc_params.hpp"
#include "common/mpc_state.hpp"
#include "common/reference_path.hpp"


// Reference tangent/normal frame tracking error.  This is pure geometry: it
// owns no ROS or solver state and may be used independently by tests and
// diagnostics.
struct TrackingError {
    double contour = 0.0;  // left-normal error: [-sin(psi), cos(psi)] dot dp
    double lag = 0.0;      // tangent error: [cos(psi), sin(psi)] dot dp
};

TrackingError computeTrackingError(const MPCState& state,
                                   const ReferencePath& ref,
                                   size_t ref_idx);

double computeContourErrorCost(const TrackingError& error, double weight);
double computeLagErrorCost(const TrackingError& error, double weight);

double computeHeadingErrorCost(const MPCState& state, const ReferencePath& ref,
                               size_t ref_idx, double weight);

double computeSpeedErrorCost(const MPCState& state, double v_ref, double weight);

// Control effort는 reference-relative residual에 벌점을 준다:
//   steering:     delta - delta_ff
//   acceleration: accel - accel_ref_used
// computeTotalCost/computeCostBreakdown이 raw ReferencePath.a_ref를 actuator
// 범위로 clip한 값을 accel_ref_used로 전달한다.
// 직선/등속 reference(delta_ff=0, accel_ref=0)에서는 기존 delta^2/accel^2
// semantics와 정확히 같다.
double computeControlEffortCost(const MPCControl& u, double weight_steer,
                                double weight_accel, double delta_ff = 0.0,
                                double accel_ref = 0.0);

double computeControlRateCost(const MPCControl& u_prev, const MPCControl& u_cur,
                              double weight_steer, double weight_accel);

// 전체 trajectory 의 cost
double computeTotalCost(
    const std::vector<MPCState>&   states,
    const std::vector<MPCControl>& controls,
    const ReferencePath&           ref,
    const MPCControl&              prev_control,  // 이전 사이클 마지막 명령 (rate cost용)
    const MPCParams&               params);

// computeTotalCost와 동일한 계산을 항목별로 분리해서 반환 (진단용).
// 급브레이크/급조향 사이클에서 어느 항이 지배적인지 확인하는 데 쓴다.
struct CostBreakdown {
    double contour = 0.0;
    double lag = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    // control/control_rate 총합 안에 포함된 종방향 부분. total()에는 다시
    // 더하지 않으며 runtime 진단에서 acceleration reference 추종과 rate
    // smoothing의 상대 크기를 분리해 보기 위한 값이다.
    double acceleration_control = 0.0;
    double acceleration_rate = 0.0;
    double control = 0.0;
    double control_rate = 0.0;
    double terminal = 0.0;
    double total() const {
        return contour + lag + heading + speed + control + control_rate +
               terminal;
    }
};

CostBreakdown computeCostBreakdown(
    const std::vector<MPCState>&   states,
    const std::vector<MPCControl>& controls,
    const ReferencePath&           ref,
    const MPCControl&              prev_control,
    const MPCParams&               params);

#endif // MPC_COST_FUNCTION_HPP
