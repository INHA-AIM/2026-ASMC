#ifndef MPC_COST_FUNCTION_HPP
#define MPC_COST_FUNCTION_HPP

#include <cstddef>
#include <vector>

#include "common/mpc_control.hpp"
#include "common/mpc_params.hpp"
#include "common/mpc_state.hpp"
#include "common/reference_path.hpp"


// 단일 스텝 비용 (참조 인덱스를 외부에서 결정하여 캐싱)
double computePathErrorCost(const MPCState& state, const ReferencePath& ref,
                            size_t ref_idx, double weight);

double computeHeadingErrorCost(const MPCState& state, const ReferencePath& ref,
                               size_t ref_idx, double weight);

double computeSpeedErrorCost(const MPCState& state, double v_ref, double weight);

// steer effort는 절대 조향각이 아니라 curvature feed-forward(delta_ff)
// 대비 잔차 (delta - delta_ff)에 벌점을 준다 - 직선(kappa=0)에서는
// delta_ff=0이라 기존과 동일하고, 코너에서는 필요한 조향 자체를 벌하지
// 않는다 (fix/yuntae-planner Commit C). delta_ff=0.0이면 기존 semantics와
// 완전히 같다.
double computeControlEffortCost(const MPCControl& u, double weight_steer,
                                 double weight_accel, double delta_ff = 0.0);

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
    double path = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    double control = 0.0;
    double control_rate = 0.0;
    double terminal = 0.0;
    double total() const { return path + heading + speed + control + control_rate + terminal; }
};

CostBreakdown computeCostBreakdown(
    const std::vector<MPCState>&   states,
    const std::vector<MPCControl>& controls,
    const ReferencePath&           ref,
    const MPCControl&              prev_control,
    const MPCParams&               params);

#endif // MPC_COST_FUNCTION_HPP
