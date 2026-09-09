#ifndef MPC_SOLVER_HPP
#define MPC_SOLVER_HPP

#include <vector>

#include "common/mpc_control.hpp"
#include "common/mpc_params.hpp"
#include "common/mpc_result.hpp"
#include "common/mpc_state.hpp"
#include "common/reference_path.hpp"

/**
 * @brief MPC 최적화 솔버
 *   - warm-start (이전 해의 한 스텝 shift 사용, curvature-aware residual
 *     rebase 포함)
 *   - central-difference gradient
 *   - Armijo backtracking line search
 *   - constraint projection (box clip)
 *
 * @param ego_state     현재 상태 (global frame)
 * @param ref           reference path (현재 위치 주변 window)
 * @param prev_control  직전 사이클에 발행한 제어 (rate cost 의 시작점)
 * @param warm_start    이전 사이클의 최적 control sequence (없으면 비움)
 * @param warm_start_steering_feedforward
 *                      warm_start와 같은 solve가 만든 stage별 steering
 *                      feed-forward (SteeringFeedforward(이전 ref.k_ref[i],
 *                      ...)). warm_start와 크기가 다르면(또는 비어 있으면)
 *                      residual rebase를 건너뛰고 기존 fallback을 쓴다 -
 *                      반드시 warm_start를 만든 것과 같은 solve의 값이어야
 *                      한다(다른 cycle의 값과 섞으면 안 된다).
 * @param params        MPC 파라미터
 * @return MPC 결과. result.steering_feedforward는 "이번" solve의 stage별
 *         feed-forward이며, 다음 호출의 warm_start_steering_feedforward로
 *         그대로 전달되어야 한다.
 */
MPCResult solveMPC(
    const MPCState&                ego_state,
    const ReferencePath&           ref,
    const MPCControl&              prev_control,
    const std::vector<MPCControl>& warm_start,
    const std::vector<double>&     warm_start_steering_feedforward,
    const MPCParams&               params);

// 현재 reference로 stage별(SteeringFeedforward) curvature feed-forward를
// 계산한다. solveMPC 내부에서 쓰는 것과 같은 함수 - 단위 테스트가 이 값을
// 재현할 때 공식을 복붙하지 않도록 헤더에 노출한다. ROS 비의존 순수 함수.
std::vector<double> computeCurrentSteeringFeedforward(
    const ReferencePath& ref, int horizon, const MPCParams& params);

// curvature-aware steering warm-start rebase(solveMPC 주석/mpc_solver.cpp
// 정의 참고). 호출자는 먼저 WarmStartFeedforwardIsValid로 전제를 확인해야
// 한다 - 이 함수 자체는 방어적 검사를 하지 않는다(호출자가 이미 검증했다는
// 계약).
std::vector<double> RebaseSteeringWarmStart(
    const std::vector<MPCControl>& warm_start,
    const std::vector<double>& warm_start_ff,
    const std::vector<double>& current_ff,
    int horizon);

// warm_start/warm_start_ff가 RebaseSteeringWarmStart에 안전하게 쓰일 수
// 있는 전제(크기 일치, horizon 이상, 모든 값 유한)를 만족하는지 검사한다.
bool WarmStartFeedforwardIsValid(
    const std::vector<MPCControl>& warm_start,
    const std::vector<double>& warm_start_ff,
    int horizon);

#endif // MPC_SOLVER_HPP