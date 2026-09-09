#pragma once

#include <string>
#include <vector>

#include "common/mpc_control.hpp"
#include "common/mpc_state.hpp"

// MPC 결과
struct MPCResult {
    MPCControl control;
    std::vector<MPCState>   predicted_states;
    std::vector<MPCControl> controls;
    // controls[i]를 만든 이번 solve의 reference로 계산한 stage별 curvature
    // feed-forward(SteeringFeedforward(ref.k_ref[i], ...)). controls와 같은
    // 크기, 같은 인덱스로 정렬된다 - 다음 cycle에 warm-start를 shift할 때
    // "이전 최적해 - 이전 feed-forward = 이전 feedback residual"을 계산하려면
    // 이전 해와 그 해를 만든 feed-forward가 반드시 같은 solve 쌍이어야 한다
    // (다른 cycle의 값과 섞이면 잘못된 residual이 나온다).
    std::vector<double> steering_feedforward;
    // 최적화 반복 이전, projectControls까지 마친 stage-0 초기 steering
    // guess([MPC-WARM] warm_projected0 진단 전용). 최적화가 끝난 뒤의
    // control.delta(최종 해)와는 다른 값이다 - "rebase가 물리 제약을 통과한
    // 뒤 실제로 최적화를 어디서 시작했는가"를 보려면 반복 전 값이 필요하다.
    double initial_guess_delta0 = 0.0;
    double cost = 0.0;
    bool   success = false;
    std::string solver_msg;
};
