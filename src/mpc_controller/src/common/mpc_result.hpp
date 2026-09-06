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
    double cost = 0.0;
    bool   success = false;
    std::string solver_msg;
};
