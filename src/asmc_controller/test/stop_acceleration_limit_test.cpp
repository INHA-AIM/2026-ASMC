#include "MPC/MPC.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#ifndef STOP_RESPONSE_STANDALONE
#include <gtest/gtest.h>
TEST(StopAccelerationLimitTest, SolverBoundAndDefaultCompatibility) {
#else
int main() {
#endif
#ifdef STOP_RESPONSE_STANDALONE
    int failures=0;
#endif
    auto check=[&](bool ok,const char* name) {
#ifdef STOP_RESPONSE_STANDALONE
        std::cout<<(ok?"[PASS] ":"[FAIL] ")<<name<<'\n';failures+=!ok;
#else
        EXPECT_TRUE(ok)<<name;
#endif
    };
    MPCParams params;params.horizon=20;params.dt=.05;params.accel_max=2;
    // Use the operating controller weights, including the longitudinal rate weight.
    params.weight_contour_error=6;params.weight_lag_error=.5;
    params.weight_heading_error=3;params.weight_speed_error=2;
    params.weight_control=1.25;params.weight_control_rate_accel=2.5;
    params.weight_terminal=8;params.lr_init=.2;params.line_search_steps=7;
    ReferencePath reference;
    for(int i=0;i<params.horizon;++i) {
        const double t=i*params.dt;
        reference.x_ref.push_back(t+t*t);reference.y_ref.push_back(0);
        reference.yaw_ref.push_back(0);reference.k_ref.push_back(0);
        reference.v_ref.push_back(1+i*.05*2);reference.a_ref.push_back(2);
    }
    MPCState ego{0,0,0,1};
    InitMPC(params);ResetMPCOnFailSafe();auto ordinary=MPCProcess(ego,reference);
    InitMPC(params);ResetMPCOnFailSafe();auto explicit_default=MPCProcess(ego,reference,params.accel_max);
    check(ordinary.success&&explicit_default.success&&
          ordinary.control.accel==explicit_default.control.accel&&
          ordinary.control.delta==explicit_default.control.delta,
          "default call retains the existing solver result");
    // Begin with the preceding ordinary driving solution as warm-start.
    auto stop=MPCProcess(ego,reference,.5);
    check(stop.success&&stop.control.accel<=.5+1e-9,
          "STOP acceleration ceiling is enforced inside the solver");
    check(std::all_of(g_mpc.warm_start.begin(),g_mpc.warm_start.end(),
          [](const MPCControl& u){return u.accel<=.5+1e-9;}),
          "retained warm-start controls obey the active STOP ceiling");
    check(stop.first_used_reference_acceleration<=.5&&GetMPCParams().accel_max==2,
          "STOP reference clipping does not mutate the global driving limit");
    for(double limit:{0.,-.5}) {
        auto bounded=MPCProcess(ego,reference,limit);
        check(bounded.success && bounded.control.accel<=limit+1e-9,
              "zero and negative ceilings constrain the solver, not just its published output");
        check(std::all_of(g_mpc.warm_start.begin(),g_mpc.warm_start.end(),
              [=](const MPCControl& u){return u.accel<=limit+1e-9;}),
              "a previous accelerating warm start cannot bypass a nonpositive ceiling");
    }
    auto driving=MPCProcess(ego,reference);
    check(driving.success&&GetMPCParams().accel_max==2,
          "leaving STOP restores the original limit without a parameter reset");
    for(int i=0;i<params.horizon;++i) {
        const double t=i*params.dt;
        reference.x_ref[i]=t<.5?t-t*t:.25;
        reference.v_ref[i]=std::max(0.0,1-2*t);
        reference.a_ref[i]=t<.5?-2:0;
    }
    // Independently test a braking reference, without the acceleration test's
    // deliberately accelerating trajectory as its preceding warm-start.
    InitMPC(params);
    auto brake=MPCProcess(ego,reference,.5);
    check(brake.success&&brake.control.accel<0&&brake.control.accel>=params.accel_min,
          "positive STOP ceiling preserves the service braking bound");
#ifdef STOP_RESPONSE_STANDALONE
    return failures?1:0;
#endif
}
