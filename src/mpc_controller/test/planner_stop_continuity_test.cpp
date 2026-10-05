#include <gtest/gtest.h>
#include <cmath>
#include "control/mpc_controller.hpp"
#include "model/vehicle_model.hpp"
namespace {
ReferencePath Curve(const MPCState& ego,const MPCParams& p,double delta) {
    std::vector<MPCControl> controls(p.horizon);for(auto& u:controls)u={delta,0};
    auto states=predictTrajectory(ego,controls,p.dt,p.wheelbase,p.vel_min,p.vel_max,p.steering_curvature_gain);
    ReferencePath r;for(auto s:states){r.x_ref.push_back(s.x);r.y_ref.push_back(s.y);r.yaw_ref.push_back(s.yaw);r.v_ref.push_back(s.vx);r.a_ref.push_back(0);r.k_ref.push_back(p.steering_curvature_gain*std::tan(delta)/p.wheelbase);}return r;
}
}
TEST(PlannerStopContinuity, RestartIsRateBoundedAroundPublishedStopSteering) {
    MPCParams p;p.horizon=20;p.max_iterations=25;
    for(double delta:{-.42,.3}) {
        MpcController controller(p);controller.ResetForPlannerStop(delta);
        MPCState ego{0,0,0,3};auto result=controller.ControlProcess(ego,Curve(ego,p,delta));
        ASSERT_TRUE(result.success);EXPECT_LE(std::fabs(result.control.delta-delta),p.steering_rate_max*p.dt+1e-8);
    }
}
TEST(PlannerStopContinuity, EmergencyResetStillStartsFromZero) {
    MPCParams p;MpcController controller(p);controller.ResetForPlannerStop(-.42);controller.ResetOnFailSafe();
    MPCState ego{0,0,0,3};auto result=controller.ControlProcess(ego,Curve(ego,p,-.42));
    ASSERT_TRUE(result.success);EXPECT_LE(std::fabs(result.control.delta),p.steering_rate_max*p.dt+1e-8);
}
