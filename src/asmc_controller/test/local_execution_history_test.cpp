#include <gtest/gtest.h>
#include "local/execution_history.hpp"
#include "MPC/MPC.hpp"
#include "MPC/Model/steering_delay.hpp"

using asmc_local::ExecutionHistory;
TEST(LocalExecutionHistory, FixedCadencePreservesDelaySamples) {
  ExecutionHistory h;
  for (int i=0; i<4; ++i) h.Issued(1.+i*.05, {i*.1, .2});
  auto s=h.At(1.20,.05,.15);
  EXPECT_FALSE(s.restart_seed);
  ASSERT_EQ(s.steering.size(), 4u);
  for (int i=0; i<4; ++i) EXPECT_NEAR(s.steering[i], i*.1, 1e-10);
}
TEST(LocalExecutionHistory, HeldCommandAdvancesDespiteSkippedSolve) {
  ExecutionHistory h;
  for (int i=0; i<4; ++i) h.Issued(1.+i*.05, {i*.1, .2});
  h.Pending();
  auto s=h.At(1.30,.05,.15);
  ASSERT_TRUE(s.restart_seed);
  ASSERT_EQ(s.steering.size(),4u);
  EXPECT_NEAR(s.steering.front(),.2,1e-10);
  std::vector<MPCControl> commands(4, {-.2,0});
  const auto after=ApplySteeringDelay(commands,s.steering,.0,.15,.05);
  for (int i=0;i<3;++i) EXPECT_NEAR(after[i].delta,.3,1e-10);
  // Old count-based history would still predict .1, .2, .3 here.
  const auto before=ApplySteeringDelay(commands,{0,.1,.2,.3},0,.15,.05);
  EXPECT_NEAR(before[0].delta,.1,1e-10);
}
TEST(LocalExecutionHistory, RestartSeedKeepsActualControlAndNormalIsUnaffected) {
  MPCParams p; InitMPC(p);
  g_mpc.warm_start.assign(20,{.1,.5});
  ExecutionHistory h;h.Issued(1.,{.3,.2});
  asmc_local::PrepareExecution(h.At(1.05,.05,.15));
  EXPECT_FALSE(g_mpc.warm_start.empty());
  h.Pending();asmc_local::PrepareExecution(h.At(1.10,.05,.15));
  EXPECT_TRUE(g_mpc.warm_start.empty());
  EXPECT_DOUBLE_EQ(g_mpc.last_control.delta,.3);
  EXPECT_DOUBLE_EQ(g_mpc.last_control.accel,.2);
  ResetMPCOnFailSafe();
  EXPECT_TRUE(g_mpc.steering_history.empty());
  EXPECT_DOUBLE_EQ(g_mpc.last_control.delta,0);
}
TEST(LocalExecutionHistory, IrregularCadenceIsZeroOrderHoldNotInterpolation) {
  ExecutionHistory h;h.Issued(1.,{.1,0});h.Issued(1.07,{.4,0});
  auto s=h.At(1.16,.05,.15);
  ASSERT_EQ(s.steering.size(),4u);
  EXPECT_EQ(s.steering,(std::vector<double>{.1,.1,.1,.4}));
  EXPECT_TRUE(s.restart_seed);
}
TEST(LocalExecutionHistory, ResetAndInvalidTimeDoNotReuseHistory) {
  ExecutionHistory h;h.Issued(2.,{.1,0});
  EXPECT_THROW(h.At(1.9,.05,.15),std::invalid_argument);
  h.Issued(1.,{.2,0}); // clock reset: no old event survives
  EXPECT_EQ(h.At(1.05,.05,.15).steering,(std::vector<double>{.2,.2,.2,.2}));
  h.Clear();EXPECT_TRUE(h.At(1.,.05,.15).steering.empty());
  EXPECT_THROW(h.Issued(1.,{NAN,0}),std::invalid_argument);
  EXPECT_THROW(h.At(1.,.0,.15),std::invalid_argument);
}
TEST(LocalExecutionHistory, BoundedAfterLongRunAndPendingExpiresOnIssue) {
  ExecutionHistory h;
  for (int i=0;i<10000;++i) h.Issued(1.+i*.05,{.25,0});
  h.Pending();h.Issued(501.,{.3,0});
  EXPECT_FALSE(h.At(501.05,.05,.15).restart_seed);
  EXPECT_NEAR(h.At(501.15,.05,.15).steering.back(),.3,1e-10);
}
