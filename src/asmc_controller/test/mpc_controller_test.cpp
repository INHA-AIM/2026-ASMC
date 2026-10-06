#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "MPC/MPC.hpp"
#include "MPC/Model/vehicle_model.hpp"
#include "MPC/Solver/mpc_solver.hpp"

namespace {

// 30m 직선, 3 m/s 등속 목표. solveMPC가 실제로 수렴할 수 있는 단순한 참조다.
ReferencePath MakeStraightReference(double target_speed, int steps, double dt) {
    ReferencePath reference;
    for (int i = 0; i < steps; ++i) {
        const double s = target_speed * dt * i;
        reference.x_ref.push_back(s);
        reference.y_ref.push_back(0.0);
        reference.yaw_ref.push_back(0.0);
        reference.v_ref.push_back(target_speed);
        reference.a_ref.push_back(0.0);
        reference.k_ref.push_back(0.0);
    }
    return reference;
}

// 일정 곡률(kappa)의 원호 reference. Commit C의 curvature feed-forward
// 초기 guess가 실제로 곡률 방향을 향하는지 확인하는 데 쓴다.
ReferencePath MakeCurvedReference(double target_speed, double kappa,
                                  int steps, double dt) {
    ReferencePath reference;
    double x = 0.0, y = 0.0, yaw = 0.0;
    for (int i = 0; i < steps; ++i) {
        reference.x_ref.push_back(x);
        reference.y_ref.push_back(y);
        reference.yaw_ref.push_back(yaw);
        reference.v_ref.push_back(target_speed);
        reference.a_ref.push_back(0.0);
        reference.k_ref.push_back(kappa);
        const double ds = target_speed * dt;
        x += ds * std::cos(yaw);
        y += ds * std::sin(yaw);
        yaw += ds * kappa;
    }
    return reference;
}

ReferencePath MakeDeceleratingReference(double initial_speed,
                                        double acceleration,
                                        double kappa,
                                        int steps,
                                        double dt) {
    ReferencePath reference;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double speed = initial_speed;
    for (int i = 0; i < steps; ++i) {
        reference.x_ref.push_back(x);
        reference.y_ref.push_back(y);
        reference.yaw_ref.push_back(yaw);
        reference.v_ref.push_back(speed);
        reference.a_ref.push_back(acceleration);
        reference.k_ref.push_back(kappa);

        const double ds = speed * dt;
        x += ds * std::cos(yaw);
        y += ds * std::sin(yaw);
        yaw += ds * kappa;
        speed = std::max(0.0, speed + acceleration * dt);
    }
    return reference;
}

MPCParams MakeTestParams() {
    MPCParams params;
    params.horizon = 10;
    params.dt = 0.1;
    params.max_iterations = 25;
    return params;
}

MPCParams MakeDecoupledTrackingParams() {
    MPCParams params;
    params.horizon = 24;
    params.dt = 0.05;
    params.max_iterations = 25;
    params.wheelbase = 3.0;
    params.steering_curvature_gain = 0.844;
    params.steering_max = 0.6981317008;
    params.steering_rate_max = 1.0;
    params.weight_contour_error = 6.0;
    params.weight_lag_error = 0.5;
    params.weight_heading_error = 3.0;
    params.weight_speed_error = 2.0;
    params.weight_control = 1.25;
    params.weight_control_rate = 100.0;
    params.weight_control_rate_accel = 4.5;
    params.weight_terminal = 8.0;
    params.lr_init = 0.20;
    params.line_search_steps = 7;
    return params;
}

MPCResult SolveCurvedTrackingCase(const MPCParams& params,
                                  const ReferencePath& reference,
                                  double tangent_offset,
                                  double normal_offset,
                                  double ego_speed) {
    const double yaw = reference.yaw_ref.front();
    MPCState ego;
    ego.x = reference.x_ref.front() + tangent_offset * std::cos(yaw) -
            normal_offset * std::sin(yaw);
    ego.y = reference.y_ref.front() + tangent_offset * std::sin(yaw) +
            normal_offset * std::cos(yaw);
    ego.yaw = yaw;
    ego.vx = ego_speed;

    MPCControl previous;
    previous.delta = SteeringFeedforward(
        reference.k_ref.front(), params.wheelbase, params.steering_max,
        params.steering_curvature_gain);
    return solveMPC(ego, reference, previous, {}, params);
}

}  // namespace

// D15: MpcController가 이전 process-global controller state를 대체한 뒤에도
// 실제 solve 경로가 살아있는지 확인하는 smoke test.
TEST(MpcControllerTest, ControlProcessProducesForwardAccelFromStandstill) {
    MPC controller(MakeTestParams());
    MPCState ego;  // 정지 상태(vx=0)에서 출발
    const ReferencePath reference = MakeStraightReference(3.0, 10, 0.1);

    const MPCOutput output =
        controller.MPCProcess(ego, reference);

    ASSERT_TRUE(output.success);
    EXPECT_GT(output.control.accel, 0.0)
        << "정지 상태에서 전방 등속 참조를 받으면 가속해야 한다";
    EXPECT_TRUE(std::isfinite(output.control.delta));
}

// D15: 두 MPC 인스턴스가 완전히 독립적인 상태를 갖는지 확인한다.
// 이전 구조의 process-global state에서는 인스턴스 개념이 아예 없었다.
// 이 테스트는 controller state가 실제로 분리됐다는 것의 회귀다.
TEST(MpcControllerTest, TwoInstancesDoNotShareWarmStartOrParams) {
    MPCParams params_a = MakeTestParams();
    params_a.horizon = 10;
    MPCParams params_b = MakeTestParams();
    params_b.horizon = 6;  // 서로 다른 값 - 공유되면 나중 인스턴스가 덮어쓴다

    MPC controller_a(params_a);
    MPC controller_b(params_b);

    EXPECT_EQ(10, controller_a.params().horizon);
    EXPECT_EQ(6, controller_b.params().horizon);

    // a를 여러 사이클 돌려 last_control_/warm_start_를 채운다.
    const ReferencePath reference_a = MakeStraightReference(3.0, 10, 0.1);
    MPCState ego_a;
    for (int i = 0; i < 3; ++i) {
        const auto out = controller_a.MPCProcess(ego_a, reference_a);
        ASSERT_TRUE(out.success);
        ego_a.vx += out.control.accel * params_a.dt;
    }

    // b는 방금 만든 새 인스턴스이므로 정지 상태에서 시작한 것과 동일하게
    // 동작해야 한다 - a의 warm-start/last_control이 새어 들어오면 안 된다.
    const ReferencePath reference_b = MakeStraightReference(3.0, 6, 0.1);
    MPCState ego_b;
    const auto out_b = controller_b.MPCProcess(ego_b, reference_b);
    ASSERT_TRUE(out_b.success);
    EXPECT_GT(out_b.control.accel, 0.0);
    EXPECT_EQ(6, controller_b.params().horizon);
}

// D15: warm-start가 사이클 간에 실제로 이어지는지 확인한다(상태 소유권
// 회귀). 완전히 매 사이클 새로 푸는 것과 warm-start를 재사용하는 것은
// 수렴 속도/결과가 달라야 한다 - 두 번째 사이클에서 previous control과
// 이어지는 rate cost가 반영된 결과가 나오는지 본다.
TEST(MpcControllerTest, WarmStartCarriesAcrossCycles) {
    MPC controller(MakeTestParams());
    MPCState ego;
    const ReferencePath reference = MakeStraightReference(3.0, 10, 0.1);

    const auto first = controller.MPCProcess(ego, reference);
    ASSERT_TRUE(first.success);
    ego.vx += first.control.accel * 0.1;
    ego.x += ego.vx * 0.1;

    const auto second = controller.MPCProcess(ego, reference);
    ASSERT_TRUE(second.success);
    // 두 번째 사이클은 첫 사이클의 접근속도를 이어받아 계속 가속해야 한다
    // (참조 속도 3.0에 아직 도달하지 못한 상태).
    EXPECT_GT(second.control.accel, -1e-6);
}

// D15: 정지 데드락 복구가 실제로 발동하는지 확인한다. 전방 의도가 있는데
// (참조가 전진 + 충분히 진행) ego가 계속 멈춰 있으면 stuck_timeout 이후
// RECOVERY_STARTED 이벤트가 나오고, 그 다음 recovery_duration 동안
// launch_accel 이상의 가속을 강제해야 한다.
TEST(MpcControllerTest, StuckRecoveryTriggersAfterTimeout) {
    MPCParams params = MakeTestParams();
    params.control_frequency = 20.0;  // 1 cycle = 0.05s
    params.stuck_timeout = 0.1;       // 2 cycle 안에 감지
    params.stuck_recovery_duration = 0.1;
    params.stuck_launch_accel = 1.0;
    params.stuck_ego_speed = 0.1;
    params.stuck_target_speed = 0.3;
    params.stuck_min_path_progress = 0.5;

    MPC controller(params);
    // ego는 절대 움직이지 않는다(정지 데드락 시뮬레이션) - 참조는 전방으로
    // 충분히 진행하는 3 m/s 직선이라 forward_intent 조건을 만족한다.
    const ReferencePath reference = MakeStraightReference(3.0, 10, 0.1);
    const MPCState stuck_ego;  // vx = 0 고정

    bool saw_recovery_started = false;
    for (int i = 0; i < 6 && !saw_recovery_started; ++i) {
        const auto output = controller.MPCProcess(stuck_ego, reference);
        ASSERT_TRUE(output.success);
        if (output.stuck_event ==
            StuckEvent::RECOVERY_STARTED) {
            saw_recovery_started = true;
        }
    }
    EXPECT_TRUE(saw_recovery_started)
        << "ego가 멈춰있는데 참조가 전진하면 stuck_timeout 이후 복구가 "
           "시작되어야 한다";
}

// Test C-6: warm-start가 없는 curved reference 첫 solve에서 초기
// sequence가 curvature 방향의 steering을 갖되(post-hoc이 아니라
// solveMPC 내부에서 사용된 초기 guess를 통해), physical steering-rate
// constraint(clipSteeringRate가 이미 투영)를 만족하는지 확인한다.
// solveMPC를 직접 호출해 controller wrapper 없이도 testable하게 한다.
TEST(SteeringFeedforwardWarmStartTest,
    NoWarmStartCurvedSolveRespectsRateLimitAndCurvatureSign) {
    MPCParams params;
    params.horizon = 10;
    params.dt = 0.1;
    params.max_iterations = 25;
    params.wheelbase = 3.0;
    params.steering_max = 0.6981;
    params.steering_rate_max = 1.0;

    const double kappa = 0.05;  // 좌회전 원호
    const ReferencePath reference = MakeCurvedReference(3.0, kappa, 10, 0.1);
    const MPCState x0;  // x=y=yaw=0, vx=0
    const MPCControl prev_control;  // 이전 사이클 실제 steering: 0
    const std::vector<MPCControl> empty_warm_start;  // 첫 solve/fail-safe 직후

    const MPCResult result =
        solveMPC(x0, reference, prev_control, empty_warm_start, params);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(static_cast<std::size_t>(params.horizon), result.controls.size());

    // 첫 horizon point가 이전 steering(0)에서 순간이동하지 않는다.
    EXPECT_LE(std::fabs(result.controls[0].delta - prev_control.delta),
             params.steering_rate_max * params.dt + 1e-9);
    // 전체 시퀀스가 이전 스텝 대비 rate 제한을 넘지 않는다.
    double prev_delta = prev_control.delta;
    for (const auto& u : result.controls) {
        EXPECT_LE(std::fabs(u.delta - prev_delta),
                 params.steering_rate_max * params.dt + 1e-9);
        prev_delta = u.delta;
    }
    // 최종 steering이 좌회전(양의 curvature) 방향을 향한다 - solver가 곡선을
    // 따라가려면 결국 양의 delta로 수렴해야 한다.
    EXPECT_GT(result.control.delta, 0.0);
}

TEST(MpcContourLagSolverTest,
     EqualLagAndContourOffsetsProduceDifferentSteeringCorrections) {
    const MPCParams params = MakeDecoupledTrackingParams();
    constexpr double kappa = 0.04;
    const ReferencePath reference = MakeCurvedReference(
        5.0, kappa, params.horizon, params.dt);
    const double feedforward = SteeringFeedforward(
        kappa, params.wheelbase, params.steering_max,
        params.steering_curvature_gain);

    const MPCResult nominal = SolveCurvedTrackingCase(
        params, reference, 0.0, 0.0, 5.0);
    const MPCResult lagged = SolveCurvedTrackingCase(
        params, reference, -1.5, 0.0, 5.0);
    const MPCResult left_offset = SolveCurvedTrackingCase(
        params, reference, 0.0, 1.5, 5.0);
    ASSERT_TRUE(nominal.success);
    ASSERT_TRUE(lagged.success);
    ASSERT_TRUE(left_offset.success);

    const double lag_correction =
        std::fabs(lagged.control.delta - nominal.control.delta);
    const double contour_correction =
        std::fabs(left_offset.control.delta - nominal.control.delta);
    EXPECT_LT(lag_correction, contour_correction)
        << "equal longitudinal lag must influence steering less than a "
           "lateral contour error";
    EXPECT_LT(left_offset.control.delta, lagged.control.delta)
        << "a left-normal offset on a left curve must request a rightward "
           "correction relative to the lag-only case";

    for (const MPCResult* result : {&nominal, &lagged, &left_offset}) {
        EXPECT_LE(std::fabs(result->control.delta - feedforward),
                  params.steering_rate_max * params.dt + 1e-9);
    }
}

TEST(MpcContourLagSolverTest,
     DeceleratingProgressLagDoesNotReverseCurvatureSteering) {
    const MPCParams params = MakeDecoupledTrackingParams();
    constexpr double kappa = 0.04;
    const ReferencePath faster = MakeCurvedReference(
        5.0, kappa, params.horizon, params.dt);
    const ReferencePath decelerating = MakeCurvedReference(
        3.0, kappa, params.horizon, params.dt);
    MPCState nominal_ego;
    nominal_ego.vx = 5.0;
    MPCState lagged_ego = nominal_ego;
    lagged_ego.x = -1.0;  // yaw=0 reference tangent direction only
    lagged_ego.vx = 4.8;
    MPCState contour_ego = nominal_ego;
    contour_ego.y = 1.0;
    contour_ego.vx = 4.8;

    // Use stateful controllers so cycle B consumes cycle A's shifted warm
    // start and previous steering exactly like runtime.
    MPC lag_controller(params);
    const MPCOutput before =
        lag_controller.MPCProcess(nominal_ego, faster);
    const MPCOutput after =
        lag_controller.MPCProcess(lagged_ego, decelerating);
    MPC contour_controller(params);
    ASSERT_TRUE(contour_controller.MPCProcess(nominal_ego, faster).success);
    const MPCOutput contour_reference =
        contour_controller.MPCProcess(contour_ego, decelerating);
    ASSERT_TRUE(before.success);
    ASSERT_TRUE(after.success);
    ASSERT_TRUE(contour_reference.success);

    EXPECT_GT(before.control.delta, 0.0);
    EXPECT_GT(after.control.delta, 0.0)
        << "a speed/progress transition with zero initial contour error must "
           "not reverse steering against positive reference curvature";
    EXPECT_LE(std::fabs(after.control.delta - before.control.delta),
              params.steering_rate_max * params.dt + 1e-9)
        << "cycle-to-cycle steering must retain the physical rate constraint";
    EXPECT_LT(std::fabs(after.control.delta - before.control.delta),
              std::fabs(contour_reference.control.delta -
                        before.control.delta))
        << "pure progress lag should perturb feed-forward steering less than "
           "an equal lateral offset";
}

TEST(MpcAccelerationFeedforwardSolverTest,
     NegativeReferenceProducesStrongerBrakingThanZeroReference) {
    const MPCParams params = MakeDecoupledTrackingParams();
    ReferencePath with_acceleration = MakeDeceleratingReference(
        /*initial_speed=*/10.0, /*acceleration=*/-2.0,
        /*kappa=*/0.0, params.horizon, params.dt);
    ReferencePath without_acceleration = with_acceleration;
    std::fill(without_acceleration.a_ref.begin(),
              without_acceleration.a_ref.end(), 0.0);

    MPCState ego;
    ego.vx = 10.0;
    const MPCControl previous;
    const MPCResult baseline = solveMPC(
        ego, without_acceleration, previous, {}, params);
    const MPCResult feedforward = solveMPC(
        ego, with_acceleration, previous, {}, params);

    ASSERT_TRUE(baseline.success);
    ASSERT_TRUE(feedforward.success);
    EXPECT_LT(feedforward.control.accel, 0.0);
    EXPECT_LT(feedforward.control.accel, baseline.control.accel)
        << "the same decreasing speed/geometry reference must request more "
           "braking when the final-grid speed derivative is negative";
}

TEST(MpcAccelerationFeedforwardSolverTest,
     DeceleratingFixtureHasConsistentSpeedAndAcceleration) {
    const MPCParams params = MakeDecoupledTrackingParams();
    const ReferencePath reference = MakeDeceleratingReference(
        10.0, -2.0, 0.0, params.horizon, params.dt);

    for (std::size_t i = 0; i + 1 < reference.size(); ++i) {
        EXPECT_NEAR(reference.v_ref[i] +
                        reference.a_ref[i] * params.dt,
                    reference.v_ref[i + 1], 1e-12);
    }
}

TEST(MpcAccelerationFeedforwardSolverTest,
     BrakingReferencePreservesCurvatureSteeringAndRateLimit) {
    const MPCParams params = MakeDecoupledTrackingParams();
    constexpr double kappa = 0.04;
    ReferencePath with_acceleration = MakeDeceleratingReference(
        10.0, -2.0, kappa, params.horizon, params.dt);
    ReferencePath without_acceleration = with_acceleration;
    std::fill(without_acceleration.a_ref.begin(),
              without_acceleration.a_ref.end(), 0.0);

    MPCState ego;
    ego.vx = 10.0;
    MPCControl previous;
    previous.delta = SteeringFeedforward(
        kappa, params.wheelbase, params.steering_max,
        params.steering_curvature_gain);
    const MPCResult baseline = solveMPC(
        ego, without_acceleration, previous, {}, params);
    const MPCResult feedforward = solveMPC(
        ego, with_acceleration, previous, {}, params);

    ASSERT_TRUE(baseline.success);
    ASSERT_TRUE(feedforward.success);
    EXPECT_GT(baseline.control.delta, 0.0);
    EXPECT_GT(feedforward.control.delta, 0.0)
        << "longitudinal feed-forward must not reverse curve steering";
    EXPECT_LE(std::fabs(feedforward.control.delta - baseline.control.delta),
              2.0 * params.steering_rate_max * params.dt + 1e-9);

    double previous_delta = previous.delta;
    for (const MPCControl& control : feedforward.controls) {
        EXPECT_LE(std::fabs(control.delta - previous_delta),
                  params.steering_rate_max * params.dt + 1e-9);
        previous_delta = control.delta;
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
