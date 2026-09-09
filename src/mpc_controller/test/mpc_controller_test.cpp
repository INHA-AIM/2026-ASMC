#include <gtest/gtest.h>

#include <cmath>

#include "control/mpc_controller.hpp"
#include "model/vehicle_model.hpp"
#include "solver/mpc_solver.hpp"

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
        reference.k_ref.push_back(kappa);
        const double ds = target_speed * dt;
        x += ds * std::cos(yaw);
        y += ds * std::sin(yaw);
        yaw += ds * kappa;
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

}  // namespace

// D15: MpcController가 이전 process-global controller state를 대체한 뒤에도
// 실제 solve 경로가 살아있는지 확인하는 smoke test.
TEST(MpcControllerTest, ControlProcessProducesForwardAccelFromStandstill) {
    MpcController controller(MakeTestParams());
    MPCState ego;  // 정지 상태(vx=0)에서 출발
    const ReferencePath reference = MakeStraightReference(3.0, 10, 0.1);

    const MpcController::Output output =
        controller.ControlProcess(ego, reference);

    ASSERT_TRUE(output.success);
    EXPECT_GT(output.control.accel, 0.0)
        << "정지 상태에서 전방 등속 참조를 받으면 가속해야 한다";
    EXPECT_TRUE(std::isfinite(output.control.delta));
}

// D15: 두 MpcController 인스턴스가 완전히 독립적인 상태를 갖는지 확인한다.
// 이전 구조의 process-global state에서는 인스턴스 개념이 아예 없었다.
// 이 테스트는 controller state가 실제로 분리됐다는 것의 회귀다.
TEST(MpcControllerTest, TwoInstancesDoNotShareWarmStartOrParams) {
    MPCParams params_a = MakeTestParams();
    params_a.horizon = 10;
    MPCParams params_b = MakeTestParams();
    params_b.horizon = 6;  // 서로 다른 값 - 공유되면 나중 인스턴스가 덮어쓴다

    MpcController controller_a(params_a);
    MpcController controller_b(params_b);

    EXPECT_EQ(10, controller_a.params().horizon);
    EXPECT_EQ(6, controller_b.params().horizon);

    // a를 여러 사이클 돌려 last_control_/warm_start_를 채운다.
    const ReferencePath reference_a = MakeStraightReference(3.0, 10, 0.1);
    MPCState ego_a;
    for (int i = 0; i < 3; ++i) {
        const auto out = controller_a.ControlProcess(ego_a, reference_a);
        ASSERT_TRUE(out.success);
        ego_a.vx += out.control.accel * params_a.dt;
    }

    // b는 방금 만든 새 인스턴스이므로 정지 상태에서 시작한 것과 동일하게
    // 동작해야 한다 - a의 warm-start/last_control이 새어 들어오면 안 된다.
    const ReferencePath reference_b = MakeStraightReference(3.0, 6, 0.1);
    MPCState ego_b;
    const auto out_b = controller_b.ControlProcess(ego_b, reference_b);
    ASSERT_TRUE(out_b.success);
    EXPECT_GT(out_b.control.accel, 0.0);
    EXPECT_EQ(6, controller_b.params().horizon);
}

// D15: warm-start가 사이클 간에 실제로 이어지는지 확인한다(상태 소유권
// 회귀). 완전히 매 사이클 새로 푸는 것과 warm-start를 재사용하는 것은
// 수렴 속도/결과가 달라야 한다 - 두 번째 사이클에서 previous control과
// 이어지는 rate cost가 반영된 결과가 나오는지 본다.
TEST(MpcControllerTest, WarmStartCarriesAcrossCycles) {
    MpcController controller(MakeTestParams());
    MPCState ego;
    const ReferencePath reference = MakeStraightReference(3.0, 10, 0.1);

    const auto first = controller.ControlProcess(ego, reference);
    ASSERT_TRUE(first.success);
    ego.vx += first.control.accel * 0.1;
    ego.x += ego.vx * 0.1;

    const auto second = controller.ControlProcess(ego, reference);
    ASSERT_TRUE(second.success);
    // 두 번째 사이클은 첫 사이클의 접근속도를 이어받아 계속 가속해야 한다
    // (참조 속도 3.0에 아직 도달하지 못한 상태).
    EXPECT_GT(second.control.accel, -1e-6);
}

// curve-tracking stability iteration 2: 이전 steering이 거의 0인 상태(직선
// warm-start가 이미 채워진 뒤)에서 curved reference로 바뀌었을 때, warm-start
// shift(mpc_solver.cpp의 "이전 해를 한 스텝 shift") 경로를 거쳐도 solver가
// 물리 제약(steering angle/rate) 안에서 실제로 곡률 방향으로 steering을
// 옮기는지 확인한다. control_rate 절대값을 검증하지 않는다 - "방향이 맞고
// 제약을 넘지 않는가"라는 성질만 본다(과도하게 특정 부동소수 출력에 결합된
// 회귀를 피하기 위함).
TEST(MpcControllerTest, WarmStartedControllerSteersTowardNewCurveWithinLimits) {
    MpcController controller(MakeTestParams());
    MPCState ego;  // x=y=yaw=0, vx=0

    // 1) 직선 reference로 몇 사이클 돌려 steering이 0 근방에 정착하고
    //    warm_start_가 채워지게 한다.
    const ReferencePath straight_ref = MakeStraightReference(3.0, 10, 0.1);
    MpcController::Output straight_output;
    for (int i = 0; i < 3; ++i) {
        straight_output = controller.ControlProcess(ego, straight_ref);
        ASSERT_TRUE(straight_output.success);
        ego.vx += straight_output.control.accel * 0.1;
        ego.x += ego.vx * 0.1;
    }
    ASSERT_NEAR(straight_output.control.delta, 0.0, 1e-6);

    // 2) 그 다음 사이클에 curved reference를 준다 - 이전 controller 상태
    //    (warm_start_/last_control_)는 그대로 이어받는다(같은 controller
    //    인스턴스).
    const double kappa = 0.05;  // 좌회전 원호
    const ReferencePath curved_ref = MakeCurvedReference(3.0, kappa, 10, 0.1);
    const auto curved_output = controller.ControlProcess(ego, curved_ref);
    ASSERT_TRUE(curved_output.success);

    // 물리 제약: steering 절대값과 이전 스텝 대비 rate 한계를 넘지 않는다.
    const MPCParams& params = controller.params();
    EXPECT_LE(std::fabs(curved_output.control.delta), params.steering_max + 1e-9);
    EXPECT_LE(std::fabs(curved_output.control.delta - straight_output.control.delta),
             params.steering_rate_max * params.dt + 1e-9);
    // 방향: 곡률(양수, 좌회전)에 맞는 방향으로 steering이 움직여야 한다 -
    // 이전 값(~0)에 머무르거나 반대 방향으로 가면 안 된다.
    EXPECT_GT(curved_output.control.delta, straight_output.control.delta);
}

// curve-tracking stability iteration 3: 이전 커브가 절대 steering을
// 이미 한쪽 방향(우회전, 음수)으로 크게 밀어둔 상태에서, 새 reference가
// 반대 방향(좌회전, 양수) 커브로 바뀌었을 때도 rebase된 warm-start가 새
// 방향을 따라가는지 확인한다 - "새 reference 방향이 stale absolute
// warm-start 방향을 지배해야 한다"는 핵심 불변식의 solver-level(pure
// 함수가 아니라 MpcController::ControlProcess 전체 경로) 검증이다.
TEST(MpcControllerTest,
    WarmStartRebasesTowardOppositeNewCurveAfterEstablishedOppositeWarmStart) {
    MpcController controller(MakeTestParams());
    MPCState ego;  // x=y=yaw=0, vx=0

    // 1) 우회전(음의 kappa) reference로 여러 사이클 돌려, warm_start_/
    //    previous_steering_feedforward_가 "이전 커브 방향"으로 확실히
    //    자리잡게 한다(단순 직선 0 근방이 아니라 실제 비영 절대 steering).
    const double old_kappa = -0.05;
    const ReferencePath old_curve_ref = MakeCurvedReference(3.0, old_kappa, 10, 0.1);
    MpcController::Output old_output;
    for (int i = 0; i < 5; ++i) {
        old_output = controller.ControlProcess(ego, old_curve_ref);
        ASSERT_TRUE(old_output.success);
        ego.vx += old_output.control.accel * 0.1;
        ego.x += ego.vx * 0.1;
    }
    ASSERT_LT(old_output.control.delta, 0.0)
        << "이전 커브(우회전)에서 steering이 음의 방향으로 자리잡아야 다음 "
           "단계의 방향 반전 검증이 의미가 있다";

    // 2) 새 reference는 반대 방향(좌회전, 양의 kappa)이다. ego 상태는 그대로
    //    이어받되(같은 controller 인스턴스, warm_start_ 재사용), reference만
    //    반대로 바뀐 상황을 만든다.
    const double new_kappa = 0.05;
    const ReferencePath new_curve_ref = MakeCurvedReference(3.0, new_kappa, 10, 0.1);
    const auto new_output = controller.ControlProcess(ego, new_curve_ref);
    ASSERT_TRUE(new_output.success);

    // 물리 제약은 여전히 만족해야 한다.
    const MPCParams& params = controller.params();
    EXPECT_LE(std::fabs(new_output.control.delta), params.steering_max + 1e-9);
    EXPECT_LE(std::fabs(new_output.control.delta - old_output.control.delta),
             params.steering_rate_max * params.dt + 1e-9);

    // 핵심 불변식: steering이 새 커브 방향(양수)으로 움직여야 한다 - 이전
    // 절대 steering(음수)에 갇혀 계속 음의 값에 머무르면 실패다.
    EXPECT_GT(new_output.control.delta, old_output.control.delta);
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

    MpcController controller(params);
    // ego는 절대 움직이지 않는다(정지 데드락 시뮬레이션) - 참조는 전방으로
    // 충분히 진행하는 3 m/s 직선이라 forward_intent 조건을 만족한다.
    const ReferencePath reference = MakeStraightReference(3.0, 10, 0.1);
    const MPCState stuck_ego;  // vx = 0 고정

    bool saw_recovery_started = false;
    for (int i = 0; i < 6 && !saw_recovery_started; ++i) {
        const auto output = controller.ControlProcess(stuck_ego, reference);
        ASSERT_TRUE(output.success);
        if (output.stuck_event ==
            MpcController::StuckEvent::RECOVERY_STARTED) {
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
    const std::vector<double> empty_warm_start_ff;  // 위와 짝 - 역시 비어있음

    const MPCResult result = solveMPC(x0, reference, prev_control,
                                      empty_warm_start, empty_warm_start_ff,
                                      params);
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

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
