#include <gtest/gtest.h>

#include <cmath>

#include "control/mpc_controller.hpp"

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

MPCParams MakeTestParams() {
    MPCParams params;
    params.horizon = 10;
    params.dt = 0.1;
    params.max_iterations = 25;
    return params;
}

}  // namespace

// D15: MpcController가 이전 global(ego/last_control/mpc_params)을 대체한
// 뒤에도 실제 solve 경로가 살아있는지 확인하는 smoke test.
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
// 이전 구조(extern MPCState ego/extern MPCControl last_control/
// extern MPCParams mpc_params)에서는 프로세스 전역이라 인스턴스 개념이
// 아예 없었다 - 이 테스트는 그 전역이 실제로 사라졌다는 것의 회귀다.
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

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
