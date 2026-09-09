// curve-tracking stability iteration 3: curvature-aware steering warm-start
// rebase 회귀.
//
// 배경: 기존 warm-start는 "이전 최적 절대 steering을 한 스텝 shift"만
// 했다. 그 절대값은 이전 reference 기준으로 최적화된 값이라, 새 커브가
// 나타나면(reference curvature가 바뀌면) 그 절대값이 새 목표와 무관한
// "과거 방향"에 갇힌다 - MORAI 실측에서 ffpeak가 이미 반대 부호인데
// steer_cmd가 여러 cycle 동안 이전 값 근처에 머무르는 curve-entry lag로
// 나타났다. RebaseSteeringWarmStart는 절대값이 아니라 "이전 최적해가 이전
// feed-forward 대비 만든 feedback residual"만 이어받아 새 reference의
// feed-forward에 얹는다.
//
// 이 테스트는 production RebaseSteeringWarmStart/WarmStartFeedforwardIsValid/
// computeCurrentSteeringFeedforward(solver/mpc_solver.hpp)를 그대로 호출한다
// - 로직을 복사하지 않는다.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "solver/mpc_solver.hpp"

namespace {

MPCControl MakeControl(double delta, double accel = 0.0) {
    MPCControl u;
    u.delta = delta;
    u.accel = accel;
    return u;
}

}  // namespace

// TEST A: 이전 최적해가 이전 feed-forward와 정확히 같으면(residual=0),
// rebase 결과는 새 feed-forward와 같아야 한다(투영 이전).
TEST(WarmStartRebaseTest, ZeroResidualYieldsPureNewFeedforward) {
    const int N = 3;
    const std::vector<MPCControl> warm_start = {
        MakeControl(0.20), MakeControl(0.20), MakeControl(0.20)};
    const std::vector<double> warm_start_ff = {0.20, 0.20, 0.20};
    const std::vector<double> current_ff = {0.20, 0.20, 0.20};

    ASSERT_TRUE(WarmStartFeedforwardIsValid(warm_start, warm_start_ff, N));
    const std::vector<double> rebased =
        RebaseSteeringWarmStart(warm_start, warm_start_ff, current_ff, N);

    for (double v : rebased) {
        EXPECT_NEAR(0.20, v, 1e-9);
    }
}

// TEST B: 이전 residual(+0.04)이 새 feed-forward(+0.25) 위에 그대로
// 얹혀야 한다.
TEST(WarmStartRebaseTest, PreservesFeedbackResidual) {
    const int N = 2;
    // shifted_index = min(i+1, N-1)이므로 stage 0/1 모두 warm_start[1]을
    // 참조한다(N=2) - 두 stage 모두 같은 residual을 반영해야 한다.
    const std::vector<MPCControl> warm_start = {
        MakeControl(0.10), MakeControl(0.14)};
    const std::vector<double> warm_start_ff = {0.10, 0.10};
    const std::vector<double> current_ff = {0.25, 0.25};

    ASSERT_TRUE(WarmStartFeedforwardIsValid(warm_start, warm_start_ff, N));
    const std::vector<double> rebased =
        RebaseSteeringWarmStart(warm_start, warm_start_ff, current_ff, N);

    // residual = warm_start[1].delta - warm_start_ff[1] = 0.14 - 0.10 = 0.04
    // rebased[i] = current_ff[i] + 0.04 = 0.29
    for (double v : rebased) {
        EXPECT_NEAR(0.29, v, 1e-9);
    }
}

// TEST C(critical): 이전 커브가 반대 방향(음수)이었고 새 커브는 양수
// 방향이다. rebase 결과는 이전 절대값의 부호가 아니라 새 reference 방향을
// 따라가야 한다 - 이것이 이 반복의 핵심 불변식이다.
TEST(WarmStartRebaseTest, RebasesTowardNewCurveDirectionNotStaleAbsolute) {
    const int N = 2;
    const std::vector<MPCControl> warm_start = {
        MakeControl(-0.10), MakeControl(-0.12)};
    const std::vector<double> warm_start_ff = {-0.10, -0.10};
    const std::vector<double> current_ff = {0.20, 0.20};

    ASSERT_TRUE(WarmStartFeedforwardIsValid(warm_start, warm_start_ff, N));
    const std::vector<double> rebased =
        RebaseSteeringWarmStart(warm_start, warm_start_ff, current_ff, N);

    // residual = -0.12 - (-0.10) = -0.02
    // rebased[i] = 0.20 + (-0.02) = 0.18
    for (double v : rebased) {
        EXPECT_NEAR(0.18, v, 1e-9);
        // 핵심 불변: 이전 절대 steering(-0.10/-0.12, 음수)에 갇히지 않고
        // 새 reference의 방향(양수)을 따라간다.
        EXPECT_GT(v, 0.0);
    }
}

// TEST D: 크기 불일치/무효 상태에서는 안전하게 fallback(rebase 시도 자체를
// 하지 않아야 한다는 계약)을 판단할 수 있어야 한다 -
// WarmStartFeedforwardIsValid가 false를 반환해야 solveMPC가 기존 경로로
// 폴백한다.
TEST(WarmStartRebaseTest, InvalidPairingFailsSafeToFallback) {
    const int N = 3;
    const std::vector<MPCControl> warm_start = {
        MakeControl(0.1), MakeControl(0.1), MakeControl(0.1)};

    // 크기 불일치.
    const std::vector<double> mismatched_ff = {0.1, 0.1};
    EXPECT_FALSE(WarmStartFeedforwardIsValid(warm_start, mismatched_ff, N));

    // warm_start 자체가 horizon보다 짧음.
    const std::vector<MPCControl> short_warm_start = {MakeControl(0.1)};
    const std::vector<double> short_ff = {0.1};
    EXPECT_FALSE(WarmStartFeedforwardIsValid(short_warm_start, short_ff, N));

    // non-finite 값.
    const std::vector<double> nan_ff = {
        0.1, std::numeric_limits<double>::quiet_NaN(), 0.1};
    EXPECT_FALSE(WarmStartFeedforwardIsValid(warm_start, nan_ff, N));

    const std::vector<MPCControl> nan_warm_start = {
        MakeControl(0.1), MakeControl(std::numeric_limits<double>::infinity()),
        MakeControl(0.1)};
    const std::vector<double> valid_ff = {0.1, 0.1, 0.1};
    EXPECT_FALSE(WarmStartFeedforwardIsValid(nan_warm_start, valid_ff, N));
}

// TEST E: rebase 결과(투영 이전) 자체는 물리 제약을 모른다 - 이 함수는
// 순수 산술이며, 물리 제약 투영은 solveMPC의 기존 projectControls가
// 담당한다(clipControl/clipSteeringRate 재사용, 중복 구현 없음). 여기서는
// "물리 제약을 초과하는 rebase 결과가 나올 수 있다"는 사실 자체를
// 확인해서, 투영이 반드시 별도로 필요하다는 계약을 명시적으로 남긴다.
TEST(WarmStartRebaseTest, RebasedValueBeforeProjectionCanExceedPhysicalLimits) {
    const int N = 1;
    const std::vector<MPCControl> warm_start = {MakeControl(0.5)};
    const std::vector<double> warm_start_ff = {0.0};
    // current_ff가 매우 크면(비현실적인 curvature) rebase 결과가 통상적인
    // steering_max(예: 0.63268)를 넘을 수 있다 - 그래서 solveMPC는 이 결과를
    // projectControls에 반드시 통과시킨다.
    const std::vector<double> current_ff = {2.0};

    ASSERT_TRUE(WarmStartFeedforwardIsValid(warm_start, warm_start_ff, N));
    const std::vector<double> rebased =
        RebaseSteeringWarmStart(warm_start, warm_start_ff, current_ff, N);

    constexpr double kTypicalSteeringMax = 0.63268;
    EXPECT_GT(std::fabs(rebased[0]), kTypicalSteeringMax);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
