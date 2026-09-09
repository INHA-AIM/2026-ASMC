#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include "diagnostics/tracking_diagnostics.hpp"

namespace {

MPCParams MakeTestParams() {
    MPCParams params;
    params.horizon = 4;
    params.dt = 0.1;
    params.wheelbase = 3.0;
    params.steering_max = 0.6;
    params.steering_rate_max = 1.0;
    return params;
}

ReferencePath MakeReference(std::size_t n, double kappa_value) {
    ReferencePath ref;
    for (std::size_t i = 0; i < n; ++i) {
        ref.x_ref.push_back(static_cast<double>(i));
        ref.y_ref.push_back(0.0);
        ref.yaw_ref.push_back(0.0);
        ref.v_ref.push_back(5.0);
        ref.k_ref.push_back(kappa_value);
    }
    return ref;
}

}  // namespace

// TEST: ReferenceIndexAtTime는 mpc_controller_node.cpp가 예전에 쓰던 것과
// 같은 계약이다(round(time/dt), horizon 밖은 마지막 유효 인덱스로 clamp).
// 이 회귀는 [MPC-TRACK]과 MPC-D1이 같은 함수를 공유한다는 것을 확인한다 -
// 인덱스 계산을 복제하지 않는다는 계약의 실행 가능한 증거.
TEST(TrackingDiagnosticsTest, ReferenceIndexAtTimeMatchesRoundAndClampContract) {
    MPCParams params = MakeTestParams();  // dt=0.1
    EXPECT_EQ(0u, ReferenceIndexAtTime(0.0, params, 10));
    EXPECT_EQ(2u, ReferenceIndexAtTime(0.2, params, 10));
    EXPECT_EQ(5u, ReferenceIndexAtTime(0.5, params, 10));
    // horizon 밖 시간 -> 마지막 유효 인덱스로 clamp.
    EXPECT_EQ(9u, ReferenceIndexAtTime(5.0, params, 10));
    // 빈 reference -> 0 (호출자가 별도로 empty 체크하지 않아도 안전).
    EXPECT_EQ(0u, ReferenceIndexAtTime(0.5, params, 0));
}

// TEST: preview indexing이 horizon보다 훨씬 짧은 reference에서도
// out-of-bounds 접근 없이 안전해야 한다(§16 필수 항목).
TEST(TrackingDiagnosticsTest, ShortReferenceDoesNotGoOutOfBounds) {
    MPCParams params = MakeTestParams();
    TrackingDiagnosticInputs in;
    in.params = params;
    // horizon(4)보다 짧은 reference(2 포인트) - 1.0s 미리보기 인덱스가
    // reference 밖을 가리킬 수 있는 조건.
    in.reference = MakeReference(2, 0.05);

    TrackingDiagnosticSample sample;
    ASSERT_NO_THROW(sample = BuildTrackingDiagnosticSample(in));
    EXPECT_TRUE(std::isfinite(sample.kappa_ref_0s));
    EXPECT_TRUE(std::isfinite(sample.kappa_ref_10s));
    EXPECT_TRUE(std::isfinite(sample.kappa_ref_peak));
}

// TEST: reference가 완전히 비어 있어도 크래시하지 않고 0으로 안전하게
// 남아야 한다(솔버가 이미 이 경우 fail-safe STOP으로 처리하지만, 진단
// 코드 자체도 독립적으로 안전해야 한다).
TEST(TrackingDiagnosticsTest, EmptyReferenceIsSafe) {
    TrackingDiagnosticInputs in;
    in.params = MakeTestParams();
    in.reference = ReferencePath{};  // empty

    TrackingDiagnosticSample sample;
    ASSERT_NO_THROW(sample = BuildTrackingDiagnosticSample(in));
    EXPECT_DOUBLE_EQ(0.0, sample.kappa_ref_0s);
    EXPECT_DOUBLE_EQ(0.0, sample.ff_0s);
}

// TEST: 타임스탬프가 기본값(0, ROS의 "unset" 규약)이어도 age 계산이
// 유한한 값으로 안전하게 나와야 한다 - NaN/inf/크래시 없음.
TEST(TrackingDiagnosticsTest, UnsetTimestampsProduceFiniteAges) {
    TrackingDiagnosticInputs in;
    in.params = MakeTestParams();
    in.reference = MakeReference(4, 0.0);
    in.ros_time_now_s = 1000.0;
    in.vehicle_state_stamp_s = 0.0;  // "unset" sentinel
    in.trajectory_stamp_s = 0.0;
    in.trajectory_vehicle_time_delta_s = 0.0;

    const TrackingDiagnosticSample sample = BuildTrackingDiagnosticSample(in);
    EXPECT_TRUE(std::isfinite(sample.vehicle_state_age_s));
    EXPECT_TRUE(std::isfinite(sample.trajectory_age_s));
    EXPECT_DOUBLE_EQ(1000.0, sample.vehicle_state_age_s);
    EXPECT_DOUBLE_EQ(1000.0, sample.trajectory_age_s);
}

// TEST: 진단 계산은 입력을 읽기만 한다 - MpcController::Output/ReferencePath/
// MPCControl을 값으로 받아서 그대로 두 번 계산해도 같은 결과가 나와야
// 하고(부작용 없음), 특히 mpc_output 필드들이 그대로 옮겨지는지 확인한다
// (§16: "새 진단 추출이 solver 출력을 바꾸지 않는다"의 실행 가능한 증거 -
// 여기서는 "입력이 그대로 출력에 반영되고 변형되지 않는다"로 검증한다).
TEST(TrackingDiagnosticsTest, WarmStartFieldsArePassedThroughUnmodified) {
    TrackingDiagnosticInputs in;
    in.params = MakeTestParams();
    in.reference = MakeReference(4, 0.02);
    in.mpc_output.used_curvature_rebase = true;
    in.mpc_output.warm_raw0 = 0.11;
    in.mpc_output.prev_ff_shift0 = 0.07;
    in.mpc_output.warm_residual0 = 0.04;
    in.mpc_output.current_ff0 = 0.09;
    in.mpc_output.warm_rebased0 = 0.13;
    in.mpc_output.warm_projected0 = 0.12;
    in.mpc_output.prev_applied_delta = 0.10;
    in.published_control.delta = 0.115;

    const TrackingDiagnosticSample sample = BuildTrackingDiagnosticSample(in);
    EXPECT_TRUE(sample.warm_used_rebase);
    EXPECT_DOUBLE_EQ(0.11, sample.warm_raw_delta0);
    EXPECT_DOUBLE_EQ(0.07, sample.warm_prev_ff_shift0);
    EXPECT_DOUBLE_EQ(0.04, sample.warm_residual0);
    EXPECT_DOUBLE_EQ(0.09, sample.warm_current_ff0);
    EXPECT_DOUBLE_EQ(0.13, sample.warm_rebased_delta0);
    EXPECT_DOUBLE_EQ(0.12, sample.warm_projected_delta0);

    // optimized_projected_delta0는 published_control.delta를 그대로
    // 반영한다 - 현재 solveMPC 구현에서 최적화 결과와 투영 결과가 이미
    // 같은 값이라는 것을 문서화한 계약(§7-F)의 실행 가능한 증거.
    EXPECT_DOUBLE_EQ(0.115, sample.optimized_projected_delta0);
    // delta0_minus_prev_delta = published - prev_applied.
    EXPECT_NEAR(0.015, sample.delta0_minus_prev_delta, 1e-9);
}

// TEST: published_steer_cmd는 ToRosControlCommand(message_converter.cpp)와
// 정확히 같은 clip(±steering_max)을 재현해야 한다 - steering_max를 넘는
// published_control.delta가 들어와도(이론상 solver가 이미 이걸 막지만,
// 진단 코드 자체가 독립적으로 안전해야 한다) clip된 값이 나온다.
TEST(TrackingDiagnosticsTest, PublishedSteerCmdReproducesProductionClip) {
    TrackingDiagnosticInputs in;
    in.params = MakeTestParams();  // steering_max = 0.6
    in.reference = MakeReference(4, 0.0);
    in.published_control.delta = 10.0;  // 비현실적으로 큰 값, clip 확인용

    const TrackingDiagnosticSample sample = BuildTrackingDiagnosticSample(in);
    EXPECT_DOUBLE_EQ(0.6, sample.published_steer_cmd);
}

// TEST: steer_rate_limit_active는 delta 변화가 max_rate*dt에 가까울 때만
// true여야 한다(constraints.cpp의 clipSteeringRate와 같은 한계값 재사용,
// 새 클램프 아님).
TEST(TrackingDiagnosticsTest, SteerRateLimitActiveReflectsRateBoundProximity) {
    TrackingDiagnosticInputs in;
    in.params = MakeTestParams();  // steering_rate_max=1.0, dt=0.1 -> max_step=0.1
    in.reference = MakeReference(4, 0.0);

    in.published_control.delta = 0.001;
    in.mpc_output.prev_applied_delta = 0.0;
    TrackingDiagnosticSample small_change = BuildTrackingDiagnosticSample(in);
    EXPECT_FALSE(small_change.steer_rate_limit_active);

    in.published_control.delta = 0.1;  // == max_step 정확히
    TrackingDiagnosticSample at_limit = BuildTrackingDiagnosticSample(in);
    EXPECT_TRUE(at_limit.steer_rate_limit_active);
}

// TEST: CSV writer는 열 수 없는 경로에서도 실패를 삼키고 false를 반환해야
// 한다 - 컨트롤 루프를 막지 않는다는 계약(§12). 디렉터리 자체를 파일
// 경로로 주면(fopen이 디렉터리를 쓰기 모드로 열 수 없음) root 권한과
// 무관하게 항상 실패한다 - 테스트가 uid=0(컨테이너 기본)에서 돌아도
// 안정적으로 재현된다.
TEST(TrackingDiagnosticsTest, CsvWriterFailsSafelyOnUnwritablePath) {
    TrackingDiagnosticCsvWriter writer;
    const bool opened = writer.Open("/tmp");  // 디렉터리, 파일 아님
    EXPECT_FALSE(opened);
    EXPECT_FALSE(writer.is_open());

    TrackingDiagnosticSample sample;
    // Open이 실패했으니 WriteRow도 크래시 없이 false를 반환해야 한다.
    EXPECT_FALSE(writer.WriteRow(sample));
}

// TEST: 정상 경로에서는 헤더가 한 번만 쓰이고, 데이터 행이 이어 붙는다.
TEST(TrackingDiagnosticsTest, CsvWriterWritesHeaderOnceThenAppends) {
    const std::string path = "/tmp/mpc_tracking_diagnostics_test.csv";
    std::remove(path.c_str());

    {
        TrackingDiagnosticCsvWriter writer;
        ASSERT_TRUE(writer.Open(path));
        TrackingDiagnosticSample sample;
        sample.ego_speed = 3.5;
        ASSERT_TRUE(writer.WriteRow(sample));
    }
    {
        // 재오픈(재시작 시뮬레이션) - 헤더를 다시 쓰지 않아야 한다.
        TrackingDiagnosticCsvWriter writer;
        ASSERT_TRUE(writer.Open(path));
        TrackingDiagnosticSample sample;
        sample.ego_speed = 4.5;
        ASSERT_TRUE(writer.WriteRow(sample));
    }

    std::FILE* f = std::fopen(path.c_str(), "r");
    ASSERT_NE(nullptr, f);
    int header_count = 0;
    char line[2048];
    while (std::fgets(line, sizeof(line), f)) {
        if (std::string(line).rfind("ros_time,", 0) == 0) ++header_count;
    }
    std::fclose(f);
    EXPECT_EQ(1, header_count);
    std::remove(path.c_str());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
