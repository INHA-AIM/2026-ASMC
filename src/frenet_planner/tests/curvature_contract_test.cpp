// Frenet/MPC 곡률 계약(curvature contract) 회귀 (curve-tracking stability 작업).
//
// 배경: MORAI 실측에서 Frenet의 max_curvature(과거 0.28)가 MPC의 물리
// 조향 한계(kappa_max = tan(steering_max)/wheelbase ~= 0.2444, mpc_params.yaml
// 기준)보다 커서, Frenet이 "생성 가능"으로 받아들인 trajectory를 MPC
// reference_validator가 CURVATURE_OUT_OF_RANGE로 거부해 fail-safe STOP이
// 발생했다.
//
// 이 테스트는 두 계약을 검증한다:
//   1. production params.yaml의 max_curvature가 MPC의 물리 조향 한계
//      (mpc_params.yaml의 steering_max/wheelbase로 계산)를 넘지 않는다.
//   2. 그 max_curvature로 곡률 필터를 통과한 candidate는 실제로 그 한계
//      안에 있다(FilterByCurvature가 production 그대로 동작한다).
//
// 두 패키지 사이에 직접 코드 의존성을 추가하지 않는다 - 각 패키지의 실제
// 설정 파일을 읽어 독립적으로 비교한다(공유 상수/공용 헤더를 새로 만들지
// 않는다). production FilterByCurvature/ComputeGeometricPath를 그대로
// 호출한다 - 로직을 복사하지 않는다.

#include "frenet_planner/Evaluation/KinematicFilter.hpp"
#include "frenet_planner/Math/FrenetConverter.hpp"
#include "frenet_planner/Reference/Reference.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

// params.yaml/mpc_params.yaml은 일반 YAML 파서 의존성 없이 두 패키지 모두
// 단순 "key: value" 라인 스캔으로 읽는다(ParameterLoader.cpp/
// parameter_loader.cpp와 동일한 최소 파싱 방식) - 여기서는 이 테스트가
// 필요로 하는 스칼라 몇 개만 뽑는다.
bool FindScalarAfterKey(const std::string& path, const std::string& key,
                        double& out_value) {
    std::ifstream in(path);
    if (!in) return false;
    std::string line;
    while (std::getline(in, line)) {
        const auto pos = line.find(key + ":");
        if (pos == std::string::npos) continue;
        // 들여쓰기가 다른 동명 key(예: 다른 section의 steering_max)를
        // 걸러내지 않는다 - 이 테스트가 읽는 두 파일에는 각 key가 한 번씩만
        // 나온다(steering.max_angle, mpc.wheelbase,
        // kinematic_limits.max_curvature).
        std::istringstream rest(line.substr(pos + key.size() + 1));
        // 인라인 주석(#...) 제거.
        std::string token;
        rest >> token;
        const auto comment_pos = token.find('#');
        if (comment_pos != std::string::npos) token = token.substr(0, comment_pos);
        try {
            out_value = std::stod(token);
            return true;
        } catch (...) {
            continue;
        }
    }
    return false;
}

}  // namespace

// 1. production 설정 파일 기준 Frenet max_curvature <= MPC 물리 조향 한계.
void TestFrenetMaxCurvatureDoesNotExceedMpcPhysicalLimit() {
    const std::string frenet_config =
        std::string(FRENET_PACKAGE_SRC_DIR) + "/../config/params.yaml";
    const std::string mpc_config = std::string(MPC_PACKAGE_SRC_DIR) +
        "/config/mpc_params.yaml";

    double frenet_max_curvature = 0.0;
    Expect(FindScalarAfterKey(frenet_config, "max_curvature",
                              frenet_max_curvature),
           "setup: production params.yaml에서 max_curvature를 읽는다");

    double steering_max = 0.0;
    double wheelbase = 0.0;
    Expect(FindScalarAfterKey(mpc_config, "max_angle", steering_max),
           "setup: production mpc_params.yaml에서 steering.max_angle을 "
           "읽는다");
    Expect(FindScalarAfterKey(mpc_config, "wheelbase", wheelbase),
           "setup: production mpc_params.yaml에서 wheelbase를 읽는다");

    // MPC reference_validator.cpp와 동일한 공식
    // (kappa_max = tan(steering_max)/wheelbase) - 로직을 복사하지 않고 같은
    // 수식만 재사용한다. 두 패키지 사이에 코드 의존성을 추가하지 않기 위해
    // 공식 자체는 각자 갖되, 그 결과를 여기서 비교한다.
    const double mpc_kappa_max = std::tan(steering_max) / wheelbase;

    std::cout << "  frenet max_curvature = " << frenet_max_curvature
              << ", mpc physical kappa_max = " << mpc_kappa_max << '\n';
    Expect(frenet_max_curvature <= mpc_kappa_max,
           "1: Frenet max_curvature does not exceed the MPC physical "
           "steering-derived curvature limit");
}

// 2. 그 max_curvature로 만든 RefLine 위에서 FilterByCurvature를 통과한
//    candidate는 실제로 그 한계 안에 있다 - candidate feasibility 계약이
//    실제로 지켜지는지 production 함수로 직접 확인한다.
void TestFilteredCandidateRespectsConfiguredCurvatureLimit() {
    const std::string frenet_config =
        std::string(FRENET_PACKAGE_SRC_DIR) + "/../config/params.yaml";
    double max_curvature = 0.0;
    Expect(FindScalarAfterKey(frenet_config, "max_curvature", max_curvature),
           "setup: production params.yaml에서 max_curvature를 다시 읽는다");

    // 원 궤적 근사: 반지름 R=1/max_curvature보다 살짝 작은 원(허용치를
    // 살짝 넘는 곡률)을 그려 실제로 걸러지는지, 살짝 큰 원(허용치 안)은
    // 통과하는지 함께 확인한다.
    auto MakeCircleRef = [](double radius, int n_points) {
        RefLine ref;
        const double step = 2.0 * M_PI * radius / n_points;
        for (int i = 0; i <= n_points; ++i) {
            const double theta = static_cast<double>(i) / n_points * 2.0 * M_PI;
            RefPoint p{};
            p.x = radius * std::cos(theta);
            p.y = radius * std::sin(theta);
            p.s = step * i;
            p.theta = theta + M_PI / 2.0;
            p.kappa = 1.0 / radius;
            p.d_kappa = 0.0;
            ref.points.push_back(p);
        }
        ref.closed_loop = true;
        ref.length = ref.points.empty() ? 0.0 : ref.points.back().s;
        return ref;
    };

    KinematicLimits limits{};
    limits.max_lateral_accel = 3.0;
    limits.max_longitudinal_accel = 3.0;
    limits.max_curvature = max_curvature;
    limits.max_longitudinal_jerk = 6.0;

    // d=0(참조선 그대로 추종)인 단일 후보를 만든다 - ComputeGeometricPath가
    // 참조선의 기하 곡률을 그대로 반영하는지가 이 테스트의 핵심이다.
    auto MakeCenterlineCandidate = [](const RefLine& ref) {
        FrenetPath path;
        path.valid = true;
        const std::size_t n = ref.points.size();
        path.s.resize(n);
        path.d.assign(n, 0.0);
        for (std::size_t i = 0; i < n; ++i) path.s[i] = ref.points[i].s;
        return path;
    };

    {
        // 허용 한계보다 살짝 넘치는 곡률(반지름을 살짝 작게) -> 거부되어야
        // 한다. 점 간격은 FrenetConverter.cpp의 kMinSegmentLength(0.15m)보다
        // 충분히 크게 둔다 - 그보다 촘촘하면 저속 클러스터 판정으로 kappa가
        // 0(판단 보류)으로 남아 이 테스트가 무의미해진다.
        const double infeasible_radius = 1.0 / max_curvature * 0.95;
        RefLine ref = MakeCircleRef(infeasible_radius, 100);
        std::vector<FrenetPath> candidates = {MakeCenterlineCandidate(ref)};
        FilterByCurvature(candidates, ref, limits);
        Expect(!candidates[0].valid,
               "2a: max_curvature보다 큰 기하 곡률을 갖는 candidate는 "
               "FilterByCurvature가 거부한다");
    }
    {
        // 허용 한계보다 여유 있는 곡률(반지름을 넉넉히 크게) -> 통과해야
        // 한다.
        const double feasible_radius = 1.0 / max_curvature * 1.5;
        RefLine ref = MakeCircleRef(feasible_radius, 100);
        std::vector<FrenetPath> candidates = {MakeCenterlineCandidate(ref)};
        FilterByCurvature(candidates, ref, limits);
        Expect(candidates[0].valid,
               "2b: max_curvature 안의 기하 곡률을 갖는 candidate는 "
               "FilterByCurvature를 통과한다");
    }
}

int main() {
    TestFrenetMaxCurvatureDoesNotExceedMpcPhysicalLimit();
    TestFilteredCandidateRespectsConfiguredCurvatureLimit();

    if (g_failures == 0) {
        std::cout << "[ALL PASS]\n";
        return 0;
    }
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
}
