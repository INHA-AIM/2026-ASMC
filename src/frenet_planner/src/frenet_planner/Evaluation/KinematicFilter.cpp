#include "frenet_planner/Evaluation/KinematicFilter.hpp"

#include <cmath>

#include "frenet_planner/Math/FrenetConverter.hpp"

void FilterLateralByAcceleration(std::vector<FrenetPath>& candidates,
                                  const KinematicLimits& limits) {
    for (auto& path : candidates) {
        if (!path.valid) continue;
        for (double dd : path.d_dd) {
            if (std::abs(dd) > limits.max_lateral_accel) {
                path.valid = false;
                break;
            }
        }
    }
}

void FilterLongitudinalByKinematicLimits(std::vector<FrenetPath>& candidates,
                                          const KinematicLimits& limits) {
    // 부동소수점 연산으로 발생하는 -0 수준의 오차를 실제 reverse로 오인하지
    // 않기 위한 허용폭. 실제 reverse 판정에는 영향을 주지 않을 만큼 작다.
    constexpr double kReverseVelocityEpsilon = 1e-6;

    for (auto& path : candidates) {
        if (!path.valid) continue;
        for (double sdd : path.s_dd) {
            if (std::abs(sdd) > limits.max_longitudinal_accel) {
                path.valid = false;
                break;
            }
        }
        if (!path.valid) continue;
        for (double sd : path.s_d) {
            if (sd < -kReverseVelocityEpsilon) {
                path.valid = false;
                break;
            }
        }
        if (!path.valid) continue;
        for (double sddd : path.s_ddd) {
            if (std::abs(sddd) > limits.max_longitudinal_jerk) {
                path.valid = false;
                break;
            }
        }
    }
}

void FilterByCurvature(std::vector<FrenetPath>& combined,
                        const RefLine& ref,
                        const KinematicLimits& limits) {
    // 시간미분->호길이미분(TimeDerivToArcDeriv, s_dot로 나눔) 기반 계산은 저속에서
    // 발산해 STOP/EMERGENCY처럼 정지에 수렴하는 후보를 전부 무효화시켰다
    // (FrenetConverter.hpp의 ComputeGeometricPath 설계 노트 참고). 위치(x,y)
    // 기반 기하학적 곡률로 대체해 고속/저속 구분 없이 일관되게 판정한다.
    for (auto& path : combined) {
        if (!path.valid) continue;

        GeometricPath geo = ComputeGeometricPath(path.s, path.d, ref);
        for (double k : geo.kappa) {
            if (std::abs(k) > limits.max_curvature) {
                path.valid = false;
                path.rejection_reason = RejectionReason::CURVATURE;
                break;
            }
        }
    }
}

// 재생성된 후보의 실제 샘플별 속도²×곡률을 검증한다. 단순히
// 기하 곡률만 작은 경로가 아니라, 해당 속도에서 실제로 횡가속도
// 한계를 만족하는 경로만 남긴다.
void FilterByCartesianLateralAcceleration(std::vector<FrenetPath>& candidates,
                                          const RefLine& ref,
                                          double max_lateral_accel) {
    constexpr double kLateralAccelMargin = 0.8;
    const double limit = max_lateral_accel * kLateralAccelMargin;
    for (auto& path : candidates) {
        if (!path.valid) continue;
        const GeometricPath geo = ComputeGeometricPath(path.s, path.d, ref);
        const size_t count = std::min(geo.kappa.size(), path.s_d.size());
        for (size_t i = 0; i < count; ++i) {
            const double lateral_accel = path.s_d[i] * path.s_d[i] * std::fabs(geo.kappa[i]);
            if (lateral_accel > limit) {
                path.valid = false;
                path.rejection_reason = RejectionReason::CURVATURE;
                break;
            }
        }
    }
}
