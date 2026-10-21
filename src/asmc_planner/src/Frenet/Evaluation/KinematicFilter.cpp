#include "Frenet/Evaluation/KinematicFilter.hpp"

#include <cmath>
#include <algorithm>

#include "Frenet/Math/FrenetConverter.hpp"

double SteeringProfileRateLimit(double speed,const SteeringProfileLimits& limits) {
    if (limits.reference_speed<=0 || std::fabs(speed)<=limits.reference_speed)
        return limits.max_rate;
    const double ratio=limits.reference_speed/std::fabs(speed);
    return std::clamp(limits.max_rate*ratio*ratio,limits.min_rate,limits.max_rate);
}

bool PathObeysSteeringRate(const CartesianPath& path,double dt,double wheelbase,
                          const SteeringProfileLimits& limits) {
    if (path.kappa.size()<2 || path.v.size()!=path.kappa.size() ||
        !std::isfinite(dt) || dt<=0 || !std::isfinite(wheelbase) || wheelbase<=0 ||
        !std::isfinite(limits.curvature_gain) || limits.curvature_gain<=0 ||
        !std::isfinite(limits.max_rate) || limits.max_rate<=0 ||
        !std::isfinite(limits.min_rate) || limits.min_rate<=0 || limits.min_rate>limits.max_rate ||
        !std::isfinite(limits.reference_speed) || limits.reference_speed<0) return false;
    for(std::size_t i=0;i<path.kappa.size();++i) {
        if(!std::isfinite(path.kappa[i]) || !std::isfinite(path.v[i]))return false;
        if(!i)continue;
        // Controller interpolates curvature linearly. Bound the derivative
        // of atan(L*k/gain) throughout that interval, including a zero crossing
        // where the peak can exceed the endpoint angle difference / dt.
        const double before=path.kappa[i-1],after=path.kappa[i];
        const bool crosses_zero=(before<=0&&after>=0)||(before>=0&&after<=0);
        const double nearest=crosses_zero?0:std::min(std::fabs(before),std::fabs(after));
        const double scale=wheelbase/limits.curvature_gain;
        const double peak_rate=scale*std::fabs(after-before)/
            (dt*(1+scale*scale*nearest*nearest));
        const double speed=std::max(std::fabs(path.v[i-1]),std::fabs(path.v[i]));
        if(!std::isfinite(peak_rate)||peak_rate>SteeringProfileRateLimit(speed,limits)+1e-9)return false;
    }
    return true;
}

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
                        const KinematicLimits& limits,
                        CartesianGeometryMode geometry_mode) {
    // 반드시 최종 발행 경로와 같은 변환 정책을 사용한다. 과거에는 여기서
    // ComputeGeometricPath의 kappa를 검사한 뒤 ConvertToCartesianPath가 정상
    // 속도 샘플의 kappa를 analytic 값으로 바꾸었다. 그 결과 Frenet 필터를
    // 통과한 후보가 MPC 입력에서는 조향 한계를 넘을 수 있었다.
    for (auto& path : combined) {
        if (!path.valid) continue;

        const CartesianPath emitted =
            ConvertToCartesianPath(path, ref, geometry_mode);
        for (double k : emitted.kappa) {
            if (!std::isfinite(k) || std::abs(k) > limits.max_curvature) {
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
                                          double max_lateral_accel,
                                          double utilization) {
    const double limit = max_lateral_accel *
        std::max(0.0, std::min(utilization, 1.0));
    for (auto& path : candidates) {
        if (!path.valid) continue;
        const CartesianPath emitted = ConvertToCartesianPath(path, ref);
        const size_t count = std::min(emitted.kappa.size(), emitted.v.size());
        for (size_t i = 0; i < count; ++i) {
            const double lateral_accel =
                emitted.v[i] * emitted.v[i] * std::fabs(emitted.kappa[i]);
            if (!std::isfinite(lateral_accel) || lateral_accel > limit) {
                path.valid = false;
                path.rejection_reason = RejectionReason::CURVATURE;
                break;
            }
        }
    }
}
