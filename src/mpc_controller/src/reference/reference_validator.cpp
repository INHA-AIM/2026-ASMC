#include "reference/reference_validator.hpp"

#include <cmath>

#include "math/utils.hpp"

namespace {

bool AllFinite(double a, double b, double c, double d, double e) {
    return std::isfinite(a) && std::isfinite(b) && std::isfinite(c) &&
           std::isfinite(d) && std::isfinite(e);
}

}  // namespace

const char* ReferenceValidationStatusName(ReferenceValidationStatus status) {
    switch (status) {
        case ReferenceValidationStatus::OK: return "OK";
        case ReferenceValidationStatus::EMPTY: return "EMPTY";
        case ReferenceValidationStatus::SIZE_MISMATCH: return "SIZE_MISMATCH";
        case ReferenceValidationStatus::NON_FINITE: return "NON_FINITE";
        case ReferenceValidationStatus::VELOCITY_OUT_OF_RANGE:
            return "VELOCITY_OUT_OF_RANGE";
        case ReferenceValidationStatus::CURVATURE_OUT_OF_RANGE:
            return "CURVATURE_OUT_OF_RANGE";
        case ReferenceValidationStatus::POSITION_DISCONTINUITY:
            return "POSITION_DISCONTINUITY";
    }
    return "UNKNOWN";
}

ReferenceValidationResult ValidateReference(const ReferencePath& reference,
                                            const MPCParams& params) {
    ReferenceValidationResult result;

    // 1. 배열 일관성. ReferencePath::size()는 x_ref.size()만 본다 - 나머지
    // 배열이 어긋나면 이후 인덱싱이 조용히 다른 스텝의 값을 섞어 쓴다.
    if (reference.empty()) {
        result.status = ReferenceValidationStatus::EMPTY;
        return result;
    }
    const std::size_t n = reference.size();
    if (reference.y_ref.size() != n || reference.yaw_ref.size() != n ||
        reference.v_ref.size() != n || reference.k_ref.size() != n) {
        result.status = ReferenceValidationStatus::SIZE_MISMATCH;
        return result;
    }

    // 물리 한계는 params가 소유한 값만 쓴다 - 새 vehicle-max 상수를 만들지
    // 않는다.
    //
    // 곡률 한계: kinematic bicycle model의 정상상태 관계
    // (SteeringFeedforward의 역방향, model/vehicle_model.hpp 참고)
    //   tan(delta) = L * kappa  ->  kappa_max = tan(steering_max) / L
    // Frenet planner의 0.1704 곡률 한계를 복사하지 않는다 - 그건 planner
    // feasibility 정책이고, 여기는 컨트롤러가 실제로 조향할 수 있는 물리
    // 한계다.
    constexpr double kEps = 1e-6;
    const double kappa_max =
        std::tan(params.steering_max) / std::max(params.wheelbase, kEps);

    // 위치 연속성 한계: 인접 ReferencePath 샘플은 항상 정확히 params.dt만큼
    // 시간 간격이 벌어진다(path_resampler.cpp의 균일 리샘플링 계약). 한 스텝
    // 안에서 물리적으로 가능한 최대 이동거리는 vel_max * dt다. 곡선 구간의
    // 호길이 vs 직선 거리 차이, 부동소수/보간 오차를 흡수하도록 작은 배수
    // 여유를 둔다 - 이 여유는 임의로 큰 수가 아니라 "실제 최대 이동거리의
    // 50%를 더 허용"이라는 명시적 비율이다.
    constexpr double kPositionMarginFactor = 1.5;
    const double max_step_distance =
        params.vel_max * params.dt * kPositionMarginFactor;

    // 속도 범위: params.vel_min/vel_max에 작은 수치 여유만 더한다. 후진
    // 주행을 architecture가 지원하지 않으므로(현재 production trajectory는
    // 전진 전용, interfaces 계약과 동일) vel_min 미만은 그대로 거부한다.
    constexpr double kVelocityEps = 1e-3;
    const double vel_lo = params.vel_min - kVelocityEps;
    const double vel_hi = params.vel_max + kVelocityEps;

    double prev_x = 0.0, prev_y = 0.0;
    bool has_prev = false;

    for (std::size_t i = 0; i < n; ++i) {
        const double x = reference.x_ref[i];
        const double y = reference.y_ref[i];
        const double yaw = reference.yaw_ref[i];
        const double v = reference.v_ref[i];
        const double k = reference.k_ref[i];

        if (!AllFinite(x, y, yaw, v, k)) {
            result.status = ReferenceValidationStatus::NON_FINITE;
            result.failed_index = i;
            return result;
        }
        if (v < vel_lo || v > vel_hi) {
            result.status = ReferenceValidationStatus::VELOCITY_OUT_OF_RANGE;
            result.failed_index = i;
            return result;
        }
        if (std::fabs(k) > kappa_max + kEps) {
            result.status = ReferenceValidationStatus::CURVATURE_OUT_OF_RANGE;
            result.failed_index = i;
            return result;
        }
        if (has_prev) {
            const double dx = x - prev_x;
            const double dy = y - prev_y;
            const double distance = std::hypot(dx, dy);
            if (distance > max_step_distance) {
                result.status = ReferenceValidationStatus::POSITION_DISCONTINUITY;
                result.failed_index = i;
                return result;
            }
        }
        prev_x = x;
        prev_y = y;
        has_prev = true;
    }

    result.status = ReferenceValidationStatus::OK;
    return result;
}
