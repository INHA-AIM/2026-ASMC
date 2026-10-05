#pragma once

#include <cstddef>

#include "common/mpc_params.hpp"
#include "common/reference_path.hpp"

// ReferencePath 입력 유효성 검증 - Reference/input-preparation 계층
// (message conversion 이후, MpcController::ControlProcess 이전)에 위치한다.
//
//   Trajectory -> PlannedPath -> ToReferencePath -> ReferencePath
//                                                        │
//                                                  ValidateReference (여기)
//                                                        │
//                                              OK -> ControlProcess
//                                              NG -> 기존 fail-safe STOP
//
// 이 계층은 "물리적으로 말이 되는 참조인가"만 본다 - 경로 품질/비용은 다루지
// 않는다(그건 cost_function의 책임). solver/vehicle_model/cost_function은
// 이 계층에 의존하지 않는다 - upstream planner 데이터가 이미 정상이라고
// 가정하고 최적화만 한다. ROS 비의존(ros::/asmc_msgs::/morai_msgs:: 없음) -
// roscore 없이 단위 테스트할 수 있다.
enum class ReferenceValidationStatus {
    OK = 0,
    EMPTY,
    SIZE_MISMATCH,
    NON_FINITE,
    VELOCITY_OUT_OF_RANGE,
    CURVATURE_OUT_OF_RANGE,
    POSITION_DISCONTINUITY,
};

const char* ReferenceValidationStatusName(ReferenceValidationStatus status);

struct ReferenceValidationResult {
    ReferenceValidationStatus status = ReferenceValidationStatus::EMPTY;
    // 실패를 유발한 첫 인덱스. 진단/테스트용이며 OK일 때는 의미가 없다.
    std::size_t failed_index = 0;

    bool ok() const { return status == ReferenceValidationStatus::OK; }
};

// params가 소유한 물리 한계만 사용한다 - 별도 vehicle-max 상수를 새로 만들지
// 않는다:
//   속도 범위       params.vel_min / params.vel_max
//   조향/축거 기반 곡률 한계  atan/tan(params.steering_max) / params.wheelbase
//   위치 연속성 상한  params.vel_max * params.dt + 작은 보간 여유
// a_ref는 최종 MPC time grid의 v_ref 미분값이므로 size/finiteness는 검증하되
// actuator 범위로 reference 정보를 잘라내거나 거부하지 않는다. cost와 fresh
// solver initialization에서 params.accel_min/max로 clip한 값이 authoritative다.
ReferenceValidationResult ValidateReference(const ReferencePath& reference,
                                            const MPCParams& params);
