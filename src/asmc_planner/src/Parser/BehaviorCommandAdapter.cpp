#include "Parser/BehaviorCommandAdapter.hpp"

#include <cmath>

namespace frenet_planner_ros {

namespace {

using Command = asmc_msgs::BehaviorCommand;
using Feedback = asmc_msgs::FrenetFeedback;

// quaternion 노름이 1에서 벗어나도 허용하는 폭. 전송 중 양자화를 고려해
// 넉넉하게 둔다 - 정규화되지 않은 쓰레기값을 걸러내는 것이 목적이다.
constexpr double kQuaternionNormTolerance = 1e-2;

bool IsValidManeuverType(uint8_t value) {
    return value <= Command::EMERGENCY_STOP;
}

bool IsValidExecutionRequest(uint8_t value) {
    return value <= Command::CANCEL;
}

bool IsValidDirection(uint8_t value) {
    return value <= Command::DIRECTION_RIGHT;
}

bool IsValidMergeType(uint8_t value) {
    return value <= Command::MERGE_HIGHWAY;
}

// -1.0(기본값 사용) 또는 0 이상의 유한값.
//
// sentinel은 정확히 비교한다. 4.4절의 허용오차는 fingerprint 비교용이며
// validation에 쓰면 -0.9999995 같은 값이 -1.0으로 인정된다. 계약은
// "== -1.0 또는 >= 0.0"이므로 그 사이 구간(-1 < value < 0)은 거부해야 한다.
bool IsValidDesiredSpeed(double value) {
    if (!std::isfinite(value)) return false;
    return value == -1.0 || value >= 0.0;
}

bool IsValidObjectId(int32_t value) {
    return value >= -1;
}

// PREPARE/EXECUTE/CANCEL handshake를 쓰는 maneuver.
bool UsesHandshake(uint8_t maneuver_type) {
    return maneuver_type == Command::MERGE ||
           maneuver_type == Command::LANE_CHANGE ||
           maneuver_type == Command::AVOID;
}

CommandValidationResult Invalid(const std::string& detail) {
    return CommandValidationResult{CommandValidationCode::INVALID_COMMAND,
                                   detail};
}

CommandValidationResult Stale(const std::string& detail) {
    return CommandValidationResult{CommandValidationCode::STALE_COMMAND, detail};
}

}  // namespace

const char* CommandValidationCodeName(CommandValidationCode code) {
    switch (code) {
        case CommandValidationCode::OK: return "OK";
        case CommandValidationCode::INVALID_COMMAND: return "INVALID_COMMAND";
        case CommandValidationCode::STALE_COMMAND: return "STALE_COMMAND";
    }
    return "UNKNOWN_VALIDATION_CODE";
}

bool ConvertManeuverType(uint8_t maneuver_type, uint8_t direction,
                         BehaviorState& out_mode) {
    // switch에 default를 두지 않고 정의된 값만 명시적으로 나열한다.
    // 새 maneuver가 추가되면 여기에 case를 추가해야 하고, 빠뜨리면 아래의
    // 최종 return false로 떨어져 INVALID_COMMAND가 된다 - 조용히 잘못된
    // 모드로 실행되는 것보다 안전한 실패 방향이다.
    switch (maneuver_type) {
        case Command::KEEP:
            out_mode = LANE_KEEPING;
            return true;

        case Command::FOLLOW:
            out_mode = FOLLOWING;
            return true;

        case Command::MERGE:
            out_mode = MERGE;
            return true;

        case Command::LANE_CHANGE:
            // 유일한 1:2 매핑. 방향이 없으면 모드를 정할 수 없다.
            if (direction == Command::DIRECTION_LEFT) {
                out_mode = LANE_CHANGE_LEFT;
                return true;
            }
            if (direction == Command::DIRECTION_RIGHT) {
                out_mode = LANE_CHANGE_RIGHT;
                return true;
            }
            return false;

        case Command::AVOID:
            out_mode = AVOID;
            return true;

        case Command::STOP:
            out_mode = STOP;
            return true;

        case Command::EMERGENCY_STOP:
            out_mode = EMERGENCY;
            return true;

        default:
            break;
    }
    return false;
}

CommandValidationResult ValidateBehaviorCommand(
    const asmc_msgs::BehaviorCommand& message, const ros::Time& now) {
    // --- 3.1 공통 -------------------------------------------------------
    if (message.header.frame_id != "map") {
        return Invalid("header.frame_id must be \"map\"");
    }
    // command_id=0은 "처리 중인 명령 없음"을 뜻하는 값이므로 명령의 ID로
    // 쓸 수 없다(FrenetFeedback.command_id 주석 참고).
    if (message.command_id == 0) {
        return Invalid("command_id must not be 0");
    }
    if (message.valid_until.isZero()) {
        return Invalid("valid_until must be set");
    }
    if (!IsValidManeuverType(message.maneuver_type)) {
        return Invalid("unknown maneuver_type");
    }
    if (!IsValidExecutionRequest(message.execution_request)) {
        return Invalid("unknown execution_request");
    }
    if (!IsValidDirection(message.direction)) {
        return Invalid("unknown direction");
    }
    if (!IsValidMergeType(message.merge_type)) {
        return Invalid("unknown merge_type");
    }
    if (!IsValidDesiredSpeed(message.desired_speed_mps)) {
        return Invalid("desired_speed_mps must be -1.0 or a finite value >= 0");
    }
    if (!IsValidObjectId(message.primary_object_id) ||
        !IsValidObjectId(message.preferred_front_object_id) ||
        !IsValidObjectId(message.preferred_rear_object_id)) {
        return Invalid("object ids must be >= -1");
    }

    // --- 3.2 FOLLOW gap 필드 --------------------------------------------
    // 이 검사는 FOLLOW 전용이 아니다. use_custom_follow_gap이 켜져 있으면
    // 어느 maneuver에서든 값이 유효해야 한다.
    if (message.use_custom_follow_gap) {
        if (!std::isfinite(message.desired_time_gap_s) ||
            message.desired_time_gap_s <= 0.0) {
            return Invalid("desired_time_gap_s must be > 0 when custom gap is on");
        }
        if (!std::isfinite(message.minimum_gap_m) ||
            message.minimum_gap_m <= 0.0) {
            return Invalid("minimum_gap_m must be > 0 when custom gap is on");
        }
    } else {
        // sentinel 정확 비교 (IsValidDesiredSpeed 주석 참고).
        if (message.desired_time_gap_s != -1.0) {
            return Invalid("desired_time_gap_s must be -1.0 when custom gap is off");
        }
        if (message.minimum_gap_m != -1.0) {
            return Invalid("minimum_gap_m must be -1.0 when custom gap is off");
        }
    }

    // --- 3.3 maneuver별 -------------------------------------------------
    // execution_request: handshake를 쓰지 않는 maneuver는 EXECUTE만 허용한다.
    if (!UsesHandshake(message.maneuver_type) &&
        message.execution_request != Command::EXECUTE) {
        return Invalid("this maneuver only accepts EXECUTE");
    }

    // merge_type: MERGE만 값을 갖는다.
    if (message.maneuver_type == Command::MERGE) {
        if (message.merge_type == Command::MERGE_NONE) {
            return Invalid("MERGE requires a merge_type");
        }
    } else if (message.merge_type != Command::MERGE_NONE) {
        return Invalid("merge_type must be MERGE_NONE unless maneuver is MERGE");
    }

    // direction / has_target_pose / 객체 ID
    switch (message.maneuver_type) {
        case Command::KEEP:
        case Command::STOP:
        case Command::EMERGENCY_STOP:
            if (message.direction != Command::DIRECTION_NONE) {
                return Invalid("this maneuver requires DIRECTION_NONE");
            }
            if (message.primary_object_id != -1 ||
                message.preferred_front_object_id != -1 ||
                message.preferred_rear_object_id != -1) {
                return Invalid("this maneuver requires all object ids to be -1");
            }
            // STOP만 목표 pose가 필수다.
            if (message.maneuver_type == Command::STOP) {
                if (!message.has_target_pose) {
                    return Invalid("STOP requires has_target_pose");
                }
            } else if (message.has_target_pose) {
                return Invalid("this maneuver must not carry a target pose");
            }
            break;

        case Command::FOLLOW:
            if (message.direction != Command::DIRECTION_NONE) {
                return Invalid("FOLLOW requires DIRECTION_NONE");
            }
            if (message.primary_object_id < 0) {
                return Invalid("FOLLOW requires primary_object_id >= 0");
            }
            if (message.preferred_front_object_id != -1 ||
                message.preferred_rear_object_id != -1) {
                return Invalid("FOLLOW requires merge gap object ids to be -1");
            }
            if (message.has_target_pose) {
                return Invalid("FOLLOW must not carry a target pose");
            }
            break;

        case Command::LANE_CHANGE:
            // 방향이 없으면 내부 모드를 결정할 수 없다(2.2절).
            if (message.direction != Command::DIRECTION_LEFT &&
                message.direction != Command::DIRECTION_RIGHT) {
                return Invalid("LANE_CHANGE requires DIRECTION_LEFT or RIGHT");
            }
            if (message.primary_object_id != -1 ||
                message.preferred_front_object_id != -1 ||
                message.preferred_rear_object_id != -1) {
                return Invalid("LANE_CHANGE requires all object ids to be -1");
            }
            if (!message.has_target_pose) {
                return Invalid("LANE_CHANGE requires has_target_pose");
            }
            break;

        case Command::AVOID:
            if (message.primary_object_id < 0) {
                return Invalid("AVOID requires primary_object_id >= 0");
            }
            if (message.preferred_front_object_id != -1 ||
                message.preferred_rear_object_id != -1) {
                return Invalid("AVOID requires merge gap object ids to be -1");
            }
            // direction은 NONE/LEFT/RIGHT 모두 허용(Frenet이 방향을 정할 수
            // 있다). has_target_pose도 선택이다.
            break;

        case Command::MERGE:
            if (message.primary_object_id != -1) {
                return Invalid("MERGE uses preferred_front/rear, not primary_object_id");
            }
            // front/rear는 선택. has_target_pose 필수 여부는 Q1 미확정이므로
            // 여기서 강제하지 않는다(계약 11.9절).
            break;

        default:
            // IsValidManeuverType에서 이미 걸러졌다.
            return Invalid("unhandled maneuver_type");
    }

    // --- 3.4 target pose 유한성 / quaternion --------------------------
    // RefLine 투영과 heading 검증은 planner가 필요하므로 호출자가 수행한다.
    if (message.has_target_pose) {
        const auto& position = message.target_pose_map.position;
        const auto& orientation = message.target_pose_map.orientation;
        if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z)) {
            return Invalid("target pose position must be finite");
        }
        if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
            !std::isfinite(orientation.z) || !std::isfinite(orientation.w)) {
            return Invalid("target pose orientation must be finite");
        }
        const double norm = std::sqrt(
            orientation.x * orientation.x + orientation.y * orientation.y +
            orientation.z * orientation.z + orientation.w * orientation.w);
        if (std::fabs(norm - 1.0) > kQuaternionNormTolerance) {
            return Invalid("target pose orientation must be a unit quaternion");
        }
    }

    // --- valid_until 만료은 마지막에 --------------------------------------
    // 형식 오류를 STALE로 가리지 않기 위해 순서가 중요하다. 형식이 잘못된
    // 명령이 늦게 도착했다면 FSM이 알아야 하는 것은 형식 오류다.
    if (now > message.valid_until) {
        return Stale("command expired at valid_until");
    }

    return CommandValidationResult{};
}

bool ConvertBehaviorCommand(const asmc_msgs::BehaviorCommand& message,
                            PlannerCommand& out_command) {
    BehaviorState mode = LANE_KEEPING;
    if (!ConvertManeuverType(message.maneuver_type, message.direction, mode)) {
        return false;
    }

    PlannerCommand converted{};
    converted.mode = mode;

    // FOLLOW/AVOID 공용 대상 ID. MERGE는 primary를 쓰지 않는다(3.3절).
    converted.target_object_id = message.primary_object_id;

    // 계약 3.5절: -1.0이면 planner 기본 목표속도를 쓴다.
    if (message.desired_speed_mps < 0.0) {
        converted.has_target_speed = false;
        converted.target_speed = 0.0;
    } else {
        converted.has_target_speed = true;
        converted.target_speed = message.desired_speed_mps;
    }
    // BehaviorCommand에 대응 필드가 없다. v2 경로에서는 상한을 쓰지 않는다.
    converted.has_max_speed = false;
    converted.max_speed = 0.0;

    // 계약 11.8절: custom gap이 꺼져 있으면 0.0을 sentinel로 남긴다.
    // -1.0을 그대로 옮기면 음수 gap이 계산식에 흘러든다. config 기본값은
    // 소비 지점(D5)에서 적용한다.
    if (message.use_custom_follow_gap) {
        converted.time_gap = message.desired_time_gap_s;
        converted.min_gap = message.minimum_gap_m;
    } else {
        converted.time_gap = 0.0;
        converted.min_gap = 0.0;
    }

    converted.source_link_id = message.source_link_id;
    converted.target_link_id = message.target_link_id;
    switch (message.merge_type) {
        case Command::MERGE_ROUNDABOUT:
            converted.merge_type = MergeType::ROUNDABOUT;
            break;
        case Command::MERGE_HIGHWAY:
            converted.merge_type = MergeType::HIGHWAY;
            break;
        default:
            converted.merge_type = MergeType::NONE;
            break;
    }
    converted.merge_sa_id = message.preferred_front_object_id;
    converted.merge_sb_id = message.preferred_rear_object_id;

    // 요청 방향 (D6-1). static_cast로 넘기지 않는다 - 두 enum의 번호가
    // 우연히 같아도 계약 2절이 명시적 매핑을 요구한다.
    //
    // AVOID만 세 값을 모두 쓴다. LANE_CHANGE는 이미 mode로 갈라졌고
    // (LANE_CHANGE_LEFT/RIGHT), 나머지 maneuver는 validation이
    // DIRECTION_NONE을 강제하므로 자연히 NONE이 된다.
    switch (message.direction) {
        case Command::DIRECTION_LEFT:
            converted.requested_direction = LateralDirection::LEFT;
            break;
        case Command::DIRECTION_RIGHT:
            converted.requested_direction = LateralDirection::RIGHT;
            break;
        default:
            converted.requested_direction = LateralDirection::NONE;
            break;
    }

    // stop_position은 채우지 않는다 - target_pose_map을 RefLine에 투영해야
    // 하므로 planner를 가진 호출자의 책임이다. 기본값 NaN이 유지되며, 그
    // 상태로 STOP을 실행하면 ValidateExternalCommand가 INVALID_COMMAND로
    // 거부한다(잘못된 값으로 주행하는 것보다 안전한 실패 방향).

    out_command = converted;
    return true;
}

uint8_t ConvertToFeedbackManeuver(BehaviorState mode) {
    switch (mode) {
        case LANE_KEEPING:
            return Command::KEEP;

        case FOLLOWING:
            return Command::FOLLOW;

        case MERGE:
            return Command::MERGE;

        // 2:1 축약. 좌/우는 FSM이 보낸 direction으로 알 수 있다.
        case LANE_CHANGE_LEFT:
        case LANE_CHANGE_RIGHT:
            return Command::LANE_CHANGE;

        case AVOID:
            return Command::AVOID;

        case STOP:
            return Command::STOP;

        case EMERGENCY:
            return Command::EMERGENCY_STOP;

        // legacy 전용 모드. v2 계약에 대응 값이 없다.
        case INTERSECTION_WAIT:
        case TURN_LEFT:
        case TURN_RIGHT:
            return Feedback::MANEUVER_NONE;
    }
    return Feedback::MANEUVER_NONE;
}

}  // namespace frenet_planner_ros

