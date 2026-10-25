#include "Parser/PlanningInput.hpp"
#include "Parser/vehicle_curvature.hpp"

#include <cmath>
#include <unordered_set>
#include <utility>

#include "Frenet/Geometry/PolygonUtils.hpp"

namespace frenet_planner_ros {
namespace {

bool IsMapFrame(const std::string& frame_id) {
    return frame_id == "map";
}

}  // namespace

const char* ConversionStatusName(ConversionStatus status) {
    switch (status) {
        case ConversionStatus::OK: return "OK";
        case ConversionStatus::INVALID_FLAG: return "INVALID_FLAG";
        case ConversionStatus::INVALID_FRAME: return "INVALID_FRAME";
        case ConversionStatus::INVALID_TIMESTAMP: return "INVALID_TIMESTAMP";
        case ConversionStatus::NON_FINITE_VALUE: return "NON_FINITE_VALUE";
        case ConversionStatus::UNSUPPORTED_BEHAVIOR:
            return "UNSUPPORTED_BEHAVIOR";
        case ConversionStatus::DUPLICATE_OBJECT_ID:
            return "DUPLICATE_OBJECT_ID";
        case ConversionStatus::INVALID_OBJECT_GEOMETRY:
            return "INVALID_OBJECT_GEOMETRY";
        case ConversionStatus::INVALID_PATH: return "INVALID_PATH";
    }
    return "UNKNOWN_CONVERSION_STATUS";
}

ConversionStatus ConvertVehicleState(const asmc_msgs::VehicleState& message,
                                     CartesianState& state) {
    if (!message.valid) return ConversionStatus::INVALID_FLAG;
    if (!IsMapFrame(message.header.frame_id)) {
        return ConversionStatus::INVALID_FRAME;
    }
    if (message.header.stamp.isZero()) {
        return ConversionStatus::INVALID_TIMESTAMP;
    }
    if (!std::isfinite(message.x) || !std::isfinite(message.y) ||
        !std::isfinite(message.yaw) || !std::isfinite(message.speed) ||
        !std::isfinite(message.acceleration) ||
        !std::isfinite(message.yaw_rate)) {
        return ConversionStatus::NON_FINITE_VALUE;
    }

    CartesianState converted{};
    converted.x = message.x;
    converted.y = message.y;
    converted.yaw = message.yaw;
    converted.kappa = VehicleCurvature(message.speed, message.yaw_rate);
    converted.v = message.speed;
    converted.a = message.acceleration;
    state = converted;
    return ConversionStatus::OK;
}

ConversionStatus ConvertBehaviorContext(
    const asmc_msgs::BehaviorContext& message, PlannerCommand& command) {
    if (!message.valid) return ConversionStatus::INVALID_FLAG;
    if (message.header.stamp.isZero()) {
        return ConversionStatus::INVALID_TIMESTAMP;
    }
    if ((message.has_desired_speed && !std::isfinite(message.desired_speed)) ||
        (message.has_max_speed && !std::isfinite(message.max_speed))) {
        return ConversionStatus::NON_FINITE_VALUE;
    }

    PlannerCommand converted{};
    switch (message.selected_behavior) {
        case asmc_msgs::BehaviorContext::KEEP:
            converted.mode = LANE_KEEPING;
            break;
        case asmc_msgs::BehaviorContext::STOP:
            converted.mode = STOP;
            break;
        default:
            return ConversionStatus::UNSUPPORTED_BEHAVIOR;
    }

    // STOP은 정지 지점이 반드시 지정되어야 한다. BehaviorContext는 이를
    // Cartesian (stop_x, stop_y)로 준다.
    //
    // 여기서 converted.stop_position에 stop_x를 넣으면 안 된다:
    // PlannerCommand.stop_position은 Frenet 종방향 좌표 s이고 stop_x/y는 map
    // 좌표라 단위계가 다르다. converter는 reference line을 모르므로 투영을
    // 할 수 없다. 따라서 이 단계에서는 Cartesian 목표의 유효성만 검증하고,
    // stop_x/y -> s 투영은 RefLine을 가진 노드가 수행한다(C4).
    // 그때까지 converted.stop_position은 기본값 NaN으로 남고, 그 상태로
    // Plan()에 들어가면 INVALID_COMMAND로 거부되어 즉시 정지한다 - 잘못된
    // 값으로 주행하는 것보다 안전한 실패 방향이다.
    if (converted.mode == STOP) {
        if (!message.has_stop_target) {
            return ConversionStatus::INVALID_FLAG;
        }
        if (!std::isfinite(message.stop_x) || !std::isfinite(message.stop_y)) {
            return ConversionStatus::NON_FINITE_VALUE;
        }
    }

    converted.target_lane = 0;
    converted.target_object_id = -1;
    converted.has_target_speed = message.has_desired_speed;
    converted.target_speed = message.desired_speed;
    converted.has_max_speed = message.has_max_speed;
    converted.max_speed = message.max_speed;
    command = converted;
    return ConversionStatus::OK;
}

ConversionStatus ConvertDetectedObjects(
    const planning_inputs::DetectedObjectArray& message,
    std::vector<TrackedObject>& objects) {
    if (!IsMapFrame(message.header.frame_id)) {
        return ConversionStatus::INVALID_FRAME;
    }
    if (message.header.stamp.isZero()) {
        return ConversionStatus::INVALID_TIMESTAMP;
    }
    std::vector<TrackedObject> converted_objects;
    converted_objects.reserve(message.objects.size());
    std::unordered_set<int32_t> object_ids;

    for (const auto& message_object : message.objects) {
        if (!object_ids.insert(message_object.id).second) {
            return ConversionStatus::DUPLICATE_OBJECT_ID;
        }
        if (!std::isfinite(message_object.center.x) ||
            !std::isfinite(message_object.center.y) ||
            !std::isfinite(message_object.center.z) ||
            !std::isfinite(message_object.yaw) ||
            !std::isfinite(message_object.vx) ||
            !std::isfinite(message_object.vy) ||
            !std::isfinite(message_object.yaw_rate) ||
            !std::isfinite(message_object.width) || message_object.width <= 0.0 ||
            !std::isfinite(message_object.length) || message_object.length <= 0.0 ||
            !std::isfinite(message_object.height) || message_object.height <= 0.0) {
            return ConversionStatus::NON_FINITE_VALUE;
        }
        if (message_object.motion_state >
            planning_inputs::DetectedObject::MOTION_DYNAMIC ||
            (message_object.velocity_unavailable &&
             (message_object.motion_state != planning_inputs::DetectedObject::MOTION_UNKNOWN ||
              message_object.vx != 0. || message_object.vy != 0.))) {
            return ConversionStatus::INVALID_FLAG;
        }

        TrackedObject object;
        object.id = message_object.id;
        object.type = message_object.type;
        object.motion_state =
            static_cast<ObjectMotionState>(message_object.motion_state);
        object.velocity_unavailable = message_object.velocity_unavailable;
        object.x = message_object.center.x;
        object.y = message_object.center.y;
        object.z = message_object.center.z;
        object.heading = message_object.yaw;
        object.vx = message_object.vx;
        object.vy = message_object.vy;
        object.speed = std::hypot(message_object.vx, message_object.vy);
        object.yaw_rate = message_object.yaw_rate;
        object.width = message_object.width;
        object.length = message_object.length;
        object.height = message_object.height;

        // authoritative geometry: perception이 제공하는 obb_corners만
        // 사용한다. center/width/length/yaw로 geometry를 재생성하지 않는다.
        for (std::size_t i = 0; i < object.corners.size(); ++i) {
            const auto& corner = message_object.obb_corners[i];
            object.corners[i] = {corner.x, corner.y, corner.z};
        }
        const geometry::CornerValidationResult validation =
            geometry::ValidateCorners8(object.corners);
        const geometry::ZRange z_range =
            geometry::ComputeZRange(object.corners);
        if (!validation.valid || !z_range.valid ||
            z_range.max - z_range.min <= 1e-6) {
            return ConversionStatus::INVALID_OBJECT_GEOMETRY;
        }
        object.geometry_valid = true;
        object.z_valid = true;
        converted_objects.push_back(object);
    }

    objects = std::move(converted_objects);
    return ConversionStatus::OK;
}

ConversionStatus ConvertCartesianPath(
    const CartesianPath& path, double dt,
    const asmc_msgs::VehicleState& ego_at_plan,
    asmc_msgs::Trajectory& trajectory) {
    const std::size_t count = path.x.size();
    if (count == 0 || path.y.size() != count || path.yaw.size() != count ||
        path.kappa.size() != count || path.v.size() != count ||
        path.a.size() != count || !std::isfinite(dt) || dt <= 0.0) {
        return ConversionStatus::INVALID_PATH;
    }
    asmc_msgs::Trajectory converted;
    // provenance, not freshness: 이 trajectory는 ego_at_plan 스냅샷으로부터
    // 생성됐으므로 source time도 그 스냅샷의 것이다. ego_at_plan이 무효/미측정
    // (header.stamp가 0)이면 그 사실을 그대로 반영한다 - now()로 대체해
    // "방금 측정한 것"처럼 보이게 하지 않는다. 소비자(MPC ConvertTrajectory)가
    // 이미 header.stamp.isZero()를 INVALID_TIMESTAMP로 거부한다.
    converted.header.stamp = ego_at_plan.header.stamp;
    converted.header.frame_id = "map";
    converted.valid = true;
    converted.failure_reason = asmc_msgs::Trajectory::FAILURE_NONE;
    converted.dt = dt;
    converted.ego_at_plan = ego_at_plan;
    converted.points.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(path.x[i]) || !std::isfinite(path.y[i]) ||
            !std::isfinite(path.yaw[i]) || !std::isfinite(path.kappa[i]) ||
            !std::isfinite(path.v[i]) || !std::isfinite(path.a[i])) {
            return ConversionStatus::NON_FINITE_VALUE;
        }
        asmc_msgs::TrajectoryPoint point;
        point.x = path.x[i];
        point.y = path.y[i];
        point.yaw = path.yaw[i];
        point.curvature = path.kappa[i];
        point.velocity = path.v[i];
        point.acceleration = path.a[i];
        converted.points.push_back(point);
    }
    trajectory = std::move(converted);
    return ConversionStatus::OK;
}

asmc_msgs::Trajectory MakeStopTrajectory(
    const asmc_msgs::VehicleState& ego_at_plan, double dt,
    uint8_t failure_reason) {
    asmc_msgs::Trajectory trajectory;
    // ConvertCartesianPath와 같은 provenance 규칙: STOP도 ego_at_plan 스냅샷
    // 기준 trajectory이므로 그 소스 시각을 그대로 쓴다. ego_at_plan이 알 수
    // 없는 상태(zero stamp, valid=false)라면 이 STOP 자체가 그 사실을 그대로
    // 전달해야 한다 - MPC의 ConvertTrajectory가 ego_at_plan.valid/timestamp를
    // 검사해 독립적으로 fail-safe STOP을 낼 수 있어야 하기 때문이다(계약
    // 9절). now()로 유효성을 위장하지 않는다.
    trajectory.header.stamp = ego_at_plan.header.stamp;
    trajectory.header.frame_id = "map";
    trajectory.valid = true;
    trajectory.failure_reason = failure_reason;
    trajectory.dt = dt;
    trajectory.ego_at_plan = ego_at_plan;
    asmc_msgs::TrajectoryPoint point;
    point.x = std::isfinite(ego_at_plan.x) ? ego_at_plan.x : 0.0;
    point.y = std::isfinite(ego_at_plan.y) ? ego_at_plan.y : 0.0;
    point.yaw = std::isfinite(ego_at_plan.yaw) ? ego_at_plan.yaw : 0.0;
    point.curvature = 0.0;
    point.velocity = 0.0;
    point.acceleration = 0.0;
    trajectory.points.push_back(point);
    return trajectory;
}

}  // namespace frenet_planner_ros

