// 외부(perception/시뮬레이터)에서 들어오는 주변 객체 표현.

#ifndef PLANNER_COMMON_OBJECT_TYPES_HPP
#define PLANNER_COMMON_OBJECT_TYPES_HPP

#include <cstdint>
#include "Global/Global.hpp"
#include <limits>

#include "Frenet/Geometry/GeometryTypes.hpp"

using ObjectMotionState = asmc_planner::MotionState;

// asmc_msgs/DetectedObject.msg 기반 canonical 객체 표현. 스칼라 motion 필드
// (ObjectInfo)와 authoritative OBB corner geometry(TrackedObstacle)를 하나로
// 합친 것이다 - 같은 객체를 두 개의 병렬 vector로 나눠 들고 다니지 않는다.
//
// authoritative geometry의 유일한 출처는 perception의 obb_corners[8]이다
// (message_converter::ConvertDetectedObjects). center/width/length/yaw로
// geometry를 재생성하는 production 경로는 없다 - corners/geometry_valid/
// z_valid만으로 유효성을 판단한다. 출처를 구분하는 별도 enum은 두지 않는다.
struct TrackedObject {
    int id = -1;
    int type = 0;
    ObjectMotionState motion_state = ObjectMotionState::UNKNOWN;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double width = 0.0;
    double length = 0.0;
    double height = 0.0;
    double yaw_rate = 0.0;  // [rad/s] planner가 연속 heading 관측으로 추정

    // Internal prediction policy, not a ROS field. Infinity preserves the
    // existing constant-turn model. A provisional turn ends along its tangent.
    double turn_prediction_duration = std::numeric_limits<double>::infinity();

    // Optional continuation for an additional short sensor-velocity hypothesis.
    // NaN retains the tangent continuation used by ordinary predictions.
    double continuation_vx = std::numeric_limits<double>::quiet_NaN();
    double continuation_vy = std::numeric_limits<double>::quiet_NaN();

    // Optional departure-only speed interval. NaNs preserve ordinary point
    // prediction. Its swept footprint covers intermediate speeds continuously;
    // these internal fields do not change the ROS message/wire contract.
    double prediction_min_speed = std::numeric_limits<double>::quiet_NaN();
    double prediction_max_speed = std::numeric_limits<double>::quiet_NaN();

    // ---- 아래는 충돌검사용 authoritative geometry (구 TrackedObstacle) ----
    geometry::Corners8 corners{};
    bool geometry_valid = false;
    bool z_valid = false;
    bool velocity_unavailable = false;  // observed occupancy, not measured rest
};

#endif  // PLANNER_COMMON_OBJECT_TYPES_HPP

