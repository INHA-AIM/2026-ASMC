// 외부(perception/시뮬레이터)에서 들어오는 주변 객체 표현.

#ifndef PLANNER_COMMON_OBJECT_TYPES_HPP
#define PLANNER_COMMON_OBJECT_TYPES_HPP

// --- ObjectStatus 한 오브젝트 -------------
struct ObjectInfo {
    int id = -1;
    int type = 0;
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
};

#endif  // PLANNER_COMMON_OBJECT_TYPES_HPP
