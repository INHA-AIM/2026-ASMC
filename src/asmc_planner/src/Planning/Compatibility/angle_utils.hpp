#pragma once

#include <cmath>

#include <geometry_msgs/Quaternion.h>

namespace frenet_planner_ros {

// quaternion -> yaw. D3-2 validation이 단위 quaternion을 이미 보장하므로
// 정규화는 하지 않는다.
inline double YawFromQuaternion(const geometry_msgs::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// 각도를 (-pi, pi]로 정규화. 단순 뺄셈으로 비교하면 179도와 -179도의 차이가
// 358도로 계산되어 실제 2도 차이를 큰 오차로 오판한다.
inline double NormalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace frenet_planner_ros
