#include "frenet_planner/Maneuver/ManeuverUtils.hpp"

#include <algorithm>
#include <cmath>

#include "frenet_planner/Math/FrenetConverter.hpp"

void ProjectObjectToFrenet(const RefLine& ref, const TrackedObject& obj,
                           double& out_s, double& out_d, double& out_s_dot) {
    CartesianState cs{};
    cs.x = obj.x;
    cs.y = obj.y;
    cs.yaw = obj.heading;
    cs.kappa = 0.0;   // CartesianToFrenet의 s_dot/d 계산에는 안 쓰임(FrenetToCartesian 역방향 전용 필드)
    cs.v = obj.speed;
    cs.a = 0.0;       // 가속도 정보 없음 - s_ddot은 안 쓰므로 무관

    double s, s_dot, s_ddot, d, d_prime, d_pprime;
    CartesianToFrenet(ref, cs, s, s_dot, s_ddot, d, d_prime, d_pprime, nullptr);

    out_s = s;
    out_d = d;
    out_s_dot = s_dot;
}

// 폐루프 트랙에서 current_s -> target_s의 전방 호길이.
// 단순 (target - current)는 finish line 이음새에서 음수로 크게 튄다
// (예: ego_s=2200, target_s=20이면 실제로는 전방 30m인데 -2180m).
double ForwardArcDistance(double current_s, double target_s, double s_max) {
    double ds = target_s - current_s;
    if (!std::isfinite(ds)) return ds;
    if (s_max > 0.0) {
        // 이음새를 갓 지난 목표(작은 음수)는 "뒤"가 아니라 한 바퀴 앞이 아니다.
        // 절반 이상 뒤로 떨어진 경우만 wrap으로 간주해 전방 거리로 되돌린다.
        if (ds < -0.5 * s_max) ds += s_max;
        else if (ds > 0.5 * s_max) ds -= s_max;
    }
    return ds;
}

bool CartesianTrajectoryClear(const CartesianPath& path,
                              const std::vector<TrackedObject>& obstacles,
                              const VehicleShape& ego_shape,
                              const CollisionCheckConfig& collision_cfg,
                              double dt, int* blocking_id) {
    if (blocking_id) *blocking_id = -1;
    const size_t count = std::min({path.x.size(), path.y.size(), path.yaw.size()});
    for (size_t i = 0; i < count; ++i) {
        CartesianState ego{};
        ego.x = path.x[i];
        ego.y = path.y[i];
        ego.yaw = path.yaw[i];
        const double t = static_cast<double>(i) * dt;
        const double margin = collision_cfg.safety_margin +
                              collision_cfg.margin_growth_rate * t;
        // 기존 구현은 ego와 obstacle 양쪽에 margin을 적용했다. 장애물 원본
        // corner를 변경하지 않으면서 총 여유를 보수적으로 유지하도록 ego에
        // 두 배 margin을 적용한다.
        const geometry::Polygon2D ego_footprint =
            MakeEgoFootprint(ego, ego_shape, 2.0 * margin);
        for (const auto& obj : obstacles) {
            if (CheckObstacleOverlap(ego_footprint, {}, obj, t)) {
                if (blocking_id) *blocking_id = obj.id;
                return false;
            }
        }
    }
    return true;
}
