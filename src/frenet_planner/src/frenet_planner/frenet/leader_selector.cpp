#include "frenet_planner/frenet/leader_selector.hpp"

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
