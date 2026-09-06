#ifndef FRENET_MANEUVER_MERGE_HPP
#define FRENET_MANEUVER_MERGE_HPP

#include <unordered_map>
#include <vector>

#include "frenet_planner/Common/ObjectTypes.hpp"
#include "frenet_planner/Common/PlannerConfig.hpp"
#include "frenet_planner/Evaluation/CollisionChecker.hpp"
#include "frenet_planner/Reference/Reference.hpp"

// package-internal MERGE gap/TTC 순수 계산. FrenetPlanner public API가
// 아니므로 frenet_planner.hpp에는 선언하지 않는다. "지금 이 gap이 안전한가"
// 만 답한다 - MERGE를 자동 활성화할지, 언제 COMMIT할지는 이 파일의 책임이
// 아니다(FSM이 지정한 MERGE 실행을 다루는 별도 호출자의 책임).

struct RoundaboutGap {
    bool safe = false;
    bool confirmed = false;
    double entry_time = 0.0;
    double clear_time = 0.0;
    int preceding_id = -1;
    double preceding_time = -1.0;
    int following_id = -1;
    double following_time = -1.0;
    size_t crossing_vehicle_count = 0;
};

// 순환 차량을 등속 직선 예측했을 때 conflict point에 가장 가까워지는 시간을
// 구하고, Ego가 들어갈 수 있는 첫 시간 gap을 반환한다. 횡경로는 global path가
// 담당하며 이 함수는 회전교차로 진입 타이밍만 판단한다.
RoundaboutGap FindRoundaboutGap(const RefLine& ref, const FrenetState& ego,
                                const std::vector<TrackedObject>& obstacles,
                                const MergeConfig& cfg, double desired_speed,
                                double max_ego_accel,
                                const std::unordered_map<int, double>* yaw_rates = nullptr);

// Global Path가 램프에서 본선으로 이어지는 고주로 합류용. 목표 corridor에 투영된
// 차량의 합류 완료시점 앞/뒤 거리와 TTC를 함께 만족하는 첫 진입시간을 찾는다.
RoundaboutGap FindHighwayMergeGap(const RefLine& ref, const FrenetState& ego,
                                  const std::vector<TrackedObject>& obstacles,
                                  const std::vector<TrackedObject>& collision_obstacles,
                                  const HighwayMergeConfig& cfg,
                                  const HighwayMergeCheckpoint& checkpoint,
                                  const VehicleShape& ego_shape,
                                  const CollisionCheckConfig& collision_cfg,
                                  double desired_speed, double max_ego_accel);

#endif  // FRENET_MANEUVER_MERGE_HPP
