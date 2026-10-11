#ifndef FRENET_MANEUVER_MERGE_GAP_EVALUATOR_HPP
#define FRENET_MANEUVER_MERGE_GAP_EVALUATOR_HPP

#include <unordered_map>
#include <vector>

#include "Frenet/Common/ObjectTypes.hpp"
#include "Frenet/Config/FrenetConfig.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Frenet/Reference/Reference.hpp"

// package-internal MERGE gap/TTC 순수 계산. FrenetPlanner public API가
// 아니므로 frenet_planner.hpp에는 선언하지 않는다. "지금 이 gap이 안전한가"
// 만 답한다 - MERGE를 자동 활성화할지, 언제 COMMIT할지는 이 파일의 책임이
// 아니다(FSM이 지정한 MERGE 실행을 다루는 별도 호출자의 책임).
//
// R6-F3: 파일명을 Merge.hpp -> MergeGapEvaluator.hpp로 바꿨다(순수 rename,
// 로직/시그니처 변경 없음) - 이 파일의 실제 역할이 gap/TTC 계산뿐이라는
// 것을 이름에 그대로 반영한다. MERGE production 실행(활성화/COMMIT)은
// 여전히 이 파일의 책임이 아니고, 여전히 어디에도 구현되어 있지 않다.

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
// objects의 각 TrackedObject는 motion 예측(gap/TTC)과 collision 재검사
// (CheckObstacleOverlap) 양쪽에 그대로 쓰인다 - 같은 객체를 두 개의 병렬
// vector로 나눠 받지 않는다(authoritative corners를 이미 갖고 있으므로).
RoundaboutGap FindHighwayMergeGap(const RefLine& ref, const FrenetState& ego,
                                  const std::vector<TrackedObject>& objects,
                                  const HighwayMergeConfig& cfg,
                                  const HighwayMergeCheckpoint& checkpoint,
                                  const VehicleShape& ego_shape,
                                  const CollisionCheckConfig& collision_cfg,
                                  double desired_speed, double max_ego_accel);

#endif  // FRENET_MANEUVER_MERGE_GAP_EVALUATOR_HPP
