#ifndef FRENET_PIPELINE_PLANNER_CONFIG_BUNDLE_HPP
#define FRENET_PIPELINE_PLANNER_CONFIG_BUNDLE_HPP

#include "frenet_planner/Config/FrenetConfig.hpp"
#include "frenet_planner/Evaluation/CollisionChecker.hpp"
#include "frenet_planner/Evaluation/Cost.hpp"
#include "frenet_planner/Evaluation/KinematicFilter.hpp"
#include "frenet_planner/Generator/GeneratorConfig.hpp"

// ExecuteManeuverPipeline(KEEP/FOLLOW/STOP 공용 candidate 생성 ->
// 곡률/충돌 필터 -> cost 평가 -> 최적 후보 선택 pipeline)이 참조하는
// config 묶음. 값을 복사하지 않고 FrenetPlanner가 Init()에서 채운 config
// struct들을 참조로만 모은다 - Parameter Single-Source 원칙상 이 struct
// 자신은 아무 필드도 소유/로드하지 않고, 여러 config를 한 번에 넘기기 위한
// 얇은 view일 뿐이다.
struct PlannerConfigBundle {
    const PathGeneratorConfig& path;
    const KinematicLimits& limits;
    const CostWeights& cost_weights;
    const VehicleShape& vehicle_shape;
    const CollisionCheckConfig& collision;
    const StopConfig& stop;
    const MergeConfig& merge;
    const AvoidConfig& avoid;
    const FollowingConfig& following;
    double lane_width;
};

#endif
