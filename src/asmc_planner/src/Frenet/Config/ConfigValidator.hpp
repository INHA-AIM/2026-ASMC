#ifndef PLANNER_CONFIG_CONFIG_VALIDATOR_HPP
#define PLANNER_CONFIG_CONFIG_VALIDATOR_HPP

#include <string>

#include "Frenet/Config/FrenetConfig.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Frenet/Evaluation/Cost.hpp"
#include "Frenet/Evaluation/KinematicFilter.hpp"
#include "Frenet/Generator/GeneratorConfig.hpp"
#include "Frenet/Reference/Reference.hpp"
#include "Frenet/visualization/planner_debug_writer.hpp"

// LoadParams()가 key 존재/타입을 확인한 뒤, 그 값 자체가 물리적으로/구조적으로
// 말이 되는지 검사한다(range, min<=max, dt>0, cross-config consistency).
// 하나라도 위반하면 false를 반환하고 error_message에 위반 내용과 관련 YAML
// key를 남긴다 - FrenetPlanner::Init()은 이 경우 초기화를 실패시켜야 한다.
//
// highway_merge zone/checkpoint의 s 순서 검증도 여기서 한다 - 잘못된 설정을
// 발견했다고 런타임에 enabled를 조용히 false로 바꾸지 않는다. 설정이
// 유효하지 않으면 Init() 자체가 실패해야 한다.
bool ValidateConfig(const ReferenceConfig& reference_cfg,
                     const ObjectTrackingConfig& object_tracking_cfg,
                     const PathGeneratorConfig& path_cfg,
                     const KinematicLimits& limits,
                     const CostWeights& cost_weights,
                     const KeepCruiseConfig& keep_cruise_cfg,
                     const VehicleShape& vehicle_shape,
                     const CollisionCheckConfig& collision_cfg,
                     const CurveSpeedConfig& curve_speed_cfg,
                     const FollowingConfig& following_cfg,
                     const AvoidConfig& avoid_cfg,
                     const LaneChangeConfig& lane_change_cfg,
                     const StopConfig& stop_cfg,
                     const EmergencyStopConfig& emergency_stop_cfg,
                     const MergeConfig& merge_cfg,
                     const HighwayMergeConfig& highway_merge_cfg,
                     const PlannerVisualizationConfig& visualization_cfg,
                     double wheelbase,
                     double lane_width,
                     double replanning_period_s,
                     double max_target_speed_accel,
                     std::string& error_message);

#endif  // PLANNER_CONFIG_CONFIG_VALIDATOR_HPP
