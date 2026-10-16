#ifndef PLANNER_CONFIG_CONFIG_LOADER_HPP
#define PLANNER_CONFIG_CONFIG_LOADER_HPP

#include <string>

#include "Frenet/Config/FrenetConfig.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Frenet/Evaluation/Cost.hpp"
#include "Frenet/Evaluation/KinematicFilter.hpp"
#include "Frenet/Generator/GeneratorConfig.hpp"
#include "Frenet/Reference/Reference.hpp"
#include "Frenet/visualization/planner_debug_writer.hpp"

// params.yaml -> 각 config struct 로드 (yaml-cpp 기반, ROS 의존성 없음).
//
// 필수 key 누락, YAML 타입 오류, 시퀀스/객체 모양 불일치가 하나라도 있으면
// false를 반환하고 error_message에 원인이 된 key를 남긴다 - 어떤 필드도
// 조용히 이전 값(대개 미정의)에 머무르지 않는다. 여기서는 개별 key의
// 존재/타입만 검사하며, range/순서/물리적 정합성 같은 값 자체의 타당성은
// ConfigValidator::ValidateConfig()의 책임이다.
bool LoadParams(const std::string& yaml_path,
                ReferenceConfig& reference_cfg,
                ObjectTrackingConfig& object_tracking_cfg,
                PathGeneratorConfig& path_cfg,
                KinematicLimits& limits,
                CostWeights& cost_weights,
                KeepCruiseConfig& keep_cruise_cfg,
                VehicleShape& vehicle_shape,
                CollisionCheckConfig& collision_cfg,
                CurveSpeedConfig& curve_speed_cfg,
                FollowingConfig& following_cfg,
                AvoidConfig& avoid_cfg,
                LaneChangeConfig& lane_change_cfg,
                StopConfig& stop_cfg,
                EmergencyStopConfig& emergency_stop_cfg,
                MergeConfig& merge_cfg,
                HighwayMergeConfig& highway_merge_cfg,
                PlannerVisualizationConfig& visualization_cfg,
                double& wheelbase,
                double& lane_width,
                double& replanning_period_s,
                double& max_target_speed_accel,
                std::string& waypoint_file,
                std::string& error_message);

#endif  // PLANNER_CONFIG_CONFIG_LOADER_HPP
