#ifndef PLANNER_CONFIG_CONFIG_LOADER_HPP
#define PLANNER_CONFIG_CONFIG_LOADER_HPP

#include <string>

#include "frenet_planner/Config/FrenetConfig.hpp"
#include "frenet_planner/Evaluation/CollisionChecker.hpp"
#include "frenet_planner/Evaluation/Cost.hpp"
#include "frenet_planner/Evaluation/KinematicFilter.hpp"
#include "frenet_planner/Generator/GeneratorConfig.hpp"
#include "frenet_planner/visualization/planner_debug_writer.hpp"

// params.yaml -> 각 config struct 로드 (yaml-cpp 기반, ROS 의존성 없음)
void LoadParams(const std::string& yaml_path,
                PathGeneratorConfig& path_cfg,
                KinematicLimits& limits,
                CostWeights& cost_weights,
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
                std::string& waypoint_file);

#endif  // PLANNER_CONFIG_CONFIG_LOADER_HPP
