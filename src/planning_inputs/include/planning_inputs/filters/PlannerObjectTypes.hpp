#pragma once
#include <array>
#include <cstdint>
#include <string>
// Portable final-publication DTO. No sensor pipeline or shared ROS schema dependency.
struct PlannerPoint3D { double x=0, y=0, z=0; };
struct PlannerObject {
  std::int32_t id=-1;
  std::string classification, yolo_class;
  bool is_dynamic=false;
  std::int32_t motion_state=0;
  PlannerPoint3D cluster_center_map, obb_center_map, velocity_map_mps;
  std::array<PlannerPoint3D,8> obb_corners_map{};
  double obb_yaw_map_rad=0, length=0, width=0, height=0;
};
