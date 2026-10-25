#pragma once
#include <asmc_lidar/core/types.hpp>
#include <array>
#include <string>

namespace asmc::lidar::filtering {
struct DeliveryPoint { double x=0., y=0., z=0.; };
// Minimal ROS-independent view used by the ported final-publication policies.
// Former Main PlannerObject. It never owns/copies a point cloud or mutates OBBs.
struct DeliveryObject {
  int id=-1, motion_state=0;
  std::string classification, yolo_class;
  bool is_dynamic=false;
  DeliveryPoint cluster_center_map, obb_center_map;
  std::array<DeliveryPoint,8> obb_corners_map{};
  double length=0., width=0., height=0.;
  double obb_yaw_map_rad=0.;
  pipeline::ObjectGroundEvidence ground_evidence;
};
}  // namespace asmc::lidar::filtering
