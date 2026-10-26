#pragma once
#include <asmc_lidar/core/types.hpp>
#include <string>

namespace asmc::lidar::filtering {
struct DeliveryPoint { double x=0., y=0., z=0.; };
// Minimal ROS-independent view used by the ported final-publication policies.
// Former Main PlannerObject. It never owns/copies a point cloud or mutates OBBs.
struct DeliveryObject {
  int id=-1;
  std::string classification, yolo_class;
  DeliveryPoint obb_center_map;
  double length=0., width=0., height=0.;
  pipeline::ObjectGroundEvidence ground_evidence;
};
}  // namespace asmc::lidar::filtering
