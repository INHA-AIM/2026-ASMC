#pragma once

#include <cstddef>
#include <string>

#include <ros/ros.h>

#include <Global/Global.hpp>
#include <PlannerInterface/DrivableBoundaryFilter.hpp>
#include <PlannerInterface/KnownStaticObjectFilter.hpp>
#include <PlannerInterface/PlanningRelevanceFilter.hpp>
#include <PlannerInterface/StaticStructureExclusionFilter.hpp>

struct PlannerObject;

class PlannerObjectRosPublisher {
public:
    explicit PlannerObjectRosPublisher(ros::NodeHandle& node_handle);

    void Publish(const LiDAR& lidar, const GPS_IMU& gps_imu,
                 const VehicleSpec& vehicle, const ros::Time& stamp);

    void SuppressMap(const ros::Time& stamp);

private:
    ros::Publisher objects_publisher_;
    ros::Publisher markers_publisher_;
    ros::Publisher last_objects_publisher_;
    ros::Publisher last_markers_publisher_;
    ros::Publisher motion_debug_publisher_;
    std::size_t max_objects_ = 200;
    double velocity_arrow_seconds_ = 1.0;
    DrivableBoundaryFilter drivable_filter_;
    KnownStaticObjectFilter known_static_filter_;
    StaticStructureExclusionFilter static_structure_exclusion_filter_;
    PlanningRelevanceFilter planning_relevance_filter_;
    ros::Time last_stamp_;
};
