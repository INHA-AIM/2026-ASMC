#ifndef LIDAR_OBJECTS_ADAPTER_CONVERTER_HPP_
#define LIDAR_OBJECTS_ADAPTER_CONVERTER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include <asmc_msgs/LidarTrackingObservation.h>
#include <planning_inputs/DetectedObjectArray.h>

namespace planning_inputs {

// Planner-specific categories; these are not YOLO class or group IDs.
// Unknown retains geometry/motion; camera static/dynamic groups are not this enum.
constexpr std::uint8_t kObjectUnknown = 0;
constexpr std::uint8_t kObjectVehicle = 1;
constexpr std::uint8_t kObjectPedestrian = 2;

constexpr std::uint8_t kMotionUnknown = 0;
constexpr std::uint8_t kMotionLowThreat = 1;
constexpr std::uint8_t kMotionDynamicPending = 2;
constexpr std::uint8_t kMotionDynamic = 3;

enum class ConversionStatus {
  OK = 0,
  INVALID_ARGUMENT,
  INVALID_FRAME,
  INVALID_TIMESTAMP,
  INVALID_ID,
  DUPLICATE_ID,
  NON_FINITE_VALUE,
  INVALID_DIMENSIONS,
  INVALID_OBB,
  INVALID_MOTION_STATE,
  INVALID_OBSERVATION,
  INVALID_VELOCITY,
};

struct ConversionStats {
  std::size_t total = 0;
  std::size_t vehicle = 0;
  std::size_t pedestrian = 0;
  std::size_t unknown = 0;
  std::size_t unsupported_classification = 0;
};

const char* ConversionStatusName(ConversionStatus status);

// Direct common contract. No legacy wire, JSON join, clipping or second
// classifier. Invalid/pose-unavailable/unrepresentable frames are withheld,
// never converted into an observed-empty planning frame.
ConversionStatus ConvertLidarObservation(
    const asmc_msgs::LidarTrackingObservation& input,
    planning_inputs::DetectedObjectArray* output,
    ConversionStats* stats = nullptr, std::string* detail = nullptr);

}  // namespace planning_inputs

#endif  // LIDAR_OBJECTS_ADAPTER_CONVERTER_HPP_
