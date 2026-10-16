#ifndef LIDAR_OBJECTS_ADAPTER_CONVERTER_HPP_
#define LIDAR_OBJECTS_ADAPTER_CONVERTER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include <asmc_msgs/TrackedObjectArray.h>
#include <planning_inputs/DetectedObjectArray.h>

namespace planning_inputs {

// Canonical ASMC object categories. Msg/TrackedObject.type uses a different
// numeric convention, so the source integer must never be copied directly.
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
};

struct ConversionStats {
  std::size_t total = 0;
  std::size_t vehicle = 0;
  std::size_t pedestrian = 0;
  std::size_t unknown = 0;
  std::size_t unsupported_classification = 0;
};

const char* ConversionStatusName(ConversionStatus status);

// Converts one source frame atomically. On failure, output is not modified;
// callers must not publish an empty substitute for a rejected frame.
ConversionStatus ConvertTrackedObjects(
    const asmc_msgs::TrackedObjectArray& input,
    planning_inputs::DetectedObjectArray* output,
    ConversionStats* stats = nullptr,
    std::string* detail = nullptr);

}  // namespace planning_inputs

#endif  // LIDAR_OBJECTS_ADAPTER_CONVERTER_HPP_
