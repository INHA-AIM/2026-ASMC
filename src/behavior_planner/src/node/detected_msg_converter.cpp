#include "behavior_planner/node/detected_msg_converter.hpp"

#include <cmath>

namespace behavior_planner {
namespace {

constexpr std::uint8_t kUnknown = 0;
constexpr std::uint8_t kVehicle = 1;
constexpr std::uint8_t kPedestrian = 2;
constexpr std::uint8_t kStaticObstacle = 3;

double PositiveOr(double value, double fallback) {
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

}  // namespace

bool DetectedMsgConverter::toTrackedObject(
    const asmc_msgs::DetectedObject& message, ObjectType type,
    TrackedObject* object) {
  if (!object || message.id < 0 || !std::isfinite(message.center.x) ||
      !std::isfinite(message.center.y) || !std::isfinite(message.yaw) ||
      !std::isfinite(message.vx) || !std::isfinite(message.vy)) {
    return false;
  }

  object->id = message.id;
  object->type = type;
  object->x = message.center.x;
  object->y = message.center.y;
  object->yaw = message.yaw;
  object->speed = std::hypot(message.vx, message.vy);
  object->accel = 0.0;
  object->length = PositiveOr(message.length, 1.0);
  object->width = PositiveOr(message.width, 1.0);
  return true;
}

PerceptionObjects DetectedMsgConverter::toPerceptionObjects(
    const asmc_msgs::DetectedObjectArray& message) {
  PerceptionObjects result;
  result.vehicles.reserve(message.objects.size());
  result.pedestrians.reserve(message.objects.size());
  result.obstacles.reserve(message.objects.size());

  for (const auto& source : message.objects) {
    TrackedObject object;
    switch (source.type) {
      case kVehicle:
        if (toTrackedObject(source, ObjectType::VEHICLE, &object)) {
          result.vehicles.push_back(object);
        }
        break;
      case kPedestrian:
        if (toTrackedObject(source, ObjectType::PEDESTRIAN, &object)) {
          result.pedestrians.push_back(object);
        }
        break;
      case kStaticObstacle:
        if (toTrackedObject(source, ObjectType::STATIC_OBSTACLE, &object)) {
          result.obstacles.push_back(object);
        }
        break;
      case kUnknown:
      default:
        // Classification이 불확실하더라도 geometry를 버리지 않는다.
        // UNKNOWN 상태 자체는 보존하고, 이후 FeatureBuilder가 OBB 크기와
        // 전역경로 주행 corridor의 실제 겹침을 확인한 경우에만 AVOID한다.
        if (toTrackedObject(source, ObjectType::UNKNOWN, &object)) {
          result.obstacles.push_back(object);
        }
        break;
    }
  }
  return result;
}

}  // namespace behavior_planner
