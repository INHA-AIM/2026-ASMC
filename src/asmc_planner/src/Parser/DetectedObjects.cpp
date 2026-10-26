#include "Parser/DetectedObjects.hpp"

#include <cmath>

namespace behavior_planner {
namespace {

constexpr std::uint8_t kUnknown = 0;
constexpr std::uint8_t kVehicle = 1;
constexpr std::uint8_t kPedestrian = 2;
constexpr std::uint8_t kStaticObstacle = 3;
constexpr std::uint8_t kMotionDynamic = 3;

double PositiveOr(double value, double fallback) {
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

}  // namespace

bool DetectedMsgConverter::toTrackedObject(
    const planning_inputs::DetectedObject& message, ObjectType type,
    TrackedObject* object) {
  if (!object || message.id < 0 || !std::isfinite(message.center.x) ||
      !std::isfinite(message.center.y) || !std::isfinite(message.yaw) ||
      !std::isfinite(message.vx) || !std::isfinite(message.vy) ||
      message.motion_state > kMotionDynamic ||
      (message.velocity_unavailable && (message.motion_state != 0 || message.vx != 0. || message.vy != 0.))) {
    return false;
  }

  object->id = message.id;
  object->type = type;
  object->motion_state = static_cast<MotionState>(message.motion_state);
  object->velocity_unavailable = message.velocity_unavailable;
  object->x = message.center.x;
  object->y = message.center.y;
  object->yaw = message.yaw;
  object->vx = message.vx;
  object->vy = message.vy;
  // LOW_THREAT는 정적에 준한다. 원본 vx/vy는 진단과 상태 전이를 위해
  // 보존하지만 Behavior의 gap/TTC에는 잡음 속도를 섞지 않는다.
  object->speed = object->motion_state == MotionState::LOW_THREAT
                      ? 0.0
                      : std::hypot(message.vx, message.vy);
  object->accel = 0.0;
  object->length = PositiveOr(message.length, 1.0);
  object->width = PositiveOr(message.width, 1.0);
  return true;
}

PerceptionObjects DetectedMsgConverter::toPerceptionObjects(
    const planning_inputs::DetectedObjectArray& message) {
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
