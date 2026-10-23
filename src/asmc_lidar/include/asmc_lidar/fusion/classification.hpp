#pragma once
#include <cstdint>
#include <string>

namespace asmc::lidar::pipeline {
// Camera evidence attached to a track; mobility group is not measured motion.
struct CameraClassEvidence {
  // Camera class state (Global.hpp:530-541). str_ObjectClass is the consumer
  // group, str_YoloClass the original detector label.
  std::string str_ObjectClass = "unknown";
  std::string str_YoloClass = "unknown";
  std::string last_seen_class = "unknown";
  int continuity_counter = 0;
  int class_miss_counter = 0;
  std::uint64_t camera_observation_ns = 0;
  std::uint64_t camera_observation_epoch = 0;
  bool is_class_locked = false;

};

}  // namespace asmc::lidar::pipeline
