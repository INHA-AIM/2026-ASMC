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
  // Exact latest matched YOLO observation, separate from confirmation/retention.
  // Unknown or a miss clears it even if str_YoloClass retains an older class.
  std::string observed_class = "unknown";
  std::string observed_raw_class = "unknown";
  bool is_class_locked = false;
  bool multiview_evidence=false, group_conflict=false;
  // Fresh common protection eligibility can survive vehicle/dynamic disagreement.
  // It is not a confirmed class and never derives from measured speed.
  std::string protection_group="unknown";
  std::uint64_t evidence_scan_ns=0;
  std::string evidence_ids; // at most one positive token per view; diagnostics only

};

}  // namespace asmc::lidar::pipeline
