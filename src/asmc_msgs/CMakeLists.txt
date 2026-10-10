cmake_minimum_required(VERSION 3.0.2)
project(asmc_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  VehicleState.msg
  DetectedObject.msg
  DetectedObjectArray.msg
  LaneBoundaryObservation.msg
  LaneBoundaryObservationArray.msg
  LaneBoundaryFamilyMember.msg
  LaneBoundaryFamily.msg
  UrbanFeatureDebug.msg
  BehaviorContext.msg
  TrajectoryPoint.msg
  Trajectory.msg
  PlanFeedback.msg
  # v2 인터페이스 (FSM <-> Frenet). FSM 담당자가 정의한 원본을 그대로 등록한다.
  # 필드/상수를 Frenet 편의로 수정하지 않는다
  # (docs/contracts/planning/fsm-frenet-v2.md 1.4절).
  BehaviorCommand.msg
  FrenetFeedback.msg
  # HD Map / competition metadata contract v1. These are additive message
  # types; do not edit the legacy messages above because their ROS1 MD5 sums
  # are already consumed by the planning stack and recorded in P1-A.
  HdMapMeta.msg
  LaneMarking.msg
  LaneMarkingArray.msg
  Lane.msg
  LaneArray.msg
  LaneAssociation.msg
  DrivableArea.msg
  DrivableAreaArray.msg
  CompetitionCheckpoint.msg
  CompetitionCheckpointArray.msg
  MissionZone.msg
  MissionZoneArray.msg
  # Phase 4 perception contracts.  These are additive replacements for the
  # legacy traffic-light String and nested Msg/TrackedObject* interfaces.
  # The legacy producers/topics remain available during consumer cutover.
  TrafficLightDetection.msg
  TrackedObject.msg
  TrackedObjectArray.msg
  # LiDAR/camera handoff types: explicit validity. Producers are implemented
  # during Jaeho's migration; existing operational message MD5s stay frozen.
  LidarObject.msg
  LidarObjectArray.msg
  CameraObjectBox.msg
  CameraObjectBoxArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  std_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_message_serialization.py)
endif()
