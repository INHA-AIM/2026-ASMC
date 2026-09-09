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
  BehaviorContext.msg
  TrajectoryPoint.msg
  Trajectory.msg
  PlanFeedback.msg
  # v2 인터페이스 (FSM <-> Frenet). FSM 담당자가 정의한 원본을 그대로 등록한다.
  # 필드/상수를 Frenet 편의로 수정하지 않는다
  # (docs/fsm-frenet-interface-v2.md 1.4절).
  BehaviorCommand.msg
  FrenetFeedback.msg
  # D7: Route/FSM -> Frenet 전역 경로 전달 계약(§19). 실차 production Route
  # Manager 연결은 이번 작업 범위가 아니다(§20) - Frenet 쪽 수신/검증
  # infrastructure만 추가한다.
  GlobalPath.msg
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
