#pragma once

#include <cstddef>
#include <vector>

#include "common/reference_path.hpp"

// Trajectory 메시지(asmc_msgs::Trajectory)에서 points만 뽑아 만든 순수 경로.
// message_converter.cpp의 ConvertTrajectory가 asmc_msgs::TrajectoryPoint[]를
// 이 구조로 옮긴다. Trajectory.ego_at_plan(계획 당시 Ego snapshot)은 여기
// 담기지 않는다 - provenance 검증에만 쓰이고 MPC의 control 입력으로는
// 쓰이지 않기 때문이다. ControlProcess의 x0(현재 상태)는 별도로 최신
// /vehicle/state를 ConvertVehicleState로 변환해서 만든다.
struct PlannedPath {
    std::vector<double> x, y, yaw, kappa, v, a;
    // a는 Frenet CartesianPath의 kinematic/diagnostic acceleration을 그대로
    // 보존한다. MPC actuator reference는 이 배열이 아니라 ToReferencePath가
    // 최종 destination grid의 v_ref를 미분해 만드는 ReferencePath::a_ref다.
    double dt = 0.0;      // 샘플 간격 [s] (frenet_planner.sample_dt())
    double d = 0.0;       // 진단용: 차선 중앙 기준 횡방향 오프셋
    double d_dot = 0.0;   // 진단용: 횡방향 속도

    std::size_t size() const { return x.size(); }
    bool empty() const { return x.empty(); }
};

// start_time_offset [s]: path.x[0]/y[0]/... 기준 t=0에서 얼마나 앞선 시점부터
// 참조를 시작할지. trajectory 생성 시각과 현재(x0) 시각 사이의 age를 보정하는
// 용도다 (기본값 0.0이면 기존과 동일). offset이 path 길이를 넘어가면 마지막
// 점으로 clip한다 - out-of-range access가 발생하지 않는다.
ReferencePath ToReferencePath(const PlannedPath& path, double destination_dt,
                              int steps, double start_time_offset = 0.0);
