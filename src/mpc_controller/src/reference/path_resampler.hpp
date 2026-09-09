#pragma once

#include <cstddef>
#include <vector>

#include "common/reference_path.hpp"

// frenet_planner_node로부터 받는 "PlannedPath" 패킷의 파싱 결과.
// frenet_planner 패키지가 MORAI의 ego/object를 받아 로컬 경로를 계산하고,
// 그 경로 + 중계용 ego 상태(ego)를 이 프로세스로 보내준다
// (message_converter.cpp가 파싱). mpc_controller는 자체 EgoInfoReceiver를
// 두지 않고 이 값을 그대로 쓴다.
struct PlannedPath {
    std::vector<double> x, y, yaw, kappa, v, a;  // frenet_planner의 CartesianPath와 동일 필드
    double dt = 0.0;      // 샘플 간격 [s] (frenet_planner.sample_dt())
    double d = 0.0;       // 진단용: 차선 중앙 기준 횡방향 오프셋
    double d_dot = 0.0;   // 진단용: 횡방향 속도

    // 중계된 ego 상태 (frenet_planner가 MORAI에서 받은 값 그대로 전달)
    double ego_x = 0.0, ego_y = 0.0, ego_yaw = 0.0;
    double ego_v = 0.0, ego_steer = 0.0, ego_accel = 0.0;

    std::size_t size() const { return x.size(); }
    bool empty() const { return x.empty(); }
};

// start_time_offset [s]: path.x[0]/y[0]/... 기준 t=0에서 얼마나 앞선 시점부터
// 참조를 시작할지. trajectory 생성 시각과 현재(x0) 시각 사이의 age를 보정하는
// 용도다 (기본값 0.0이면 기존과 동일). offset이 path 길이를 넘어가면 마지막
// 점으로 clip한다 - out-of-range access가 발생하지 않는다.
ReferencePath ToReferencePath(const PlannedPath& path, double destination_dt,
                              int steps, double start_time_offset = 0.0);
