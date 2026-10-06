#pragma once

#include <vector>

#include "Global/Global.hpp"

// Solver/Model/Cost/Constraints 위에서 warm-start/정지 데드락 감지/
// 복구 pulse/last_control 갱신을 소유하는 계층. Control은
// 이 클래스를 통해서만 solveMPC를 부른다 - trajectory 수신/freshness
// 검증/ROS 변환/reference resampling/ROS publish/fail-safe만 Control에 남고,
// 그 외 제어 상태와 최종 solver 진단은 여기서 관리한다.
class MPC {
 public:
    explicit MPC(const MPCParams& params);


    // ego_state/reference는 이미 유효하다고 검증된 뒤(freshness/파싱 성공)
    // 호출한다. 실패(success=false)면 warm_start_/last_control_을 건드리지
    // 않는다 - 호출자가 fail-safe STOP을 publish하고, 다음 사이클에 이전
    // 상태로 다시 시도할 수 있게 한다. stuck_event/stuck_event_*는 Control이
    // 기존 ROS_WARN/ROS_INFO 로그를 그대로 남기기 위한 진단값이고, 제어
    // 로직 자체에는 영향을 주지 않는다.
    MPCOutput MPCProcess(const MPCState& ego_state,
                         const ReferencePath& reference);

    // trajectory 유실/stale 등으로 fail-safe STOP을 낼 때 Control이 호출한다.
    // warm-start/정지 데드락 상태를 모두 비워 다음 정상 사이클이 이전
    // 실패의 잔상 없이 새로 시작하게 한다.
    void ResetOnFailSafe();
    void ResetForPlannerStop(double published_steering);

    const MPCParams& params() const { return params_; }

 private:
    MPCParams params_;
    MPCControl last_control_;
    std::vector<MPCControl> warm_start_;

    // 모드 독립 정지 데드락 감지/복구 (control_frequency 기준 cycle 카운트).
    int stuck_cycles_ = 0;
    int recovery_cycles_left_ = 0;
    int stuck_timeout_cycles_ = 1;
    int recovery_duration_cycles_ = 1;
};
