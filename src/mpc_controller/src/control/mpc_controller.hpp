#pragma once

#include <vector>

#include "common/mpc_control.hpp"
#include "common/mpc_params.hpp"
#include "common/mpc_result.hpp"
#include "common/mpc_state.hpp"
#include "common/reference_path.hpp"

// solver/model/cost/constraint 자체는 그대로 두고(수식 변경 없음), 그
// 위에서 warm-start/정지 데드락 감지/복구 pulse/last_control 갱신을
// 소유하는 계층. mpc_controller_node는 이 클래스를 통해서만 solveMPC를
// 부른다 - trajectory 수신/freshness 검증/ROS 변환/reference resampling/
// ROS publish/fail-safe만 node에 남고, 그 외 제어 상태는 여기로 옮겼다.
class MpcController {
 public:
    explicit MpcController(const MPCParams& params);

    enum class StuckEvent {
        NONE,
        RECOVERY_STARTED,
        RECOVERY_COMPLETE,
    };

    // ego_state/reference는 이미 유효하다고 검증된 뒤(freshness/파싱 성공)
    // 호출한다. 실패(success=false)면 warm_start_/last_control_을 건드리지
    // 않는다 - 호출자가 fail-safe STOP을 publish하고, 다음 사이클에 이전
    // 상태로 다시 시도할 수 있게 한다. stuck_event/stuck_event_*는 node가
    // 기존 ROS_WARN/ROS_INFO 로그를 그대로 남기기 위한 진단값이고, 제어
    // 로직 자체에는 영향을 주지 않는다.
    struct Output {
        bool success = false;
        MPCControl control;
        const char* failure_reason = nullptr;

        StuckEvent stuck_event = StuckEvent::NONE;
        double stuck_event_ego_v = 0.0;
        double stuck_event_max_ref_v = 0.0;
        double stuck_event_path_progress = 0.0;
    };
    Output ControlProcess(const MPCState& ego_state,
                          const ReferencePath& reference);

    // trajectory 유실/stale 등으로 fail-safe STOP을 낼 때 node가 호출한다.
    // warm-start/정지 데드락 상태를 모두 비워 다음 정상 사이클이 이전
    // 실패의 잔상 없이 새로 시작하게 한다.
    void ResetOnFailSafe();

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
