#ifndef FRENET_PIPELINE_TRAJECTORY_REQUEST_HPP
#define FRENET_PIPELINE_TRAJECTORY_REQUEST_HPP

#include <vector>

#include "Frenet/Generator/GeneratorConfig.hpp"

// R4: PlannerCommand와 TrajectoryRequest의 역할을 분리한다.
//
//   PlannerCommand  = FSM이 무엇을 요구했는가 (외부 intent, [A] 그룹)
//   TrajectoryRequest = 이번 cycle에서 어떤 Frenet trajectory를 생성해야
//                       하는가 (Generator가 실제로 필요로 하는 입력)
//
// 각 Maneuver(Keep/Follow/Stop)는 PlannerCommand + 자신의 context를 해석해
// TrajectoryRequest를 만들고, TrajectoryPipeline/Generator는 이 struct만
// 보고 후보를 생성한다 - BehaviorState나 PlannerCommand를 다시 들여다보지
// 않는다.
//
// 그래서 이 struct에는 다음이 들어가면 안 된다:
//   - BehaviorState mode
//   - PlannerCommand(또는 그 일부를 그대로 보존하는 필드)
//   - ROS 메시지 타입
// 이런 걸 넣으면 "이름만 Request로 바뀐 PlannerCommand"가 되어 R4의 목적
// (Generator가 "FOLLOW가 뭔데?"를 몰라도 되게 만드는 것)이 무너진다.
//
// AVOID/LANE_CHANGE/MERGE는 R4 범위 밖이다(lifecycle/target lock/merge
// timing이 복잡해 기본 인터페이스가 안정된 다음에 다룬다) - 그래서 이 struct는
// KEEP/FOLLOW/STOP이 필요로 하는 필드만 담고, MergeArrivalRequest 등은 아직
// 두지 않는다. 필요해지면 R5+ 에서 그 모드를 전환할 때 추가한다.

enum class LongitudinalMode {
    VELOCITY_KEEPING,
    FOLLOWING,
    STOPPING,
};

// 횡방향 목표. 대부분의 모드는 고정폭(search_tolerance=0)으로 target_d
// 하나만 추종한다 - AVOID/LANE_CHANGE처럼 목표 주변을 넓게 탐색해야 하는
// 모드는 tolerance가 의미를 갖는다.
//
// R6-A 감사: hold 여부(STOP lateral hold가 걸렸는지)는 Generator가 필요로
// 하지 않는 lifecycle state였다 - GenerateRequestedCandidates는 target_d/
// search_tolerance만 읽는다. hold 판단은 Maneuver 계층(Keep.cpp)에서 이미
// 끝나고, hold가 걸리면 target_d=hold_d, search_tolerance=0으로 완전히
// 해석된 값만 여기 담긴다. 그래서 이 struct에는 hold 자체를 나타내는
// 필드를 두지 않는다 - 있으면 Generator가 참조하지 않는 죽은 입력이자,
// "target_d==0이 곧 hold 없음"이라는 잘못된 sentinel 해석을 유발할 여지만
// 남긴다.
struct LateralRequest {
    double target_d = 0.0;
    double search_tolerance = 0.0;
};

// FOLLOWING 전용 종방향 생성 입력. leader_s/leader_speed/leader_accel은
// 이미 Frenet으로 투영되고 FollowingContext가 검증한 값이다 - Generator는
// "누가 leader인지"를 몰라도 이 값들로 종방향 후보를 만들 수 있다.
//
// R6-A: leader의 충돌 예외 처리는 여기 두지 않는다. "어떤 물체를 reactive
// coast 연장에서 봐줄지"는 종방향 생성과 무관한 안전 필터 정책이므로
// TrajectoryRequest.collision.coast_exempt_object_id로 옮겼다 - Generator가
// longitudinal_mode==FOLLOWING인지 보고 스스로 추론하지 않게 하기 위함이다.
// 호출자(Keep.cpp)는 여전히 leader_object_id를 이 필드로도 채워 넣지만,
// 그건 collision 쪽 필드고 이 struct 자체는 순수 종방향 생성 입력만 담는다.
struct FollowingRequest {
    double leader_s = 0.0;
    double leader_speed = 0.0;
    double leader_accel = 0.0;

    double time_gap = 0.0;
    double min_gap = 0.0;
    double gap_gain = 0.0;
};

// STOPPING 전용 종방향 생성 입력. stop_s는 이미 phase(APPROACH/BRAKING)
// 판단이 끝난 뒤의 목표 s다 - lifecycle 자체는 이 struct에 없다(Stop.cpp가
// 계속 소유).
struct StopRequest {
    double stop_s = 0.0;
    // STOP 계열은 정지 목표를 지나치지 않도록 delta_s 범위를 제한한다.
    // R5부터 GenerateRequestedCandidates(CandidateCombiner.cpp)가 이 값을
    // 직접 읽어 종방향 후보를 만든다 - 호출자(Keep.cpp)가
    // StopConfig.delta_s_override를 그대로 옮겨 채우므로 단일 출처는
    // 여전히 config지만, generator는 config를 다시 읽지 않고 이 필드만 본다.
    SamplingRange terminal_offset{};
};

// R6-A: 충돌 필터가 필요로 하는 예외 정책을 request.longitudinal_mode로부터
// 추론하지 않고 명시적으로 전달한다. Generator는 이 struct만 읽는다 -
// "지금 FOLLOWING인가"를 보고 coast_exempt를 스스로 정하지 않는다(§3).
//
// coast_exempt_object_id: reactive coast 연장 단계에서 장애물로 보지 않을
// 물체(예: FOLLOW의 leader). margin_recovery_object_id: 현재 pose가 이미
// 확장 margin 안에 있는 AVOID 대상. 실제 OBB 충돌을 허용하는 예외가 아니며
// 이격을 개선해 margin 밖으로 나가는 후보에만 쓰인다. 둘 다 없으면 -1.
struct CollisionRequest {
    int coast_exempt_object_id = -1;
    int margin_recovery_object_id = -1;
};

struct TrajectoryRequest {
    LateralRequest lateral;

    LongitudinalMode longitudinal_mode = LongitudinalMode::VELOCITY_KEEPING;
    double target_speed = 0.0;

    // Optional position-dependent ceiling, already resolved by the maneuver.
    // Empty preserves the existing generator semantics for STOP/FOLLOW/AVOID.
    std::vector<SpeedEnvelopePoint> speed_envelope;

    FollowingRequest following;
    StopRequest stop;
    CollisionRequest collision;
};

#endif  // FRENET_PIPELINE_TRAJECTORY_REQUEST_HPP
