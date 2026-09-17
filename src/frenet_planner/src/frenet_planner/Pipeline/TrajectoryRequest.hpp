#ifndef FRENET_PIPELINE_TRAJECTORY_REQUEST_HPP
#define FRENET_PIPELINE_TRAJECTORY_REQUEST_HPP

#include "frenet_planner/Generator/GeneratorConfig.hpp"

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
// 모드가 이 struct를 쓰게 되면(R5+) tolerance가 의미를 갖는다.
struct LateralRequest {
    double target_d = 0.0;
    double search_tolerance = 0.0;
};

// FOLLOWING 전용 종방향 생성 입력. leader_s/leader_speed/leader_accel은
// 이미 Frenet으로 투영되고 FollowingContext가 검증한 값이다 - Generator는
// "누가 leader인지"를 몰라도 이 값들로 종방향 후보를 만들 수 있다.
// leader_object_id는 예외다: 충돌 필터가 "이 물체는 추종 대상이니 reactive
// coast 연장 단계에서는 장애물로 보지 않는다"를 판단하는 데 필요한 식별자로,
// FOLLOW의 의미(어떤 behavior인지)가 아니라 "어떤 물체를 봐줄지"라는 안전
// 필터 입력이라 여기 남긴다.
struct FollowingRequest {
    int leader_object_id = -1;
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
    // STOP 계열은 정지 목표를 지나치지 않도록 delta_s 범위를 제한한다
    // (CandidateCombiner의 stop_cfg.delta_s_override와 동일한 값).
    SamplingRange terminal_offset{};
};

struct TrajectoryRequest {
    LateralRequest lateral;

    LongitudinalMode longitudinal_mode = LongitudinalMode::VELOCITY_KEEPING;
    double target_speed = 0.0;

    FollowingRequest following;
    StopRequest stop;
};

#endif  // FRENET_PIPELINE_TRAJECTORY_REQUEST_HPP
