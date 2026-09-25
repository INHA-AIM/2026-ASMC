# P5-C perception runtime acceptance v1

> **문서 상태:** Active Acceptance Gate
>
> **계약 ID:** `asmc_perception_runtime_acceptance_v1`
>
> **기계 정본:** `config/contracts/perception_runtime_acceptance_v1.yaml`

## 1. 목적과 비변경 범위

P5-A/P5-B 뒤의 실제 ROS graph가 문서와 다르게 연결되지 않았는지 반복해서 확인한다. 이번
단계에서는 topic을 rename하거나 판제 consumer를 재작성하지 않는다. 다음 현재 계약을 그대로
고정한다.

- camera semantic → per-view BEV → fused BEV → lane vectorizer
- typed 신호등 `/perception/traffic_light/detection`
- 호환 신호등 `/traffic_light/state`, `/traffic_light/detail`
- 판제의 typed 우선·legacy fallback
- planning 소유 `lane_boundary_adapter`의 read-only typed lane 입력

LiDAR canonical runtime은 담당 팀원의 구현과 handoff가 끝나지 않았으므로 수락 필수항목으로
가장하지 않는다.

## 2. 검사 범위

| 구분 | 펼친 topic 수 | 실행 조건 |
|---|---:|---|
| camera/BEV/lane | 33 | 기본 검사 |
| traffic light | 3 | `--traffic-light` |
| canonical rqt | 2 | `--visualization` |
| 기본-OFF debug | 19 | 기본적으로 광고되지 않아야 함 |

각 필수 topic에서 type, 유일 publisher, 필요한 내부 subscriber, 최소 rate, source stamp 역행,
마지막 수신 age와 지정된 `frame_id`를 확인한다. diagnostics에 ERROR가 있으면 실패한다.
`--require-planning`은 `/behavior_planner`가 typed와 legacy state를 모두 구독하는지 확인하고,
`--require-lane-adapter`는 `/lane_boundary_monitor` 연결을 확인한다. 두 consumer gate는 해당
프로세스를 함께 실행한 수락시험에서만 사용한다.

정적 검사는 host wrapper, canonical launch, semantic include, 판제 source/config와 adapter
source/config의 anchor를 확인한다. 이동이나 topic 오타가 생기면 live 시험 전 CI에서 실패한다.

## 3. 실행

먼저 bridge와 필요한 perception/planning process를 각자의 foreground terminal에서 실행한다.
verifier는 프로세스를 대신 시작하지 않는다.

```bash
# camera/BEV/lane 기본 graph
./scripts/verify_perception.sh

# 기본 graph + 신호등 + canonical rqt 두 화면
./scripts/verify_perception.sh --traffic-light --visualization

# 판제를 같이 띄운 신호등 consumer 연결
./scripts/verify_perception.sh --traffic-light --require-planning

# 판제 팀원이 adapter를 검토할 때만
./scripts/verify_perception.sh --require-lane-adapter
```

측정 창은 기본 15초다. `--duration 30`으로 늘릴 수 있다. 기계 판독 결과를 남길 때
`--yaml-out /root/ws/artifacts/...yaml`을 사용하며, 경로는 ROS container 안의 경로다.

## 4. 구현 시점 검증

다음 정적 검사는 2026-09-25 working tree에서 통과했다.

```text
camera_lane topics       33
all selectable topics    38
default-off topics       19
static source bindings    8
optional consumer gates   2
deferred LiDAR topics     2
unit tests                8 passed
asmc_perception tests    205 passed
```

같은 날 연결된 MORAI camera와 기본 canonical perception을 15.038초 측정한 결과도 통과했다.

| 계층 | 관측 rate | 결과 |
|---|---:|---|
| full-resolution semantic, 각 view | 19.20~19.21 Hz | PASS |
| model-grid road marking, 각 view | 11.29~11.30 Hz | PASS |
| projected BEV semantic, 각 view/layer | 14.76~14.92 Hz | PASS |
| fused BEV 4종 | 9.93 Hz | PASS |
| lane boundary observations | 11.21 Hz | PASS |
| diagnostics 4종 | 0.49~0.50 Hz | PASS |

33개 topic 모두 type, 유일 publisher, 필수 subscriber, nonzero/non-regressing stamp, 지정 frame,
staleness와 diagnostics ERROR 검사를 통과했고 기본-OFF publisher 19개는 광고되지 않았다. 이
실행은 camera/BEV/lane 기본 graph만 검증했다. 신호등·판제·rqt·lane adapter 선택 모드는 해당
process를 함께 띄운 수락시험에서 각각 별도로 수행해야 하며, 기본 live 통과를 그 결과로
확대 해석하지 않는다.

## 5. 다음 gate

- P5-D에서 live 결과와 대표 bag을 이용해 compatibility consumer를 다시 감사한다.
- legacy traffic-light String은 모든 consumer 전환 전 삭제하지 않는다.
- `lane_boundary_adapter`는 판제 담당자가 흡수 여부와 삭제 시점을 결정한다.
- P5-E에서 실행 문서·RViz/rqt·acceptance 명령과 package 이동 인수표를 최종 교차검증한다.
- 웹 콘솔은 실제 수요가 생길 때까지 보류한다.
