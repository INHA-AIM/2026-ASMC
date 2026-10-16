# 역사·레거시 문서

> **문서 역할:** Active Archive Index — 현재 실행에 사용하지 않는 설계·이식·연구 기록
>
> **실행 금지:** archive의 명령, 포트, package 경로와 TODO를 현재 환경에 그대로 적용하지 않는다.

Archive는 실패·변경 이유와 과거 의사결정 맥락을 보존한다. 오래됐다는 이유로 내용을
축약하거나 삭제하지 않지만, 각 기록의 현재 대체 문서를 먼저 확인한다.

## 1. 기록과 현재 행선지

| 역사 기록 | 당시 범위 | 현재 먼저 볼 문서 |
|---|---|---|
| [Wave 1/2 이식 계획](./wave1-wave2-plan.md) | 개인 repo 이식, 초기 package·transport 선택 | [시스템 구조](../../architecture/system-overview.md), [통합 로드맵](../../architecture/consolidation-roadmap.md) |
| [인지 agentic loop](./agentic-loop.md) | Loc-Aid·MGeo bake·BEV 초기 연구 루프 | [Perception guide](../../../src/asmc_camera/docs/integration.md), [현재 roadmap](../../architecture/perception-roadmap.md) |
| [D1 legacy behavior 분석](./d1-legacy-behavior-boundary.md) | commit `fa530f9`의 policy/computation 분류 | [FSM↔Frenet v2 contract](../../contracts/planning-fsm-frenet-v2.md), [Planning guide](../../architecture/planning-control.md) |
| [standalone UDP 기준선](./standalone-udp-baseline.md) | ROS runtime 전환 전 Frenet/MPC 실행 | [Planning 운영 규약](../../operations/planning-control.md) |
| [generated message export](./generated-msg-index.md) | v2 message 외부 텍스트 사본 | `src/asmc_msgs/msg/`와 [message guide](../../../src/asmc_msgs/README.md) |

## 2. 읽는 방법

- 완료되지 않은 checkbox와 `다음 작업`은 당시 계획이며 현재 backlog가 아니다.
- `현재`, `정본`, `권장`이라는 표현은 archive가 작성된 시점을 가리킨다.
- source line number는 문서에 적힌 commit에서만 의미가 있다.
- 외부 개인 repo·워크스페이스 링크는 provenance이며 다른 clone에서 없을 수 있다.
- 현재 구현과 다른 사실은 원문을 고쳐 역사를 바꾸지 않고 상단 안내와 이 index에서 분리한다.

## 3. Generated message 사본

`BehaviorCommand.msg.txt`와 `FrenetFeedback.msg.txt`는 현재 `.msg`의 대체 정본이 아니다.
D2 감사에서 첫 filename 행, CRLF와 마지막 newline을 정규화하면 현재 source와 내용이 같음을
확인했다. D3 삭제 후보지만 외부 인수인계 링크와 parity를 확인하기 전에는 유지한다.

## 4. D3 삭제·축소 gate

Archive를 삭제하거나 상위 워크스페이스 문서로 옮기려면 다음을 모두 확인한다.

1. 대체하는 current document와 마지막 유효 commit을 index에 기록한다.
2. `rg`로 팀 저장소 내부 참조가 0인지 확인한다.
3. 고유한 수치·실패 원인·결정 근거가 current record 또는 Git 이력에서 재현되는지 확인한다.
4. 외부 링크와 handoff 사용자를 확인한다.
5. 삭제는 내용 통합과 다른 커밋으로 수행한다.

이번 D2 묶음에서는 archive 본문과 generated export를 삭제하지 않는다.
