# D2 본문 축약 정보 보존 감사

> **상태:** 2026-09-22 검토 완료
>
> **대상:** D2-1 온보딩·협업, D2-2 대회·운영 문서

## 결론

문서 줄 수는 크게 줄었지만 현재 구현·운영에 필요한 수치, 경계와 재현 절차가 같은 비율로
사라진 것은 아니다. 반복 설명, 이미 끝난 Phase 0 작업 방식, 현재 존재하지 않는 경로별 파일
목록과 날짜별 실측을 active guide에서 걷어낸 영향이 가장 크다. 이번 감사에서 다음을 원문 절,
표, 명령과 현재 정본을 다시 대조했다.

- 팀원과 GitHub 계정, 현재 package ownership과 공용 변경 review gate
- OS, Docker image, Python/CUDA, 데이터 mount와 pinned `morai_msgs`
- 차량·센서·카메라 pose, 공개 자산, 경로·체크포인트와 채점 수치
- 현재 UDP/IP/Hz와 아직 적용하지 않은 Phase 3 목표값
- raw/result bag lineage, privileged 입력 금지와 날짜별 transport 실측
- 파트별 시뮬레이터 acceptance 기준

현재 팀이 따라야 하는 계약·수치의 누락은 발견하지 않았다. 다만 축약된 시뮬 검증표에서
여전히 유효한 저속/곡률, process/socket 종료, 3-view timestamp·latency, LiDAR sync와 결과 기록
양식은 [운영 가이드](../operations/README.md#7-변경-범위별-시뮬-검증)에 다시 넣었다.

## 온보딩·협업 원문 대응

| 이전 절 | 현재 위치 | 판단 |
|---|---|---|
| `getting-started` 준비·clone·submodule·Docker | [개발 시작](../onboarding/README.md) §1~3 | 현재 명령과 금지사항 보존 |
| `development-environment` OS·언어·image·data | [개발 시작](../onboarding/README.md) §1, §4~5 | 현재 기준만 통합 |
| `team-dev-guide` 계정·담당 영역 | [ownership](../onboarding/ownership.md) §1~2, `CODEOWNERS` | 현재 경로로 교정 |
| `team-dev-guide` 공용 경계·검증 책임 | [ownership](../onboarding/ownership.md) §4~5 | 변경 종류별 gate로 통합 |
| `collaboration`·`git-workflow` 현재 Git 절차 | [협업 규칙](../governance/README.md) §1~6 | 짧은 branch+PR로 단일화 |
| Phase 0 개인 repo, Wave 1/2 | [Wave 기록](../archive/integration/wave1-wave2-plan.md) | 현재 지침에서 제외, 역사 보존 |
| 담당자별 “자주 수정하는 파일” | package README와 [repository layout](../architecture/repository-layout.md) | 빠르게 낡는 중복 목록은 복제하지 않음 |
| IDE 개인 선호, 지난 주간 일정 | 없음 | 계약·재현 정보가 아니므로 의도적 제외 |
| `main` 직접 push Path A | Git 이력과 Wave 기록 | 현재 보호 정책과 충돌하여 의도적 폐기 |

과거 문서의 `src/planner/` 경로처럼 현재 tree와 맞지 않는 문장은 복원하지 않았다. 이후
D2-7 source audit에서 Planning/Control은 현재 ROS1/catkin runtime임을 확인했으며, 실행은
[Planning/Control 규약](../operations/planning-control.md), standalone UDP 설명은
[역사 기준선](../archive/planning/standalone-udp-baseline.md)을 따른다.

## 대회·운영 원문 대응

| 이전 절 | 현재 위치 | 판단 |
|---|---|---|
| 대회 PC·당일 제약·차량·센서·채점 | [대회 환경](../competition/README.md) §1~6 | 규정 수치와 운영 gate 보존 |
| 공개 map/path/scenario/checkpoint | [대회 환경](../competition/README.md) §3~4 | 파일명·개수·경로 수치 보존 |
| Docker image·host network·GUI | [운영 가이드](../operations/README.md) §3, §6 | 실행·장애 대응 통합 |
| UDP/ROS 구조와 UI 포트표 | [운영 가이드](../operations/README.md) §1~5 | 현재값/후속 목표를 분리 |
| 2026-09-05/09 rate·latency·GT 실험 | [transport 기록](../records/simulator/runtime-transport-2026-09.md) | 현재 설정과 분리해 날짜별 사실 보존 |
| raw/result recorder·replay | [rosbag 가이드](../operations/rosbag-evaluation.md) | active 절차와 lineage 보존 |
| 파트별 시뮬 checklist | [운영 가이드](../operations/README.md) §7 | 현재 gate로 압축, 유효 세부 기준 복원 |
| LBC·direct rosbridge 세부 항목 | package/archive와 transport 기록 | 현재 full-stack gate가 아니므로 active 표에서 제외 |
| 오래된 현장 일정·예정 문구 | 공식 기록과 Git 이력 | 지난 일정을 현재 지침으로 복원하지 않음 |

특히 `127.0.0.1`, Ctrl `9111/9112`, Ego `909`, Collision `9091/9092` 방향, v2 sensor
`9290~9303`, LiDAR 10 Hz는 현재 회귀값으로 남아 있다. Ctrl `9093/9094`, Status
`7603/7502`, 현장 `192.168.0.1→192.168.0.10`과 LiDAR 15 Hz는 **승인됐지만 아직 적용하지
않은 Phase 3 목표**로 분리돼 있다. 축약 과정에서 두 상태를 합치지 않았다.

## 의도적으로 보존하지 않은 정보의 기준

아래 내용은 삭제가 아니라 active 문서의 책임 범위에서 제외한 것이다.

1. 현재 tree에서 실행할 수 없는 명령·경로는 현재 가이드로 복원하지 않는다.
2. 날짜가 있는 측정값은 `records/`에 두고 최신 성능으로 일반화하지 않는다.
3. 파일 단위 담당 목록은 `CODEOWNERS`, package README와 실제 tree를 우선한다.
4. 개인 IDE·개인 repo 사용법과 이미 지난 일정은 팀 계약으로 취급하지 않는다.
5. 원문은 각각 `b84335b`, `4a25a94` 시점의 Git blob으로 재검토할 수 있다. Git 이력은
   현재 정본을 대신하지 않으며, 역사적 문장의 원형을 확인하는 보조 수단이다.

## 후속 통합에 적용할 기준

- Frozen contract는 줄 수를 줄이기 위해 의미를 다시 쓰지 않는다.
- active contract와 rollout/experiment 기록이 한 파일에 섞였으면 먼저 역할을 표시하고,
  절 단위 대응표를 만든 뒤에만 이동한다.
- 수치·enum·MD5·topic·좌표계·상태 전이는 기계 판독 정본 또는 구현과 대조한다.
- 호환 안내 파일은 D3 parity와 외부 링크 유예가 끝나기 전까지 유지한다.
