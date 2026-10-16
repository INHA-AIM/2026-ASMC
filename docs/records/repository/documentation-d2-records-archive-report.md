# D2 records·archive 문서 통합 기록

> **상태:** 2026-09-22 D2 마지막 묶음 완료
>
> **범위:** `docs/records/`, `docs/archive/`의 역할·시점·현재 행선지와 보존 감사

## 결과

[records index](../README.md)와 [archive index](../archive/README.md)를 추가해 특정
날짜의 사실, 생성 product, 과거 설계와 현재 정본을 분리했다. 기존 Markdown 1,800행과
generated message export 506행은 삭제·축약하지 않았다.

P1-A runtime v1은 현재 HEAD에 계속 맞춰 쓰는 manifest가 아니라 content commit `73c0a8c`의
snapshot임을 명확히 했다. 반면 P2-B compact drivable은 현재 machine source와 product SHA가
일치할 때 사용하는 generated product baseline이다. 두 문서를 모두 `baseline`이라는 이유로
같은 갱신 정책에 두지 않는다.

## 기존 문서와 행선지

| 기존 문서 | 고유 내용 | 처리 |
|---|---|---|
| `records/baselines/runtime-v1.md` | package/topic/message/sensor/bag/HD Map P1-A snapshot | 본문·machine manifest 보존, snapshot commit과 v2 생성 원칙을 index에 명시 |
| `records/baselines/kcity2025-drivable-input-v1.md` | P2-A link별 clip/guard 입력 판단 | 생성 본문 보존, P2-B 선행 audit로 분류 |
| `records/baselines/kcity2025-compact-drivable-v1.md` | 현재 compact product 개수·SHA·소비 규칙 | 생성 본문 보존, current product 검증 조건 명시 |
| `records/hd-map/kcity2025-route-curation-20260917.md` | stage01~14 source 선택과 검수 근거 | 전체 본문 보존, record/current machine policy 경계 추가 |
| `records/perception/twinlitenet-v4-evaluation.md` | epoch 10/18 수치·overlay·한계 | 전체 본문 보존, artifact 외부 보존 규칙 연결 |
| `records/simulator/morai-msgs-versions.md` | beta/26.R1 field 조사와 팀 repo 비교 | 전체 본문 보존, current contract 우선 명시 |
| `records/simulator/runtime-transport-2026-09.md` | host network·rate·latency·GT 성공/실패 | 전체 본문 보존, current operations 우선 유지 |
| `archive/integration/wave1-wave2-plan.md` | 초기 이식 일정·upstream·미완료 판단 | 역사 원문 보존, 현재 planning/roadmap 링크 교정 |
| `archive/perception/agentic-loop.md` | 초기 반복 평가 설계와 metric schema | 역사 원문 보존, archive index 연결 |
| `archive/planning/d1-legacy-behavior-boundary.md` | commit 기반 policy/computation 상세 분류 | 424행 본문 보존, current contract 연결 |
| `archive/planning/standalone-udp-baseline.md` | 이전 standalone 실행 원문 | 전체 보존, 현재 실행 금지 유지 |
| `archive/generated-msg/*.txt` | 외부 검토용 v2 message 사본 | 원본 보존, 정규화 parity와 D3 gate를 별도 README에 기록 |

## 물리 이동을 보류한 package-local 기록

`drivable_bev`, `camera_semantic_perception`, `twinlite_morai`, `data_collection` README의 날짜별
절은 현재 실행·I/O 절과 강하게 연결돼 있다. 이번에 옮기면 문서 개수는 줄지 않으면서 anchor와
문맥만 끊기므로 원문 위치에 보존하고 records index에서 찾도록 했다. 추후 이동은 제목 단위
대응표, compatibility anchor와 word-diff가 준비된 경우에만 수행한다.

## 현재와 다른 사실을 처리한 방법

- runtime v1 `--check`가 현재 HEAD에서 launch rename, `LaneAssociation`, topic/message 변화로
  실패하는 것을 확인했다. v1을 재생성하지 않고 새 v2 freeze가 필요하다고 기록했으며 active
  repository/metadata/tools 가이드의 일상 검증 명령에서 v1 check를 분리했다.
- Wave 기록의 standalone UDP는 당시 결정을 유지하되 현재 ROS 운영 규약과 historical UDP
  baseline을 함께 연결했다.
- generated message export는 filename wrapper·CRLF·마지막 newline만 다르고 현재 source와
  본문 parity가 있음을 확인했다.
- 상위 개인 workspace에만 존재하는 team-status·대회 UI·clone inventory 링크는 portable하지
  않으므로 provenance 경로로 표시했다. 팀에 필요한 upstream commit과 UI 관측 표는 기존
  팀 문서 본문에 그대로 보존했다.
- 날짜별 rate·latency·모델 IoU를 현재 성능이나 본선 보장으로 승격하지 않았다.

## 변경하지 않은 것

- baseline YAML/JSON/NPZ, HD Map geometry와 product SHA
- message source·MD5, sensor/network config와 package 경로
- 실험 수치, 실패 분석, checkpoint·artifact 경로와 열린 질문
- archive의 당시 일정·결정·upstream commit
- generated message export의 byte content

## 검증

- 기존 records/archive Markdown 11개 1,800행과 generated export 506행 보존
- 기존 Markdown 11개의 heading sequence가 변경 전과 동일함을 확인
- runtime v1 Markdown과 YAML이 content commit `73c0a8c`의 byte content와 각각 일치
- generated export 두 개의 정규화 semantic parity 확인
- P2-A `39 links / 984.488 m guarded / 1200.123 m clip candidate`, P2-B deterministic
  product generator check 통과
- records/archive 14개와 연결 진입점 9개의 로컬 target·anchor 검사 통과
- `git diff --check`, 빈 `docs/` 디렉터리 검사

이 묶음으로 D2 디렉터리별 본문 통합을 마친다. 다음 단계 D3에서는 compatibility stub,
generated duplicate와 실제 중복을 대체 정본·외부 참조 gate를 통과한 항목부터 별도 커밋으로
정리한다.
