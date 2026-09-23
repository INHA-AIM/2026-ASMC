# 검증·실험 기록

> **문서 역할:** Active Records Index — 특정 commit·날짜·artifact에서 관측한 사실의 탐색
>
> **주의:** `records/`의 수치와 상태는 현재 설정을 자동으로 뜻하지 않는다. 현재 실행은
> `contracts/`, `packages/`, `operations/`의 연결 문서를 우선한다.

기록은 당시 판단을 재현하고 회귀 비교의 기준점을 제공한다. 새 구현이 생겼다고 기존 결과를
고쳐 쓰지 않으며, 새 측정은 별도 버전 또는 날짜 기록으로 추가한다.

## 1. 기록 종류와 해석

| 종류 | 의미 | 현재 사용 조건 |
|---|---|---|
| Frozen repository snapshot | 한 commit의 package/topic/message/config 상태 | 해당 snapshot commit에서만 byte-for-byte 재생성 |
| Generated product baseline | 기계 정본에서 만든 배포 후보의 hash·개수 | 현재 source와 product build ID·SHA가 일치하고 generator check 통과 |
| Input/audit baseline | 후속 product를 만들기 전 입력·판정 근거 | 결과 product의 정책으로 사용하지 않고 provenance로 참조 |
| Experiment record | 특정 데이터·환경·checkpoint의 측정 결과 | 같은 조건의 비교에만 사용 |
| Version reference | 특정 조사일의 외부 버전·필드 차이 | 현재 contract와 pinned dependency를 다시 확인 |

기록 안에 `다음 단계`, `권장`, `정본`이라는 표현이 있어도 기록 시점의 문맥이다. 현재 팀
결정과 충돌하면 [문서 허브의 정본 우선순위](../README.md#2-정본-우선순위)를 따른다.

## 2. Baseline·HD Map 기록

| 기록 | 시점·식별자 | 보존하는 사실 | 현재 권위 |
|---|---|---|---|
| [P1-A runtime v1](./baselines/runtime-v1.md) | content commit `73c0a8c` | 당시 24 packages, 33 runtime messages, sensor/bag/HD Map snapshot | 현재 구조는 package guide와 contracts |
| [P3-A bridge v1](./baselines/bridge-p3a-v1.md) | 2026-09-23, `p3a_bridge_profiles_v1` | bridge source/topic SHA, 공통 runtime, transport/network/sensor profile | [bridge profile contract](../../config/bridge/bridge_profiles_v1.json) |
| [P3-E local live v1](./baselines/bridge-p3e-local-live-v1.md) | 2026-09-23, `p3e_bridge_local_live_v1` | MolitComp03 v3 packet/rate, azimuth framing, Competition header 불일치 | [bridge profile contract](../../config/bridge/bridge_profiles_v1.json) |
| [P3-F consolidated v1](./baselines/bridge-p3f-consolidated-v1.md) | 2026-09-23, `p3f_bridge_consolidated_v1` | bridge 4→1 package 이관, 663 tests와 post-move local smoke | [bridge package guide](../packages/asmc_bridge/README.md) |
| [P2-A drivable input](./baselines/kcity2025-drivable-input-v1.md) | P2-A, stage14 SHA | link별 clip/guard 입력 판정 | P2-B 결과가 후속 product |
| [P2-B compact drivable](./baselines/kcity2025-compact-drivable-v1.md) | `kcity2025_roundabout_extended_drivable_v5` | Lane/vector/grid 개수와 product SHA | [HD Map runtime contract](../contracts/hd-map/runtime-v1.md)와 현 product hash |
| [route curation 2026-09-17](./hd-map/kcity2025-route-curation-20260917.md) | stage01~14 audit | source별 선택, 사용자 검수와 링크별 보정 근거 | machine policy와 current package guide |

`runtime-v1`은 최초 통합 전 상태를 동결한 snapshot이다. 이후 HD Map launch 이름,
`LaneAssociation`, topic reference와 message가 의도적으로 바뀌었으므로 현재 HEAD에서
`build_runtime_baseline.py --check`가 실패하는 것이 기록 손상을 뜻하지 않는다. v1을
덮어쓰지 말고 다음 저장소 동결 시 `runtime_baseline_v2`를 새로 만든다.

P2-A는 P2-B 이전의 입력 감사이므로 문서 끝의 “다음 gate”를 현재 미완료 작업으로 해석하지
않는다. 현재 주행가능영역은 P2-B product와
[asmc_hd_map guide](../../src/asmc_hd_map/README.md)를 따른다.

## 3. Perception·simulator 기록

| 기록 | 조건 | 보존하는 사실 | 현재 권위 |
|---|---|---|---|
| [TwinLiteNet+ v4](./perception/twinlitenet-v4-evaluation.md) | 2026-09-04~05, v4 val 1,404 samples | epoch 10/18 학습·평가와 한계 | [Perception guide](../packages/asmc_perception/README.md) |
| [MORAI message versions](./simulator/morai-msgs-versions.md) | 최종 조사 2026-09-15 | `beta_drive`·26.R1 field 차이와 direct ROS 실패 | [simulator contract](../contracts/simulator/morai-msgs-beta-policy.md) |
| [runtime transport](./simulator/runtime-transport-2026-09.md) | 2026-09-05~23 | host network, rate/latency, bag, 종료된 GT와 P3-E live 실험 | [운영 가이드](../operations/README.md) |

성능값은 dataset, checkpoint, simulator profile과 측정 창을 함께 인용한다. 예를 들어 당시
LiDAR `8.x Hz` 관측은 현재 10 Hz baseline이나 승인된 15 Hz 목표를 대체하지 않는다.

## 4. Package에 남아 있는 날짜별 로그

다음 기록은 실행법·I/O 변경 과정과 강하게 연결돼 있어 이번 D2 정리에서 물리적으로 옮기지
않았다.

| 위치 | 내용 | 유지 이유 |
|---|---|---|
| `src/perception/drivable_bev/README.md` | 2026-09-10~15 calibration, replay, lane-family와 runtime profile | 현재 명령·topic·handoff 판단과 절 단위로 연결 |
| `src/perception/camera_semantic_perception/README.md` | 2026-09-05 live 실측 | live 실행·진단 문맥에 인접 |
| `src/perception/twinlite_morai/README.md` | v1~v4 학습 변화와 actor 정책 | adapter·loss·배포 후보 설명과 연결 |
| `src/data_collection/README.md` | Capture·bag·cache·manifest 구현 기록 | 현재 수집 절차와 동일 파일에서 관리 |
| `docs/packages/asmc_perception/roadmap.md` | camera/BEV/GT 후보의 시간순 결정 | Working Proposal의 결정 근거 |

이 절들을 `records/perception/`으로 옮기는 것은 문서 수를 줄이지 않고 기존 anchor와 실행
문맥을 끊을 위험이 더 크다. D3에서 이동할 경우 제목 단위 대응표, compatibility anchor와
word-diff를 먼저 준비한다.

## 5. Artifact와 재현 규칙

- 대용량 bag·dataset·checkpoint는 Git에 복사하지 않고 `$ASMC_DATA` 상대 경로, 크기와 SHA를
  기록한다.
- `artifacts/` 링크는 Git 밖의 로컬 산출물일 수 있다. 파일 부재를 결과 부재로 해석하지 말고
  기록된 run ID·SHA와 원본 저장 위치를 확인한다.
- machine-readable YAML/JSON/NPZ가 있으면 Markdown보다 이를 먼저 비교한다.
- 수치를 새로 측정했으면 기존 행을 수정하지 말고 날짜·commit·환경을 포함한 새 record를
  추가한다.
- 실패·중단 결과도 성공 결과와 같은 수준으로 보존한다.

## 6. 새 기록의 최소 항목

1. record ID와 날짜, Git commit
2. simulator·sensor/network profile 또는 dataset/split
3. 입력 artifact 경로·SHA와 생성 명령
4. 측정 구간·sample 수·metric 정의
5. 결과와 실패·제외 조건
6. 현재 contract에 미치는 영향과 미치는 영향이 없는 범위
7. 대용량 artifact의 외부 위치와 보존 담당

현재 지침을 바꾸는 결론은 record에만 남기지 않고 해당 contract/package/operations 문서에
별도 리뷰로 반영한다.
