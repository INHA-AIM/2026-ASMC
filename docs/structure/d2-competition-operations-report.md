# D2 대회·운영 문서 통합 기록

> **상태:** 2026-09-22 통합 완료
>
> **범위:** `docs/competition/`, `docs/operations/`와 직접 연결된 실행 문서

## 결과

| 이전 문서 | 새 정본 | 처리 |
|---|---|---|
| `competition/environment.md` | `competition/README.md` | 규정·환경·미션 요약 통합 |
| `competition/assets.md` | `competition/README.md` | 공개 자산·센서셋·경로·체크포인트 통합 |
| `operations/docker.md` | `operations/README.md` | container·GPU·GUI·장애 대응 통합 |
| `operations/network-udp-ros.md` | `operations/README.md` | 현재 UDP profile·진단·후속 현장 gate 통합 |
| `operations/sim-verification-checklist.md` | `operations/README.md` | 범위별 최소 검증표 통합 |
| `operations/rosbag-evaluation.md` | 같은 경로 | active raw/result 절차만 유지 |
| 위 두 운영 문서의 날짜별 실측 | `records/simulator/runtime-transport-2026-09.md` | rate·latency·GT 성공/실패를 기록으로 분리 |
| `operations/planning-control.md` | 같은 경로 | 당시 standalone 책임과 port 전환 경계 명시; D2-7에서 후속 교정 |

기존 active 문서 7개 1,546줄은 active 정본 4개 610줄로 줄였다. 날짜별 검증 144줄은
현재 설정과 섞이지 않도록 record로 분리했다. 기존 5개 경로에는 외부 링크를 위한 호환
안내만 남겼다.

> **D2-7 후속 교정:** source·launch·package manifest 재감사에서 현재 Planning/Control이
> ROS1/catkin runtime으로 복귀한 것을 확인했다. active 운영 문서는 현재 구조로 교정했고,
> 이 보고서 작성 당시 standalone 내용은
> [historical baseline](../archive/planning/standalone-udp-baseline.md)에 보존했다.

## 현재 상태와 의미 보존

- 규정 v1.1의 PC, UDP 제한, 차량·센서, 미션·패널티, 주행 불능과 당일 절차를 보존했다.
- 공개 asset 4개, MGeo 원본, 전역경로 4,429점, 길이 2,184.61 m, 5% 109.23 m와
  체크포인트 16개·3 m 기준을 보존했다.
- 실행 센서셋은 `config/morai/sensors/ASMC_sensor_set_v2.json`, 현장 `Autonav_sensor.json`은 이전 제출 provenance,
  다른 팀 센서셋은 비정본이라는 경계를 명시했다.
- 현재 개발값 `127.0.0.1`, Ctrl `9111/9112`, Ego `909`, LiDAR 10 Hz와 승인됐지만 미적용인
  Phase 3 목표 `9093/9094`, Status `7603/7502`, field IP, LiDAR 15 Hz를 한 표에서 분리했다.
- 기존 source code/config는 바꾸지 않았다. 포트·센서 주기 전환은 `asmc_bridge` 단계 gate다.
- Docker host network, RGB/LiDAR rate·latency, raw replay 결과, Streaming GT strict pairing
  실패와 direct ROS advertise 실패를 기록 문서에 보존했다.
- Capture, Sensor Record, Real Time UDP bag, privileged GT의 용도를 분리하고 raw/result lineage,
  replay 순서, reject/audit 절차를 유지했다.

원문의 상세 문장은 commit `4a25a94` 이전 이력에서 확인할 수 있다. 현재 실행자가 따라야 할
값은 records가 아니라 `operations/README.md`가 정본이다.

## 검증 결과

- 저장소 내부 Markdown target 402개와 절 anchor 22개가 모두 유효하다.
- 이전 정본 경로를 가리키는 실행 링크가 남지 않았다.
- 현재 launch의 Ego `909`, Collision `9092`, Ctrl `9111`과 v2 LiDAR `9298→9299`, 10 Hz를
  운영 정본과 대조했다.
- `git diff --check`를 통과했고 비어 있는 `docs/` 디렉터리가 없다.
- 문서 변경이므로 package build와 simulator 주행은 수행하지 않음

다음 D2 묶음은 `contracts/`다. Frozen Interface의 수치나 MD5는 변경하지 않고 공용 규약,
HD Map, perception, planning과 simulator 계약의 진입점·중복 설명을 먼저 정리한다.
