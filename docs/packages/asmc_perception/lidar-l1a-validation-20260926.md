# L1-A LiDAR 실험 기준선과 판제 인계 검증 — 2026-09-26 자료, 09-27 마감

> **판정:** 조사·재실행 기준선은 고정, LiDAR→판제 운영 전환은 **미수락**.
> 정차/텔레포트 자료로 터널 벽 잔여의 원인과 frame 계약 문제는 재현했지만,
> 전 미션 검출·추적 정확도나 정상↔음영 연속 주행은 검증하지 않았다.
> 설계 의미는 [판제 공통 소비 계약](../../contracts/perception/lidar-planning-consumer-v1.md),
> 숫자·SHA는 [manifest](../../../config/baselines/lidar_l1a_20260926.yaml)가 정본이다.

## 1. 증거와 재현 경계

| 고정 입력/산출물 | 확인한 사실 | 한계 |
|---|---|---|
| 팀 main `1ce434d`, 손재호 `6435af1`, 서준 `8b3f0b1` | 손재호 detached source worktree가 clean, nested catkin build 통과, 독립 합성 GTest 27개 통과 | catkin의 `54`는 JUnit 이중 집계. 이강태 미공개 음영 코드 및 서준 브랜치 전체 통합 빌드는 미포함 |
| 26R1/ASMC_sensor_set_v3, local network, LiDAR mount `(0.80,0,1.45)m` | source config SHA를 manifest에 고정. live 완성 cloud 약 13.1 Hz; 손재호는 ~12 Hz도 허용한다고 확인 | 사용자 보고 sensor/profile이며 시뮬 UI 순간 상태의 증명은 아님. 15 Hz 메시지 복제 불필요 |
| `$ASMC_DATA/lidar_assessment/20260926/stationary_v2/stationary_0.bag` | 1,968,191,252 bytes, SHA `c4ce3c2afe53167c38dba3f284fe166ef5ebc6cb755581815ea1200eb6c8b076`; 원래 Ego pose/control 복원, 5개 장면 각 약 35초 | stationary/teleport. actor snapshots는 비동기 GT 참고이며 정확한 frame-paired 객체 label 아님 |
| 같은 data root의 `stationary_v2_scene_audit_final.json` | raw/header/출력/터널 진단의 live 관측 5장면 요약 | bag 수신시각과 관측 stamp를 구별. 입력-to-출력 end-to-end latency 단정 금지 |
| `offline_v2/frames.jsonl`, `offline_v2_scene_summary.json` | 동일 bag 2,562 scan, 팀원 evaluator의 baseline/no_protection/camera_missing 3 sweep | `processing_delay=0.13s`로 미래에 도착한 camera/pose state가 보일 수 있어 callback-live 대용 아님 |
| `ground_benchmark_v1/report.json` | Capture 단일 프레임 100개, 각 후보 3회 비교, Patchwork++ `3e6903a1` | 팀원 pipeline 전체 대체 실험 아님. capture pause 때문에 추적·속도 성능 평가 불가 |

bag, actor snapshots, 실행 params, live audit, offline frames/summary, ground report 및 build/test
log의 경로·byte·SHA는 [manifest](../../../config/baselines/lidar_l1a_20260926.yaml)에 있다.
팀 repo에는 bag을 복사하지 않는다. `offline_v2/invocation.json`은 명령·private ROS master·
0.13 s delay를 기록하지만 **그 당시 버전은 evaluator source SHA를 저장하지 않았다**.
현재 원본 revision `6435af1`과 clean detached worktree, bag/params/결과 SHA를 함께 연결한다.
완전한 bit-identical binary provenance가 확보됐다고 표현하지 않는다. 재실행 도구는 이제
resolved `Main` 경로와 source SHA를 기록하며, 기존 결과를 덮어쓰지 않는다.

```bash
# WSL/호스트: SHA, 측정치, 2개 브랜치의 메시지 MD5, 미션/회귀 입력 검사
python3 tools/lidar/validate_l1a_baseline.py --data-root ../data
python3 -m unittest discover -s tools/lidar -p 'test_*.py'

# Noetic 컨테이너: 같은 bag의 scene 감사 재생성 (새 출력 경로만 사용)
python3 /root/ws/tools/lidar/audit_stationary_scenes.py \
  --manifest /data/lidar_assessment/20260926/stationary_v2/manifest.json \
  --output /data/lidar_assessment/<new-run>/stationary_scene_audit.json

# 팀원 evaluator 재실행 전: 손재호 6435af1 nested workspace를 별도 source한다.
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
source /data/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/devel/setup.bash --extend
rospack find Main  # 아래 --expect-main-path와 같아야 한다.
# 도구가 private ROS master를 띄우므로 기존 주행 master를 건드리지 않는다.
python3 /root/ws/tools/lidar/run_teammate_offline.py \
  --bag /data/lidar_assessment/20260926/stationary_v2/stationary_0.bag \
  --params /data/lidar_assessment/20260926/main-live-params.yaml \
  --expect-main-path /data/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/src/Main \
  --processing-delay 0.13 \
  --output /data/lidar_assessment/<new-run>/offline
```

마지막 명령의 `--expect-main-path`는 현재 `asmc-ros-noetic` 컨테이너에서
`rospack find Main`으로 확인한 경로다. mount가 달라지면 그 절대 경로로 지정한다.
evaluator 재실행 값은 CPU 부하와
비동기 state selection으로 변할 수 있으므로 기존 `frames.jsonl`의 SHA와 같아야 한다는
수락 조건은 없다. 동일 bag·source·파라미터로 통계·실패 장면을 비교한다.

## 2. live와 offline이 다른 결정적 이유

GPS-invalid A126 정차 구간에서 YOLO는 약 16.65 Hz, LiDAR는 약 13.1 Hz다.
보호 box는 최근접 camera stamp가 **50 ms 이내**일 때만 유효하지만, 실제 스케줄링에서
그 범위를 벗어난 프레임이 재현됐다.

| 구간 | live 보호 불가 / 진단 frame | 불가 시 YOLO offset | 불가/가능 시 잔여점 중앙값 | 불가/가능 시 cluster 중앙값 |
|---|---:|---:|---:|---:|
| A126 진입 정차 | 112 / 464 | 53.9–63.2 ms | 1,438 / 13점 | 3 / 0개 |
| A126 내부 정차 | 130 / 462 | 50.7–63.6 ms | 1,458 / 4점 | 3 / 0개 |

이때 후보 벽점이 `ambiguous` 보호 경로로 남는 현재 코드와 일치한다. 주변에 동기 GT
차량이 없었으므로 여기서 센서 miss/차량 recall을 판단하지 않는다. 반대로 offline
evaluator는 130 ms processing delay를 두므로 A126 활성 924 frame 모두
`protection_available=true`였다. 따라서 offline에서 잔여점이 적다고 **live 문제가 해결**된
것은 아니다. `camera_missing` sweep는 보호 없는 순수 LiDAR와 같지 않다. fallback이
ambiguous 전방점을 유지하며, `no_protection` sweep도 현 runtime flag OFF와 구현이
완전히 동일하지 않다. evaluator `A`는 이미 tunnel B를 tracker에 투입한 통합 runtime이므로
legacy-only A/B 비교가 아니다. 활성 1,011 frame의 `replica_mismatch`를 정확도 실패로
해석하지 않는다.

실험으로 확인한 것은 **매칭 실패↔벽 잔여점의 강한 동일-input 상관 및 코드 경로**다.
인과 확인을 더 엄밀히 하려면 추후 같은 scan에 camera arrival만 바꾼 deterministic
replay를 수행한다. 타임아웃 완화만으로 고치는 것은 stamp age/지연 위험을 키우므로
다음 단위에서 buffer·관측시각 매칭·valid-empty/missing 분리를 설계한다.

## 3. 지면분할 결과를 읽는 방법

semantic `StandingOBJ(109)`에는 일반 구조물이 대량 포함될 수 있어 차량/보행자/장애물
보존률에서 **분리**했다. 전체 `Obstacle` 163,667점 중 153,904점이 109다. 이 라벨을
합산하면 PCL 단일 plane보다 Patchwork++가 압도적으로 좋아 보이지만, 비교 목적과 다르다.

| Capture 100 frame, 109 제외 | Patchwork++ 센서높이 1.80 m | PCL single-plane 참고 |
|---|---:|---:|
| 실제 obstacle/actor 보존 | 9,014 / 9,763 (92.3%) | 9,087 / 9,763 (93.1%) |
| 낮은 `Obstacle(164)` | 423 / 640 | 332 / 640 |
| `Pedestrian(118)` | 387 / 427 | 420 / 427 |
| ground 제거 | 536,578 / 556,161 (96.5%) | 546,680 / 556,161 (98.3%) |

이 Capture 자료는 과거 **10 Hz/28,800 rays** 설정이므로 v3의 15 Hz 설정·약 13 Hz 완성
cloud와 동일하지 않다. Patchwork++가 낮은 장애물에는 이롭지만 보행자 보존은 악화됐다. GT box는 sparse하고
일부에는 점이 없으므로 per-object AP/recall 결론도 아니다. 현재 손재호 터널 ground
refit 전체를 이 benchmark가 대체해서 측정한 것도 아니다. **교체 보류**가 현재 판정이다.
센서 장착 `+1.45m`와 base-link 노면 `~-0.35m`를 합쳐 실제 센서 노면 높이는 약 1.80m;
A126 live fit 중앙값은 약 -1.77m/-1.795m다. 소스의 fallback -1.25m는 낮은 물체 삭제
위험이 있어 낮은 장애물·경사·높은 차량을 함께 회귀해야 한다.

## 4. 코드상 결함, 관측 현상, 미검증 가설

| 분류 | 내용 | 다음 검증 |
|---|---|---|
| 코드상 계약 불일치 | `/lidar_nogps`는 leveled sensor-origin인데 `velodyne`; `PlannerObjectAdapter`는 pitch만 inverse, sensor→base 오프셋 없음. 무효 GPS에도 `/lidar/tracked_objects_map` 발행. mode 전환 tracker clear. | roll/pitch+mount OBB/속도 roundtrip, GPS valid/stale/invalid/복구 수락 |
| 실제 입력 관측 | 50ms camera miss에서 A126 벽 잔여점 1.4k 이상, 여러 cluster. offline 130ms buffer가 이를 숨김. | 동일 scan 시각/도착 변경 replay, camera empty/late/missing/OFF 분리 |
| 가설·미측정 | 벽 인접 차량 cluster 합쳐짐, 낮은 장애물 삭제, GPS 복귀 점프에 ID 깨짐, 고주로 차로 귀속 흔들림. | 실제 NPC 배치 + 연속 주행 GT/holdout, profile별 latency·CPU 측정 |

## 5. 미션별 입력 가용성과 다음 회귀 fixture

| 미션 | 지금 있는 입력 | 추가로 필요한 수락 입력 |
|---|---|---|
| 일반 도로 | initial 정차 | 연속 주행·일반 차량/보행자 |
| 정적 장애물 교차로 | 없음 | 공식 미션 구간에 정적 객체 배치·우회 주행 |
| 동적 장애물/횡단 | 없음 | 횡단·진입 속도와 occlusion GT가 있는 연속 주행 |
| 로터리 | 없음 | 합류 차량/보행자 및 차로 경계 |
| 고주로 추종·합류·차로 변경 | A423 정차 | 인접 2/3/4차로 차량, 경계 침범, 진입/이탈 연속 주행 |
| 톨게이트 | 없음 | 섬 근처 장애물·합류 차량 |
| 음영 진입 | A126 진입 정차 | GPS 정상→무효 연속 구간, 벽 근접 차량 |
| 음영 내부 | A126 내부 정차 | 지면 낮은 장애물·차량·보행자, camera late/missing/OFF |
| 음영 복귀 | 시작점 복귀 정차 | GPS 무효→정상·점프·수신 중단을 포함한 연속 구간 |

합성 회귀에는 roll/pitch, 센서 offset, OBB/중심/속도 일관성, GPS NaN/stale/점프,
camera 50 ms 경계·valid-empty·미수신, 정지 객체 jitter/ID split·merge/occlusion,
차로 footprint overlap ambiguity를 넣는다. 현재 실패할 것으로 예상되는 사례도
manifest의 `regression_cases`에 `observed_current_failure` 또는 입력 부족으로 명시했다.
첫 좌표·시간 수정의 구체 숫자 입력은
[`l1a_contract_cases.yaml`](../../../tools/lidar/fixtures/l1a_contract_cases.yaml)에 고정했다.
이는 roll/pitch·장착 오프셋·OBB/속도 기대값, GPS/카메라 경계 상태를 포함하며
`test_contract_fixture.py`가 자체 일관성을 검사한다. **현 LiDAR 구현 통과 시험은 아니다.**
**현재 정차 bag은 전체 미션 수락셋이 아니다.**

다음 진행은 ① frame/관측·출력 시간, ② GPS 전환, ③ camera 지연·벽 잔여,
④ HD맵/카메라 주행 관련성, ⑤ 지면·군집·추적 독립 비교, ⑥ 고주로 차로 귀속·전 미션
연속 주행 순서다. 한 단위마다 같은 fixture의 전후 지표·대표 실패·소비자 호환·rollback
기준을 보고하고, 사용자의 검토 후 다음 단위로 넘어간다.
