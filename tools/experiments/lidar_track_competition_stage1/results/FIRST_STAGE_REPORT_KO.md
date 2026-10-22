# 1단계 보고서 (GPT 검토용 진입 문서)

- 실험 루트: `/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204` (이하 `$RR`)
- Windows: `\\wsl.localhost\Ubuntu-20.04\home\autonav\2026AIM\Lidar_experimments\track_competition_stage1_20261022_134204`
- branch `exp/lidar-track-competition-stage1-20261022_134204` (worktree `$RR/repo`, 공유 .git = 2026-ASMC)
  - 출발 39cffc57
  - 1단계 commit **669859c1**
  - 로컬 전용, push·merge·rebase 없음. 제품 저장소(2026-ASMC, 2026-ASMC-nogps, Yangtest)는 변경 0
- 작업 시각: 2026-10-22 13:42 KST 시작, 이 보고서는 약 2시간 30분 시점에 작성(1단계 예산 12시간 중)

## 1. 핵심 결론

1. **채택된 과거 코드는 없다. 출발 코드는 P0(916e4d9e + 하네스 39cffc57)로 정했다.** [확인]
   - 실험 01~06은 모두 탈락했고(selected_config null 또는 REJECTED), 실험 commit은 exp branch에만 있다.
   - 정본 P0 출력은 exp03 `work/runs/p0`이며, 6개 실험의 기본 off 출력과 동일하다.
   - 상세: `EXPERIMENT_LINEAGE_KO.md`
2. **HISTORICAL_REPRODUCTION 통과** [실행]
   - P0를 같은 commit·같은 옵션(-O0)으로 다시 빌드해 scen93·scen21·scen58·static_bag를 돌렸다.
   - 결과: jsonl 4/4(1,398줄, timing 제외), pts.bin 4/4 byte 동일, 평가 샘플 동일 (`$RR/work/runs/HR_p0_rebuild/IDENTITY_VS_EXP03_P0.json`)
3. **Competition 입력 조건** [확인·실행]
   - P0 하네스는 모든 자차 입력(IMU, CAN, GPS)을 판정용 ego pose에서 합성했다.
   - 운영 Parser는 `/Ego_topic`(판정 전용) remap과 속도 노름(vel_y·vel_z 포함)을 쓴다. 둘 다 계약 위반이다.
   - 평가 데이터에는 vel_x·IMU·GPS가 없다.
   - 그래서 다음과 같이 처리했다.
     - 허용 채널(vel_x, IMU yaw)만 라벨을 붙여 에뮬레이션했다.
     - roll·pitch는 NaN으로 두었다.
     - GPS는 공급하지 않았다.
     - 하네스가 허용 열만 이름으로 읽도록 했다(`INPUT_CONTRACT.md`, `INPUT_ALLOWLIST.json`).
   - 금지 필드 격리 시험 PASS: 금지 열을 교란하거나 삭제해도 출력이 동일하고, 허용 열 vel_x × 1.1은 출력이 바뀌었다(`INPUT_ISOLATION_TEST.json`).
4. **COMPETITION_BASELINE(CB) ≈ P0** [실행]
   - 114개 장면 + 정지 bag에서 모든 local 지표가 ±0.1% 이내다.
   - 공통 기하의 local 추적은 GPS를 쓰지 않기 때문이다.
   - 대신 map 출력, motion_state, /last_lidar planner 경로는 이 데이터로 평가할 수 없다.
5. **확인된 원인 두 가지를 최소 수정했다(기본 OFF 옵션). 결정론적 원인 시험과 실데이터 모두에서 확인했다.**
   - A (C1): miss 뒤 재획득된 새 트랙의 2번째 관측 속도가 (k+1)배가 되는 구현 오류. Kalman.cpp에서 경과시간으로 나누도록 수정
   - B (C2): 상대 CA 상태에 CAN 즉시값을 더하는 구조라서, 자차 가감속·회전이 가짜 속도가 됨. FrameBoundary에서 vel_x와 IMU yaw로 base_link 원호 변위를 빼 지면 기준 상태로 바꿈
   - 기본 OFF 출력은 P0와 115/115 동일하다(`IDENTITY_DEFAULT_OFF_vs_P0.json`).
6. **A+B 효과 (CB 대비, primary 84개 장면, 동일 평가 계약 v2)** [실행]
   - 개선
     - 속도 오차 MAE 0.881 → 0.560 m/s(−36%), p95 2.45 → 1.73(−29%), p99 6.67 → 4.46(−33%)
     - 정지 객체 가짜 이동(>0.5 m/s) 38.0% → 10.4%
     - 이동 차량 속도 MAE −26%
     - IDSW −5.6%, 트랙 ID −6.9%
   - 변화 없음: TP, FN, FP, 출력 scan, OBB 형상·안정성 지표(±1%)
   - **악화: 이동 차량 속도 추종 지연** 평균 2.02 → 2.25 scan(+11%), 중앙값 2 → 3
7. **A+B 이후 남은 주원인은 측정 기준점(C3)이다.** 클러스터 점 평균이 가시면을 따라 움직인다.
   - 차량 GT 중심 대비 p50 1.46 m, 정지 차량 프레임 간 이동 p95 0.36 m
   - 합성 진단: 잡음이 없어도 주차 차량을 지나치면 정지 객체 가짜 이동이 57%다.
   - 1단계에서는 수정하지 않았다(우선순위 B, 다음 단계).
8. **15 Hz 실시간성** (실제 Main 노드, 15 Hz 절대 deadline 입력, 조용한 머신) [실행]
   - 운영 관례 빌드(-O0)
     - drop 0.4%(2/508), 종단 지연 p95 66~90 ms, p99 69~126 ms, max 146 ms
     - LidarProcess p95 56~67 ms로 주기 66.7 ms와 거의 같다 → **여유가 없다**
     - 카메라 decode, YOLO 결합, GPS/map, MotionClassifier, legacy 발행은 이 측정에 포함되지 않았다.
   - -O2 빌드: drop 0, p95 20~24 ms(약 5배 여유)
   - 병목은 -O0 Eigen 점별 변환(ROI p50 13 ms, leveling 4 ms)과 추적 단계다(`REALTIME_REPORT.json`).

## 2. 실행·검토·미검증 구분

| 구분 | 항목 |
|---|---|
| 실제 실행 | P0 재빌드·4개 장면 재현. 후보 빌드(-O0, -O2). 기본 OFF 동일성 115개 장면. trace ON 중립성 4개 장면. CB/A/B/A+B 115개 장면 + ARM 평가 + 보조 지표. 금지 필드 격리. 합성 10개 장면 × 4 arm(trace 포함). 실데이터 진단 9개 장면 × 2 arm trace. 원인별 시험 9개(정상/강제 OFF). gtest 전체(P0 232, 후보 241 통과). 15 Hz 실시간 10회 |
| 코드 검토만 | legacy 경로(운영 main.launch 기본)의 레버암 누락과 Body2ENU pitch 이중 적용. 운영 Parser의 계약 위반. MotionClassifier 프레임 혼합. dt 가드 부재. /last_lidar 소비자 경로와 valid 처리(STATE_AND_FRAME_CONTRACT.md 7·9절) |
| 미검증 | 실제 IMU roll/pitch, 50 Hz 샘플 지연, GPS 5 Hz 조건. map pose가 필요한 motion_state·planner 속도. 실제 Competition Vehicle Status 패킷(105 B) 디코드. 카메라·YOLO를 포함한 15 Hz 부하. 실제 15 Hz LiDAR. 실차/MORAI 실시간 연결. **독립 최종 평가 데이터(현재 114개 장면은 모두 개발·회귀 데이터)** |

## 3. 비교 결과 (요약, 전체 표는 `$RR/work/analysis/COMPARE_*.md`)

### 3.1 HISTORICAL(P0) 대 COMPETITION_BASELINE(CB) — primary
- 모든 정확도·안정성·ID 지표의 차이는 ±0.1% 이내다.
- not-ready scan 54 → 53. 정차 중 vel_x 음수 샘플은 AddCan이 거부한다(전체 115개 장면에서 CB 118 scan, P0 121 scan).
- map 출력 459 → 0 (scen93 예시). GPS가 없기 때문이다.

### 3.2 CB 대 후보 (동일 Competition 입력, 동일 계약)

| 지표 (primary) | CB | A | B | A+B |
|---|---|---|---|---|
| TP / FN / FP | 53768 / 18130 / 187658 | 동일 | 53769 / 18129 / 187650 | 53769 / 18129 / 187648 |
| 속도 오차 MAE (m/s, kfc ≥ 2 TP 52.4k) | 0.881 | 0.783 (−11%) | 0.602 (−32%) | **0.560 (−36%)** |
| 속도 오차 p95 / p99 | 2.45 / 6.67 | 2.22 / 5.01 | 1.83 / 4.87 | **1.73 / 4.46** |
| 정지 객체 가짜 이동 >0.5 m/s (n = 16.2k) | 38.0% | 31.8% | 11.9% | **10.4%** |
| 가감속 중 / 등속 중 가짜 이동 | 50.2% / 26.9% | 45.8 / 19.2 | 14.6 / 10.8 | 13.1 / 9.2 |
| 회전 중 가짜 이동 | 35.2% | 30.8 | 10.5 | 9.0 |
| 이동 차량 속도 MAE / p95 | 1.03 / 2.86 | 0.96 / 2.67 | 0.79 / 2.10 | 0.76 / 2.05 |
| IDSW / 트랙 ID 수 | 3882 / 24543 | 3820 / 23390 | 3682 / 23585 | 3663 / 22839 |
| 차량 중심 MAE / yaw MAE | 1.058 / 46.3° | 동일 | 동일 | 동일 |
| 차량 yaw 흔들림 p95 | 27.7° | 27.8 | 27.3 | 27.5 |
| OBB 회전 추종 지연 (xcorr scan) | 2.24 | 2.25 | 2.17 | 2.17 |
| **속도 추종 지연 평균 / 중앙값 (scan)** | 2.02 / 2 | 1.91 / 2 | **2.35 / 3** | **2.25 / 3** |
| highspeed 속도 MAE / 정지 가짜 이동 | 1.23 / 90% | 1.12 / 91% | 0.89 / 60% | 0.88 / 60% |
| scen93 속도 MAE | 0.494 | 0.380 | 0.353 | 0.338 |

- 판정: 아직 확정하지 않는다. 평가 계약상 사전 기준이 없고, 평가 장면도 개발·회귀 데이터다.
- 정확도와 안정성은 명확히 개선됐지만 속도 추종 반응은 나빠졌다. 따라서 "흔들림만 낮춘 결과"로 성공을 주장하지 않으며, 지연 대응은 다음 단계에서 다룬다.

### 3.3 결정론적 원인 시험 (DIAGNOSTIC_ONLY, `STAGE1_PATCH_SUMMARY.md`)
- 옵션 ON: 9/9 PASS
- 옵션 OFF(P0 경로): 수정 단언 5개 FAIL
  - 재획득 오차 10 m/s
  - 선회 중 0.65 m/s
  - 가속 중 0.46 m/s
  - 선회 중 이동 차량 0.65 m/s
  - 12 m/s에서 2 scan miss 시 ID 교체
- 가드 3개(등속 직진, 제자리 회전 레버암, miss 없을 때 옵션 중립)는 두 모드 모두 PASS

### 3.4 합성 진단 (DIAGNOSTIC_ONLY, `$RR/work/analysis/SYNTHETIC_REPORT.txt`)
- syn01 자차·객체 정지: 모든 arm에서 오차 0
- syn04 제자리 회전(레버암): 모든 arm에서 p50 0.075
- syn09 제동: 가짜 이동 79% → B 17.5%
- syn02, syn03(주차 차량을 지나침, 잡음 없음): 모든 arm에서 57% → 측정 기준점 문제(C3)
- syn10(자차와 같은 곡률로 도는 선행 차량): B에서 p95 2.32 → 2.77로 악화(C4)
- syn07(불규칙 dt): A 단독 p95 악화. trace로 트랙 계보가 갈라진 결과임을 확인했다(DIAGNOSIS C5).

## 4. 다음 단계 권고와 근거

1. **측정 기준점 개선 (우선순위 B, 최대 효과 예상)**
   - 근거: A+B 이후 남은 정지 객체 가짜 이동과 속도 p95의 주원인이다(실데이터 점 평균 흔들림, 합성 57%).
   - 후보: 가시면을 고려한 박스 기준점(가시 모서리 또는 장축 끝점 + 크기 이력)을 KF 측정으로 쓰고, 관측 품질에 따라 R을 조절한다.
   - 조건: 초기화, 예측, 매칭, 갱신, 출력, 이력을 함께 바꿔야 하므로 사전에 계약을 고정한다.
2. **반응 속도 회복 (우선순위 D와 모델)**
   - 근거: B의 대가인 속도 추종 지연 +11%.
   - 후보: dt 비례 과정잡음(이산 white-noise jerk), 선회 대상에 CTRV/CA 혼합.
   - 평가: 속도 추종 지연, 회전 추종, 정지 가짜 이동을 함께 본다.
3. **운영 입력 계약 수정**
   - 내용: Parser가 velocity.x만 읽고 `/Ego_topic` remap을 제거하며, 음수 vel_x를 처리하게 한다.
   - 선행 조건: Competition Vehicle Status 105 B 형식을 현장 live capture로 확인해야 한다(수정하지 않으면 모든 LiDAR scan이 skip될 위험).
4. **빌드 최적화**
   - 근거: 운영 nested 빌드가 -O0이라 15 Hz 여유가 없다.
   - 내용: -O2와 -O0의 출력 수치 동등성을 확인한 뒤 Release(-O2) 빌드로 전환한다.
   - 근거 수치: -O2에서 종단 지연 p95 약 22 ms, drop 0.
5. **장기 coast 후 재획득 속도 점프(C5)와 legacy 경로 결함**은 별도 원인 시험으로 다룬다.
6. **데이터 공백**
   - 다음을 포함한 10~20 s MORAI 수집이 필요하다: 실제 /imu, /gps, vel_x(Competition), 3D GT(/Object_topic), 2 s 이상 정차, 회전 중 주차 차량.
   - 수집 후 형식·시간·좌표계·GT 정합성을 먼저 검사한다.
   - 수집 절차: `/velodyne_points`(또는 브리지 입력), `/imu`, `/gps`, `/Competition_topic`(디코드가 가능하면), `/Ego_topic`(평가 전용), `/Object_topic`(평가 전용)을 rosbag으로 함께 기록한다. 기존 수집기(morai_3d_live.py)의 GT 변환 규약을 그대로 사용한다.
7. **실험 06 E06의 OBB 안정화 기능**(IDSW −59%, yaw 흔들림 −78%)은 별도 계약 아래 기능 단위로 재구현해 검토한다. 탈락 스택 전체는 가져오지 않는다.

## 5. 남은 시간

- 사용: 약 2.5시간(13:42 → 약 16:15)
- 1단계 예산 12시간 중 약 9.5시간 남음
- 전체 96시간 중 약 93.5시간 남음(마감 2026-10-26 13:42 KST, 마지막 24시간은 판제 연결·MORAI 재생·실시간 검증용으로 확보)

## 6. 실제 실행한 명령 (모두 WSL, `$RR` 기준)

```
# 출발 worktree
git -C /home/autonav/2026AIM/2026-ASMC worktree add -b exp/lidar-track-competition-stage1-20261022_134204 $RR/repo 39cffc57355a76eb35332e0fe70452f36cc70b42
git -C /home/autonav/2026AIM/2026-ASMC-nogps/src/MORAI-ROS_morai_msgs archive 45c6baf1 | tar -x -C $RR/repo/src/MORAI-ROS_morai_msgs
# 빌드 (P0 / 후보 / -O2)
bash scripts/build_outer.sh
OUTER_DEVEL=$RR/repo/devel BUILD_JOBS=4 bash scripts/build_nested.sh p0
OUTER_DEVEL=$RR/repo/devel BUILD_JOBS=6 bash scripts/launch.sh scripts/build_nested.sh cand
OUTER_DEVEL=$RR/repo/devel BUILD_JOBS=3 bash scripts/launch.sh scripts/build_nested_o2.sh cand_o2
# 입력
python3 -B scripts/prepare_competition_inputs.py <exp03>/work/inputs <exp03>/work/static_bag work/inputs_comp
python3 -B scripts/make_isolation_inputs.py work/inputs_comp work/isolation scen58 scen93
python3 -B scripts/make_synthetic_scenes.py work/synthetic --ground-z-base -0.35
# 실행 (각 arm = 격리 namespace + private roscore, UDP 0 확인)
SCENES="scen93 scen21 scen58 static_bag" bash scripts/launch.sh scripts/run_arm2.sh HR_p0_rebuild p0 4 historical
bash work/queue_stage1.snapshot.sh   # ID_OFF_hist, CB, ISO_*, CAND_A/B/AB, SYN_*, TRC_hist
bash work/queue_stage1b.snapshot.sh  # TRC_CB, TRC_AB, 15 Hz 실시간 10회 (rt_job.sh via ns_batch.sh)
#   CAND_A: --set lidar_kf_reacquire_elapsed_init=true ; CAND_B: --set lidar_kf_ego_translation_comp=true
# 평가·분석
python3 -B <exp03>/repo/tools/experiments/lidar_obb_init_turn_follow/preholdout_eval.py arm --pre .../PREHOLDOUT_EVAL_CONTRACT.json --root <exp03>/repo --samples work/runs/<arm>/samples --out work/runs/<arm>/ARM.json
bash scripts/post_stage1.sh          # stage1_metrics, compare_arms, reference point, isolation, synthetic, trace neutrality
bash scripts/make_trace_examples.sh
python3 -B scripts/rt_analyze.py ... ; python3 -B scripts/make_realtime_report.py ; python3 -B scripts/make_manifest.py results/BASELINE_MANIFEST.json
# 시험
bash scripts/run_cause_tests.sh ; bash work/tests/run_both.sh (scripts/run_gtests.sh p0|cand in ns_batch)
```

## 7. 재현·복구

- 소스: `git -C $RR/repo checkout 669859c1` (또는 `git am results/STAGE1_669859c1.patch` on 39cffc57)
- P0 출력 재현: `SCENES="scen93 scen21 scen58 static_bag" bash scripts/launch.sh scripts/run_arm2.sh <NAME> p0 4 historical` 실행 뒤 `identity_check.py`로 exp03 `work/runs/p0/out` 부분집합과 비교
- 기준선과 후보 재실행: `bash scripts/launch.sh scripts/run_arm2.sh <NAME> cand 6 competition [--set ...]`
- 복구
  - 실험 branch와 worktree만 추가했다. 제거는 `git -C /home/autonav/2026AIM/2026-ASMC worktree remove $RR/repo`와 `git branch -D exp/lidar-track-competition-stage1-20261022_134204`다(사용자 승인 후에만 실행).
  - 기존 실험·제품 저장소는 변경하지 않았다(`PROTECTED_SNAPSHOT_BEFORE/AFTER.json` 전 항목 SAME).

## 8. 산출물과 원시 결과 경로

| 산출물 | 경로 |
|---|---|
| 계보 | `$RR/results/EXPERIMENT_LINEAGE_KO.md` |
| 재현 manifest | `$RR/results/BASELINE_MANIFEST.json`, `ENVIRONMENT.txt` |
| 입력 계약 | `$RR/results/INPUT_CONTRACT.md`, `INPUT_ALLOWLIST.json`, `INPUT_ISOLATION_TEST.json` |
| 상태·좌표계 계약 | `$RR/results/STATE_AND_FRAME_CONTRACT.md` |
| 진단 | `$RR/results/DIAGNOSIS_KO.md`, `results/trace_examples/` (9개 객체 × CB/A+B, csv + txt), 도구 `scripts/trace_link.py`, `diag_reference_point.py`, `stage1_metrics.py`, `diag_arm_diff.py` |
| 수정 | `$RR/results/STAGE1_PATCH_SUMMARY.md`, `STAGE1_669859c1.patch`, 원인별 시험 `work/tests/cause_tests_*` |
| 동일성 | `$RR/results/IDENTITY_DEFAULT_OFF_vs_P0.json`, `IDENTITY_TRACE_ON_vs_P0.json`, `work/runs/HR_p0_rebuild/IDENTITY_VS_EXP03_P0.json` |
| 비교 | `$RR/work/analysis/COMPARE_HIST_vs_COMP.{md,json}`, `COMPARE_CANDIDATES.{md,json}`, `stage1_metrics_all.json`, `reference_point_{CB,CAND_AB}.json`, `SYNTHETIC_REPORT.{txt,json}` |
| 실시간 | `$RR/results/REALTIME_REPORT.json`, 원시 `$RR/work/realtime/<run>/` |
| 원시 run | `$RR/work/runs/{HR_p0_rebuild,ID_OFF_hist,CB,CAND_A,CAND_B,CAND_AB,ISO_*,SYN_*,TRC_*}/` (out/*.jsonl, samples/*.pkl, ARM.json, trace/, params.yaml, binary_sha256.txt, jobs.txt, udp_snmp.json) |
| 보호 | `$RR/results/PROTECTED_SNAPSHOT_{BEFORE,AFTER}.json` |
