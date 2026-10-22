# 1단계 보고서 R2 (GPT 검토용 진입 문서, 2026-10-22 23:30 KST 개정)

- 실험 루트: `/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204` (이하 `$RR`)
- Windows: `\\wsl.localhost\Ubuntu-20.04\home\autonav\2026AIM\Lidar_experimments\track_competition_stage1_20261022_134204`
- branch `exp/lidar-track-competition-stage1-20261022_134204` (worktree `$RR/repo`, 공유 .git = 2026-ASMC, 로컬 전용, push·merge·rebase 없음)
  - 출발 39cffc57 (= P0 916e4d9e + 하네스)
  - 1단계 코드 669859c1, 1차 기록 5c12c346, 이번 개정 기록은 이 문서 끝의 commit 표에 적는다.
- 1차 보고(R1) `FIRST_STAGE_REPORT_KO.md`(16:01 작성)는 그대로 둔다. R2는 R1을 대체하는 진입 문서이며, 바뀐 항목은 **[정정]**·**[보강]**으로 표시한다. 바뀌지 않은 수치는 R1과 같다.

## 0. 이 개정의 성격과 가정

- 2026-10-22 23:15 KST에 1단계 지시문을 다시 받았다. 같은 루트에 1단계 산출물 9종이 이미 있었고(commit 5c12c346), 실행 중인 인스턴스는 없었다.
- 새 실험 디렉터리를 또 만들지 않고 **같은 루트에서 재검증·정정·보강**했다. 1차 산출물은 덮어쓰지 않고 R2 문서와 보강 문서를 추가했다. 이 선택이 지시와 다르다면 다음 단계에서 새 루트로 옮긴다.
- 이번 세션에서 한 일: (1) 1차 산출물 무결성 재검증, (2) 지시문 요구 항목 대조와 공백 보강 2건(원시 OBB 중심 비교, 속도 추종 지연 분해), (3) Competition 채널 식별 정정 1건, (4) 기록 commit. **코드 수정은 없다**(669859c1 그대로).

## 1. 핵심 결론

1. **채택된 과거 코드는 없고 출발 코드는 P0(916e4d9e + 39cffc57)다.** [확인] `EXPERIMENT_LINEAGE_KO.md`. 실험 06 기록(ccfdba25, 0b4eddd5, 916e4d9e)은 원본 FINAL_RESULT·COMMITS와 일치했다.
2. **HISTORICAL_REPRODUCTION 통과.** [실행] P0 재빌드(-O0, 39cffc57) 4개 장면 jsonl·pts.bin·평가 샘플이 exp03 정본 `work/runs/p0`와 동일(`work/runs/HR_p0_rebuild/IDENTITY_VS_EXP03_P0.json`). 이번 세션에서 바이너리 3종 × 3 빌드의 SHA256이 `BASELINE_MANIFEST.json`과 모두 일치함을 다시 확인했다(`STAGE1_REVERIFY_20261022T2315.json`).
3. **Competition 입력 조건.** [확인·실행]
   - 평가 데이터(Lidar_Data 114 장면 + 정지 bag)에는 vel_x·IMU·GPS가 없다. 허용 채널(vel_x, IMU yaw)만 라벨을 붙여 에뮬레이션하고, roll·pitch는 NaN, GPS는 공급하지 않았다(`INPUT_CONTRACT.md`, `INPUT_ALLOWLIST.json`).
   - 금지 필드 격리 시험 PASS: 금지 열 교란·삭제 시 출력 동일, 허용 열 vel_x × 1.1은 출력 변화(`INPUT_ISOLATION_TEST.json`).
   - **[정정] 채널 식별.** Competition Vehicle Status는 PUBSUB **295**(`#MoraiInfo$` 181 B, 확장 필드 없음)이며 저장소 template(molitcomp03)에서 9099 → bridge ego_port → `/Ego_topic`으로 간다. 787은 Ego Detail Status(`#DetailEgoInfo$` 105 B, 판정 전용, 9798 → `/Competition_topic`, 디코드 안 됨)다. R1은 이를 거꾸로 적었다(저장소 문서와 같은 역전). 따라서 R1의 **P1(remap 위반) 주장과 "remap 제거" 권고를 철회**한다. `main.launch`의 `/Competition_topic → /Ego_topic` remap은 올바르다. P2(Parser 속도 노름, vel_y·vel_z 읽음, 부호 손실)는 위반으로 유지한다. 근거 수준과 미재확인 항목은 `INPUT_CONTRACT_ADDENDUM_KO.md`, 기계 판독은 `COMPETITION_CHANNEL_IDENTITY.json`.
   - **[정정·신규 위험 H3]** 이 PC의 MORAI 로컬 네트워크 저장본은 770(판정 전용 Ego Vehicle Status, 229 B, 타이어·슬립 확장 포함) → 9099, 295 → 909다. 이대로 live 실행하면 bridge가 229 B를 디코드해 `/Ego_topic`으로 내보내고 Main이 판정 전용 채널을 **경고 없이** 소비한다. live·MORAI 재생 전에 template(295 → 9099)로 바꾸고 bridge payload 181 B를 확인해야 한다.
   - 1단계 수치에는 영향이 없다(하네스는 vel_x 열만, 실시간 feeder는 velocity.x만 공급).
4. **COMPETITION_BASELINE(CB) ≈ P0.** [실행] 114 장면 + 정지 bag에서 local 지표 차이 ±0.1% 이내(local 추적은 GPS를 쓰지 않음). map 출력·motion_state·/last_lidar는 이 데이터로 평가 불가.
5. **확인된 원인 2개를 최소 수정(기본 OFF 옵션)했고, 원인별 시험과 실데이터 양쪽에서 확인했다.** [실행]
   - A (C1) miss 후 재획득 트랙의 2번째 관측 속도 (k+1)배 → 경과시간으로 나눔(Kalman.cpp)
   - B (C2) 상대 CA 상태 + CAN 즉시값 가산 → vel_x와 IMU yaw로 base_link 원호 변위를 빼 지면 기준 상태(FrameBoundary.cpp)
   - 기본 OFF 출력은 P0와 115/115 동일(`IDENTITY_DEFAULT_OFF_vs_P0.json`). 원인별 시험 9개: 옵션 ON 9/9 PASS, 옵션 OFF(P0 경로) 수정 단언 5개 FAIL(`STAGE1_PATCH_SUMMARY.md`).
   - 기존 회전 정렬이 이미 처리하는 항(유한각 재표현 R(−Δψ)·r = −ω×r 항 포함, 속도·가속도 전달항, 레버암, roll/pitch 변화)과 P0가 빼지 않는 항(자차 병진 R_kᵀΔb)의 구분은 `STATE_AND_FRAME_CONTRACT.md` 4절에 유도했다. "명시적 ω×r 항이 없다"는 사실은 누락이 아니라고 확인했다.
6. **A+B 효과(CB 대비, primary 84 장면, 평가 계약 v2 고정).** [실행]
   - 속도 MAE 0.881 → 0.560 m/s(−36%), p95 2.45 → 1.73, p99 6.67 → 4.46
   - 정지 객체 가짜 이동(>0.5 m/s) 38.0% → 10.4%, 이동 차량 속도 MAE −26%, IDSW −5.6%
   - TP/FN/FP·출력 scan·OBB 형상·안정성 지표 변화 없음(±1%). 검출·발행 감소로 얻은 개선이 아니다.
   - **악화**: 속도 추종 지연 평균 2.02 → 2.25 scan(+11%), 중앙값 2 → 3.
   - **[보강]** 지연 분해(`DIAGNOSIS_ADDENDUM_KO.md` 2절): 자차 정속 구간(n=68)에서는 A+B 1.66 vs CB 1.71로 늦지 않다. 증가는 자차 가감속 구간(n=518, 2.06 → 2.32), 특히 대상이 자차와 함께 가감속하는 동반 운동 구간(1.48 → 1.70)에 한정된다. P0의 즉시 CAN 가산이 그 상황에서만 우연히 유리했음을 수치로 확인했다. 독립 운동 구간(2.54 → 2.77)의 증가는 아직 가설 단계다.
7. **A+B 이후 남은 주원인은 측정 기준점(C3)이다.** [실행]
   - 114 장면: 점 평균의 GT 중심 오차 p50 1.46 m, 정지 차량 프레임 간 이동 p95 0.36 m(GT 0.03). 합성: 잡음 0에서도 주차 차량 지나칠 때 가짜 이동 57%.
   - **[보강] 원시 OBB 중심 비교**(9개 진단 장면, 6,380 차량 TP, `DIAGNOSIS_ADDENDUM_KO.md` 1절): 원시 L-shape OBB 중심은 자차 쪽 치우침을 절반으로 줄이지만(p50 1.34 → 0.69 m, 20~35 m에서는 1.09 m), 정지 차량 프레임 간 이동 p95는 0.539 m로 점 평균 0.541 m와 같다. **점 평균을 OBB 중심으로 교체하는 것만으로는 흔들림(가짜 속도의 원인)이 줄지 않는다.** 다음 단계 기준점은 가시 모서리·가시면 anchor + 크기 이력이어야 하며, 초기화·상태·예측·매칭·갱신·출력·이력을 함께 바꾸는 계약이 먼저 필요하다. 연속 프레임 표: `results/trace_examples/REFPOINT_*.csv`.
8. **15 Hz 실시간성(실제 Main 노드, 15 Hz 절대 deadline 입력).** [실행]
   - 운영 관례 빌드(-O0): drop 0.4%(2/508), 종단 지연 p95 66~90 ms, p99 69~126 ms, max 146 ms, LidarProcess p95 56~67 ms ≈ 주기 66.7 ms → **여유 없음**. 카메라·YOLO·GPS/map·MotionClassifier·legacy 발행은 미포함.
   - -O2: drop 0, p95 20~24 ms. 병목은 -O0 Eigen 점별 변환(ROI p50 13 ms)과 추적 단계.
   - 로그 ON/OFF: -O0에서 trace ON p95 122 ms vs OFF 90 ms, drop 3 vs 2(약 12.7 MB/34 s, 전체 트랙). -O2에서는 23.4 vs 22~24 ms로 차이 없음. 운영에서는 선택 활성화만 쓴다.
   - 과거 "p95 145 ms"는 최대 속도 오프라인 하네스의 경합 포함 처리시간이며 15 Hz 지연이 아니다(사용하지 않음).

## 2. 실행·코드 검토·미검증 구분

| 구분 | 항목 |
|---|---|
| 실제 실행 | P0 재빌드·4 장면 재현. 후보 빌드(-O0, -O2). 기본 OFF 동일성 115 장면. trace ON 중립성 4 장면. CB/A/B/A+B 115 장면 + ARM 평가 + 보조 지표. 금지 필드 격리. 합성 10 장면 × 4 arm. 실데이터 진단 9 장면 × 2 arm trace. 원인별 시험 9개(정상/강제 OFF). gtest(P0 232, 후보 241 통과). 15 Hz 10회. **[R2] 바이너리 SHA256 재검증, 원시 OBB 기준점 비교(8,198 + 8,222 TP), 지연 분해(4 arm × 586~592 세그먼트)** |
| 코드·설정 검토만 | legacy 경로 레버암 누락과 pitch 이중 적용. Parser 속도 노름. MotionClassifier 프레임 혼합. dt 가드 부재. /last_lidar 소비자 경로와 valid 처리. **[R2] bridge 포트·패킷 크기 표, 저장소 network template, MORAI 로컬 저장본 라우팅** |
| 과거 세션 확인(이번 미재확인) | **[R2] 설치된 MolitComp03 시뮬레이터 자산의 PUBSUB 이름(295/770/787). 이번 세션은 UnityPy(Python ≥ 3.9 필요)로 디코드 재확인에 실패**. 코드·설정·과거 관측(9798의 105 B)과는 정합 |
| 미검증 | 실제 IMU roll/pitch, 50 Hz 샘플 지연, GPS 5 Hz. map pose가 필요한 motion_state·planner 속도. **실제 295 패킷 live capture(181 B, velocity.x km/h)**. 카메라·YOLO 포함 15 Hz 부하. 실제 15 Hz LiDAR. 실차/MORAI 실시간 연결. **독립 최종 평가 데이터(114 장면 모두 개발·회귀 데이터)** |

## 3. 비교 결과 요약 (전체 표 `work/analysis/COMPARE_*.md`, `DIAGNOSIS_ADDENDUM_KO.md`)

| 지표 (primary) | CB | A | B | A+B |
|---|---|---|---|---|
| TP / FN / FP | 53768 / 18130 / 187658 | 동일 | 53769 / 18129 / 187650 | 53769 / 18129 / 187648 |
| 속도 오차 MAE (m/s) | 0.881 | 0.783 | 0.602 | **0.560** |
| 속도 오차 p95 / p99 | 2.45 / 6.67 | 2.22 / 5.01 | 1.83 / 4.87 | **1.73 / 4.46** |
| 정지 객체 가짜 이동 >0.5 m/s | 38.0% | 31.8% | 11.9% | **10.4%** |
| 이동 차량 속도 MAE / p95 | 1.03 / 2.86 | 0.96 / 2.67 | 0.79 / 2.10 | 0.76 / 2.05 |
| IDSW / 트랙 ID 수 | 3882 / 24543 | 3820 / 23390 | 3682 / 23585 | 3663 / 22839 |
| 차량 중심 MAE / yaw MAE | 1.058 / 46.3° | 동일 | 동일 | 동일 |
| 차량 yaw 흔들림 p95 | 27.7° | 27.8 | 27.3 | 27.5 |
| 속도 추종 지연 평균 / 중앙값 (scan) | 2.02 / 2 | 1.91 / 2 | 2.35 / 3 | 2.25 / 3 |
| └ 자차 정속 구간 (n=68) [R2] | 1.71 | 1.79 | 1.87 | **1.66** |
| └ 자차 가감속 ∧ 동반 운동 (n=259) [R2] | 1.48 | 1.26 | 1.93 | 1.70 |
| └ 독립 운동 (n=305) [R2] | 2.54 | 2.46 | 2.76 | 2.77 |

기준점 비교(차량, 9개 진단 장면, TRC_CB) [R2]

| 기준점 | GT 중심 오차 p50 (m) | 자차 쪽 치우침 p50 (m) | 정지 차량 프레임 간 이동 p95 (m) |
|---|---|---|---|
| 점 평균(현재 KF 측정) | 1.36 | +1.34 | 0.541 |
| 원시 L-shape OBB 중심 | 0.75 | +0.69 | 0.539 |
| 안정화(출력) OBB 중심 | 0.75 | +0.68 | 0.501 |
| KF 위치 | 1.36 | +1.34 | 0.257 |
| GT | 0 | 0 | 0.020 |

- 판정: 확정하지 않는다. 평가 계약에 사전 합격 기준이 없고 모든 장면이 개발·회귀 데이터다. 정확도·안정성은 개선됐고, 추종 지연의 악화는 자차 가감속 구간에 한정됨을 확인했다. "흔들림만 낮춘 결과"가 아니라는 근거는 TP/FN/FP·출력 scan 불변과 자차 정속 구간 지연 불변이다.

## 4. 다음 단계 권고와 근거

1. **측정 기준점 개선(우선순위 B, 최대 효과 예상).** 근거: C3가 남은 정지 객체 가짜 이동 10%와 속도 p95의 주원인이고, 원시 OBB 중심 교체만으로는 흔들림이 불변(0.54 m)이다. 후보: 가시 모서리·가시면 anchor로 박스 중심을 추정하고 비가시 치수를 이력으로 채우는 측정 모델 + 관측 품질별 R. 선행: 초기화·상태·예측·매칭·갱신·출력·이력 계약 고정. 평가: 정지 차량 프레임 간 이동 p95, 치우침 p50, 20~35 m 구간, 회전 추종을 함께 본다.
2. **반응 속도 회복(우선순위 D).** 근거: B의 지연 증가는 자차 가감속 ∧ 동반 운동 구간(1.48 → 1.70 scan)에 한정된다. 후보: dt 비례 과정잡음(이산 white-noise jerk), 가속 상태 사전 공분산 상향, 혁신 기반 적응. 합격 조건: 해당 범주 지연이 CB 수준으로 돌아오고 자차 정속 범주(1.66)와 정지 객체 가짜 이동(10.4%)이 나빠지지 않아야 한다.
3. **운영 입력 경로 [정정].** Parser가 `velocity.x`만 읽고 음수 vel_x를 처리하게 한다. **remap은 유지한다.** live 전 MORAI 네트워크 저장본을 template(295 → 9099)로 교체하고, bridge ego_port에 Competition 가드(181 B만 허용)를 추가한다. 실제 295 패킷 live capture(181 B, velocity.x km/h)로 확인한다. 저장소 문서(README.md 57·91·95행 등)의 역전 기록은 별도 수정 과제다.
4. **빌드 최적화.** -O2와 -O0의 출력 수치 동등성을 확인한 뒤 Release(-O2)로 전환한다(-O0는 15 Hz 여유 없음).
5. **장기 coast 후 재획득 속도 점프(C5)와 legacy 경로 결함**은 별도 원인 시험으로 다룬다.
6. **데이터 공백.** 10~20 s MORAI 수집(실제 /imu, /gps, 295 vel_x, /Object_topic GT, 2 s 이상 정차, 회전 중 주차 차량)을 먼저 짧게 받아 형식·시간·좌표계·GT 정합을 검사한다. 독립 최종 평가는 이 데이터로만 한다.
7. 실험 06 E06의 OBB 안정화 기능은 별도 계약 아래 기능 단위로 재구현해 검토한다.

## 5. 남은 시간

- 1단계: 13:42 시작, R1 16:15, 이번 세션 23:15~23:45. 1단계 예산 12시간 중 약 2시간 남음(01:42 KST까지). R2 제출로 1단계를 마감하고 GPT 검토를 기다린다.
- 전체 96시간(마감 2026-10-26 13:42 KST) 중 약 86시간 남음. 마지막 24시간(2026-10-25 13:42 이후)은 판제 연결·MORAI 재생·실시간 검증용으로 확보한다.

## 6. 실제 실행한 명령

1차(R1 6절 전체, 요약):
```
git -C /home/autonav/2026AIM/2026-ASMC worktree add -b exp/lidar-track-competition-stage1-20261022_134204 $RR/repo 39cffc57
bash scripts/build_outer.sh ; OUTER_DEVEL=$RR/repo/devel BUILD_JOBS=4 bash scripts/build_nested.sh p0
OUTER_DEVEL=$RR/repo/devel BUILD_JOBS=6 bash scripts/launch.sh scripts/build_nested.sh cand
OUTER_DEVEL=$RR/repo/devel BUILD_JOBS=3 bash scripts/launch.sh scripts/build_nested_o2.sh cand_o2
python3 -B scripts/prepare_competition_inputs.py <exp03>/work/inputs <exp03>/work/static_bag work/inputs_comp
python3 -B scripts/make_isolation_inputs.py work/inputs_comp work/isolation scen58 scen93
python3 -B scripts/make_synthetic_scenes.py work/synthetic --ground-z-base -0.35
SCENES="scen93 scen21 scen58 static_bag" bash scripts/launch.sh scripts/run_arm2.sh HR_p0_rebuild p0 4 historical
bash work/queue_stage1.snapshot.sh ; bash work/queue_stage1b.snapshot.sh
python3 -B <exp03>/repo/tools/experiments/lidar_obb_init_turn_follow/preholdout_eval.py arm --pre ... --out work/runs/<arm>/ARM.json
bash scripts/post_stage1.sh ; bash scripts/make_trace_examples.sh
python3 -B scripts/rt_analyze.py ... ; python3 -B scripts/make_realtime_report.py ; python3 -B scripts/make_manifest.py results/BASELINE_MANIFEST.json
bash scripts/run_cause_tests.sh ; bash work/tests/run_both.sh
```
이번 세션(R2, 모두 WSL `$RR` 기준, 읽기 전용 분석):
```
# 무결성 재검증 (manifest SHA256 vs 현재 바이너리, git 상태, 실행 인스턴스 없음)
python3 -B - <<EOF ... EOF   # -> results/STAGE1_REVERIFY_20261022T2315.json
# 원시 OBB 중심 기준점 비교 (trace 있는 9 장면)
python3 -B scripts/diag_reference_point_raw.py work/runs/TRC_CB work/inputs_comp work/analysis/reference_point_raw_TRC_CB.json \
  --table scen92:8:results/trace_examples/REFPOINT_TRC_CB_scen92_gt8.csv --table scen16:22:... --table scen75:16:...
python3 -B scripts/diag_reference_point_raw.py work/runs/TRC_AB work/inputs_comp work/analysis/reference_point_raw_TRC_AB.json --table ...
# 속도 추종 지연 분해 (4 arm, 114 장면)
python3 -B scripts/diag_speed_lag_split.py work/analysis/speed_lag_split.json work/inputs_comp CB=work/runs/CB CAND_A=work/runs/CAND_A CAND_B=work/runs/CAND_B CAND_AB=work/runs/CAND_AB
# 채널 식별 (읽기 전용): 저장소 template JSON, morai_udp_bridge.py/launch, MORAI 로컬 저장본 JSON 조회
```

## 7. 재현·복구

- 소스: `git -C $RR/repo checkout 669859c1` (또는 `git am results/STAGE1_669859c1.patch` on 39cffc57). R2 문서·도구는 기록 commit(아래 표)에 있다.
- P0 출력 재현: `SCENES="scen93 scen21 scen58 static_bag" bash scripts/launch.sh scripts/run_arm2.sh <NAME> p0 4 historical` 뒤 `identity_check.py`로 exp03 `work/runs/p0/out` 부분집합과 비교.
- 기준선·후보 재실행: `bash scripts/launch.sh scripts/run_arm2.sh <NAME> cand 6 competition [--set lidar_kf_reacquire_elapsed_init=true] [--set lidar_kf_ego_translation_comp=true]`. trace가 필요하면 `TRACE_SCENES="scen92 ..."`.
- R2 분석 재실행: 6절의 두 python 명령(입력은 기존 run 디렉터리, 결과는 `work/analysis/`).
- 복구: 실험 branch와 worktree만 추가했다. 제거는 `git -C /home/autonav/2026AIM/2026-ASMC worktree remove $RR/repo`와 `git branch -D exp/lidar-track-competition-stage1-20261022_134204`(사용자 승인 후에만). 제품 저장소·기존 실험 01~06·MORAI 설정은 변경하지 않았다.

## 8. 산출물과 원시 결과 경로

| 산출물 | 경로 |
|---|---|
| 진입 문서 | `$RR/results/FIRST_STAGE_REPORT_KO_R2.md` (이 문서), R1 `FIRST_STAGE_REPORT_KO.md` |
| 계보 | `$RR/results/EXPERIMENT_LINEAGE_KO.md` |
| 재현 manifest | `$RR/results/BASELINE_MANIFEST.json`, `ENVIRONMENT.txt`, **[R2]** `STAGE1_REVERIFY_20261022T2315.json` |
| 입력 계약 | `$RR/results/INPUT_CONTRACT.md`, `INPUT_ALLOWLIST.json`, `INPUT_ISOLATION_TEST.json`, **[R2 정정]** `INPUT_CONTRACT_ADDENDUM_KO.md`, `COMPETITION_CHANNEL_IDENTITY.json` |
| 상태·좌표계 계약 | `$RR/results/STATE_AND_FRAME_CONTRACT.md` |
| 진단 | `$RR/results/DIAGNOSIS_KO.md`, **[R2]** `DIAGNOSIS_ADDENDUM_KO.md`, `results/trace_examples/` (TRC_* 9 객체 × CB/A+B, **[R2]** `REFPOINT_*` 6개), 도구 `scripts/trace_link.py`, `diag_reference_point.py`, **[R2]** `diag_reference_point_raw.py`, `diag_speed_lag_split.py` |
| 수정 | `$RR/results/STAGE1_PATCH_SUMMARY.md`, `STAGE1_669859c1.patch`, 원인별 시험 `work/tests/cause_tests_*` |
| 동일성 | `$RR/results/IDENTITY_DEFAULT_OFF_vs_P0.json`, `work/analysis/IDENTITY_TRACE_ON_vs_P0.json`, `work/runs/HR_p0_rebuild/IDENTITY_VS_EXP03_P0.json` |
| 비교 | `$RR/work/analysis/COMPARE_HIST_vs_COMP.{md,json}`, `COMPARE_CANDIDATES.{md,json}`, `stage1_metrics_all.json`, `reference_point_{CB,CAND_AB}.json`, **[R2]** `reference_point_raw_{TRC_CB,TRC_AB}.json`, `speed_lag_split.json`, `SYNTHETIC_REPORT.{txt,json}` |
| 실시간 | `$RR/results/REALTIME_REPORT.json`, 원시 `$RR/work/realtime/<run>/` |
| 원시 run | `$RR/work/runs/{HR_p0_rebuild,ID_OFF_hist,CB,CAND_A,CAND_B,CAND_AB,ISO_*,SYN_*,TRC_*}/` |
| 보호 | `$RR/results/PROTECTED_SNAPSHOT_{BEFORE,AFTER}.json` |

commit 표(branch `exp/lidar-track-competition-stage1-20261022_134204`)

| commit | 내용 |
|---|---|
| 39cffc57 | 출발(P0 + 하네스) |
| 669859c1 | 1단계 코드: trace, competition 입력, 옵션 A·B, 원인별 시험 |
| 5c12c346 | 1차 기록(R1 산출물) |
| (R2 기록) | 이 문서, 보강·정정 문서, 분석 도구 2개, 재검증 JSON — 해시는 `git log -1`로 확인 |
