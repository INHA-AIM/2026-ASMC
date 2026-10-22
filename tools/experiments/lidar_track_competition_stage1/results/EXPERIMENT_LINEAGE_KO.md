# 실험 01~06 계보와 이번 출발 코드 선정

작성: 2026-10-22, 실험 루트 `/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204`

## 0. 조사 방법과 근거 수준

- 실험 01~06의 원본 파일(FINAL_RESULT.json, SELECTED_CONFIG.json, COMMITS.txt, 계약 JSON, RUN_ENV, logs, 실행 스크립트)과 공유 저장소 `/home/autonav/2026AIM/2026-ASMC`의 git 이력(`git log`, `git branch -a --contains`, `git rev-parse <c>:path`)을 읽기 전용으로 확인했다.
- 실험별 조사 6건과 P0 실행 경로 조사 1건을 병렬로 수행했다. 이어서 그 결과를 원본 파일로 다시 대조하는 교차 검증을 별도로 수행했다(`work/understand/split/reconcile.txt`). 원자료는 `work/understand/workflow_output.json`에 있다.
- 표기 구분
  - [확인]: 파일·git·해시로 직접 확인함
  - [추정]: 간접 근거만 있음
- 기존 실험 디렉터리의 results 트리 해시와 worktree 상태는 작업 전에 `results/PROTECTED_SNAPSHOT_BEFORE.json`에 기록했다.

## 1. 요약

| 실험 | 루트 | 개발 출발 코드 (B) | 평가 후보 (C) | 판정 | 채택 (D) |
|---|---|---|---|---|---|
| 01 innovation gate | obb_innovation_gate_20261018_113148 | a4c373d2 = 916e4d9e + 하네스 v1 | G1~G5, cand_on G3 (a8215603) | REJECTED_EXPERIMENT | 없음 |
| 02 quality temporal | obb_quality_temporal_20261018_170239 | 916e4d9e (하네스 v2 39cffc57 추가) | A1~A5, B1~B3, C1~C3, D1~D2. 선택 C3 = A1+B2+C3 | REJECTED_EXPERIMENT (holdout scen93 yaw +104%) | 없음 |
| 03 init/turn follow | obb_init_turn_follow_20261019_032258 | 60602a5f (실험 02 기록 HEAD) | S1_R1a~c, S2_R2/R3a~d, S3_R4a/b. 평가 S3_R4a | REJECTED_PREHOLDOUT | 없음 |
| 04 provisional/turn v2 | obb_provisional_turn_v2_20261019_202308 | 30eb0c0b (실험 03 기록 HEAD) | C1~C5 (16개). evaluated C5_D2 | REJECTED_PREHOLDOUT (G13/G14) | 없음 |
| 05 geometry decoupled | obb_geometry_decoupled_20261020_200235 | cea41cf8 (실험 04 기록 HEAD) | 12개. evaluated S3_ALL_noQ | REJECTED_PREHOLDOUT (G13 G14 T01~T03 M01) | 없음 |
| 06 measurement identity | obb_measurement_identity_20261021_045723 | 0b4eddd5 (실험 05 기록 HEAD) | 11개. evaluated S3_F4L3 (바이너리 commit ccfdba25) | REJECTED_PREHOLDOUT (G13 G14 T03 M02) | 없음 |

핵심 결론 [확인]
1. 실험 01~06 중 실제로 채택된 코드나 설정은 없다.
   - 실험 commit은 모두 `exp/*` branch에만 있다.
   - 제품·원격·백업 ref 22개와 Yangtest ref 7개에서 실험 식별자 `git grep` 결과가 0건이다(ObbOutputStabilizer, TrackGuidedFragments, InnovationGate, `lidar_obb_*` 파라미터 등).
   - 모든 FINAL_RESULT에 `selected_config=null`이거나 탈락으로 기록되어 있고, 신규 holdout은 한 번도 실행되지 않았다.
2. 02→06은 "이전 실험 기록 HEAD를 다음 실험 출발점으로 삼은" 선형 사슬이다. 이는 채택이 아니라 실험 계보다. 최신 실험 코드라는 이유로 가장 좋은 코드라고 볼 근거는 없다.
3. 원본 기준선 P0는 하나로 확정된다(2절).

## 2. 원본 기준선 P0 (A) [확인]

| 항목 | 값 |
|---|---|
| 제품 commit | 916e4d9efb38f9066aa4b079df05d16ef011c063 (`/home/autonav/2026AIM/2026-ASMC-nogps` HEAD, branch feature/lidar-nogps-main-integration, status 0줄) |
| 하네스·계측 commit | 39cffc57355a76eb35332e0fe70452f36cc70b42 (parent 916e4d9e). tools 밖의 변경은 `Main/CMakeLists.txt` +5줄(lidar_dataset_replay target)과 `test/lidar_dataset_replay.cpp`(blob 4120ef29) 두 파일뿐 |
| 평가기 | `tools/experiments/lidar_obb_quality_temporal/eval_lib.py` 8053c756…, `EVAL_CONTRACT.json` 71b4ef6f… (obb_quality_temporal_eval_v2), identity_check.py 465de2f1… |
| 빌드 | catkin_make `--only-pkg-with-deps Main`, `CMAKE_BUILD_TYPE` 빈값, `CATKIN_ENABLE_TESTING=ON`, flags.make `-std=gnu++17` (최적화 플래그 없음 = gcc -O0, sanitizer 없음). 제품 nogps nested 빌드도 `CMAKE_BUILD_TYPE` 빈값이므로 같은 관례 |
| 정본 바이너리 | exp03 `work/src_p0/.../exp_p0/devel/lib/Main/lidar_dataset_replay` sha256 cdc2c00e…, libmyHeader.so dc6cc664… |
| 파라미터 | `roslaunch --dump-params Main main_integration.launch planner_udp_enabled:=false` (/Main param 96개, params.yaml 35204d35…). 공통 기하 ON, roll ON. 장면마다 `--set imu_yaw_is_true_enu=false` |
| 실행 | ns_batch.sh(unshare user/net/pid/mount, lo만 사용, private roscore), scan마다 동기 처리(최대 속도, 실시간 pacing 없음), sim time은 ros::Time::setNow(source stamp), UDP 송신 0 |
| 정본 출력 | `/home/autonav/2026AIM/Lidar_experimments/obb_init_turn_follow_20261019_032258/work/runs/p0` (jsonl 115, pts.bin 115, samples 115; ARM.json 8bd475e9…, METRICS.json 144afcd3…) |
| 동일성 | exp01 baseline(하네스 v1), exp02 baseline, exp03 p0, exp04 cand_off, exp05 X_OFF, exp06 X_OFF가 timing(과 exp01의 prof)을 제거하면 115/115 동일하다. "byte-identical"이라는 과거 표현의 정확한 의미는 "timing 제거 후 동일"이며, pts.bin만 진짜 byte 동일이다 |
| 입력 | Lidar_Data 114개 장면 + 정지 bag(291 scan). 하네스 frames 모드는 평가용 ego pose로 IMU(yaw만, roll=pitch=0), CAN(위치 중앙차분 속도의 부호 있는 노름), GPS(ego 위치 + lever arm, age 0)를 합성한다 → Competition 입력 조건을 충족하지 않는다(INPUT_CONTRACT.md) |

주의: 평가기 버전에 따라 같은 P0 출력에서도 숫자가 다르다(v1 scen93 차량 yaw MAE 7.20°, v2 8.67°). 이후 비교는 v2 계약(71b4ef6f)만 사용한다.

## 3. 실험별 계보

### 실험 01 (innovation gate)
- commit: 916e4d9e → a4c373d2(하네스 v1 + 계약 v1) → a8215603(InnovationGate, 기본 off) → 91bcd60d(기록)
- 결과: 개선 기준 0/4. holdout scen93 차량 yaw MAE +48%(v1 평가기). REJECTED_EXPERIMENT
- 넘긴 원인: 흔들림 대부분이 이상치가 아니라 연속적인 가시면 변화다. 이상치 gate 형태는 다시 시도하지 말 것
- a4c373d2는 39cffc57의 조상이 아니다. 이후 실험은 하네스 v2를 쓴다

### 실험 02 (quality temporal)
- commit: 916e4d9e → 39cffc57(하네스 v2 + 평가기 v2) → 990ed2c0(A/B/C/D, 기본 off) → 60602a5f
- 선택 C3(A1 품질 가중 크기·yaw + B2 장축 anchor + C3 track 유도 병합·분할)
- 결과: jitter −50~80%였지만 holdout scen93 yaw 8.67→17.66°(+104%). REJECTED_EXPERIMENT
- 넘긴 원인: 저품질 첫 관측 고정, 실제 회전 시 EMA 지연

### 실험 03 (init/turn follow, pre-holdout)
- commit: 60602a5f → 245925e7(계약 + trace) → 34c9c2d8(R1/R2) → 1ee8fcde → 9f24d231(R3/R4) → c614dd89 → 30eb0c0b
- P0를 39cffc57 export로 다시 빌드해 정본 P0 run을 만들었다(이것이 이후 실험의 P0)
- 평가 S3_R4a. REJECTED_PREHOLDOUT(over-merge +5.6%, E2 short p95, scen93 11개 gate)

### 실험 04 (provisional/turn v2)
- commit: 30eb0c0b → 6f5e649e … adb27db5 → cea41cf8
- evaluated C5_D2(A+B+C+D). G13/G14(scen93)만 실패
- A(병합 근거)와 D(회전 추종)는 재사용 가치가 있다고 기록함

### 실험 05 (geometry decoupled)
- commit: cea41cf8 → d50fda8d → ef6ed519 → 0b4eddd5
- evaluated S3_ALL_noQ. G13 G14 T01~T03 M01 실패
- Q의 축 혼합, SG의 보행자 흡수, Q 없는 회전 tail을 확인함

### 실험 06 (measurement identity)
- commit: 0b4eddd5 → d56a7b54(계약) → ccfdba25(F1~F4, 후보 바이너리 f85921d1) → d209d6e7(기록)
- 사용자 기록과 대조 [확인]
  - FINAL_RESULT.json: final_result REJECTED_PREHOLDOUT, selected_config null, evaluated_config S3_F4L3
  - binary_commit ccfdba257382c7e9e2518ee7c4d5ab75ea348814, experiment05_base 0b4eddd5304dfeffd7ebbf124f8c96242c922ec0, production_base 916e4d9efb38f9066aa4b079df05d16ef011c063
- 정정 사항
  - FINAL_RESULT의 `final_head`(ccfdba25)는 실제 branch head(d209d6e7)가 아니라 바이너리 commit을 뜻한다
  - 후보 바이너리 f85921d1은 commit 전 작업트리에서 링크됐다(05:58:39 < commit 06:00:03). 06:00 재빌드는 compile 0회인 no-op이었다. 따라서 ccfdba25와 내용이 같다는 것은 mtime과 status로 강하게 추정한 것이고, relink로 증명된 것은 아니다
- 정지 bag 차량 yaw p95는 P0 기준 4.06→9.51°(메모리의 2.26→9.51°는 B0 기준)

## 4. 이번 개발의 출발 코드 E 선정

선정: **E = P0 = 916e4d9e + 하네스 39cffc57.** 이 commit에서 새 branch `exp/lidar-track-competition-stage1-20261022_134204`와 worktree `repo/`를 만들었다(`logs/worktree_add.log`: "HEAD is now at 39cffc57").

근거
1. 채택된 코드가 없으므로 "받아들여진 상태"는 P0뿐이다.
2. P0는 서로 다른 하네스 2개, 빌드 6개, 기본 off 실행 6회에서 출력이 같다. 재현성 근거가 가장 강하다.
3. 실험 06 S3_F4L3은 평균 지표가 좋다(primary 차량 흔들림 p95 길이/폭/yaw 0.870/0.625/27.7 → 0.300/0.200/6.18). 그러나 다음 이유로 출발 코드에서 제외했다.
   - REJECTED_PREHOLDOUT이고, 남은 scen93 실패 원인이 물려받은 C3 인계와 provisional 경로에 있다.
   - 탈락 flag 73개와 약 6,000줄이 섞여 새 변경의 효과를 분리할 수 없다.
   - 114개 장면을 6개 실험에 걸쳐 튜닝에 노출했다.
4. S3_F4L3은 고정 비교 arm(E06)으로만 남긴다. 출력·ARM은 보존되어 있다(`obb_measurement_identity_20261021_045723/work/runs/S3_F4L3`, ARM d072ed45…).

P0 대비 이번 branch에 들어간 변경은 `results/STAGE1_PATCH_SUMMARY.md`와 commit 목록에 기록한다. 모든 변경은 기본 OFF이며, 기본 OFF 출력이 P0와 동일함을 실행으로 확인했다(`results/IDENTITY_*.json`).

## 5. 이번 단계의 재현 결과

HISTORICAL_REPRODUCTION은 P0 재빌드로 수행했다(같은 commit, 같은 옵션, 이번 루트의 outer devel).
- 대상 장면: scen93, scen21, scen58, static_bag
- 결과: jsonl 4/4(1,398줄, timing 제거), pts.bin 4/4 byte 동일, 평가 샘플 4/4 동일
- 근거 파일: `work/runs/HR_p0_rebuild/IDENTITY_VS_EXP03_P0.json`
- 재빌드 바이너리 sha256은 cbeea5b6…로 정본(cdc2c00e…)과 다르다. 빌드 경로가 RUNPATH에 들어가므로 예상된 차이다.

## 6. 기억·요약 문서와 원본의 차이 (정정 목록)

- "P0/E2 byte-identical": timing 제거 후 동일이라는 뜻이다(pts.bin만 byte 동일).
- 메모리의 "Reusable: test/lidar_dataset_replay.cpp (exp01)": 실험 02~06이 실제로 재사용한 것은 39cffc57 재작성판(4120ef29)이다.
- exp06 static bag yaw 2.26→9.51°: 기준이 B0다. P0 기준으로는 4.06→9.51°다.
- exp04 C의 "0.381/0.200/8.0": C5_D2(A+B+C+D)의 값이다.
- Yangtest에서 발견된 OBB yaw 이중 회전: 916e4d9e에는 없다(5a32c9ab에서 이미 수정됨). P0에는 해당하지 않는다.
- 사용자 지시의 "39cffc57이 trace 계측 포함": 39cffc57이 추가한 것은 prof와 `--points-out`이다. OBB trace는 245925e7/dba90534(실험 03/04)에 있고 P0에는 포함되지 않는다.
- exp05·06 프롬프트 원문 파일은 run root에서 찾지 못했다.
