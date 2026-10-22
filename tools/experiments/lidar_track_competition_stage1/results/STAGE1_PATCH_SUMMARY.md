# 1단계 코드 변경 요약 (branch exp/lidar-track-competition-stage1-20261022_134204)

- 출발점: 39cffc57 (= P0: 916e4d9e + 하네스)
- 1단계 commit: **669859c1** `feat(lidar): add opt-in track trace, competition replay inputs and tracking frame options` (10개 파일, +747/−12, 로컬 전용, push 없음)
- patch: `results/STAGE1_669859c1.patch` (`git format-patch -1 669859c1`)
- 기본 동작 보존 [실행]: 옵션을 모두 끄고 trace를 꺼서 115개 장면을 historical 입력으로 실행한 결과가 정본 P0와 같다. jsonl 115/115(54,895줄, timing 제외), pts.bin 115/115 (`results/IDENTITY_DEFAULT_OFF_vs_P0.json`). trace를 켠 경우의 중립성은 `results/IDENTITY_TRACE_ON_vs_P0.json`에 기록한다.

## 변경 함수와 전후 흐름

| 파일 | 함수 | 변경 | 기본값 |
|---|---|---|---|
| Lidar/Tracking/TrackTrace.hpp (신규) | track_trace::Recorder/Event, tracking_options::Options | 상한 있는 버퍼링 JSONL trace, 트랙 선택. 옵션 플래그 2개 | trace off, 옵션 false |
| Global/Global.hpp | LiDARObjectInfo | tracking_stamp_ns, tracking_ego_speed_mps, kf_elapsed_since_meas_s (기록용 필드) | 동작 영향 없음 |
| Geometry/FrameBoundary.cpp | RecordTrackingFrame, EgoShiftSinceRecord(신규), AlignTrackingFrame | 기록 시각과 CAN 속도 저장. **옵션 B**: yaw frame에서 base_link 원호 변위를 빼고 재표현 | B off면 P0와 같은 식 |
| Lidar/Kalman/Kalman.cpp | predictKalmanPosition, kalman_filter_predict, kalman_filter_update | 정렬 복사본에 새 필드 복사, miss 동안 경과시간 누적. **옵션 A**: miss 뒤 두 번째 관측 속도 = 변위/경과시간, 위치 = z − v·dt. KF trace(prior, 혁신, S, K, posterior) | A off이거나 miss가 없으면 P0 식 그대로 |
| Lidar/Tracking/Tracking.cpp | Objectinfo_for_km, ConsolidateCurrentDetections, CompareCentroids | 검출·병합·연관 후보·할당·OBB 안정화 전후·생성·retire·삭제 trace. 매칭 복사 필드 추가. **옵션 B**에서 절대속도에 CAN을 더하지 않음 | B off면 AbsV = u + v_CAN |
| Lidar/LidarProcess/LidarProcess.cpp | LidarProcess | scan 입력(dt, IMU, CAN, GPS, map 유효)과 단계별 시간 trace | trace off |
| main.cpp | ConfigureTrackingExperiment(신규), TemporalThread, main | 파라미터 `lidar_kf_reacquire_elapsed_init`, `lidar_kf_ego_translation_comp`, `lidar_track_trace_*` 읽기, scan 번호(generation) 설정, 종료 시 flush | 모두 off |
| test/lidar_dataset_replay.cpp | ReadCompetitionFrames, ImuFromRpy(신규), Replay::TracePublish(신규), main | `--input-mode competition`(허용 열만 읽고 GPS는 공급하지 않음), `--track-trace*`, `--run-id`, 발행/미발행 사유/planner 값 trace | historical(P0와 같음) |
| test/test_tracking_frame_options.cpp (신규), CMakeLists.txt | – | 원인별 시험 9개 | – |

## 원인별 시험 (수정 전 실패, 수정 후 통과)

- 실행: `scripts/run_cause_tests.sh`. 결과는 `work/tests/cause_tests_{normal,force_off}.{log,xml}`, `cause_tests_rc.txt`에 있다.
- 체인: 운영 FrameFilter → CompareCentroids → Kalman → frame_boundary를 이상적 측정으로 실행한다(DIAGNOSTIC_ONLY).

| 시험 | 옵션 ON | 옵션 OFF (= P0 경로) |
|---|---|---|
| FixReacquiredNewTrackVelocityUsesElapsedTime (자차 5 m/s, 새 트랙 2 scan miss) | PASS (오차 < 0.05 m/s) | **FAIL: 주차 차량 속도 오차 10 m/s** |
| FixParkedCarHasNoFakeSpeedWhileEgoTurns (v = 8, ω = 0.3) | PASS | **FAIL: 0.65 m/s** |
| FixParkedCarHasNoFakeSpeedWhileEgoAccelerates (2 m/s²) | PASS | **FAIL: 0.46 m/s** |
| FixMovingCarGroundVelocityWhileEgoTurns | PASS | **FAIL: 0.65 m/s** |
| FixFastEgoMissedParkedCarKeepsId (12 m/s, 2 scan miss) | PASS | **FAIL: ID 100 → 102** |
| GuardP0ReacquireDefectIsReproduced | PASS (P0 결함 고정) | 건너뜀 |
| GuardReacquireOptionIsNeutralWithoutMisses | PASS | PASS |
| GuardStraightConstantSpeedParkedCarIsStillInBothModes | PASS | PASS |
| GuardRotationInPlaceLeverArmHandledInBothModes | PASS | PASS |

옵션 OFF 실행은 `TRACKING_OPTIONS_FORCE_OFF=1`로 했다. 이때 바이너리 코드 경로는 P0와 같다(위 동일성 결과로 확인).

## 외부 구현 참고

없음. 모든 규칙(원호 변위 Δb, 경과시간 기반 초기화, trace 형식)은 이번 단계에서 직접 작성한 자체 규칙이다. 원호 변위는 일정 곡률(unicycle) 운동학의 표준 식이다.
