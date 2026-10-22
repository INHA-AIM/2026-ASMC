# Competition 입력 계약과 제한 어댑터

기계 판독용 허용 목록은 `results/INPUT_ALLOWLIST.json`이다. 금지 필드 격리 시험 결과는 `results/INPUT_ISOLATION_TEST.json`에 있다.

## 1. 규칙

추론에 허용되는 입력은 다음과 같다.
- Competition Vehicle Status의 **vel_x**
- 실제 제공되는 **자세와 각속도**(IMU 센서)
- 별도 **GPS** 센서

추론에 쓸 수 없는 입력은 다음과 같다.
- Competition Vehicle Status의 pos_x/y/z, vel_y, vel_z, accel_x/y/z
- 타이어 횡력·슬립각 등 제공되지 않는 필드
- 판정 전용 채널(Ego Vehicle Status `/Ego_topic`, Ego Detail Status)

평가 전용 데이터는 다음과 같다.
- 객체 GT(labels_3d_lidar_v1)와 완전한 자차 정답(Lidar_Data ego pose)
- 매칭, 관측 선택, 필터, 분류, 온라인 파라미터 조절, 최종 출력에 쓰지 않는다.

출처
- 사용자 지시(2026-10-22)
- 2026-ASMC-nogps `docs/competition/README.md:70-71,86-91`
- `docs/competition/rule-v1.2-impacts.md:12-24`
- `config/morai/README.md:57-73`

## 2. 실제 bridge 필드 확인 결과 [코드·문서 확인]

| 채널 | 원천 → topic | 단위·축 | timestamp | 현재 상태 |
|---|---|---|---|---|
| vel_x | Competition Vehicle Status (MolitComp03 type 787, UDP 9797→9798) → morai_udp_bridge → `/Competition_topic` (morai_msgs/EgoVehicleStatus.velocity.x) | m/s (ego_velocity_in_ms=true), 차체 전방 | 브리지 수신 시각(기본 timestamp_mode) | **미확정**. 2026-10-17 local에서 관측한 패킷은 `#DetailEgoInfo$` 105 B로, 브리지가 기대하는 `#MoraiInfo$` 152 B와 달라 디코드되지 않았다. `/Competition_topic` 0건. 현장 패킷은 live capture로 확인해야 한다 |
| IMU 자세·각속도 | `#IMUData$` → morai_sensor_bridge → `/imu` (sensor_msgs/Imu) | quaternion, rad/s, m/s²(중력 포함) | 수신 시각 | 사용 가능. EgoVehicleStatus.msg에는 angular_velocity 필드가 없다 |
| GPS | NMEA GGA → `/gps` (morai_msgs/GPSMessage) | 위경도, status | 수신 시각 | 사용 가능. 실측 약 5 Hz(setting.yaml에는 30) |

## 3. P0의 계약 위반 지점 [코드]

**제품 (916e4d9e)**
- P1: `main.launch:126-128`이 `/Competition_topic`을 판정 전용 `/Ego_topic`으로 remap한다.
- P2: `Parser.cpp:532-533`이 v = sqrt(vx² + vy² + vz²)를 쓴다(부호 없음, vel_y·vel_z 포함). 이 값이 다음 경로로 퍼진다.
  - SensorPoseHistory 게이트·외삽
  - Tracking 절대속도
  - MotionClassifier
  - planner 속도·관련성 필터
- pos_xyz, accel, heading은 제품 경로에서 읽지 않는다.

**P0 하네스 (frames 모드)**
- H1: 평가용 ego pose만으로 IMU(yaw, roll = pitch = 0), CAN(위치 중앙차분의 부호 있는 노름, 미래 프레임 사용), GPS(위치 + lever arm, 매 scan age 0)를 합성한다. 실험 01~06의 모든 추론 입력이 판정 전용 pose에서 나왔다.
- H2: bag 모드도 EgoVehicleStatus의 속도 노름을 쓴다.

## 4. 데이터의 허용 입력 보유 여부 [데이터 확인]

- Lidar_Data 114개 장면과 원본 dataset
  - ego_pose CSV에는 frame_id, timestamp, ego_x/y/z, heading만 있다.
  - 수집 topic은 /Ego_topic, /Object_topic, 카메라, /lidar3D뿐이다.
  - vel_x, IMU, GPS는 **없다**.
- 실제 /imu·/gps·/Ego_topic이 있는 bag(shahn03 13개, tunnel, lidar_manual)에는 3D 객체 GT가 **없다**.
- 정지 GT bag은 /imu quaternion이 모두 0이다.
- 결론: 정답이 있는 평가 데이터에서는 허용 입력을 그대로 쓸 수 없다. 그래서 다음과 같이 **명시적으로 라벨링한 에뮬레이션**을 사용하고, 가능한 진단 범위를 제한한다.

| 채널 | 이번 단계 처리 | 라벨 | 한계 |
|---|---|---|---|
| vel_x | 평가 pose의 중앙차분 속도를 heading 방향으로 사영(vel_y 제외), t_lidar 시각(age 0) | VEL_X_EMULATED_FROM_EVAL_POSE | 시각이 낙관적이다(실제는 최대 20 ms 앞선 50 Hz 샘플). bag 실측에서 중앙차분과 vel_x의 차이는 p95 0.3~0.9 m/s(수신 시각 매칭 지터 포함) |
| IMU yaw | 평가 heading을 그대로 사용 | IMU_YAW_EMULATED_FROM_EVAL_HEADING | bag 실측 IMU yaw − Ego heading p95 0.2~0.8°. 시각은 낙관적(age 0) |
| IMU roll/pitch | **없음**. frames_comp.csv에 NaN으로 기록하고 0으로 채우지 않는다 | rp_available = 0 (scan마다 기록) | 하네스는 수평 가정(roll = pitch = 0)으로 leveling을 수행한다. 실제 roll·pitch(최대 약 3°)에서 leveling 경로는 검증되지 않는다 |
| 각속도 | 없음 (NaN). 추론 경로도 사용하지 않음 | – | – |
| GPS | **공급하지 않음**. 평가 pose를 GPS처럼 쓰지 않는다 | gps = 0 | map pose가 무효다. map 출력, motion_state, /last_lidar·UDP 경로는 이 데이터로 평가할 수 없다 |

## 5. 제한 어댑터 구현 [실행]

- 입력 생성: `scripts/prepare_competition_inputs.py` → `work/inputs_comp/<scene>/frames_comp.csv` (115개 장면, 54,895 scan; `COMP_INPUT_MANIFEST.json`)
  - 열 = 허용 열(idx, stem, stamp_ns, npy, vel_x_mps, imu_yaw_rad, imu_roll_rad, imu_pitch_rad, imu_wz_rps) + 금지 열(ego_x/y/z, ego_speed_norm_mps, vel_y_mps, vel_z_mps, accel_x/y/z)
  - 금지 열은 격리 시험을 위해 일부러 함께 기록한다.
  - frames.csv와 gt.csv는 평가 전용 심볼릭 링크다.
- 하네스: `lidar_dataset_replay --input-mode competition` (commit 669859c1)
  - `ReadCompetitionFrames()`는 허용 열만 **이름으로** 찾는다. 다른 열은 읽지 않는다.
  - scan마다 AddImu(t_lidar, yaw, roll/pitch는 NaN이면 0)와 AddCan(t_lidar, vel_x)을 넣는다. **AddGps는 호출하지 않는다.**
  - jsonl에 `ego_in{vel_x, imu_yaw, rp_available, gps:0}`을 기록한다.
- 부호: vel_x 음수 샘플(정차 중 잡음 111개)은 제품 AddCan이 거부하므로 해당 scan은 not-ready가 된다. 이는 P0와 같은 동작이며, 실제 vel_x 사용 시 위험 요소로 보고한다.

## 6. 금지 필드 격리 시험

- 방법: scen58과 scen93에서 같은 바이너리와 같은 옵션으로 다음 변형을 실행한다(`scripts/make_isolation_inputs.py`, `work/isolation/`).
  - orig
  - forbid_perturb: 금지 열 9개를 큰 무작위 값으로 교체(pos ±500 m, vel_y·vel_z ±20 m/s, accel ±30 m/s²)
  - forbid_removed: 금지 열 삭제
  - allowed_perturb (양성 대조군): vel_x × 1.1
- 판정
  - forbid_perturb와 forbid_removed는 orig와 timing을 제외하고 동일해야 한다.
  - allowed_perturb는 달라야 한다(시험이 변화를 감지할 수 있음을 증명).
- 결과: `results/INPUT_ISOLATION_TEST.json` (실행 결과로 채움).

## 7. 운영 경로에 남은 위반 (이번 단계에서는 수정하지 않음)

- main.launch의 `/Competition_topic`→`/Ego_topic` remap과 Parser의 속도 노름은 운영 제품 코드의 계약 위반이다.
- 수정 방향
  - CAN 콜백을 velocity.x만 읽는 함수로 바꾼다.
  - remap을 제거한다.
  - `/Competition_topic` 디코드(105 B 형식)를 확인하고, 미수신 시 무효·성능 저하 처리를 한다.
- 이 수정은 브리지 패킷 형식 확인(현장 live capture)이 필요하다. 수정하지 않으면 속도 입력이 없어 모든 LiDAR scan이 skip될 위험이 있다(SensorPoseHistory.cpp:415-418). 이번 단계에서는 제품에 반영하지 않고 다음 단계 과제로 넘긴다.
