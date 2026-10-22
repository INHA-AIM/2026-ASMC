# 상태·측정·좌표계 계약 (P0 = 916e4d9e + 39cffc57, 1단계 옵션 포함)

- 코드 위치: `repo/src/lidar+sensorfusion/src/Main/src/...`. 행 번호는 916e4d9e 기준이다. 1단계 commit 669859c1에서는 계측이 추가되어 행이 밀린다.
- 근거 수준
  - [코드]: 코드를 읽어 확인함
  - [시험]: 원인별 시험 또는 replay로 실행해 확인함
  - [추정]: 간접 근거만 있음
- 수식 유도와 코드 복제 시뮬레이션: `work/understand/trackmath/` (kf_frame_sim*.py, kf_reacquire_sim.py, kf_turn_bias_table.py)

## 1. 좌표계와 장착

| 이름 | 정의 | 사용처 |
|---|---|---|
| sensor (velodyne) | LiDAR 원점. x 전방, y 좌측, z 위 | Lidar_Data npy, PointCloud2 입력 |
| leveled sensor (tracking input) | 원점은 LiDAR. 축 = L·sensor, L = Mᵀ·Ry(pitch)·Rx(roll)·M (M = 장착 회전, 현재 I) | P0 기준. 공통 기하(main_integration)에서 KF 상태, 측정, OBB가 모두 이 프레임에 있다 |
| yaw frame | 원점 base_link. 수평으로 정렬하고 차량 heading을 x축으로 둔 프레임. Y = Rz(ψ)ᵀ·R_att·M·Lᵀ, o = Rz(ψ)ᵀ·R_att·t_mount | AlignTrackingFrame 재표현 (FrameBoundary.cpp:168-175) |
| base_link | 후륜축 중심. GT 프레임 기준으로 지면 위 약 0.35 m | V2 local 출력, GT, 평가 |
| map | UTM52N에서 HD map offset을 뺀 좌표 | V2 map 출력, /last_lidar, legacy 연관 |

- LiDAR 장착 (x, y, z) = (0.80, 0.00, 1.45) m, rpy = 0이다. GPS 안테나는 (0.35, 0, 1.25) m다 (setting.yaml:12-22, autonav_v3.yaml과 1e-6 이내로 일치 검사).
- 상태는 base_link가 아니라 **LiDAR 원점** 프레임에 있다. 장착 오프셋은 AlignTrackingFrame의 OriginInYaw 항에서만 추적 수식에 들어간다 [코드].
- 실행 경로
  - 운영 기본 main.launch는 legacy 경로다(enable_common_geometry=false, pitch만 보정).
  - P0 평가와 이번 단계는 main_integration.launch, 즉 공통 기하 + roll 경로다.
  - legacy 경로의 추가 결함(레버암 −ω×l 누락, Body2ENU pitch 이중 적용)은 코드로만 확인했고 이번 평가에는 포함되지 않았다(9절).

## 2. KF 상태와 측정

- 상태: x = [px, py, vx, vy, ax, ay]ᵀ (Eigen_KfState_CA, float). 단일 CA 모델이다. IMM 코드는 모두 주석 처리되어 있다 [코드].
- 위치 기준점: 클러스터 점의 **산술 평균** (Clustering.cpp:191-199 → Tracking.cpp:517-522, 병합 시 RecomputeGeometry). OBB 중심이 아니다.
- 측정
  - z = 현재 검출 점 평균 (CoreX, CoreY), H = [I₂ 0 0], R = 0.01·I₂ (σ = 0.1 m).
  - F: 표준 CA. Q = 0.003·I₆로 **dt와 무관한** scan당 상수다. P₀ = I₆.
  - 갱신: K = P⁻Hᵀ(HP⁻Hᵀ+R)⁻¹. Joseph form은 쓰지 않는다 (Kalman.cpp:141-206).
- dt = ts_CurrTime − ts_PrevTime (FrameFilter.cpp:82).
  - 공통 기하에서는 /velodyne_points header.stamp(브리지 수신 시각)다. Lidar_Data replay에서는 t_lidar다.
  - clamp나 가드가 없다. 첫 init의 dt = 0.1은 init 분기에서 쓰이지 않는다.
- 속도 상태의 의미
  - P0: **자차 대비 상대속도**를 현재 tracking 축으로 표현한 값이다. u = R_kᵀ(V_obj − V_base). 정지 물체면 u ≈ [−v, 0]이다.
  - 옵션 ego_translation_comp: **지면 절대속도**를 현재 tracking 축으로 표현한 값이다. 정지 물체면 u = 0이다.

## 3. 한 scan의 호출 순서

- main.cpp TemporalThread
  1. CopyData(최신 프레임 우선)
  2. DeadReckoning, Camera
  3. LookupLocalStateForLidarNs (IMU와 CAN 필수, age ≤ 0.12 s)
  4. MapAt (GPS age ≤ 0.30 s면 map 유효)
  5. PrepareScan
  6. LidarProcess
  7. FusionProcess
  8. GateInputs/Decide/PublishV2
  9. PlanLegacy → PlannerObjectRosPublisher → UDP
- 하네스는 3~8을 같은 함수로 호출한다. 센서 스레드, 최신 프레임 우선, 발행, UDP는 없다.
- LidarProcess (LidarProcess.cpp:153-349)
  1. ROI → Voxel 0.1 m → CorrectingPoints(leveling, StoreLeveling) → RANSAC 지면 → Clustering
  2. Objectinfo_for_km (검출마다 L-shape OBB, 점 평균)
  3. FrameFilter
     1. ConsolidateCurrentDetections (OBB 중심 거리 ≤ 0.6 m이고 AABB IoU ≥ 0.35면 병합)
     2. CompareCentroids
        1. 이전 트랙마다 predictKalmanPosition: 복사본에 AlignTrackingFrame을 적용한 뒤 CA 예측
        2. 비용 = d + (1 − ShapeAgreement) − 0.2·min(age/30, 1). gate 3.0
        3. Hungarian
        4. 매칭 쌍마다 stabilizeMatchedOBB(aligned 복사본) → kalman_filter_update(AlignTrackingFrame → 2번째 관측이면 속도 초기화 → CA 예측 → 갱신) → 절대속도
        5. 중복 retire
        6. 미매칭 검출은 즉시 새 ID + init
        7. 미매칭 트랙: miss < 25면 kalman_filter_predict, 25면 삭제
  4. MotionClassifier (map 무효면 InvalidateMapHistory)
  5. RecordTrackingFrame (관측 트랙)
- **회전 정렬 → 예측 → 매칭 → 갱신** 순서는 지켜진다. 연관용 예측과 갱신 시 예측은 같은 공식이다. 단, **OBB 안정화가 KF 갱신보다 먼저** 일어난다 [코드].

## 4. 자차 운동 처리 (이산 변환 유도)

기호: ψ_k = IMU yaw(rad, CCW+), Δψ = wrap(ψ_{k+1} − ψ_k), r = LiDAR 원점 기준 위치, l = (0.8, 0) 장착, b = base_link 위치, P = 물체 세계 위치.

정확한 재표현 (평지, 장착 회전 없음):

  r_{k+1} = R(−Δψ)·[ r_k + l + R_kᵀ(P_{k+1} − P_k) − R_kᵀ(b_{k+1} − b_k) ] − l

P0 AlignTrackingFrame (FrameBoundary.cpp:190-214) [코드]:

  p' = Y_{k+1}ᵀ·( Rz(ψ_k − ψ_{k+1})·(Y_k·p + o_k) − o_{k+1} ),  v' = R·v,  a' = R·a

이미 처리된 항
- (a) 위치의 유한각 재표현 R(−Δψ)·r. 연속시간 ṙ = −ω×r + … 의 −ω×r 항을 정확히 포함한다. 따라서 "명시적 ω×r 항이 없다"는 사실 자체는 누락이 아니다.
- (b) 속도·가속도 벡터의 재표현(전달항 −ω×u, −ω×a).
- (c) 레버암 (R(−Δψ) − I)·l. o_k, o_{k+1} 항으로 처리한다(공통 기하만 해당).
- (d) scan 사이 roll/pitch 변화. Y_k ≠ Y_{k+1}로 처리한다.

P0가 상태에서 빼지 않는 항
- 자차 병진 R_kᵀ·Δb. 주석 "Do not subtract CAN displacement: relative CA already models ego motion"이 이를 밝힌다. 그래서 u는 상대속도가 된다.

절대속도 (Tracking.cpp:786-796) [코드]
- 매칭된 트랙에서 KF 갱신 직후 AbsV = u + [v_CAN, 0]이다. v_CAN = f32_CurrentVel_KPH/3.6 (m/s)이며 tracking 축 +x에만 더한다.
- v_CAN은 scan stamp 이전의 최신 CAN 샘플(age ≤ 0.12 s, 보간 없음)이다.
- 운영 Parser는 v_CAN = ‖velocity‖ (부호 없음, vel_y·vel_z 포함, Parser.cpp:532)를 쓴다. 이것은 입력 계약 위반이다(INPUT_CONTRACT.md).

P0 모델 오차 (원인 시험으로 확인함) [시험]
- 상대속도 u의 자차 성분 [−v, 0]과 상대가속도의 자차 성분 −[v̇, ωv]는 **차체에 고정된** 값이다. 그런데 코드는 이를 매 scan R(−Δψ)로 세계 고정 벡터처럼 회전시킨다. CA 모델은 이 변화를 a 상태로 학습해야 하므로 선회와 가감속 중에 편향과 지연이 생긴다.
- 원인 시험(`test_tracking_frame_options`, 이상적 측정, 운영 FrameFilter 체인)에서 옵션 OFF(P0 경로)의 결과는 다음과 같다.
  - 선회 v = 8 m/s, ω = 0.3 rad/s 중 주차 차량: 가짜 속도 **0.65 m/s**
  - 자차 가속 2 m/s² 중 주차 차량: 가짜 속도 **0.46 m/s**
  - 선회 중 이동 차량: 속도 오차 **0.65 m/s**
- 등속 직진과 제자리 회전(레버암)은 P0도 0.05 m/s 미만이다. 즉 공통 기하의 레버암 처리는 정확하다.

**옵션 ego_translation_comp** (FrameBoundary.cpp EgoShiftSinceRecord)
- yaw frame에서 base_link의 원호 변위 Δb를 뺀다.
  - Δb = [ρ·sin Δψ, ρ·(1 − cos Δψ), 0], ρ = v̄·Δt/Δψ
  - v̄ = (v_CAN,k + v_CAN,k+1)/2
  - Δt = 현재 scan stamp − 기록 stamp
  - Δψ = IMU yaw 변화
- translation' = Y_{k+1}ᵀ·(Rz(−Δψ)·(o_k − Δb) − o_{k+1})
- 결과
  - 상태 속도는 지면 절대속도가 되고 AbsV = u가 된다(CAN을 더하지 않음).
  - 새 트랙의 사전값 u = 0은 "세계에서 정지"를 뜻한다(P0에서는 "자차와 같이 이동"을 뜻했다).
- 사용하는 입력은 vel_x와 IMU yaw 두 가지이며 모두 허용 입력이다.
- 원인 시험에서 위 세 경우 모두 0.05 m/s 미만이다. 자차 12 m/s에서 2 scan을 놓친 주차 차량도 ID를 유지한다(P0는 100→102로 ID가 바뀐다) [시험].

## 5. 두 번째 관측 속도 초기화와 miss 후 재획득

- P0 (Kalman.cpp:166-171)
  - v₀ = (z − pred)/dt, x = [pred, v₀, 0]이다. 바로 이어지는 CA 예측이 z에 도달하므로 혁신은 0이다.
  - P는 v₀ 오차를 반영하지 않는다.
- 결함: 새 트랙(InitCount = 1)이 k scan 동안 miss되면 InitCount가 유지된 채 재획득된다. 이때 z − pred는 (k+1) scan 동안의 변위인데 1 scan dt로 나눈다. 그래서 속도가 **(k+1)배**가 된다 [코드, 시험].
  - 원인 시험: 자차 5 m/s, 2 scan miss, 주차 차량에서 **|AbsV| 오차 10 m/s**.
  - 실데이터 P0: 직전 scan에 miss가 있던 정지 객체 TP의 속도 p95가 13.5 m/s다(miss가 없으면 2.1 m/s) [실행: `work/analysis/p0_stationary_breakdown.json`].
- 옵션 reacquire_elapsed_init: miss가 있었던 경우에만 v₀ = (z − pred)/(누적 miss dt + dt), x = [z − v₀·dt, v₀, 0]으로 둔다(CA 예측 후 정확히 z). miss가 없으면 P0 식을 그대로 쓴다(bit 동일). 원인 시험에서 오차는 0.05 m/s 미만이다.

## 6. OBB

- 검출 L-shape (Tracking.cpp:142-201)
  - 0~89°를 1° 간격으로 탐색하고 closeness 점수를 쓴다(최소 거리 0.05).
  - 점이 10개 미만이면 yaw = 0인 AABB를 쓴다.
  - 검출 단계에서는 w ≥ h를 보장한다.
  - yaw는 [−π/2, π/2)의 π 주기 **축 방향**이다. heading이 아니다.
- 중심 = 외접 사각형 중심. 점 평균은 가시면 쪽으로 치우친다. 둘 사이에 보정 관계식은 없다.
- 매칭 트랙: stabilizeMatchedOBB (Tracking.cpp:227-261)
  - 이전 yaw를 −Δψ만큼 재표현한다. 종횡비 ≥ 1.2면 raw 쪽으로 α = 0.25로 EMA하고, 1.2 미만이면 예측 yaw만 쓴다.
  - 그 yaw로 **현재 점**을 다시 감싼다. 길이 ≥ 폭 재정규화가 없어 length < width가 생길 수 있다.
- miss 후 재획득 첫 프레임은 raw L-shape를 쓴다.
- miss 트랙: CoreX/Y만 KF 예측으로 바뀐다. OBB, AABB, AbsV는 마지막 관측 값으로 정체되며 출력에서는 제외된다.
- OBB yaw 이중 회전: Yangtest에서 발견된 결함은 916e4d9e에 **없다**. 5a32c9ab에서 rotate_obb_yaw 인자로 수정되었다. update는 false, miss predict는 true다 [코드].

## 7. 출력 경로와 소비자

| 출력 | 내용 | 실제 소비자 |
|---|---|---|
| /lidar/local_object (base_link), /lidar/map_object (map) — LidarObjectArrayV2 | center/corners = 현재 scan의 OBB(KF 위치가 아님), length = Width, width = Height. **속도 필드는 0이고 relative/absolute_velocity_valid = false**. motion_state만 유효 | repo 안에 런타임 구독자 없음 |
| /last_lidar (map) + /lidar/motion_debug | MakePlannerObjects → 4개 필터 → clip. velocity = MotionClassifier planner 속도(LowThreat = 0, Pending은 blend)를 map으로 회전. raw_velocity = AbsV | **planner가 실제로 읽는 유일한 경로** (tracked_wire_bridge → planning_objects_adapter → asmc_planner, 0.30 s freshness) |
| UDP 127.0.0.1:7510 | MakePlannerObjects(필터 전, 최대 20개) | repo 안에 수신자 없음 |

- main_integration에서 /last_lidar는 map scan이고 모든 객체가 motion_valid일 때만 나간다(warm-up 객체가 하나라도 있으면 프레임 전체가 보류된다). main.launch(legacy)에서는 처리된 scan마다 나간다.
- valid 처리
  - V2 속도는 항상 valid = false다(소비자 없음).
  - tracked_wire_bridge는 ID 집합 불일치나 무효 motion code가 있으면 프레임 전체를 버린다.
  - converter와 planner는 무효 기하가 하나라도 있으면 프레임 전체를 거부한다.
  - planner는 vx/vy를 그대로 쓴다(별도 valid 플래그 없음).
- Competition 입력(이 데이터에는 GPS가 없음)에서는 map pose가 무효다. 따라서 V2 local만 나가고 motion_state·/last_lidar·UDP는 나가지 않는다. 이번 단계의 속도 평가는 KF 절대속도(하네스 `vb` = base_link로 회전한 AbsV)를 대상으로 한다. planner 경로 값은 trace의 `publish.planner`로 기록하지만, map pose가 있는 historical 모드에서만 채워진다.

## 8. 시간·단위·부호

- yaw
  - IMU quaternion에서 rpy를 계산한다(rad, ENU, CCW+).
  - Kalman 함수는 deg를 받아 ×π/180으로 바꾼다. AlignTrackingFrame은 rad를 받는다.
  - makeYawTransform(−dpsi) = R(−Δψ)이다. 좌회전(Δψ > 0)이면 전방 정지물체가 −y 쪽으로 이동하므로 부호가 맞다 [코드].
- 시각
  - 원천 stamp(ROS clock, 브리지 수신 시각)
  - 하네스: ros::Time::setNow(t_lidar) sim time과 logical steady
  - 처리 시간: steady_clock(CLOCK_MONOTONIC)
- 서로 다른 clock의 값은 빼지 않는다(REALTIME_REPORT.json 정의 참고).
- trace 레코드
  - t_ns: LiDAR source stamp
  - mono_ns: 노드의 CLOCK_MONOTONIC
  - scan: 하네스에서는 frame idx, 런타임에서는 callback generation

## 9. 코드로만 확인한 결함 (현 평가 경로 밖이거나 아직 재현하지 않음)

- legacy 경로의 레버암 누락: 제자리 회전 ω = 0.5에서 정지물체 횡속도 −0.47 m/s (코드 복제 시뮬레이션) [추정].
- legacy Body2ENU pitch 이중 적용 [코드].
- dt 가드 부재와 Q의 dt 비스케일 [코드]. 역행 stamp는 공통 기하에서 거부된다.
- CAN 부호 없음: 후진 시 AbsVx ≈ +2|v| [코드]. Competition vel_x로 바꾸면 AddCan이 음수 속도를 거부해 scan이 skip된다(P0 데이터에서는 121 scan이 not-ready) [실행].
- MotionClassifier가 서로 다른 scan의 차체축 속도로 중앙값을 낸다(선회 중 프레임이 섞임) [추정].
- LiDAR deskew 없음, IMU yaw 비보간(최신 샘플) [추정].
