# 카메라·LiDAR 좌표 계약과 external 구현 대조

검토일: 2026-10-08. PR #71은 `534ddc5`로 main에 병합됐다. 이 검토는
PR #72의 opt-in 공통 좌표 경계를 대상으로 하며, 외부 모델을 설치하거나 운영
의존성으로 추가하지 않는다. external clone은 원본 상태를 유지한다.

## 판단

공개 local 객체를 후륜축 `base_link`로 통일하고 관측별 수평화를 되돌린 뒤
카메라/map으로 변환하는 설계는 적절하다. 현재 카메라 Python과 LiDAR C++의
장착·광학 축·투영 식도 일치한다. `base_link`는 차량 고정 좌표이며 수평화 센서
좌표가 아니다. 후륜축 원점은 우리 프로젝트의 선택이다. ROS 표준 자체가 후륜축을
강제하지는 않는다. [REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst),
[REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)

이 판단은 합성 좌표의 수학적 일치와 설정 출처 일치를 뜻한다. 실제 보정 정확도,
서로 다른 시각의 센서 정합, 지면 모델의 적용 범위와 모든 bag의 객체 보존까지
수락한 것은 아니다. roll·pitch 후보와 새 운영 경계의 기본값은 각각 OFF다.

## 고정 레포와 좌표 처리

| 참고 구현·고정 SHA | 실제 기준 좌표와 변환 | 우리 코드에서 취할 점 |
|---|---|---|
| [Frustum PointNets](https://github.com/charlesq34/frustum-pointnets/blob/2ffdd345e1fce4775ecb508d207e0ad465bcca80/kitti/kitti_util.py#L49) `2ffdd345` | raw Velodyne → camera reference → rectified camera → image. `P2 × R0_rect × Tr_velo_to_cam`; 센서 x전방/y좌측/z위, 광학 x우측/y아래/z전방 | 각 행렬의 출발·도착 좌표를 명시한다. KITTI rectification/stereo baseline을 MORAI 단안 설정으로 복사하지 않는다. |
| [PointPainting community](https://github.com/Song-Jingyu/PointPainting/blob/ec732f1ae103234d60040194df3bf4067bc37b92/painting/pcdet_utils/calibration_kitti.py#L22) `ec732f1a` | 같은 KITTI 변환으로 원본 LiDAR 점에 영상 의미 점수를 추가. 논문 저자의 원본 구현이 아닌 재구현 | 함수 이름보다 실제 행렬 방향을 확인한다. `painting.py`의 `cam_to_lidar`라는 이름은 내부 `Tr_velo_to_cam` 방향과 다르다. 카메라 FOV 밖 점을 삭제하는 정책은 전체 LiDAR 운영 정책으로 가져오지 않는다. |
| [CLOCs](https://github.com/pangsu0613/CLOCs/blob/d7d78f69fcd2b30147f0448f0606df3551975472/second/core/box_np_ops.py#L592) `d7d78f69` | LiDAR/camera 좌표의 후보 박스를 변환해 late fusion. 박스 차원 순서도 변환 | 이 revision의 `project_to_image`는 동차좌표에 0을 붙여 `P[:,3]`을 무시한다. 비영 stereo baseline에서는 그대로 재사용하면 안 된다. 다른 helper는 1을 사용한다. 아래 합성 시험은 해당 helper만 검사하며 전체 CLOCs 경로의 오작동을 주장하지 않는다. |
| [MIT BEVFusion](https://github.com/mit-han-lab/bevfusion/blob/326653dc06e0938edf1aae7d01efcd158ba83de5/tools/data_converter/nuscenes_converter.py#L324) `326653dc` | 기준 LiDAR 시각/원점에 정렬. sensor(tS) → ego(tS) → global → ego(tL) → LiDAR(tL). 각 sensor의 calibrated pose와 ego pose 사용 | ego라는 이름만 보고 후륜축이라고 간주하지 않는다. 서로 다른 시각 정렬은 고정 extrinsic과 별개다. 데이터셋 pose를 쓰는 구현은 GPS 없는 실시간 처리의 증거가 아니다. |
| [Lift-Splat-Shoot](https://github.com/nv-tlabs/lift-splat-shoot/blob/2903467c91ee9c12f0917a12c22ab1f04e607ae0/src/models.py#L166) `2903467c` | 영상 augmentation 역변환 → depth/K 역변환 → camera-to-ego → ego BEV | 영상 resize/crop/flip의 행렬도 추적한다. 이 camera 경로를 BEVFusion converter의 센서 시각 보상과 동일하다고 해석하지 않는다. |
| [NVIDIA CUDA-BEVFusion](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-BEVFusion/src/bevfusion/camera-geometry.cu#L123) `7c1623f2` | inverse image augmentation, inverse K, camera-to-LiDAR를 GPU에서 적용. depth 경로는 LiDAR-to-image와 image augmentation 적용 | 행렬을 합성해 중간 점군 복사를 줄일 수 있다. `base_link`라는 공개 기준을 둔다고 점군 전체를 두 번 복사할 필요가 없다. |

행벡터 코드의 `points @ R.T + t`와 열벡터 코드의 `R × point + t`는 같은
변환이다. 전치 여부만 비교하지 않고 방향·축·원점·동차좌표·시각을 함께 확인했다.

## 우리 변환과 보정 설정

표기 `T_A_from_B`는 B 좌표를 A 좌표로 바꾼다. 관측별 수평화 `L_scan`에 대해:

```text
p_leveled = L_scan × p_sensor
p_base = T_base_from_sensor × L_scan⁻¹ × p_leveled
q_image = K × O × T_camera_mount_from_base × p_base
O = [0 -1 0; 0 0 -1; 1 0 0]
```

raw 점에는 `L_scan⁻¹`을 적용하지 않는다. `L_scan`은 해당 scan에 실제 적용한
행렬이며 최신 IMU로 다시 계산하지 않는다. 중심·8코너·방향축에 동일한 정의를
사용한다. 공개 `base_link`는 차량의 roll·pitch를 가진 고정 body frame이고,
`lidar_leveled`는 scan별 전처리 frame이다. ground plane/BEV의 평면 가정과도 구분한다.

센서 원본은 `ASMC_sensor_set_v3.json`, SHA는
`4c2e93cc4d1139caf92a4e7ed4e20b9c7a705599e7e3faaf247b596e0fd1ae9f`다.

| 항목 | 현재 설정과 검토 |
|---|---|
| LiDAR 장착 | 후륜축에서 `[0.80, 0, 1.45]m`, RPY 0°. 비영 장착 회전도 별도 합성 시험에 포함 |
| 전방 카메라 | `[1.90, 0, 1.20]m`, pitch 2°, 1280×720/HFOV 90°. `fx=fy=640`, 중심 `(640,360)` |
| 좌/우 카메라 | `[1.15, ±0.65, 1.20]m`, pitch 10°/yaw ±70°, 640×480/HFOV 130°. 전방 파라미터를 복사하지 않음 |
| 광학 축 | body x전방/y좌측/z위 → optical x우측/y아래/z전방. 원본 영상 frame_id와 별도 3D optical frame을 혼동하지 않음 |
| GPS lever | `[0.35, 0, 1.25]m`. GPS 관측 당시 자세로 회전 후 한 번 제거 |
| map | UTM 52N에서 `[302595, 4124145, 0]` 원점을 뺀 기존 HD맵 좌표. 임의 WGS84 ENU 원점으로 바꾸지 않음 |
| 카메라 지면 | `base_link z=-0.35m`, 2026-09-11 지면 fit의 평면. 장착 높이 1.20/1.45m를 도로 위 높이로 다시 해석하지 않음 |

시작 시 JSON SHA·장착·영상 크기/FOV·map 원점을 검사한다. 선언과 설정의
일치는 드리프트 검사이며 실제 영상의 보정 오차를 자동 보증하지 않는다.
보정·HD맵·모델 pin은 유지했다.

## 수치 대조

[연구용 검사기](../../../tools/perception/audit_fusion_coordinates.py)는 실제
카메라 Python calibration과 실제 LiDAR C++ core의
[probe](../../../tools/perception/frame_projection_probe.cpp)를 실행한다. 외부
딥러닝 runtime을 설치하지 않고 고정 clone의 순수 calibration 함수만 사용한다.
3뷰 × 장착 회전 2종 × roll/pitch 3종 × 점 3개 = **54조건**이다.

- raw와 동일 scan의 수평화 점 투영은 카메라 기준과 최대 **2.28×10⁻¹³ pixel** 차이였다.
- Frustum/PointPainting 단안 투영과 CLOCs rigid roundtrip도 각각 오차 기준을 통과했다.
- 수평화 역변환 누락·중복·다른 관측 자세를 적용한 음성 대조는 최소
  **11.91 / 11.87 / 8.03 pixel** 오차가 나 정상 경로와 구분됐다.
- `P[:,3]`이 비영인 stereo 합성 시험에서 CLOCs 선택 helper는 **2.69–69.12 pixel**
  차이가 났다. 우리 MORAI 단안 `K` 경로에는 이 stereo translation이 없다.

고정 SHA·파일 SHA·설정·결과는
[수치 증거](../../../config/baselines/evidence/lidar_camera_frame_boundaries_v1/fusion-coordinate-audit.json)에
기록한다. 이는 54개의 독립 실환경 보정 GT 시험이나 모델 성능 측정이 아니다.

기록된 typed local/map 코너에서 실제 적용한 rigid pose도 복원해, 관측 이전 Ego
위치와 읽기 전용으로 비교했다. 주/음영의 첫 normal/pitch-only 실행에서 각각
637/205개를 비교했고 local→map rigid 잔차는 최대 `2.64e-13m`였다. Ego에 대한
XY 차이 p95는 각각 **0.170/0.257m**, 최대 **0.404/0.337m**였다. Ego 표본은
최대 28.7/33.4ms 이전 값이며, rear-axle map/ENU heading이라는 참조 가정과
시차가 포함된다. 이 값은 독립 객체 GT recall이나 pose 수락 임계값이 아니다.
검사기는 [pose 출력 감사](../../../tools/perception/audit_map_pose_outputs.py), 원본은
[주 bag](../../../config/baselines/evidence/lidar_camera_frame_boundaries_v1/frame-pose-audit-primary.json)과
[음영 bag](../../../config/baselines/evidence/lidar_camera_frame_boundaries_v1/frame-pose-audit-shadow.json)에 있다.

## 남은 한계와 후속 담당

1. **센서 시간:** bridge의 stamp는 수신 시각이며 카메라 노출/레이저 ray 시각이
   확인된 것이 아니다. scan 내부 deskew도 없다. ±50ms matcher는 가까운 시각을
   고를 뿐 서로 다른 관측 시각 사이의 차량 움직임을 보상하지 않는다. 별도
   IMU/CAN 상대 운동 보상 실험은 이후 기하 단계에서 검토하며 GPS를 local 필수로
   만들거나 음영 내부 전역 위치 추정을 추가하지 않는다.
   Main의 내부 `RAW_LIDAR_DATA.u64_Timestamp`는 header를 ms로 양자화한다.
   `PrepareScan`과 `SensorPoseHistory::MapAt`는 이 source-ms 값을
   `Observation.receipt_ns`에도 넣는다. 실제 Main callback 수신 시각을 기록한
   값이 아니다. 따라서 현재 GPS 도착 cutoff는 scan source-ms이며 더 보수적인
   기준이다. 원본 ns 보존·실제 callback receipt 캡처·clock 지연을 구분하기 전
   관측 시간 계약을 완료로 표시하지 않는다. 주 bag 649개 원본 LiDAR header 모두
   ms 미만 값이 있었고, replay observer의 ROS clock도 일부 header보다 늦게 갱신됐다.
   원본 header를 바꾸거나 receipt를 임의로 보정해서 숨기지 않는다. 이 수정은
   고정 baseline 이후 별도 묶음으로 시험한다.
2. **평면/높이:** 카메라 고정 지면은 검증한 음영 ODD 밖의 경사·roll 변화까지
   보장하지 않는다. 차선 피팅 소비는 음영에 한정한다. 주 bag 높이 감사의 최대
   0.047803m 차이도 다른 bag/장비의 altitude datum 보장은 아니다.
3. **pose 제공자:** HD맵 GPS 경로와 새 LiDAR 경계는 안테나 lever를 제거한다.
   기존 bridge의 선택적 `publish_map_tf`는 원시 안테나 XY와 z=0을 `base_link`로
   발행하며 기본 OFF다. 기존 GPS/IMU VehicleState estimator도 raw GPS XY를 쓴다.
   이것들을 새 후륜축/3D pose의 대체품으로 사용하지 않는다. 판제의 권위 pose
   제공자·원점·레버 정책을 공동 확인해야 한다. 이번 작업에서 다른 담당 운영
   pose 소스를 자동 교체하거나 중복 `map → base_link` TF를 만들지 않았다.
   또한 현재 CAN parser는 `norm(velocity)`로 방향 없는 속력을 만들고 bounded
   map 예측은 이를 body `+x` 속도로 쓴다. 후진/횡이동까지 정확하다고 볼 수 없다.
   부호·좌표가 검증된 body velocity 또는 명시적인 전진 ODD/gate를 확인해야 한다.
4. **수락:** state 54회와 구독 비용 24회 재생을 완료했다. local 공급 지속과
   map observer gate를 확인했지만, GPS 대비 24쌍 모두 strict payload가 달랐다.
   정상 반복도 달라 원인 분리에 결정론적 raw/IMU/CAN 재현이 필요하다.
   동시 구독 비용은 [운영 경계 결과](lidar-camera-frame-boundaries.md)에 기록했다.
   객체 보존·정상 map 추적 품질·실제 GPU fusion은 남아 있다.
   행렬 대조만으로 터널 필터·추적 품질을 수락하지 않는다.

이 한계·연결 지점은 [운영 경계 인계](lidar-camera-frame-boundaries.md)와
[재호 개선 과제](../../packages/asmc_lidar/b1a-improvement-tasks.md)에 연결한다.
센서 장착 위치에서 카메라로 직접 합성 투영하는 효율과 공개 `base_link` 계약은
함께 유지할 수 있다. map 출력은 유효 pose가 있을 때 같은 검출·추적의 결과를
변환하며, 별도 검출을 반복하지 않는다.
