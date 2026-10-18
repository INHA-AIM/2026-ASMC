# 원본 시각 IMU 기반 차체 자세 보정

## 적용 경계

현재 구현은 TwinLite Large TensorRT 이후의 **model-grid→지면 근거→family** 경로에
roll/pitch 보정을 연결하는 M1 시험 경로다. 모델·정규화·해상도·mask·피팅 알고리즘·기존 header/MD5는 유지한다.
정본 `imu_motion.yaml`의 enabled 기본값은 false이며, 내부 대조 launch에서만 켤 수 있다.
일반 BEV/RGB 영상 warp와 차량 병진 이동·yaw 정렬·높이 추정·시간 누적은 이번 M1의 범위가 아니다.
따라서 고정 평면 RGB BEV 배경에 M1 선을 겹쳐 위치 정확도를 판단하지 않는다.

평지의 세계 법선 `[0,0,1]`을 영상 원본 시각의 IMU quaternion으로 차량 좌표에 변환한다.
`R_WB=R_WI·R_BIᵀ`, `n_B=R_WBᵀ·[0,0,1]`이며 카메라 장착 변환은 고정한다.
MORAI v3 IMU id7의 장착 회전은 `[0,0,0]`이다. 센서 축/장착이 달라지면 별도 검증한다.
평면의 법선 거리는 사용자 선택의 0.36m로 유지한다. IMU가 suspension/heave까지 측정한 값이 아니다.
Euler 각도 차감이나 임의의 첫 프레임을 평지 기준으로 만드는 M1-rest는 사용하지 않는다.

기존 선택 픽셀의 광선을 새 평면으로 옮기는 sparse homography를 사용한다.
원래 픽셀 gate·confidence·기하 가중치는 고정하고 카메라 뒤/범위 밖 근거를 제거한다.
Family의 x/y 피팅 후 z는 family header를 선택한 뷰의 보정 평면으로 계산한다.
이는 뷰별 이동·yaw 정렬이 아니며, 최대 35ms 시각 차이에 따른 병진 이동 오차는 남는다.
그리드 전체 lookup을 매 프레임 재생성하지 않으며, 기존 gate 밖에서 새로 보이는 픽셀을 복원하는 후보는 아니다.

## 정확한 시각·관측 연결

Bridge의 [SensorFrameTiming](bridge-sensor-timing.md)을 사용한다.
원본 JPEG는 topic·frame·stamp·실제 wire sequence로 metadata와 연결한다.
ROS는 파생 Image의 sequence를 재할당하므로 새
[`CameraModelGridTiming`](../../config/contracts/camera_model_grid_timing_v1.yaml)에
원 JPEG header와 실제 class/confidence wire header를 따로 보존한다.
기존 영상 header를 다시 쓰거나 원본을 receive 시각으로 추측하지 않는다.

M1 소비는 class/confidence/timing을 exact stamp로 모은 뒤 **header 전체·view·생산자 UUID·generation**도 검사한다.
IMU는 원 센서와 timing을 정확히 연결하고 동일 source 재전송을 독립 자세로 세지 않는다.
같은 clock domain·bridge session의 두 유효 quaternion 사이에서만 SLERP한다.
보간 간격은 최대 100ms, 수신 age는 최대 350ms, 뷰간 source skew는 최대 35ms다.
외삽·최신 자세 대입·다른 clock 직접 차감은 하지 않는다.

Source/자세/범위 근거가 부족하면 세 뷰 모두 고정 평면으로 처리한다.
일부 뷰만 보정하거나 과거 family를 새 결과처럼 발행하지 않는다.
파생 grid timing 자체가 없으면 M1 입력이 성립하지 않는다. 구형 result bag은 M0로 재생한다.
구형 raw bag을 새 inference로 재생하면 원본 timing 부재가 명시된 sidecar가 만들어져 M0로 fallback한다.
Bridge 재시작·source clock 역행·충돌·중복 wire key는 기록하고 잘못 연결하지 않는다.
모델 세대 전환에서는 기존 epoch gate·cutoff와 큐 초기화를 유지한다.

## 실행·진단

같은 master에서 `camera.launch`와 시험 launch를 동시에 켜지 않는다.
일반 실행을 종료한 뒤, 동일한 workspace를 source한 터미널에서 직접 대조한다.

```bash
# 같은 세 모델·설정으로 M0 대조
roslaunch asmc_camera _imu_compare.launch enabled:=false
# M1 시험. 앞 실행을 종료한 뒤 실행
roslaunch asmc_camera _imu_compare.launch enabled:=true
# 별도 터미널의 시각화는 그대로 사용
roslaunch asmc_camera view.launch
```

`/perception/lane/diagnostics`의 `imu_motion`에서 보정 계산 수·fallback 사유·raw 시각·bracket·
사용 quaternion/평면과 pending pose 수를 확인한다. 보정 활성 설정은 family config SHA에 포함한다.
기존 family 유효성과 IMU 보정 사용 여부는 별개다. M0 fallback을 M1 성공으로 세지 않는다.
Lifecycle trace를 켜면 프레임별 `imu_projection`의 원본 시각/자세/사용 여부를 기록한다.

소비 버퍼는 bounded이며 디코딩/NN 추론과 별도 mutex로 관리한다. IMU metadata를 기다리느라
family 계산을 멈추지 않고, 준비된 두 샘플이 없으면 명시적으로 고정 평면을 사용한다.
회귀와 실시간 대조는 [검증 기록](../records/camera/2026-10-18-source-time-imu-projection.md)에 고정한다.
기본값 승격은 이 구현 자체의 완료와 구분한다.
