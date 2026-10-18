# 원본 시각 IMU 기반 차체 자세 보정

## 적용 경계

현재 구현은 TwinLite Large TensorRT 이후의 **model-grid→지면 근거→family** 경로에
roll/pitch 보정과 관측 폭·공통 B-spline 공동 계산을 사용하는 기본 경로다.
정본 `imu_motion.yaml`은 enabled=true다. 모델·정규화·해상도·mask·family 표현·기존 header/MD5는 유지한다.
[동일 입력·실시간 수락](../records/camera/2026-10-18-joint-family-stabilization.md) 후 기본 전환했다.
Comparison RGB/semantic BEV는 family가 선택한 같은 프레임별 지면을 재사용한다.
차량 병진 이동·yaw 정렬·높이 추정·시간 누적은 이번 M1의 범위가 아니다.
구형 고정 평면 BEV 데이터 토픽은 유지하므로 그 배경으로 M1의 절대 정확도를 판단하지 않는다.

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
# 기본 M1+공동 폭 계산. 앞 실행을 종료한 뒤 실행
roslaunch asmc_camera camera.launch
# IMU만 대조하는 내부 설정. 앞 실행을 종료한 뒤 실행
roslaunch asmc_camera _imu_compare.launch enabled:=true corrected_width_prior:=0.35 joint_width_fit:=false
# 별도 터미널의 시각화는 그대로 사용
roslaunch asmc_camera view.launch
```

`/perception/lane/diagnostics`의 `imu_motion`에서 보정 계산 수·fallback 사유·raw 시각·bracket·
사용 quaternion/평면과 pending pose 수를 확인한다. 보정 활성 설정은 family config SHA에 포함한다.
기존 family 유효성과 IMU 보정 사용 여부는 별개다. M0 fallback을 M1 성공으로 세지 않는다.
Lifecycle trace를 켜면 프레임별 `imu_projection`의 원본 시각/자세/사용 여부를 기록한다.

소비 버퍼는 bounded이며 디코딩/NN 추론과 별도 mutex로 관리한다. IMU metadata를 기다리느라
세 뷰 전체에 최대10ms의 조건부 대기 예산만 사용하며, 유효한 두 샘플을 얻지 못하면
명시적으로 고정 평면을 사용한다. 대기 중 timeline lock을 해제한다.
회귀와 실시간 대조는 [검증 기록](../records/camera/2026-10-18-source-time-imu-projection.md)에 고정한다.
구현·이전 실패·최종 수락은 각 실행 기록으로 구분한다.

## 터널 운동 후속 판단

[기존 bag·새 원본 시각 주행 대조](../records/camera/2026-10-18-tunnel-imu-motion-and-fit.md)에서
IMU가 큰 가감속 기하 변화를 줄였고, nominal 폭 prior가 영상 차선 밀착도를 떨어뜨리는 기여를 확인했다.
이전 [폭 완화 후보](../records/camera/2026-10-18-imu-width-policy.md)는 오른쪽 흰선 잔차와
약3.1cm 지원 손실로 미통과했다. [공동 계산 후속](../records/camera/2026-10-18-joint-family-stabilization.md)은
같은 지원 기준에서 이 회귀를 없애고 공급 최적화·10분 수락을 마쳤다.
정속/미세 조향의 노이즈와 절대 지면 가정은 내부 일관성·독립 GT 정확도를 구분해 해석한다.

## 프레임별 폭 사전값 정책

`imu_motion.corrected_width_prior_strength`의 기본값은0이다. 세 뷰 모두 원본 시각의 IMU로
보정된 프레임에만 적용한다. 하나라도 시각/자세 근거가 부족하면 전 뷰가 고정 평면으로 돌아가며,
`lane_fitting.yaml`의 원래 prior0.35를 사용한다. 폭 범위2.4–4.6m와 shared B-spline·support 검사는 유지한다.

프레임별 설정은 읽기 전용이며 계산 중 공유 설정을 수정하지 않는다. 보정 시/복귀 시의 두 값을
family config SHA에 포함한다. 기존 diagnostics의 `imu_motion.last`와 shared family audit에
`fit_policy`의 mode/reason/width_prior_strength와 `fitted_widths_m`을 남긴다. Lifecycle trace는
`family_fit_policy`로 source stamp와 선택한 정책을 기록한다. 공개 메시지·서비스·MD5는 바꾸지 않는다.

## 공동 폭 계산과 IMU 도착 순서

[후속 잔차·공급 검증](../records/camera/2026-10-18-joint-family-stabilization.md)은 황색 기준 역할이
관측된 프레임에서 공통 B-spline 제어점과 관측한 폭을 함께 계산한다. 폭 경계·support·robust 반복은
유지하며, 황색 역할이 없으면 기존 계산을 사용한다. `joint_width_fit` 진단은 이 정책의 선택을 뜻하며
모든 프레임에 황색 역할이 있었다는 뜻은 아니다.

Camera source가 마지막 유효 IMU source보다 조금 앞서 도착하면, 세 뷰 합계 최대10ms 예산으로
다음 실제 IMU 샘플을 기다린다. Condition 대기 중 timeline lock을 풀고, 도착 후 같은 source/session·
100ms bracket·350ms freshness를 다시 검사한다. 오래된 source나 잘못된 세션에는 기다리지 않는다.
외삽·최신 quaternion 대입·시간 상한 연장은 하지 않는다. `pose_timeline.counters`의
`wait_requested`, `wait_recovered`, `wait_unresolved`와 `last_bracket_failure`를 확인한다.
기다려도 근거가 없거나3뷰 source skew가35ms를 넘으면 기존 M0 복귀가 유지된다.

일반 `camera.launch`가 M1·공동 폭 계산을 사용한다. M0 복구는 기존 실행을 종료한 뒤
`_imu_compare.launch enabled:=false`로 명시한다. 두 launch를 동시에 켜지 않는다.
원본 timing이 없는 구형 bag과 IMU 오류는 기본 전환 후에도 M0로 처리한다.

M0 복귀는 드물지만 사라지지 않았다. 최종10분의27/8,983프레임과 전환 때15m 지점의
최대약0.318m 인접 곡선 차이를 기록했다. 이는 독립 GT 오차가 아니며 평활화로 감추지 않는다.
다음 우선 과제는 source 시각에 맞춘3뷰 묶음 선택·중첩 근거 정합이다.
병진/yaw 정렬과 과거 근거 누적은 유효한 터널 이동 입력과 별도 검증이 필요하다.
판제의 구 SHA guard와 고정 평면 bbox 재투영은 [인수 규칙](../handoffs/camera-to-planning.md)을 따른다.

## 두 시각화의 보정 재사용

Family worker가 원본 시각 IMU를 조회하고 M1/M0를 선택하는 계산은 기존대로 한 번이다.
`/perception/lane/ground_projection`의 작은 비래치 metadata에 뷰별 실제 지면과 원 JPEG header,
추론 session/generation·family 생산 세션·설정·보정 ID를 담는다.
[내부 계약](../../config/contracts/camera_ground_projection_v1.yaml)은 기존 메시지 MD5를 바꾸지 않는다.
메시지는 구독자가 있을 때만 조립·발행하며 family 선 시각화는 실제 family 결과를 그대로 그린다.

Comparison은 원 JPEG를 frame/stamp/wire sequence로, 원본 크기 semantic 3개 head를
같은 추론 epoch 안의 유일한 frame/stamp로 대응한다. Head별 ROS sequence가 JPEG와 같다고
가정하지 않는다. 중복 stamp가 모호하거나 일부 입력이 없으면 다른 시각의 영상으로 대체하지 않는다.
세 뷰가 모이면 뷰별 raster 투영/coverage/quality를 한 번 만들고 RGB와 semantic에 함께 사용한다.
이미 보정된 raster를 다시 보정하지 않으며 모델 추론·IMU 조회·family 피팅을 추가 실행하지 않는다.

기본 comparison만 구독하면 기존 고정 BEV 생성·합성 체인은 구독하지 않는다. 별도 recorder나
legacy 소비자는 종전 데이터 토픽을 계속 사용할 수 있다. Comparison은 최신 완성 묶음만 최대10Hz로
처리하고 JPEG를 마지막에 한 번 인코딩한다. 캐시는 뷰별8개/500ms, 렌더 대기는 한 개다.
세대/생산자 변경 시 캐시를 비우며 렌더 도중 변경된 결과도 발행하지 않는다.
화면의 `IMU shared` 또는 `M0: 사유`가 실제 family의 선택을 표시한다.
수신/발행 source stamp는 보존한다. 차량 이동·yaw에 따라 Ego 기준 도로가 움직이는 것은 정상이다.
