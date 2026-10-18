# 정차 지면 보정·M1-rest·표시 주기 대조

## 현재 판단

Family는 **새 결과가 도착할 때 즉시 표시하는 방식**을 권고한다. 큰 RGB/semantic 합성은
최신 정확 대응 프레임을 최대10Hz로 표시한다. 판제 입력의15Hz 발행은 표시와 독립적이다.
이번 즉시 표시 구현은 별도 trial 토픽에서만 실행했다. 운영 family 화면은 개선한10Hz timer,
semantic 화면은 최신 프레임10Hz timer이며 즉시 표시를 기본으로 승격하지 않았다.
Large E8·추론 전후처리·피팅·지면0.35m·bridge header 정책은 바꾸지 않았다.

앞선 [M0/M1 분석](2026-10-18-lane-motion-review.md)의 M1-rest는 기하 민감도 후보다.
더 매끄러운 결과가 독립 GT의 정확도 향상을 뜻하지 않는다. 이번 결과에서도 보정 승격은 보류한다.

## 실행과 표시 대조

이미 실행 중인 `asmc-ros-noetic`의 모델 세 개를 그대로 사용했다. 추가 GPU 추론·차량 제어·
날씨 변경·객체 스폰은 하지 않았다. `view.launch`는 매번 한 세트만 실행했고 rqt는 시작하지 않았다.
아래는 루프백 ROS 구독자가 받은 JPEG의 새 source stamp 수다. 같은 stamp 반복은 제외한다.
시뮬레이터의 현재 정차 도로 위치이며 음영 ODD 전체 주행 수락은 아니다.

| 구성 | 시간 | Semantic 새 출력: 완전한5초 창 | Family 화면 새 출력 | Family 발행→화면 수신 p95 |
|---|---:|---:|---:|---:|
| 기존 callback 제한 | 35초 | 7.6–7.8Hz | 9.0–9.8Hz | 79.10ms |
| 최신 프레임 timer·중복 방지 | 60초 | 10.0Hz | 9.8–10.0Hz | 88.28ms |
| Family 도착 즉시 표시 trial | 60초 | 10.0Hz | 14.8–15.2Hz | 3.72ms |

시각화의 의미에 따라 주기를 분리해야 한다. Family 렌더링 자체는 가벼우므로10Hz timer의
다음 차례를 기다릴 이유가 약하다. 입력15Hz를 따라 표시하면 판제로 발행한 결과를 빨리 볼 수 있다.
큰 semantic 합성은 인코딩·화면 크기 때문에10Hz 상한을 유지하는 편이 합리적이다.
두 화면의 Hz가 달라도 입력 시각·좌표·generation이 맞으면 계약상 문제는 없다.

발행 이후3.72ms는 전체 인지 지연이 아니다. 즉시 표시 trial의 원본 header→JPEG 수신 p95는
149.78ms였고, timer는184.60ms였다. 모델·family 계산과 표시 대기를 포함한 값이다.
실제 rqt 페인팅·모니터 표시 시각은 측정하지 않았다. 두60초 구간은 순차 실행이며 같은 동적
프레임의 paired 품질 대조가 아니다. 전체 모델 지연이83ms 개선됐다고 해석하지 않는다.

Timer 대조에서 invalid family7건·readiness false3건이 있었다. Trial60초에서는 valid900개,
readiness298회 모두 true였으나 이를 모든 위치의 회귀 해결로 해석하지 않는다. 원본 기록을 보존한다.
YOLO 화면은 이번 변경 대상이 아니며 같은 구간9.2–10Hz 창이 있었다. 객체 mosaic의 header는
렌더링 시각이므로 semantic의 카메라 source age와 지연 순위를 비교하지 않는다.

10분 시험은 표시 주기 선택을 먼저 검토해 달라는 사용자 요청 시 중단했다. 부분 기록은 보존하고
완료·통과로 세지 않는다. 표시 경로 확정 후 최대10분으로 재검증할 항목이다.

## 적용한 시각화 변경

- 정확한 RGB/drivable/road-marking stamp 묶음을 뷰별 하나만 보관한다. 입력 callback은
  디코딩하지 않으며15Hz callback을100ms로 걸러7.5Hz가 되는 경로를 제거했다.
- 별도10Hz timer가 최신 묶음을 선택한다. 직전 source를 반복할 상황에는 최대40ms만 새 입력을
  기다린다. deadline은 원래 주기로 유지하며 오래된 결과를 새 stamp로 바꾸지 않는다.
- 혼합 타입의 exact synchronizer는 수요·세대 전환 시 큐를 비운다. 구독 당시 context를 보존해
  status 전환 사이에 도착한 과거 묶음을 새 generation으로 표시하지 않는다.
- Family의 격자를 재사용하고 렌더링·JPEG 인코딩을 수신/세대 잠금 밖에서 수행한다.
  발행 직전에 세대와350ms freshness를 다시 확인한다. LEFT/FORWARD/RIGHT 기준을 표시한다.
- mono8 확률을 미리 float로 복사하지 않고 기존 renderer에서 정규화한다. 기존 float 경로와
  인코딩 전 픽셀 일치를 시험했다. 추론 확률·클래스·원본 RGB는 변경하지 않는다.
- 구독이 없으면 입력·디코딩·렌더링을 중단한다. 중단 뷰는 WAITING/STALE로 지우며 다른 뷰를 막지 않는다.

도착 즉시 trial은 `probe_family_arrival_view.py`의 단일 최신 대기 worker를 사용했다.
원본 family를 그대로 그리며 별도 피팅·역행 stamp 생성·결과 재발행은 하지 않는다.

## M1-rest가 M0와 다른 선을 그리는 이유

두 후보는 같은 shared B-spline 코드와 인자를 사용한다. 바뀌는 것은 픽셀→지면 좌표다.
그 결과 지면 점의 위치·클러스터·연결·역할 선택·피팅 폭·support 범위가 달라질 수 있다.
보정 뒤 화면에 선이 덜 붙는 현상은 단순한 색/표시 문제가 아니라 실제 곡선 차이일 수 있다.
운영은 M0를 유지한다. 앞선 RGB 재투영도 각 후보의 해당 plane을 사용했으며 공통 plane으로
잘못 그린 오류는 확인되지 않았다.

기존 M1-rest는 per-run 저속 표본의 첫 자세를 Euler roll/pitch에서 빼는 민감도 실험이었다.
완전한 정차 평균·실측 도로 법선 보정이 아니다. 실제 정차 기울기까지 제거할 수 있다.
후속 정차 후보는 측정 법선과 quaternion을 함께 보존해 다음 관계로 비교한다.

```text
n_B(t) = R_WB(t)^T R_WB(rest) n_B(rest)
n_B(t) · p_B + h_rest = 0
```

카메라 고정 장착 회전을 다시 적용하지 않으며, 상수 h는 차체 heave가 없다는 가정이다.
IMU만으로 절대 높이를 추정했다고 설명하지 않는다. 현재 IMU 장착 회전0을 전제로 한다.

## 정차 실측과 채택 보류

12초 Capture 대신 수신 구독과 read-only gRPC로 LiDAR·IMU·RGB·model-grid를 보존했다.
VehicleSpec의 실제 반환 wheelRadius는0.36m다. 총 차체 높이와 rear-axle→지면 높이를 혼동하지 않는다.
라이브 TF의 LiDAR 장착은 base_link 기준(0.8,0,1.45)m·회전0이었다.

초기 대조는 자세·속도 변동으로 정차 gate를 실패했다. 세 번째 샘플은 이동1.27cm,
최대 속도0.026m/s, roll/pitch 표준편차0.00045°/0.00092°로 정차 조건을 통과했다.
그러나 넓은 LiDAR ROI는 연석·도로 바깥 지점을 섞어 추정 높이가 약0.36–0.48m로 바뀌었다.
인식된 도로 영역만 고르면0.3811m, inlier 약99.6%, 잔차 p95 약6.3mm였지만42개 cloud 중
영상 시각이100ms 이내인 유효 지면은13개이며 원거리 독립 ROI 지지도 부족했다.
모델의 drivable 출력을 도로 선정에 썼으므로 이것도 독립 GT가 아니다.

새16묶음의 유효 family는 M0·기존 M1-rest·측정 plane 후보 모두16/16이었다.
프레임별 semantic 재투영 p95의 중앙값은 M0약2.20px, 기존 M1-rest약2.21px,
측정 plane 후보약3.06px였다. 자체 semantic 일치가 좋아졌다는 근거도 없다.
**현재 위치의 제한된 지면 fit으로0.35m를0.381m로 교체하지 않는다.**
별도 노면 표본·보정 외부 정확도·진입/내부/출구 대조를 통과해야 적용할 수 있다.

## 원본 시각과 수신 시각

현재 bridge는 camera/IMU 패킷의 원본 sec/nsec를 읽지만 `timestamp_mode=receive`에서는
ROS header에 처리 시각을 넣는다. 두 시각을 각각 지속 보존하려면 bridge의 metadata 경로 보완이
필요하다. `header.stamp` 하나를 source로 바꾸는 것만으로 둘 다 보존되지는 않는다.
이번에는 bridge나 전역 timestamp_mode를 변경하지 않았다.

후속에는 원본 sec/nsec·유효 여부·선택한 ROS header stamp·첫/마지막 UDP 수신 시각·
monotonic 처리 시각·센서/frame 식별자·fallback 사유를 별도 기록으로 연결하는 편이 적절하다.
기존 소비자의 header 계약은 전환 검증 전 유지한다. 서로 다른 clock을 직접 빼지 않는다.
이전 receive-only bag에서 실제 원본 패킷 시각을 복구할 수는 없다.

앞선 passive UDP20초에서는 카메라 source↔수신의 중앙값이 약1.1ms였으나 WSL/Windows
clock offset과 전송/생성 지연이 합쳐진 값이다. IMU는840개 수신 중217개가 같은 원본 시각의
동일 quaternion/gyro였으며 고유 시각623개였다. 임의1ns를 더해 별도 자세로 세지 않는다.
카메라 시각을 둘러싼 IMU bracket p50약42ms·p95약46.2ms였으므로 각 영상의 시각에
SLERP하는 방법을 검토한다. 최신 결과 시점의 IMU를 그대로 붙이지 않는다.
정차만으로 동적 acquisition offset을 식별할 수 없으며 원본 camera stamp가 정확한 노출 시각인지도
별도 검증해야 한다.

## 산출물·검증

- [표시 주기 대조](http://127.0.0.1:8000/camera-seg-large-deploy-20261018-r1/display-review-r1/index.html)
- [정차 보정과 같은 RGB 재투영](http://127.0.0.1:8000/camera-seg-large-deploy-20261018-r1/stationary-review-r1/index.html)
- 원본: `data/runtime/family-stationary-20261018-r1/r2/r3`(각각 별도 디렉터리),
  `data/runtime/lane-display-*`, `artifacts/camera-seg-large-deploy-20261018-r1/time-alignment-review-r1/`.
- 기존0.35m 근거는 `artifacts/perception_eval/analysis/twinlite_v6_ground_plane_audit_20260911/metadata.json`
  (작업 checkout 안)이며 narrow 약0.3542m·shadow median 약0.3315m를 근거로 택한 근사다.
- 정차 수집: `tools/perception/calibrate_stationary_lane_plane.py`.
- 오프라인 대조: `python3 -m tools.perception.review_stationary_lane_plane --help`.
- 실제 표시 측정: `probe_lane_display.py`; 즉시 표시 후보: `probe_family_arrival_view.py`.

로컬 시험은 최신 입력/반복 stamp·잠금 밖 인코딩·세대 전환·구독 해제·중단 뷰·확률 픽셀 동등성,
혼합 타입 exact sync·정차 plane/법선/quaternion과 기존 family 회귀를 포함한다.
Hosted CI·10분 연속 수락·rqt 실제 표시 시간·운동 보정 운영 승격은 완료로 표시하지 않는다.

최종 로컬 회귀는 호스트43개와 Noetic 컨테이너44개로 총87개를 통과했다.
문서 이전·실행 명령 검사와 Python3.8 구문 검사도 통과했다.

최종 코드20초 smoke에서도 semantic/family는 완전한5초 창3개 모두10Hz였다.
이 짧은 확인을10분 연속 수락으로 대체하지 않는다.
