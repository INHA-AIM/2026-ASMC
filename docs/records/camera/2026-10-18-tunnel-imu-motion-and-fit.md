# 터널 주행 IMU 효과와 segmentation–family 밀착도

## 결정

**다음 실시간 후보는 원본 시각 IMU M1 + nominal 폭 사전값 완화다. 운영 기본값은 이번에 바꾸지 않았다.**
정차 한 위치의 결과만으로 IMU 효과를 판단하지 않고 기존 터널 주행 bag 6개와 새 원본 시각 가감속·조향을 대조했다.
모델·보정 파일·family 알고리즘·공개 계약을 변경하지 않았다. 주행 제어는 진단용 gRPC 한 생산자만 사용했다.
Push·PR·GitHub Actions 실행은 하지 않았다.

작업은 [동시 작업 경계](../repository/2026-10-18-parallel-task-worktrees.md)에 따라
`2026-ASMC-camera-motion` / `experiment/camera-imu-motion`에서 진행했다.
`59b43a58`에 분리 기준을, `8e1e731f`에 기록·분석 도구와 검사를 저장했다.
기존 worktree의 데이터 이관 스크립트 미커밋 변경과 별도 LiDAR 연구 파일은 가져오지 않았다.

## 선명한 segmentation에 family가 밀착하지 않는 이유

현재 경로는 raster BEV에서 선을 그대로 복사하는 구조가 아니다.
Model-grid의 차선 픽셀을 연속 지면 좌표로 투영하고 0.25m 전방 구간에서 굵기·중복 근거를 묶은 뒤
세 역할을 공통 B-spline과 법선 방향 폭으로 맞춘다.

- `width_prior_strength=0.35`는 측정 폭을 nominal 3.48/3.52m 쪽으로 당긴다.
  프레임 내 폭은 역할별 상수여서 세 선의 픽셀을 각자 독립적으로 따라갈 수 없다.
- `spline_smoothing=0.08`, control spacing 2.5m와 robust 반복은 매끈한 공통 형상을 우선한다.
- `OBSERVED`는 근거와 허용 거리 안에 있다는 뜻이다. 모든 곡선 점이 segmentation 양성 픽셀 위라는 뜻이 아니다.
  `STRUCTURE_INFERRED`는 실제 검출과 구분한 구조 추정이므로 paint에 붙는 정도로만 평가하지 않는다.
- 이중 황색선은 같은 클래스의 근거가 묶이면 두 줄 사이를 대표할 수 있다. 최종 역할과 개별 paint 경계는 다르다.
- 서로 다른 시각 또는 고정 평면 RGB BEV 배경과 IMU 곡선을 겹치면 별도의 표시 불일치도 생긴다.
  이번 갤러리는 정확히 같은 JPEG와 각 후보의 투영으로 원본 RGB에 재투영했다.

기존 터널 216묶음에서 selected raw pixel과 병합 node의 전방 재투영 거리 중앙값은 0px였다.
최종 family만 바꾸는 단일 대조에서 폭 prior를 0으로 했을 때 전방 흰선/황색선 거리 중앙값이
1.0/1.4px에서 0/0px로 줄었다. Smoothing만 0으로 한 효과는 작았다.
따라서 이 자료에서 폭 prior의 기여가 확인됐다. 단, 같은 카메라의 round-trip 정합성이
실제 장착 보정·절대 위치 정확도까지 증명하지는 않는다.

폭 prior를 0으로 해도 폭 범위 2.4–4.6m, 공통 곡선·평행 형상, 다중 역할·confidence·support 유효성은 유지했다.
폭 제약 전체나 피팅 유효성 검사를 없앤 후보가 아니다. 운영 YAML은 수정하지 않았다.

## 기존 터널 주행 bag 6개

SUNNY/FOGGY × 11/13/15시의 진입·내부·출구 연속 구간을 사용했다.
고정 Large 그리드 216묶음과 RGB SHA를 재사용해 모델 재실행에 따른 차이를 피했다.
기존 bridge diagnostics는 모두 `timestamp_mode=receive`였다. 원본 timing sidecar가 없어
해당 bag의 header를 촬영 source time으로 부르지 않는다. IMU ±40ms 대조는 민감도 시험이다.

프레임별 곡선의 관측/추정 support를 존중하고, 같은 x/role이 두 프레임과 모든 대조 후보에서 유효한 지점만 사용했다.
개발용 Ego pose의 SE(2)로 이전 곡선을 정렬한 뒤 쌍별 횡방향 변화 p95를 계산하고 그 분포를 집계했다.
독립 차선 GT 위치 오차가 아니며 새로운 미사용 평가본으로 설명하지 않는다.

| 기존 bag 구간 | M0 변화량 p95 | 수신 시각 M1 변화량 p95 |
|---|---:|---:|
| 진입 | 2.23cm | 10.70cm |
| 내부 | 21.74cm | 7.07cm |
| 출구 | 20.84cm | 7.73cm |

각 구간 66쌍, 전체 198쌍이다. M1이 내부·출구의 큰 흔들림을 줄였지만 진입에서는 악화됐다.
Roll은 선택 프레임에서 약 -0.92~2.38도, pitch는 -0.49~0.79도였다.
수신 시각 오프셋·폭 prior·지면 가정이 함께 남으므로 이 결과로 기본값을 바꾸지 않았다.

## 원본 시각을 보존한 새 터널 시험

사용자가 허용한 MORAI 이동·가감속·조향만 수행했다. 다른 actor가 없는 것을 확인하고 A126 내부에서
25초 명령 구간을 포함해 35초 RGB/grid/IMU를 기록했다. 날씨·센서·모델·기존 actor를 변경하거나 새 actor를 만들지 않았다.
최고 속도 5.54m/s(약19.9km/h), 최대 경로 중심 오차 0.131m였다.
가속→정속→제동→정차→재가속→좌우 조향→제동을 사용했다. 제어는 perception 결과를 소비하지 않았다.

- 원본 RGB/grid 226묶음, IMU와 timing 각각 1,434개, journal 탈락 0개.
- 원본 중복을 제외한 IMU 1,035자세. 정확한 wire key로 연결하고 같은 source clock에서만 보간했다.
  새 분석기는 source 충돌·역행이 있으면 구간을 분리하도록 실패시킨다.
- Capture 요청은 사용하지 않았다. 동적 자세 비교용 ROS 원본 시각 기록이며 학습 자료 채택이 아니다.
- RPC Ego 상태 499개와 원 API timestamp를 보존했다. 정수의 Unix microsecond 해석은 실제 clock과 대조한
  개발용 기준이다. 조회 왕복의 중간 시각보다 원 timestamp가 중앙값 17ms, 최대 약84ms 이전이었다.
  조회 수신 시각을 pose의 측정 시각으로 대체하지 않았다.
- 원 RPC timestamp에 맞춘 IMU의 자세 차이 p95는 roll 0.0015도·pitch 0.0111도·yaw 0.0089도였다.
  이는 정합 근거이며 RPC를 대회 운영 pose 입력이나 독립 위치 GT로 승격한 것이 아니다.

같은 RGB와 semantic으로 226/226묶음이 모든 후보에서 유효했다. 공통 비교가 성립한 인접 쌍은 128개다.

| 새 기록 조건 | M0 변화량 p95 | source M1 변화량 p95 |
|---|---:|---:|
| 전체 | 24.58cm | 3.96cm |
| 첫 가속 | 28.37cm | 5.82cm |
| 첫 제동 | 54.49cm | 9.66cm |
| 재가속 | 25.15cm | 3.72cm |
| 두 번째 제동 | 22.23cm | 2.41cm |
| 정속 | 1.93cm | 2.78cm |
| 좌우 조향 | 1.34cm | 1.69cm |

큰 가감속에서 이득이 있었고 작은 변화 구간에서는 M1이 조금 더 흔들렸다. 전체 중앙값도 M0 0.72cm보다 M1 1.18cm가 크다.
평균적으로 항상 더 매끈하다고 말하지 않는다. 큰 순간 변화를 줄이는 효과와 미세한 자세 노이즈를 구분한다.

동일 자료에서 추가한 후보 하나인 **M1+폭 prior 0**은 변화량 p95 4.02cm로 운동 안정화 이득을 유지했다.
M1의 전방 관측 흰선/황색선 픽셀 거리에서 프레임별 중앙값의 p95는 2/3px였고, 조합 후보는 0/0px였다.
모든 점의 오차가 0이거나 절대 위치가 정확하다는 뜻이 아니다. 자기 segmentation과의 정합성을 평가한 값이다.

0.36m 법선 거리와 평면 세계 가정은 유지했다. Heave·경사/횡단 경사·카메라 절대 보정 오차는 독립적으로 검증되지 않았다.
과거 근거 누적·yaw/병진 정렬은 적용하지 않았다. 구간별 동작 차이를 숨기기 위해 readiness나 freshness를 완화하지 않았다.

## 시험 종료·복원과 검증

첫 실행은 주행·기록을 완료했지만, 위치/제어 모드 설정 후 기어가 D로 남아 복원 검사가 실패했다.
실패 report와 원본을 보존했다. 브레이크 heartbeat를 유지하고 **원위치→원 제어 모드→원 기어** 순서로 보완했다.
별도 복원 검사에서 원래 A748 위치, P단·키보드 모드, 속력 0.000054m/s를 확인했다.
위치 오차는 2.46mm, 방향 차이는 1.34도였다. 방향까지 정확히 복원했다고 표시하지 않는다.
보완된 runner의 전체 주행을 다시 반복한 것은 아니며 다음 사용에도 실제 상태 검증이 필요하다.

원본은 `data/runtime/camera-imu-motion-20261018-r1/`, 갤러리는
`artifacts/camera-tunnel-imu-motion-20261018-r1/`에 보존한다.
[E392 비교 목차](http://127.0.0.1:8000/camera-tunnel-imu-motion-20261018-r1/index.html),
[E390 기존 bag](http://127.0.0.1:8000/camera-tunnel-imu-motion-20261018-r1/legacy/index.html),
[E391 새 source 기록](http://127.0.0.1:8000/camera-tunnel-imu-motion-20261018-r1/source/index.html)을 실험 목록에 등록했다.
새 페이지의 끊어진 정적 링크는 0개다. 전체 갤러리 검사의 다른 페이지 링크 25개 누락은 별도 문제로 남긴다.
고정 집계는 [evidence](../../../config/baselines/evidence/camera_tunnel_motion_v1/source-summary.json),
[기존 bag 집계](../../../config/baselines/evidence/camera_tunnel_motion_v1/legacy-summary.json),
[실패 report](../../../config/baselines/evidence/camera_tunnel_motion_v1/live-trial.json),
[복원 확인](../../../config/baselines/evidence/camera_tunnel_motion_v1/restoration-verified.json)을 따른다.
Source/wire 연결·충돌 배제, 유효 pixel 거리, 같은 위치/역할의 temporal 비교, 운동 명령 범위 검사와 Python compile을 확인했다.
분석 재현은 아래처럼 새 출력 폴더로 수행한다. 같은 버전의 NumPy/OpenCV를 사용하는 Noetic 분석 환경 결과를 정본으로 보존했다.

```bash
PYTHONPATH=src/asmc_camera/src:tools/perception OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/perception/review_tunnel_imu_motion.py --inputs ../data/runtime/family-motion-review-20261018-r1 --config src/asmc_camera/config --output /tmp/tunnel-legacy-review
PYTHONPATH=src/asmc_camera/src:tools/perception OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/perception/review_source_tunnel_motion.py --joint-width0 --trial ../data/runtime/camera-imu-motion-20261018-r1/live-motion-r1 --config src/asmc_camera/config --output /tmp/tunnel-source-review
```

ROS 공개 메시지·모델·정본 입력 코드는 바꾸지 않아 재빌드나 전체 모델 품질 재시험이 필요한 변경은 없다.
이동 기록은 실시간 M1의 공급률·10분 수락 시험이 아니다. 모델과 family는 기존 M0로 계속 실행한다.

## 다음 구현 하나

M1+폭 prior 완화의 내부 실험 프로필을 실제 family worker에 연결해 음영구간에서 품질·공급을 확인한다.
IMU 사용 시 완화한 prior와, source가 없을 때 기존 M0 fallback의 prior를 명시적으로 구분해야 한다.
새 설정은 config SHA에 포함하고 기본값 승격은 ≥10Hz·공백≤250ms·freshness350ms 및 주행 품질 확인 후 결정한다.
미세 흔들림은 근거 없는 자세 평활화로 숨기지 말고 IMU 시간 오차·지면 법선/높이 가정·프레임별 폭 추정을 먼저 구분한다.
