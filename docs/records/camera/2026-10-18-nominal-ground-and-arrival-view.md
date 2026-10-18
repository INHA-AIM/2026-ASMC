# 명목 노면0.36m·결과 도착 기반 시각화 적용

## 적용 결정

사용자가 MORAI VehicleSpec의 wheelRadius=0.36m를 기본으로 선택했다.
정본 `config/cameras_v3.yaml`의 노면은 `z=-0.36m`, `dz_dx=dz_dy=0`이며 source도 명목 제원으로 기록한다.
전방 카메라 장착(1.9,0,1.2)m·pitch2°와 IMU 장착은 유지한다. 평면 기준 전방 카메라 높이는1.56m다.
이는 측정한 suspension/heave 보정이나 M1-rest 채택이 아니다. IMU 운동 보정은 아직 미적용이다.
구형 `bev/cameras_v2.yaml`은 기존0.35m를 재현용으로 보존했다.

| 표시 | 기본 처리 |
|---|---|
| 실제 shared family 선 | 도착 즉시 최신 worker로 렌더링. 고정10Hz 대기 제거 |
| 신호등 | 기존 결과 도착 즉시 표시 유지. 생산자 목표10Hz |
| 객체3뷰 합성·개별 패널 | 최신 결과를 선택하는 worker, 최대10Hz |
| RGB/semantic 합성·개별 패널 | exact-stamp 묶음을 선택하는 worker, 최대10Hz |
| RGB BEV 비교 | 수요가 있을 때만 처리, 최대10Hz(종전5Hz) |

Raw source header·모드/generation·350ms freshness·좌표·mask·모델·추론 head·피팅 인자는 유지한다.
ROS 메시지/서비스 MD5, bridge 동작과 Main/LiDAR 매칭은 변경하지 않았다.
렌더링 thread에서 한 건 계산·최신 알림 한 건만 대기하며 새 알림은 이전 것을 대체한다.
주기 예산이 남았으면 즉시 처리하고, 제한 중이면 기다린 뒤 그 시점의 최신 표본을 읽는다.
무구독·무변경 처리는 표시 예산을 소모하지 않는다. 구독 해제/종료는 대기 worker를 깨워 종료한다.
Family의 무구독 처리에서는 재귀적으로 자신을 다시 깨우지 않도록 시험했다.

현재 PC의 runtime overlay에도 변경을 복사하고 카메라 모델을 한 세트만 재시작했다.
새 session에서 세 모델의 ACTIVE·prepared·pipeline_ready를 확인했으며 ground param도-0.36m였다.
추론/worker 오류를 자동 backend fallback이나 오래된 결과 재발행으로 숨기지 않는다.

## 지면 변경의 오프라인 대조

기존 Large semantic 연속216묶음(SUNNY/FOGGY×11/13/15시·진입/내부/이탈)에서0.35/0.36m를 비교했다.
두 경우 모두 family216/216 유효, 새 무효0이었다. 공통 유효 역할/지지 구간 x0.5–14.5m에서
횡방향 곡선 차이 p50=1.64cm·p95=3.94cm·p99=4.93cm였다.
독립 지면 GT의 위치 오차가 아니라 보정값 변경에 따른 곡선 차이다. 같은 장면의 연속 사진을
독립 장면으로 세지 않는다. 이전 정차 plane의 불충분한 지지 근거와 실패는
[원래 기록](2026-10-18-lane-display-and-rest-calibration.md)에 보존한다.

재현은 `tools/perception/compare_lane_ground_offset.py`의 기존 grid manifest·SHA로 수행한다.
원본 model-grid·가중치·split을 바꾸거나 새 추론을 실행하지 않았다.

## 표시 부하 비교와 남은 제한

이미 실행 중인 MORAI와 세 모델을 유지했다. 별도의 rqt 창·차량 제어·날씨 변경은 하지 않았다.
RGB BEV 비교까지 모든 표시 출력에 실제 ROS 구독자를 붙여15Hz 후보와10Hz 후보를60초씩 비교했다.
성능 값은 순차 구간의 관측이며 동일 동적 프레임의 paired 품질 비교가 아니다.

| 렌더러 CPU(한 코어100% 기준) | 큰 화면15Hz 후보 | 큰 화면10Hz 후보 |
|---|---:|---:|
| 객체 | 50.5% | 36.5% |
| Semantic | 53.9% | 43.4% |
| RGB BEV 비교 | 55.7% | 49.3% |
| 위 세 개 합 | 160.1% | 129.2% |

합계 CPU는 약19.3% 감소해 큰 합성 화면은10Hz 상한을 채택했다.
일반 표시(객체·semantic·family·신호등, RGB BEV 비교 미구독)에서도60초를 추가 측정했다.
Family는 공급된11.2–13.0Hz를 따라 표시했고, family 발행→화면 수신 지연 p95=7.41ms였다.
객체/semantic 합성의 고유 header 기준 공급은 약9.6–10.0Hz, 신호등은9.8–10.0Hz였다.
시각화가15Hz 결과를 만들어 낸 것이 아니며 생산자의 실제 공급을 그대로 따른다.

전체 원본 header→family 화면 지연 p95는 일반 표시에서도 약209ms였다. 표시 이후 대기는
작아졌으나 모델/후단 지연까지 해결됐다는 의미는 아니다. 목표15Hz·기존 p95≤150ms의
전체 수락은 아직 미달이다. 입력 camera는 짧은 별도 확인에서 약19.2–21.4Hz였다.
센서 입력 부족이나0.36m 변경만을 원인으로 확정하지 않는다.

모든 큰 출력10Hz 시험에서는 생산자 family와 화면의 동시 공백 약653ms·readiness false2회가 남았다.
일반 표시에서도 readiness false1회가 있었으며 원본 보고서에서 보존한다.
생산/수신/계측 중 어느 단계가 멈췄는지 이번 결과만으로 확정하지 않는다. 알려진 실패를
화면 smoothing·주기 하향·350ms 완화로 통과시키지 않는다. 이번을 전체 장기 수락으로 표시하지 않는다.

일반 표시 재측정은 bounded 비동기 journal로 기록했고 기록 누락0이었다.
그 구독자 GC는 최대약0.12ms였으나 앞선653ms 공백의 원인을 소급 확정할 근거는 아니다.
합성 header는 모든 뷰의 source tuple을 표현하지 못한다. 같은 합성 header의 반복이 곧 동일한
픽셀/동일 표본 재표시라는 뜻도 아니다. 객체 합성 header는 render 시각이라 source latency 순위에서 제외한다.
ROS JPEG 수신까지만 계측했으며 실제 rqt 페인팅/모니터 지연은 포함하지 않는다.

## Bridge timing 분석의 연결

원본 source와 receive 시각을 함께 보존하려면 현재 한 개 header 선택 방식에 metadata를 추가해야 한다.
[센서 timing 설계](../../architecture/bridge-sensor-timing.md)에 현재 파서·커널 수신·공용 메시지·
연결 키·clock domain·중복/역행·SLERP 활용·단계별 검증을 구체화했다.
이번에는 설계만 기록했으며 bridge 코드/메시지/`timestamp_mode`를 바꾸지 않았다.

## 결과와 재현

[E378 실험 결과](http://127.0.0.1:8000/camera-seg-large-deploy-20261018-r1/nominal-36cm-r1/index.html)에서
지면 대조·표시 CPU·공급률·실패 원본을 함께 확인한다.

- 표시 지표: `data/runtime/camera-display-15hz-20261018-r1/`,
  `camera-display-10hz-20261018-r1/`, `camera-display-normal-20261018-r1/`.
- 지면 대조: `artifacts/camera-seg-large-deploy-20261018-r1/nominal-36cm-r1/`.
- 변경 전 runtime 복구본·SHA: `data/runtime/camera-36cm-display-20261018-r1/`.
- CPU/ROS 회귀: 지면/mount 보존, arrival-driven family, stale 제거·무구독 busy loop 없음,
  latest 알림 대체·주기 대기·종료·오류 복구, 기존 exact sync/세대·mask·bbox 계약을 검사한다.
- Hosted CI·10분 연속 수락·IMU 동적 보정의 정확도 검증은 이번 완료 항목이 아니다.

로컬 결과: Noetic 카메라 회귀409개와 호스트 오프라인/문서43개를 통과했다.
Runtime overlay에는 planner/Main 원본이 없어 두 XML 통합 시험이 처음에는 경로 오류였다.
운영 경로를 바꾸지 않고 현재 저장소의 launch/config만 별도 test-support에 복사해 두 시험도 통과했다.
최초 오류와 test-support를 보존하며 hosted CI 통과로 대체하지 않는다.
