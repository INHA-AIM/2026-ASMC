# 센서 원본·수신·발행 시각 구현과 실측

## 적용 범위

`asmc_msgs/SensorFrameTiming`을 추가하고 camera JPEG·IMU·LiDAR sweep마다
`/morai_sensor_bridge/frame_timing`으로 발행한다. MD5는 `07dce68802e02b5b3f4cd89724a7cad7`이다.
원본 camera/IMU sec/nsec, LiDAR 첫/마지막 device tick, 첫/마지막 kernel/user receive,
monotonic receipt와 발행 시각·세션 UUID·독립 frame counter·실제 sensor wire sequence를 보존한다.
기존 센서 header·MD5·포트·receive 기본값과 Main의 시간 매칭은 유지했다.

Linux kernel timestamp NEW 옵션64를 우선 사용한다. ancillary truncation이나 미지원은
user-space fallback으로 명시한다. 커널 software timestamp를 노출/하드웨어 시각으로 부르지 않는다.
Raw source 검증은 선택한 header 정책과 독립적이다. 중복·충돌·역행 값을 receive로 덮어쓰지 않는다.
LiDAR packet tick 반복과 sweep endpoint 중복을 분리했다. 점별 deskew는 구현하지 않았다.

JPEG 조각 누락/재순서·미완성 timeout은 정상 영상 metadata를 만들지 않는다.
조립·source history·ROS 출력 큐와 확인기의 대응표에 상한을 두었다. Bridge 수신 경로에 파일 쓰기나
추가 JPEG 복사를 넣지 않았다. 정상 콘솔5초·진단1초 주기는 유지하며 timing counters를 진단에 추가했다.
기존 raw bag5개 정본 프로필과 이를 상속하는 alias에도 timing을 선택 항목으로 연결했다.
기록 시작 때 발행 중인 경우 함께 저장하며 구형 bridge의 raw 수집을 차단하지 않는다.

## 실제 ROS 및 MORAI 검증

`asmc-ros-noetic` overlay에서 공용 메시지와 bridge를 빌드했다.
독립 rostest master에서 실제 UDP 조각→CompressedImage, IMU와 metadata의 wire sequence·header 연결,
중복/충돌/역행 raw source와 receive header 보존, 무구독 시 sequence 무효 처리를 통과했다.
운영 포트와 다른19291/19303을 사용하고 차량 제어는 발행하지 않았다.

MORAI와 세 카메라 모델을 실행한 상태에서 queue1과 queue16을60초씩 관측했다.
초기 측정에서 raw source가 다른 IMU도 ROS 출력에서 누락됐다. 기존 비동기 IMU 발행 큐1을
16으로 늘려 짧은 burst를 보존했다. 영상의 queue1 최신 우선 정책은 유지한다.
다른 시각의 순차 표본이므로 완전히 같은 packet replay 대조나 장기 무손실 보장은 아니다.

| IMU 측정 | queue1 | queue16 |
|---|---:|---:|
| Timing 수신 | 2,462 | 2,448 |
| 센서와 정확히 연결 | 2,331 | 2,446 |
| 미연결 전체 | 131 | 2 |
| 시작/끝1초를 제외한 미연결 | 126 | 0 |

queue16의 미연결 IMU2건과 camera4 한 건은 시작0.02초 이내의 구독 연결 구간이다.
같은60초에서 cam1/2/3은 각각1,195/1,195건, LiDAR728/728건이 정확히 대응했다.
source 무효·충돌·역행, metadata sequence gap·regression, ancillary truncation·user-space timestamp
fallback·socket drop·timing 발행 오류는 관측되지 않았다. IMU673건은 같은 raw 시각의 재전송이었다.
별도 독립 자세 수로 세거나 원본 시각에 임의1ns를 더하지 않는다.

최종 표본의 camera 신규 입력은 약19.9Hz, camera4 약10Hz, LiDAR 약12.1Hz였다.
LiDAR를15Hz로 재발행하지 않았다. metadata의 last receive→조립 후 timing 생성 지연 p95는
front2.77ms·left3.87ms·right4.19ms·IMU4.60ms·LiDAR4.44ms였다.
이 값에는 기존 처리·스케줄링·센서 publish도 포함하며 metadata 추가 자체의 비용으로 해석하지 않는다.

Camera3뷰의3,585개 raw 시각은 모두 IMU raw timestamp 사이100ms 이내 bracket을 가졌다.
bracket span p50=42ms·p95=77ms·최대88ms였다. 이 집계는 timestamp 가용성이다.
실제 quaternion SLERP·노출 시각·clock offset·동적 재투영 정확도를 검증한 결과가 아니다.

## 재현·남은 작업

- [계약·직접 확인 명령](../../architecture/bridge-sensor-timing.md)
- `tools/bridge/verify_frame_timing.py`: bounded 대응표와 전체 timing/header·정확히 대응한 기록.
- 로컬 원본: `data/runtime/bridge-frame-timing-20261018-r1/`, `r2/`, `r3/`.
  r1은 초기 계측, r2는 queue1, r3는 queue16이다. 이전 실패도 보존한다.
- 실험 산출물: `artifacts/bridge-frame-timing-20261018-r1/`의 비교·두60초 보고서.
- [E380 갤러리](http://127.0.0.1:8000/bridge-frame-timing-20261018-r1/index.html).

현재 PC에는 bridge와 카메라 모델을 각각 한 세트로 실행했다. 잔류 ROS master·rqt·중복 생산자를
정리하고 확인기는 종료했다. 이 결과는 기존 카메라 전체 지연·family 순간 공백·10분 수락의 완료가 아니다.
동적 source-time 보정은 실제 센서 메시지와 timing이 대응하고 유효한 자세/이동 입력이 있을 때만 후속 대조한다.
Hosted CI는 이번 로컬 검사로 대체하지 않으며 push/병합은 하지 않았다.

최종 로컬 검사는 bridge Python45개·공용 generated message3개·실제 ROS fixture1개·
VehicleState C++ 회귀를 포함한다. 공용 계약/토픽14개, 문서/기존 수락39개,
오프라인 운동·겹침24개와 bridge 프로필6개도 통과했다.
Bridge 프로필 검사는 별도 로컬 저장 경로 작업의 `docker_ros_up.sh` 수정이 기존 SHA pin과 달라
그대로는 실패한다. 해당 사용자 파일을 바꾸지 않고, 이 파일만 committed HEAD를 쓰는
`/tmp/bridge-timing-clean-3n4prnws`의 검사 전용 fixture에서 새 bridge/설정/시험을 검증했다.
Docker 저장 경로 수정은 이 작업 커밋에 포함하지 않았다.
기존 표시10Hz 변경의 canonical config SHA 두 개도 검사 중 누락을 발견해 현행화했다.
이전 SHA·실험값·완료 상태는 보존했으며 이를 새 전체 스택 수락으로 표시하지 않는다.
