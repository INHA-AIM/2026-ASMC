# LiDAR 15Hz 설정의 실제 회전율과 bridge 진단 정리

사용자가 MORAI UI에서 Intensity·Instance LiDAR를15Hz로 설정한 상태에서 검사했다.
저장소의 이전 GT JSON10Hz를 이번 실행 설정으로 해석하지 않았다. 센서 설정·날씨·차량 위치를
변경하거나 차량 제어를 발행하지 않았다. 기존 main checkout과 운영 master도 유지했다.

## 원시 UDP → ROS 비교

| 조건 | Intensity / Instance 결과 | 관측 |
|---|---|---|
| ROS 변환 없는 원시 수신30초 | wall12.0965 /12.0959Hz; device12.0960 /12.0965Hz | 각각18,150패킷·363회전, 약605packet/s |
| 독립 ROS master의 bridge60초 | 구독자12.4893 /12.4890Hz | 각각749개 수신, 중복·역행stamp0 |
| ROS 종료 후 원시 수신30초 | wall12.5118 /12.5118Hz; device12.5121 /12.5123Hz | 각각18,750패킷·375회전, 약625packet/s |
| 최종 설치본 ROS 재확인30초 | 구독자12.1465 /12.1465Hz | 각각365개 수신, 진단30개·콘솔6줄씩 |

원시 수신은 ROS·NumPy 좌표 계산·PointCloud 생성 없이 UDP만 받는다. Exclusive bind를 사용해
기존 receiver와 경쟁하면 실행을 거부한다. 세 조건은 직렬 실행했으므로 작은 조건 간 차이를
bridge 성능 개선으로 해석하지 않는다.

60초 bridge 진단의 첫/마지막 표본 사이에는 두 LiDAR 모두 **회전 경계738회·발행738개**였다.
시작 시 불완전 회전1개만 제외했고 이후 조립 timeout·parse error·socket drop은0이다.
관측한 processing p95의 최대는 Intensity3.098ms, Instance2.629ms다.
두 원시 구간에서도 Linux `InErrors`·`RcvbufErrors`·`InCsumErrors`·socket drop은0이었다.
두 번째 원시 구간은 회전당50패킷, 방위각 증가량719/720/721(0.01도 단위)만 관측했다.
동일한 패킷 간격의 약7.2도 연속 진행으로, 이 표본에는 패킷 건너뜀 흔적이 없다.
Device 시간/실제 시간 비율도 약0.9998로 전체 시뮬레이션 시계가 느려진 증거는 없다.

**확인됨:** 이 실행의 약12–12.5Hz는 ROS PointCloud 변환 이전부터 존재한다. Bridge가 받은
회전을 추가로 잃어서15Hz가12Hz로 내려갔다는 근거가 없다. Docker는 host network, CPU quota
제한 없음·가용CPU20개·throttling0이다. 소켓 수신 버퍼의 실제 Linux 보고값은8MiB다.

**추가 대조 필요:** MORAI 내부 센서 스케줄링·점군 생성 비용·동시 GT 센서 부하 중 어느 항목이
설정15Hz와 실제 회전율 차이를 만드는지는 이번 비침습 검사만으로 확정하지 않는다.
Linux 카운터0을 Windows/Docker Desktop 이전 구간의 무손실 보장으로 사용하지 않는다.
없는 scan을 복제하거나 회전 한 바퀴를 여러 PointCloud로 쪼개15Hz처럼 표시하지 않는다.

## 로그와 진단 변경

- Network/Sensor 각각5초마다 한 줄. 최근Hz·문제 상태·idle 입력·누적/새 오류·socket drop과
  Network의 마지막 링크를 표시한다. 누적 메시지 수와 LiDAR 상세 줄은 diagnostics로 옮겼다.
- Diagnostics는1초마다 발행한다. 모든 주기는 monotonic clock을 사용하고 놓친 주기를 몰아서
  실행하지 않는다. `diagnostics.yaml`을 두 component가 함께 사용한다.
- 수신 스레드는 짧은 잠금으로 제한된 표본과 카운터를 갱신한다. 보고자는 잠금 안에서
  스냅샷을 복사한 뒤 집계·직렬화·발행을 수행하여 `deque mutated during iteration`을 해결했다.
- 보고 callback의 예외는 로그와 `reporter_failures`로 남기고 다음 주기에 재시도한다.
- 기본 수신 age0.5초, GPS1.0초, IMU0.3초를 넘으면 `STALE`. 최초 필수 미수신은 `WAITING`,
  선택 입력의 최초 미수신은 `IDLE`, 잘못된 입력은 `INVALID`, receiver 실패는 `ERROR`, 최근
  parse/send 오류·socket drop은 `DEGRADED`다. 정상 수신이 복구되면 회복을 기록한다.
- `health`는 전송·수신 상태다. GPS fix·IMU 자세·차량 상태의 유효성이나 fusion freshness를
  대신하지 않는다. 기존 estimator·인지·fusion의 기준은 변경하지 않았다.
- Hz는 최근 보고 구간이고, latency·패킷 중앙값·wall/device 회전율은 최근 최대4096표본이다.
  과거 중앙값을 현재1초 raw Hz와 동일 기간으로 비교하지 않는다.

실제60초 시험에서 각 diagnostics60개·콘솔12줄씩 수신했다. Sensor 진단 간격은
0.984–1.016초, Network0.998–1.002초, Sensor 콘솔4.998–5.006초,
Network4.999–5.002초였다. 보고 예외는0이다.
기존 `9798 #DetailEgoInfo$` 헤더 불일치는 `INVALID`와 `header_errors`로 드러낸다.
본문을 확인하지 않은 새 프로토콜을 임의로 해석하지 않았다.

## 검증과 재현

CPU 동시 접근·상한·표본 불변성·stale/회복·선택 입력·callback 실패 후 재시도·장치 시각
rollover를 포함한 Python26개를 통과했다. 실제 ROS 시험은 `/clock`이0으로 멈춘 상태에서
`WAITING → OK → STALE → OK`와1초 진단을 확인한다. 기존 protocol·launch·토픽 검사와
vehicle-state C++ 회귀, 독립 overlay build/install도 함께 확인한다.
CI에 이 브릿지 회귀를 실행하는 단계를 추가했다. Hosted 결과는 PR의 실제 결과를 따르며
로컬 시험 통과를 hosted 통과로 대신하지 않는다.

```bash
# 정상 실행 상태에서 읽기 전용 관측
python3 tools/bridge/measure_bridge_reporting.py \
  --duration 30 --json-out /data/runtime/bridge-reporting.json

# bridge를 종료한 상태에서만 실행: 기존 UDP bind가 있으면 실패
python3 tools/bridge/measure_lidar_ingress.py \
  --duration 30 --json-out /data/runtime/lidar-ingress.json

catkin_make run_tests_asmc_bridge -j2 -l2
catkin_test_results --verbose build/test_results/asmc_bridge
```

원본은 workspace 공통 `data/runtime/bridge-diagnostics-20261017/`에 보존한다.
Git에는 [집계와 SHA](../../../config/baselines/evidence/bridge_diagnostics_20261017/summary.json)를
남긴다. 첫 검사의 임시 fixture 경로 누락과 재검사 기록도 보존한다.
최종 확인을 시작하는 임시 runner의 출력 경로 조립 오류는 운영 코드와 별개였으며,
해당 실행을 정리한 뒤 `final-r2`에서30초 확인을 완료했다.
시험용 master·bridge·구독자는 종료하고 사용자 프로세스와 센서 설정을 유지했다.
