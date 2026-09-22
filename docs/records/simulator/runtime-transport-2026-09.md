# MORAI runtime·transport 검증 기록 — 2026-09

> **문서 역할:** Experiment Record — 26.R1.H3 네트워크, UDP bridge, bag과 종료된 GT 실험
>
> 현재 실행값은 [운영 가이드](../../operations/README.md)를 따른다. 이 문서의 수치는 당시
> 환경의 관측값이며 현재 설정 또는 본선 보장을 뜻하지 않는다.

## 1. 2026-09-05 Docker host network

26.R1.H3와 기존 센서 설정을 유지한 채 Docker Desktop host network를 시험했다.

| 항목 | 결과 |
|---|---|
| Compose | `network_mode: host`, published port 없음 |
| Windows→container UDP `39199` | 통과 |
| container→Windows UDP `39200` | 통과 |
| Ego·RGB 4대·LiDAR·GPS·IMU | 모두 수신 |
| LiDAR | 최초 8.23 Hz, bridge 재시작 후 8.27 Hz |
| bridge 42초 중단 후 재실행 | Docker/컨테이너 재시작 없이 복구 |

이 결과로 host network를 WSL 개발 기본값으로 채택했다. 시험 중 `udp_bridge.sh`의 `cmd_ip`가
통합 launch에서 빠지던 문제를 수정해 `127.0.0.1`이 송신 node까지 전달되게 했다. 이 결과는
Windows loopback 검증이며 실제 두 PC LAN ingress를 보장하지 않는다.

## 2. 2026-09-09 v2 UDP→ROS 60초 기준선

조건은 26.R1.H3, `ASMC_sensor_set_v2.json`, Docker host network,
`timestamp_mode=receive`다.

```bash
python3 scripts/verify_morai_topics.py \
  --profile runtime --duration 60 \
  --json-out /data/rosbags/audits/udp_bridge_v2_60s.json
```

### 수신률

| topic | 설정/기대 Hz | subscriber Hz |
|---|---:|---:|
| RGB front/left/right | 20 | 20.003 / 20.004 / 20.004 |
| RGB extra | 10 | 10.003 |
| GPS | 5 | 5.001 |
| IMU | 50 | 46.029 |
| Intensity LiDAR | 10 | 8.244 |
| Ego | 50 | 40.441; bridge interval 약 46.2 |

Linux UDP `InErrors`, `RcvbufErrors`, `SndbufErrors`, `InCsumErrors`, `MemErrors`, socket drop과
bridge parse error는 모두 0이었다. 따라서 nominal보다 낮은 LiDAR/IMU/Ego rate를 이 자료만으로
UDP packet loss라고 판정할 수 없다. 원본 sequence가 없어 SIM cadence, ingress와 ROS
scheduling을 분리할 수 없기 때문이다. `long_gap_equivalent_periods`도 drop count가 아니라
arrival jitter 지표다.

### bridge 처리·message age

| 경계 | p50 | p95 | max |
|---|---:|---:|---:|
| RGB front | 0.094 ms | 0.258 ms | 1.509 ms |
| RGB left | 0.079 ms | 0.271 ms | 4.636 ms |
| RGB right | 0.062 ms | 0.210 ms | 0.666 ms |
| Intensity LiDAR | 1.142 ms | 1.583 ms | 4.150 ms |
| Ego UDP→ROS | 0.119 ms | 0.423 ms | 0.832 ms |
| Ctrl callback→UDP send | 0.080 ms | 0.110 ms | 0.364 ms |

receive stamp 기준 subscriber age p95는 RGB `0.690/0.653/0.578 ms`, LiDAR `1.522 ms`,
Ego `0.603 ms`, GPS `0.572 ms`, IMU `0.500 ms`였다. 이는 MORAI sensor 생성부터의 end-to-end
latency가 아니다. LiDAR assembly는 p50 `109.7 ms`, p95 `215.8 ms`로 packet 축적과 도착
cadence의 영향을 포함한다.

bridge 처리값에는 UDP parsing, point 계산과 ROS message 생성이 함께 들어 있으므로 전부 제거
가능한 ROS overhead가 아니다. 이 기준선에서는 bridge가 명백한 병목이라는 근거가 없었다.
direct UDP A/B는 같은 run에서 bridge 경계가 deadline의 5~10% 이상이거나 sequence 기반 ROS
drop 또는 반복적인 control deadline miss가 확인될 때만 다시 연다.

## 3. raw→replay→result 기준선

`camera_raw` 30초 run을 고정 4-class checkpoint로 1.0배 재생했다. paused replay controller를
먼저 켜고 inference, result recorder, start service 순서로 시작해야 wall time과 sim time이
섞이지 않았다.

| 항목 | 결과 |
|---|---:|
| raw RGB front/left/right | 20.0 / 20.0 / 20.0 Hz |
| result front/left/right | 18.76 / 19.78 / 19.71 Hz |
| 3-view 처리량 | 약 56~59 frame/s |
| inference stage | 약 23~25 ms/batch |
| end-to-end p50/p95 | 53.8 / 84.8 ms |
| output bag-header p95 | 약 77~80 ms |
| decode/inference error | 0 / 0 |
| timestamp regression | 0 |

Foggy 13시 `full_raw` 19초 smoke에서는 RGB front/left/right `20.01 Hz`, extra `10.01 Hz`,
LiDAR `8.45 Hz`, GPS `5.00 Hz`, IMU `48.58 Hz`, Ego `48.67 Hz`였다. 모든 header가 단조
증가했고 LiDAR bag-header p95는 `1.51 ms`였다. metadata에는 UDP receive mode, live bridge
parameter, bag SHA-256과 complete 상태가 남았다.

## 4. UDP Streaming GT — 종료된 경로

GT 센서셋의 RGB/Semantic과 Intensity/Instance LiDAR source timestamp를 시험했다.
receive mode는 bridge가 message를 만든 ROS 시각, source mode는 MORAI camera/IMU/Ego/Object
시각과 Velodyne device time을 사용했다. 잘못되거나 역행한 source time은 receive time으로
fallback하고 diagnostics에 남겼으며 source timestamp가 없는 GPS는 항상 receive time이었다.

| 항목 | 결과 | 판단 |
|---|---:|---|
| RGB/Semantic 3-view | 약 20 Hz | rate 통과 |
| ID 5 / ID 11 | 8.61 / 7.63 Hz | rate 기준 통과 |
| RGB–Semantic p95 | 34/35/31 ms; 다른 표본 3/3/3 ms | 위상 변동, strict gate 실패 |
| ID 5–11 p50/p95 | 12.0 / 101.6 ms | p95 50 ms gate 실패 |
| source regression/fallback | 0 / 0 | 통과 |
| ID 11 schema | xyz float32 + `instance_id` uint32 | 통과 |
| Instance↔Object ID | Instance `[2,3,4]`, Pedestrian `[2,4]`, NPC `[3]` | 대응 통과 |

Foggy 장면 364만 sampled point 중 41,551개가 nonzero Instance였고 Object ID와 대응했다.
그러나 camera/LiDAR strict pairing을 만족하지 못했으므로 Streaming GT는 학습 pair 자동 생성
경로로 채택하지 않았다. `lidar_dev_gt_udp`, `dev_oracle_udp` profile은 재현용 privileged
실험이며 `deployment_allowed=false`다. 학습 정답은 Capture의 Instance point와
`_instance.txt`/`_instance_8Points.txt`를 사용한다.

bridge 누적 count는 v2로 hot reload해도 0으로 돌아가지 않는다. `lidar_instance=951(0.0Hz)`는
과거 누적 수신 이력이고 현재 v2가 GT를 송신한다는 뜻이 아니다. 괄호의 interval rate와
`last_message_age_sec`로 현재 stream을 판단한다.

별도 20초 live 표본의 ID 5/11은 `8.03/7.43 Hz`였고 camera pairing p95는 같은 조건에서도
`34/35/31 ms`와 `3/3/3 ms` 사이로 변했다. 단일 짧은 표본으로 strict sync를 확정하지 않았다.

## 5. direct ROS/rosbridge GT — 실패 후 종료

MORAI Network/Sensor를 ROS로 전환하고 `rosbridge_websocket :9090`을 시험했다.
Network Settings client 21개와 `/Object_topic` 약 39~40 Hz는 확인했지만 RGB, Semantic,
ID 5/11, GPS와 IMU는 advertise되지 않았다. `/Ego_topic`도 MORAI payload의
`angular_velocity`와 pinned beta message가 맞지 않아 거부됐다.

필수 sensor advertise gate에서 실패했으므로 5분 부하·정합 비교를 중단했다. 미병합 구현은
`origin/backup/ros-gt-experimental-20260910`에만 남아 있고 현재 지원 경로나 후속 gate가
아니다. 이 실험 때문에 공용 `morai_msgs`를 변경하지 않는다.

## 6. 보존된 결정

- 실제 운용·모델 A/B는 v2 Real Time UDP raw/result bag을 사용한다.
- 단일-frame 학습 GT는 Capture를 우선한다.
- Simulation Time Sensor Record는 장착 sensor sequence를 저장하지만 없는 GT channel을
  생성하지 않는다.
- direct ROS GT와 Streaming GT를 신규 기본 경로로 사용하지 않는다.
- `asmc-ros-noetic`과 내부 ROS bridge는 대회 UDP 정규화와 재현 평가를 위해 유지한다.
