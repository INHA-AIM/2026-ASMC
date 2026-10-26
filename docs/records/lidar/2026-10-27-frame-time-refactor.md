# C0 후속 — 공용 좌표 수학·scan 변환 재사용·IMU 거부 이력

PR #115에서 사용자 승인으로 진단 범위를 운영 수정까지 확장했다. C0 최초 대조와
[GPS0·복귀 수정](../bridge/2026-10-26-gps-validity-recovery.md)은 각각의 원본 근거를 보존한다.
검출·추적·fusion·HD맵 임계값, 공개 메시지·토픽·MD5, 카메라 모델과 M1 정책은 유지한다.

## 책임과 바뀐 코드

| 소유자 | 이번 변경 | 유지하는 책임 |
|---|---|---|
| `asmc_geometry` | 중복 WGS84→ENU/UTM·grid yaw·GPS0 유효성, 같은 clock의 past-only 조회·ns age 비교를 헤더 라이브러리로 분리 | 노드·상태 추정기·메시지·Python binding 없음 |
| bridge | 공용 geodesy 사용. 앞선 GPS0·timeout·중복/역행 이후 기준 재확보 수정 유지 | 센서 수신·원본/수신 시각 보존, 안테나 기준 상태. LAN2LAN IP/포트 수정 없음 |
| `asmc_lidar` | scan별 준비 변환, GPS별 grid yaw offset 캐시, 중간 점군 복사 제거, causal 거부 이력과 진단 | scan 시각 센서 선택·수평화·장착 변환·OBB/속도 변환·HD맵 판단 |
| `planning_inputs` | 기존 geodesy 중복을 공용 헤더로 교체하고 소비 경계를 코드 주석에 명시 | 후륜축 위치·속도 추정, gain·grid yaw·예측/복구 정책. 이미 변환된 LiDAR 객체를 재보정하지 않음 |
| 카메라 | 동일 원본 크기·mount·roll/pitch 투영 fixture로 C++/Python 정합 검증 | 원본 clock/session의 SLERP·M1/M0, 픽셀→지면, family·BEV 재사용 경로 유지 |

`base_link`는 실제 후륜축 원점이고, 내부 수평화 좌표와 다르다. LiDAR는 실제 적용한
수평화 역변환과 장착 변환을 scan context에서 한 번 합성해 중심·8코너·장축·벡터에 사용한다.
`map` 파생 결과와 HD맵 조회는 유효한 scan pose가 있을 때만 만든다. GPS 무효를 local
검출 중단과 혼동하지 않는다. 일반 판제·전역경로·HD맵은 map을 유지한다.

공통 함수가 생겼다고 세 pose 생산자가 동일해진 것은 아니다. bridge 안테나 원점과 판제
후륜축 원점, 각 추정기의 gain·예측 정책은 여전히 다르다. LiDAR의 GeographicLib와 기존
판제 UTM 다항식도 유지했다. 공통 grid-yaw 규칙에 각 투영 함수를 주입해 수치 정책을 숨겨 바꾸지 않았다.
카메라 source clock과 LiDAR device-hour clock은 대응 근거 없이 차감하지 않는다.

## 26개 IMU stale scan의 원인과 해결

원본 `lidar-frame-time-c0-live-20261026-r1/timing.bag`의 565 scan 중 26건이
`IMU is stale for LiDAR scan`으로 기록됐다. 같은 기록을 native PoseHistory로 재생했다.

- 26건에서 ROS header(bridge 수신 시각) 기준 IMU age p50 **2.267ms**, p95 **3.021ms**, 최대 **4.040ms**다.
  IMU가 오래 도착하지 않아 생긴 120ms 초과로 설명할 수 없다.
- 원본 quaternion yaw는 두 번 **147.501° / 145.430°** 급변했다. 시험의 터널 텔레포트와
  원위치 복원에 대응한다. 저장된 시작/복원 transform도 이를 뒷받침한다.
- 기존 guard는 `5° + 120°/s × 마지막 정상 표본 이후 시간`을 초과한 자세를 거부했다.
  이 기록에서 99개의 IMU가 거부됐다. 이 guard의 거부 자체를 결함으로 처리하지 않는다.
- 문제는 단일 `latest-invalid` 시각이었다. scan을 기다리는 동안 scan보다 미래의 거부 표본이
  들어오면 과거 오류를 덮어썼다. 그 결과 stale로 잘못 설명하거나, 아직120ms 이내인 옛 자세를 사용했다.
- IMU/CAN 거부 이력을 기존 history 기간(기본5초)·최대1024개로 제한하고, 정상 표본과 같이
  **scan 시각 이전의 마지막 거부**를 선택하도록 수정했다. GPS pose에 붙이는 IMU도 같은 규칙을 사용한다.

수정한 native 재생은 원래 stale26 모두 `imu_input_invalid / implausible IMU yaw jump`로
설명한다. 같은 조건에서 기존에 유효했던 **3건의 오래된 자세 재사용도 차단**해
`local_ready536 / imu_input_invalid29`가 된다. 유효 공급 수를 늘리려고 오류를 정상으로
승격하지 않는다. 구 기록539/26과 새 재생536/29를 함께 보존한다.

trace 수신 시점에 조회하는 재생이라 ROS worker 스레드의 정확한 스케줄까지 재현한 것은 아니다.
미래 오류가 과거 오류를 가리는 결함은 별도 합성 회귀에서 확정했다. 이 bag에는 충돌 토픽이
없으므로 collisionObjects의 문자열만으로 실제 충돌 또는 네트워크 누락을 원인으로 확정하지 않는다.

### 시험 reset과 운영 진단

의도적으로 정차 차량을 텔레포트하는 시험은 **정착 후 명시적 reset**으로 새 세대·추적·센서 이력을
시작한다. `run_tunnel_motion_trial.py --lidar-reset-service /migration/lidar/lidar_pipeline/reset`
옵션은 시험 소유 노드에만 지정한다. 기본은 미지정이며 서비스 이름을 추측하지 않는다.
원위치 복원 뒤에도 reset한다. 기존 reset 서비스가 generation과 대기 작업·tracker를 정리한다.
일반 주행·충돌·센서 오류에서 자동 reset하거나 guard를 완화하는 정책은 추가하지 않았다.

1Hz diagnostics와 수요 기반 trace는 선택 IMU stamp, 마지막 수신 source/receipt stamp,
해당 scan의 거부 stamp·사유를 구분한다. source와 monotonic receipt를 서로 빼지 않는다.
정상 최대 age120ms는 유지하며 정수 ns로 inclusive 경계만 명확히 했다(120ms 통과, +1ns 실패).

## 계산 최적화

- `PointCloud2 → PCL`을 최종 소유 버퍼에 직접 변환한다. 기존 임시 PCL 벡터를 전체 복사하던 한 단계를 제거했다.
  점·순서·empty metadata·원본 메시지 불변성을 유지한다.
- 객체마다 반복하던 time/mount/map 검증과 합성 행렬을 `PreparedTransform`으로 scan당 한 번 준비한다.
  벡터 출력 capacity도 미리 확보하며 동일 변환을 OBB·속도에 재사용한다.
- true ENU→UTM grid yaw offset은 GPS 표본마다 계산해 저장한다. scan마다 GeographicLib 두 번을 호출하지 않는다.
- 정렬된 이력의 표본 검색은 binary search다. 실제 보간·bounded 예측·age 한계는 변경하지 않는다.
- 점군 전처리·군집·Kalman·카메라 matching·HD맵 컷은 바꾸지 않는다. 단계별 노드를 추가하지 않는다.

## 검증과 근거

- 일반649 + 터널208 = **857 scan**에서 객체 수·순서·ID·8코너·추적 결과가 리팩토링 전과 동일하다.
- 두 bag의 bridge/planning native 상태 출력은 **바이트 단위 동일**하다. GPS0 기존 결함 수정은 별도 v1→v2 비교에 보존한다.
- 세 카메라54개 투영과 부정 대조를 포함한 C0 합성7개, 상태 probe의 GPS0·복귀 fixture 통과.
- 공용 geometry1개, bridge9개, planning_inputs7개, 최종 LiDAR CTest34개 통과.
  LiDAR에는 실제 독립 ROS master의 관측·reset·세대·카메라 상태 회귀가 포함된다.
- 운영 컨테이너 전체 workspace와 최종 LiDAR 바이너리를 재빌드했다. 기존 사용자 프로세스를 종료하지 않았다.

성능 비교는 이전/신규 native replay를 일반·터널 각각3회 교차 실행한다. 파일 IO·trace 직렬화를 포함하는
wall/CPU/RSS와 기존 pipeline timer를 구분한다. 실시간 입력→발행 지연이나 공급Hz 수락으로 확대하지 않는다.

| 자료·구성 | 전체 replay wall 중앙값 | CPU 중앙값 | peak RSS 중앙값 | 기존 pipeline p95 중앙값 |
|---|---:|---:|---:|---:|
| 일반649 scan 전 | 7.002s | 4.973s | 156.04MiB | 10.037ms |
| 일반649 scan 후 | 6.984s | 4.949s | 156.11MiB | 9.869ms |
| 터널208 scan 전 | 2.949s | 2.013s | 155.91MiB | 11.430ms |
| 터널208 scan 후 | 2.934s | 1.993s | 156.15MiB | 11.377ms |

3회 반복의 차이는 작아 **유의미한 전체 속도 향상이나 RSS 감소를 확정하지 않는다.**
반복 범위 안에서 출력 동등성과 자원 사용 유지, 중복 계산/복사 제거를 확인했다.
새 기하 코어가 처리량이나 터널 전체 공급 기준을 달성했다는 결과는 아니다.

재현 근거:

- 최초 리팩토링 `0412c06d`, causal 거부 이력 `8b51eedf`.
- v3 / `lidar-frame-time-c0-refactor-20261026-r1`: 문구 우선순위만으로 해결되지 않음을 발견한 중간 기록.
- v4 / `lidar-frame-time-c0-refactor-20261027-r2`: causal 수정 native audit, `imu-history.jsonl`,
  `imu-summary.json`, `performance-r3/summary.json`, 원본 RGB/지도 갤러리.
- 성능 첫 시도는 컨테이너 GNU time 부재로 실행 전 실패했다. 해당 디렉토리를 덮어쓰지 않고
  Python POSIX wait4로 전환했다. 두 번째 시도에서는 비교용 큰 JSON을 보유한 부모의
  RSS가 fork 자식의 최고 사용량에 섞였다. `performance-r2`의 RSS는 채택하지 않는다.
  최종 `performance-r3`는 작은 별도 supervisor에서 native replay를 실행해 계측을 분리했다.
  두 실패·중간 디렉토리는 보존한다. 입력·운영 계산 변경은 없다.
- v5 manifest는 v4와 같은 운영 소스를 유지하면서 최종 계측/IMU 집계 도구를 고정한다.
  운영 수학 변화가 없으므로 v4 native 결과를 재사용한다.
- 결과 정본은 D의 등록된 run이며 Git에는 코드·manifest·집계 설명만 둔다. 새 bag이나 모델 학습은 수행하지 않았다.

이 후속에서는 기존 실주행 기록을 재생해 결함을 고정하고 독립 ROS로 검증했다. LAN2LAN 설정을
local로 바꾸거나 새 주행을 하지 않았으며, 전체 주행 수락·터널 벽 제거·3뷰 matching 완료를 뜻하지 않는다.

## 다음 실행과 후속 과제

이 PC의 컨테이너 빌드는 완료됐다. 기존 프로세스는 시작할 때의 코드를 사용하므로 다음 실행 전
본인이 실행했던 launch를 Ctrl+C로 종료하고 다시 실행한다. 네트워크 설정은 기존 LAN2LAN 명령을 유지한다.
팀원이 pull한 뒤에는 다음과 같이 전체 의존성을 빌드한다.

```bash
docker exec -it asmc-ros-noetic bash
cd /root/ws
./scripts/build_ws.sh -j2 -l2 -DCATKIN_WHITELIST_PACKAGES=""
source devel/setup.bash
```

카메라·LiDAR·판제 실행은 [명령 모음](../../operations/commands.md)을 따른다.
추가 launch 옵션은 없다. `asmc_geometry`는 실행할 노드가 아니다.

다음 공동 과제는 **동일 기록에서 카메라 source와 LiDAR scan의 clock 대응 근거 확보**다.
그다음 3뷰 matching 시간 경계를 검증한다. 카메라 SLERP와 LiDAR의 latest-before 정책을
이름만 맞추려고 통합하지 않는다. 재호는 추적·속도·터널·matching 개선을 계속 담당하며,
판제는 현재 차량 pose·경로·행동/센서 장애 정책을 유지한다. 단일 pose 생산자는 별도 검증 후 결정한다.
