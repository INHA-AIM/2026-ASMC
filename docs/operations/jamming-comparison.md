# 세 재밍 플래너 비교 실행

2026-11-10 PR #141 병합으로 대회 최종 주행 소스 `a62360ec`가 main `d2b18f74`에 포함됐다.
`competition/2026-final` 태그는 마지막 대회 소스를 고정한다. 이 문서는 현재 launch·YAML을 기준으로 하며,
[정리 전 안내 원문](../records/planning/2026-10-30-jamming-comparison-before-closeout.md)과 당시 실험 결과는 별도로 보존한다.
사용자의 대회 주행 확인과 개별 시험 기록을 보존하며, 이번 문서 정리를 새 실주행·전체 CI 수락으로 해석하지 않는다.

## 운영 정본과 현재 비교 환경

정본은 `2026-ASMC` checkout과 `asmc-ros-noetic` 한 컨테이너다.
ROS master는11311, 판제는 <http://localhost:8088/web/index_v2.html>, LiDAR는 <http://localhost:8091/>를 사용한다.
컨테이너 이름이나 Git HEAD만으로 실행 바이너리가 최신이라고 판단하지 않는다.
재빌드와 실제 선택 파일 확인은 [명령 모음](commands.md) 및 [정본 실행 확인](../records/repository/2026-11-10-canonical-runtime-closeout.md)을 따른다.
Docker Desktop의 Windows 브라우저에서는 LiDAR `view.launch`에 `bind_address:=0.0.0.0`을 지정한다.

기본 음영 선택은 **`jamming_planner:=localization` + `localization_local_recovery:=true`**다.
기본 launch에서 localization MPC 한 개가 native 지도 경로 또는 검증된 LOCAL 대체 경로를 추종한다.
LOCAL 대체 경로 생산자가 함께 실행되지만 별도 LOCAL 제어기는 실행되지 않는다.

각 터미널은 같은 checkout의 환경을 사용한다.

```bash
docker exec -it asmc-ros-noetic bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
```

아래 각 행은 별도 터미널에서 순서대로 한 번 실행한다. 브릿지는 필요한 master를 함께 시작할 수 있다.
이미 같은 스택이 실행 중이면 중복 기동하지 않는다. 수동 실행과 전체 스택 실행기를 함께 켜지 않는다.

```bash
./scripts/udp_bridge.sh
roslaunch asmc_camera camera.launch
roslaunch asmc_lidar lidar.launch
roslaunch asmc_planner autonomy_no_bridges_v2.launch
```

LAN2LAN에서는 첫 줄만 `./scripts/udp_bridge.sh --transport lan2lan`으로 바꾸고,
MORAI의 같은 버전 LAN2LAN Network/Sensor 파일을 함께 로드한다.
브릿지는 센서·차량 상태를 제공하며 autonomy가 카메라/LiDAR 생산자를 다시 시작하지 않는다.

## 병합 전 비교 프로필의 실행

과거 `asmc-jamming-compare`의 수동11373·판제8089·LiDAR8097, 자동11374는 개인 비교 환경이었다.
현재 정본 명령의 기본 포트가 아니다. [정리 전 원문](../records/planning/2026-10-30-jamming-comparison-before-closeout.md)은 재현 근거로만 사용한다.
#125·126·138·139는 과거 병합 없이 닫혔고, #142·143·144의 커밋은 #141을 통해 main에 포함됐다.

## 공통 전환과 책임

방식을 바꾸려면 현재 autonomy를 종료한 뒤 다음 중 하나만 실행한다. 입력·경로 생산자와 제어 연결은 launch가 선택한다.
아래 표는 `autonomy_no_bridges_v2.launch` 기준이다. 전체 스택용 `autonomy.launch`의 전달 인자 기본값은 [인자 목록](launch-reference.md)에서 별도로 확인한다.

| 선택 | 기본 속도 설정 | 입력·제어 |
|---|---|---|
| localization — 기본 | `jamming_speed_kph=15`, 순항15km/h | GPS anchor·IMU·속도·map/LiDAR, localization MPC; 검증 LOCAL 대체 경로 기본ON |
| local | 상한20·순항15km/h | 관측 노면·family·local LiDAR, LOCAL MPC |
| camera | 목표20km/h, creep3km/h | 기존 camera BBox/family·LiDAR 기반 선택과 자체 제어 |

```bash
roslaunch asmc_planner autonomy_no_bridges_v2.launch jamming_planner:=local
roslaunch asmc_planner autonomy_no_bridges_v2.launch jamming_planner:=camera localization_local_recovery:=false
```

기본 localization으로 돌아갈 때는 autonomy를 종료한 뒤 인자 없는 기본 명령을 사용한다.
localization의 native 경로만 비교할 때 `localization_local_recovery:=false`, 동일 속도 상한 비교는
`jamming_speed_kph:=10`을 명시한다. 속도는 최종 유효 경로·곡률·제동·입력 상태의 제한을 추가로 받는다.

공통 진입·준비는 `src/asmc_planner/config/jamming_transition.yaml`과 GpsModeSource가 담당한다.
현재 A448 시작 station1859.641578240m에서 재밍 소유권을 선택하며 접근 목표25km/h와 실제 터널 시작1877.744798966m는 구분한다.
GPS 유효성 하나만으로 구간 안에서 NORMAL로 복귀하지 않는다.
최종 `/ctrl_cmd` 생산자는 selector 한 개이며, 각 제어기는 `/ctrl_cmd/normal` 또는 `/ctrl_cmd/jamming`을 제공한다.

검증된 LOCAL 후보와 회복 가능한 native 실패가 있으면 첫 실패 계획에서 즉시 인계한다.
native 복귀는1초 안정화를 확인한다. 모드와 준비 요청의 도착 순서가 바뀌어도 기존100ms 인계 창을 사용하며,
250ms 명령 watchdog과 센서 freshness를 늘리지 않는다.

## 회복·지도 경계

LOCAL/localization의 `jamming_motion.yaml`은 차폭1.892m·길이4.635m·후륜축 뒤0.790m,
고정 객체/노면 추가 여유0m·탈출 상한5km/h를 공유한다.
0m는 실제 차체 겹침·관측 불확실성·미래 점유·도로 외곽 검사를 제거한다는 뜻이 아니다.
일반/로터리의 충돌 여유는 별도 설정이며 이 음영 설정으로 바뀌지 않는다.

카메라 의미 그룹은 실제 운동 상태와 다르다. vehicle/dynamic_obstacle도 정지할 수 있다.
관측 누락·무효 속도를 유효0으로 바꾸거나 입력 무효를 정상 빈 장애물로 처리하지 않는다.
센서 단절·실제 통로 폐쇄·동적 충돌 위험에서는 정지할 수 있다.

camera 선택의 `jamming_creep_clear_distance=0`은 creep 중 추가 정적 정지 검사를 끄는 기존 대회 정책이다.
사람/개 대기·입력 timeout·비상정지 등 별도 조건은 유지된다. LOCAL/localization과 동일한 sweep 수락을 뜻하지 않는다.
측정 벽 통로 확장은 localization의 별도 wall_permission과 물리 터널 범위에서만 사용하며,
페인트 횡단 허용과 주행가능영역 확장을 구분한다.

## 재현과 수락 기록

동일 시나리오·배치·센서·날씨·초기 상태에서 camera/local/localization을 비교한다.
경로/명령 공백, 정지시간과 사유, 경로 출처, 추종 오차·접촉 근거를 연결한다.
진단에 source/run/세대가 없는 과거 결과를 현재 실행 결과로 합산하지 않는다.
과거 실패와 절대 위치/속도 정확도 한계는 [기록 목록](../records/README.md)과 10월30일 개별 기록을 따른다.

## 재부팅 후 콘솔 점검

판제 대시보드는 autonomy의 `dashboard:=true` 기본값으로 실행한다.
같은 master/checkout의 대시보드를 별도로 유지할 때만 `dashboard:=false`를 사용한다.
포트가 다른 서버에 점유돼 있으면 먼저 소유 master·실행 경로를 확인한다.

LiDAR 웹은 별도이며 상세 점군이 필요하면 일반 LiDAR 실행 대신 아래 명령을 사용한다.

```bash
roslaunch asmc_lidar lidar.launch debug:=true
roslaunch asmc_lidar view.launch
```

`debug` 기본값은 false이며 판제 발행을 위해 켤 필요가 없다. 기본 LiDAR는3뷰 fusion을 사용하고
`lidar_wall_vehicle.yaml`을 읽는다. LiDAR 단일 시험은 `fusion:=false`다.
웹을 보려는 목적으로 인지나 제어를 두 번 실행하지 않는다.

## 공용 코드·파라미터와 반복 정지

정지 원인은 sensor/pose age, native 경로, LOCAL 후보, selector 인계와 MPC를 같은 시각으로 확인한다.
LiDAR 객체 미검출로 native가 경로를 정상으로 판단하는 경우는 실패 직후 인계 정책만으로 해결되지 않는다.
현재 실행 안내는 코드에서 생성한 [launch 인자 목록](launch-reference.md)과 함께 검토한다.

## 2026-10-30 팀 PR 통합

대회 최종 소스에는 camera·로터리·LiDAR 팀 패치가 함께 포함된다.
현재 소스 통합은 [PR #141](https://github.com/INHA-AIM/2026-ASMC/pull/141)의 대회 종료 병합을 따른다.
과거 팀별 검증·실패 SHA는 [정리 전 원문](../records/planning/2026-10-30-jamming-comparison-before-closeout.md)과
[종료 보존 기록](../records/repository/2026-11-10-postcompetition-preservation.md)에 연결한다.
