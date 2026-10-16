# HD맵 → 판제: 전역 지도·미션 경계·GPS 상태 인수

> **문서 역할:** Active Handoff — PR #58의 HD 패키지 결과와 판제 반영 항목
> **검토 대상 main:** `8e6516d` (2026-10-06). 판제 source는 이 작업에서 수정하지 않았다.
> **상태:** HD producer 검증; 판제 연결·full-stack/실차 인수는 아래 항목을 확인한 뒤 판제에서 수행한다.
> **정본:** [기존 topic 계약](../contracts/hd-map-topic-v1.md),
> [확장 상태 계약](../../src/asmc_hd_map/config/hd_map_context_contract_v1.yaml)

## 1. HD 패키지가 제공하는 결과

운영 `roslaunch asmc_hd_map competition_map.launch rviz:=false`와 debug launch 모두 크롭을 기본으로 끈다.
RViz 프리셋은 전역 주행가능영역 외곽선을 기본으로 표시하고 `near ego` 크롭 display는 제거했다.
차선·정지선·차로·주행가능영역·미션·체크포인트·공식 경로는 `/hd_map/global/*`에서
latched 배열로 한 번 받고 필요한 레이어만 캐시한다. `/hd_map/context`와
`/hd_map/local/current_lane`은 크롭 없이 GPS 기반으로 계속 제공한다.
GPS가 끊기면 현재값을 무효화하고 정적 전역 정보와 명시적인 역사값만 유지한다.

| 입력 | 제공 내용 | 판제 사용 |
|---|---|---|
| `global/lanes` | 70개 차로 polygon/centerline, link ID·선후행·인접 관계·차로 번호 | 현재 후보 또는 전체 경로와 교차하는 차로 선택 |
| `global/drivable_areas` | 102개 영역, 교차로·support lane 포함 | 모든 후보 경로와 차량 몸체의 주행가능 여부 검사 |
| `global/drivable_grid` | 3085×7232, 0.2 m, `0=free`, `100=nonfree` | 필요 시 한번 받아 빠른 cell 조회; raster는 vector 파생물 |
| `global/lane_markings` | 물리 차선 121 + 정지선 7; 색·실선/점선·역할 | 도색 정책과 정지선 연결, 신호 색상은 별도 인지에서 수신 |
| `global/mission_zones` | 미션/정책 15개, effective speed 39개 link | ID별 payload 캐시; 구간·속도·인지 활성 조건 선택 |
| `global/checkpoints` | 15개 공식 pose, 3 m 반경, yaw, station | 체크포인트 통과와 경계 연결 |
| `global/route` | 공식 path 4430점 + XY 누적거리, route SHA·origin | HD station 정본, 경계 좌표 재투영 기준 |
| `context` | 후륜축 pose, lane/route 유효 여부, 현재 구간·속도·미션 경계·GPS 상태 | 경량 동적 상태와 GPS fallback 판단 |

`HdMapRoute`, `MissionBoundary`, `HdMapContext`는 **asmc_hd_map 패키지**의 새 타입이다.
기존 `asmc_msgs` 타입과 MD5는 바꾸지 않았다. C++ 소비자는 `asmc_hd_map` catkin 의존성과
`<asmc_hd_map/HdMapContext.h>`, `<asmc_hd_map/HdMapRoute.h>`를 추가한다.
`header.frame_id=map`, 좌표·거리는 m, context 속도는 m/s다. 기존 mission payload 속도는 km/h다.

`context.header.stamp`는 상태 발행 시각이고 `pose_stamp`는 마지막 수용 위치 관측 시각이다.
관측 callback에서 즉시 발행하고 최대 20 Hz health timer로 유효 상태를 갱신한다.
판제는 수신 중단도 감시하고 `pose_valid`, `lane_valid`, `route_valid`를 각각 검사한다.
공식 경로와 15 m 이상 떨어지면 pose는 유효해도 `route_valid=false`일 수 있다.
`route_speed_limit_mps`는 공식 경로 투영 위치의 상한이다. 임의 support link의 속도를 나타내지 않는다.

Python 소비 예제는 `hd_map_consumer_example.py`의 `~layers`를 선택한다.
`SemanticMapIndex`는 JSON payload를 한 번 파싱한 뒤 `missions_at_station`,
`speed_at_station`, `mission_boundaries`, `signals_ahead`, `spatial`을 조회한다.
C++도 정적 배열과 ID별 payload를 받아 같은 방식으로 캐시할 수 있다.
context의 `active_mission_ids`와 경계 ID를 전역 미션 payload에 연결한다.

### 신호등 인지 활성 구간

source의 `traffic_light_activation_A236_to_A308`은 **A236 시작 → A308 종료**다.
전체 ID는 `A2256W000236` → `A2256W000308`, 공식 station은
`[160.009167271, 609.876838441)` m다. 이 구간에서만 인지를 활성화한다는
`enable_within_zone_only` / `disable_outside_zone=true` 정책을 metadata에 정의했다.
HD가 camera 노드를 직접 켜고 끄지는 않는다. 판제가 유효한 context의
`active_mission_ids`에 이 ID가 포함되는지 또는 전역 zone의 station 범위를 조회해 연결한다.

이 활성 zone과 실제 신호 정지선 경계 5개를 구분한다. `mission_type=traffic_light`만
검사하면 둘 다 포함되므로 payload의 `perception_monitoring=activation_zone`과
`boundary_only=true`를 구분한다.

| 실제 신호 경계 소유 link | 신호등 ID | 공식 station (m) |
|---|---|---|
| A219 | C1256W000077 | 208.458941227 |
| A213 | C1256W000091 | 315.906098142 |
| A728 | C1256W000016 | 436.797566442 |
| A315 | C1256W000123 | 550.618735375 |
| A308 | C1256W000052 | 609.868390593 |

이 표의 5개 경계는 공식 채점 교차로 4개소와 같은 단위가 아니다. v1.2 후속 공지의
30초·서로 다른 4개소 모두 미준수 시 실패 규칙은 payload에 반영됐지만, 공식 교차로 대응은
`unverified`이고 `judged_intersection_id=null`, `intersection_failure_evaluation_available=false`다.
위 신호등 ID를 그대로 위반 집계의 교차로 ID로 쓰지 않는다. 상세 필드와 전 구간 `route_rules`는
[대회 metadata 계약](../contracts/competition-kcity2025-metadata-v1.md)을 따른다.

## 2. 교차로 패치와 현재 main 지도 갱신

HD 제품은 `kcity2025_static_junction_widened_drivable_v6`다.
main Frenet의 `config/hd_map/drivable.json`은 이전
`kcity2025_roundabout_extended_drivable_v5` 사본이다. A226(`A2256W000226`)과
A111(`A2256W000111`)의 교차로 주행 여유 및 대응 surface patch가 다르다.
개수만 비교하면 둘 다 Lane 70·Area 102라서 차이를 놓친다.

현재 geometry SHA는
`01ec671fe402baf339f8f6bcfb83a97213d560f7311961c5f072a47d654bcfbf`다.
`DrivableArea.intersection_guarded=true`의 교차로·합류 road mesh를 차선 안쪽으로 다시
잘라내지 않는다. support link를 공식 전역경로의 순서에 무조건 추가하거나, 모든 미션을
현재 support link ID 하나로만 활성화하지 않는다. 실제 차로 ID와 공식 route station을 구분한다.

판제 file reader에 맞는 JSON/RLE를 HD 패키지에서 함께 내보낸다. 출력 폴더는 비어 있어야 한다.

```bash
python3 src/asmc_hd_map/scripts/export_planning_bundle.py \
  --data-root /data --output-dir /tmp/hd-map-v6-planning
```

| 출력 | 반영 |
|---|---|
| `drivable.json`, `lines.json`, `drivable_grid.rle` | Frenet `config/hd_map/`의 세 파일을 **같이** 갱신 |
| `competition_metadata.yaml` | 미션·속도·신호 정지선 payload를 file로 소비할 때 사용 |
| `route.json` | 공식 station namespace와 points/stations |
| `manifest.json` | build/product/geometry 식별자와 각 파일 SHA |

export는 runtime pins, product self-hash, vector geometry SHA, NPZ SHA를 검증한다.
RLE는 0인 free cell만 `[begin,end)` 구간으로 기록하며 unknown/nonfree를 free로 바꾸지 않는다.
main `AvoidRoadMap::Load`가 사용하는 width/height/resolution/origin과 형식에 맞춘다.
영역 JSON만 바꾸면 Frenet의 실제 `Inside()` 조회가 이전 RLE를 계속 사용하므로 반드시 함께 바꾼다.

## 3. 누적거리와 후륜축 기준: 소유권

HD 공식 station은 공식 global path 첫 점부터의 **XY arc length**다.
전체 길이는 약 2184.611723336 m다. 고도·경로 smoothing·차로 이동으로 재산정하지 않는다.
미션 범위는 `[start,end)`, 길이 0인 신호 경계는 `POINT`다.
context 경계 조회는 닫힌 route의 끝→시작 seam을 넘는 거리도 계산한다.
직접 query할 때는 `route_length_m`을 넘기면 같은 루프 거리로 조회한다.
context 경계에는 ENTER/EXIT/POINT, 공식 station, signed route distance와 map 좌표를 같이 제공한다.
경계 좌표는 공식 path의 해당 station을 보간한 점이며 실제 정지선 polygon을 대체하지 않는다.

현재 Behavior RouteManager는 원래 MGeo link centerline을 연결해 station을 만들고,
Frenet은 reference를 smoothing/resampling한 뒤 길이를 다시 계산한다.
그 `s`에 HD 숫자를 바로 대입하지 않는다. **HD가 공식 station/경계 좌표를 완성해 제공하고,
판제가 경계 좌표를 자기 reference에 재투영하는 것이 적절하다.**
분기·루프 중첩에서는 link/route 순서와 heading으로 투영 대상을 제한한다.
HD station을 판제 station으로 이름만 바꿔 저장하거나 체크포인트 clear_s와 혼용하지 않는다.

HD GPS+IMU 입력은 다음 기준으로 후륜축 중심을 계산한다.

```text
map_antenna_xy = UTM52(GPS) - (302595, 4124145)
map_rear_axle_xy = map_antenna_xy - R(yaw) * (0.35, 0.0)
```

HD geometry와 체크포인트를 이동시키지 않는다. GPS/IMU pair 간격은 최대 100 ms,
GPS offset과 map origin 오차는 최대 1 m다. `pose_stamped` 입력은 이미 map 후륜축 좌표여야
하며 HD에서 lever arm을 다시 빼지 않는다.

main `asmc_bridge/src/vehicle_state/vehicle_state_adapter.cpp`의 `/vehicle/state` 위치는
UTM에서 origin만 빼고 이 lever arm 보정은 적용하지 않는다.
**판제/공용 localization 쪽에서 `/vehicle/state`도 동일한 후륜축 기준으로 맞춘다.**
공용 어댑터에서 한 번 고치는 것이 여러 판제 consumer에서 따로 보정하는 것보다 명확하다.
어댑터를 고칠 때 이미 보정한 pose에 0.35 m를 두 번 빼지 않는지 확인한다.
HD context의 pose와 `/vehicle/state`로 만든 후보 경로가 다른 기준점이면 몸체/cell 검사에 오차가 생긴다.

## 4. GPS 음영과 복귀

| 상태 | HD 출력 | 판제 반영 |
|---|---|---|
| 정상 | `pose_valid=true`; lane/route 독립 valid; 활성 구간과 경계 | 필요한 정보 선택 소비 |
| GPS status≤0, 좌표 sentinel | 즉시 `POSE_GPS_UNAVAILABLE`, 현재 lane/route/speed/미션/경계 무효·비움 | camera lane/drivable 입력으로 전환 |
| GPS 미수신/만료 | 0.5 s 후 `POSE_STALE`, 같은 무효 처리 | source stamp·수신 중단 감시, camera fallback |
| IMU pair/origin 부적합 | `POSE_INVALID` | 위치 기반 HD 현재값 거부 |
| GPS 복귀 | 새 유효 관측, 차로 hysteresis와 route hint 초기화, 재조회 | 유효 상태 확인 후 HD 위치 기반 소비 재개 |

음영에서 HD는 위치·차로·station을 외삽하거나 정확한 ego를 추정하지 않는다.
`last_valid_*`와 `gps_shadow_zone_ids`는 마지막 유효 관측에서 확인한 정보다.
음영 때 `gps_shadow_context_from_last_valid_pose=true`로 구분한다.
GPS 손실 자체로 특정 지도 음영구간 안에 있다고 확정하지 않는다. 시작부터 GPS가 없으면
`last_valid_context_available=false`, 음영구간 ID도 빈 배열이다.
정적 전역 미션에는 음영구간의 규칙·범위·관련 link가 계속 있으므로 판제가 ID로 조회한다.
legacy `local/current_lane`도 `valid=false/STATE_STALE`가 된다. 남아 있는 ID/좌표는 마지막 값이다.

camera 소비와 전환 정책, 음영 중 Behavior/Frenet 진행 정책, 재진입 시 연결은 판제가 소유한다.
현재 `/vehicle/state`도 GPS x/y를 반복 발행할 수 있으므로 header만 새로 온다고 위치가
복구됐다고 판단하지 않는다. HD context의 pose/source valid와 freshness를 함께 확인한다.

## 5. 조회 거리와 크롭

크롭은 운영에 필수 입력이 아니다. 원본 정보를 제한하지 않으며, 차선·영역의 근거리
조회만 필요하면 전역 vector cache에서 작은 ROI를 선택한다.
90 m 전방·30 m 후방·좌우 30 m 직사각형은 `publish_local:=true`를 켰을 때의 노면/디버그 창이다.
ego를 표시하면 하단에서 25%에 위치한다. 회전 직사각형은 이 목적에서 전방 범위를 더 확보하면서
120×120 m 정사각형보다 cell 수를 절반으로 줄인다. 이것을 모든 판제 정보의 한계로 쓰지 않는다.

검토한 main은 Behavior 신호/정지선 조회 200 m, following 60 m, AVOID 120 m이며,
Frenet 곡률/감속 preview는 후보 5초 외에 곡률 preview·제동·반응 거리가 더해진다.
현재 normal target 14.5 m/s에서는 설정으로 계산한 전방 거리 약 174 m이고,
같은 설정 가정 60 km/h는 약 208 m, 80 km/h는 약 310 m다. 실차 제동 검증값은 아니다.
따라서 context의 **미션 경계 기본 전방 200 m**는 현재 신호 조회를 연결하기 위한 시작값이고,
빠른 주행이나 고주로 추적은 판제가 필요한 horizon까지 전역 구간을 조회한다.
`~boundary_ahead_m`, `~boundary_behind_m`으로 context 창을 바꾸거나
`SemanticMapIndex.mission_boundaries(s, ahead_m=350, behind_m=100)`처럼 직접 조회한다.
원거리 차량 추적용 actor/link 정보는 전역 mission payload에서 가져온다.

현재 Frenet 회피 검사는 approach/shift/pass/return 전체 후보 경로와 차량 몸체를 확인한다.
이 제약을 ego 90/30 m cache로 축소하면 긴 후보 경로에서 지도 정보가 누락될 수 있다.
판제는 전체 후보 경로와 겹치는 vector/raster를 조회하거나 지금처럼 전역 file cache를 유지한다.

## 6. 판제 확인 항목과 재현

- export 세 파일을 함께 반영하고 v6 build/manifest SHA를 확인한다. A226/A111 진입·교차로·출구에서
  후보 경로와 차량 몸체가 보완한 영역을 사용할 수 있는지 검사한다. 교차로 전체 union을 다시 clip하지 않는다.
- 공식/판제 station을 구분해 미션 시작·종료/신호 경계 좌표를 재투영한다. 범위 경계, 경로 seam,
  병렬 차로·support link 진입에서 ID와 진행 방향이 맞는지 확인한다.
- `/vehicle/state`와 HD pose의 기준점을 후륜축으로 맞춘다. 같은 GPS+IMU에서 0.35 m 잔차가
  없어지는지, yaw 회전 시 보정 방향이 맞는지, 보정을 두 번 하지 않았는지 검사한다.
- raw MGeo max_speed만 사용 중인 Behavior에 HD effective cap을 연결한다. kph→m/s 변환,
  high-speed exception이 competition cap만 해제하고 MGeo 상한은 유지하는지 확인한다.
  route cap은 임의 support link의 raw 상한과 구분하며 적용 정책은 판제에서 결정한다.
- static 신호 ID/정지선과 camera의 현재 신호 색상을 결합한다. provisional zone은 mandatory 규칙에서 제외한다.
- GPS 무효 즉시/미수신 0.5 s/시작부터 음영/복귀/지연된 무효 packet을 확인한다.
  음영에는 camera geometry를 사용하고 HD 역사값을 현재 위치로 되살리지 않는다.
- 실제 Behavior→Frenet 후보 생성, 고주로 추적·속도 전환과 camera fallback을 포함한
  full-stack replay/closed-loop는 판제 연결 후 별도 확인한다. producer 검증만으로 배포 승인하지 않는다.

HD producer 확인 결과(2026-10-06, ROS Noetic 개발 컨테이너): 전역 topic 검증과
GPS 음영/복귀 synthetic 검증 통과. export한 세 파일은 검토 대상 main의 실제
`AvoidRoadMap::Load`에서 수용되었고, RLE free cell 전체 22,310,720칸이 원본 NPZ와 일치했다.
실제 센서 지연·판제 후보 생성/폐루프 성능을 측정한 결과는 아니다.

HD 쪽 회귀검사와 개발용 재현 명령:

```bash
source devel/setup.bash
ASMC_DATA=/data python3 -m unittest discover -s src/asmc_hd_map/test
# 반드시 별도 개발 ROS master에서 실행: synthetic GPS/IMU를 발행한다.
roslaunch asmc_hd_map competition_map.launch rviz:=false
rosrun asmc_hd_map verify_competition_map_topics.py
rosrun asmc_hd_map verify_hd_map_context.py _allow_synthetic_input:=true
```

`verify_hd_map_context.py`는 정상 후륜축 보정·공식 station·즉시 GPS 무효·역사 음영 정보·
복귀 재획득·지연 무효 packet 무시·미수신 timeout·기본 크롭 미발행을 확인한다.
map pins 또는 source SHA가 바뀌면 publisher는 시작을 거부하므로 번들 전체를 재생성한다.
롤백 시 같은 commit의 지도·metadata·NPZ/판제 RLE를 함께 복원한다.
