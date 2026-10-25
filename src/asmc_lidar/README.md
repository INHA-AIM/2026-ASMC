# asmc_lidar

ROI → voxel → roll/pitch 수평화 → 지면 분리 → 군집 → OBB → 추적 → 전방 카메라 fusion → local/map 객체를 처리한다. 현재는 **이관 검증 경로**이며 판제는 기존 `lidar+sensorfusion` Main을 사용한다.

안승현은 **HD맵 기반 필터·카메라 연결 지원**, 손재호는 **추적·속도·운동 상태·fusion 보완**을 진행한다. 좌표·시간·공용 계약과 판제 전환은 공동 확인한다.

- [판제 전환 기반·구 기능 대조·다음 작업](docs/cutover-foundation.md) — PR #108 병합 기준. 속도/운동 정보 연결과 직접 소비·구 패키지 폐기를 먼저 준비하며, GPS 없는 터널 local 개선은 손재호가 담당한다.
- [재호의 PR #104 인수 시작](../../docs/handoffs/lidar-migration.md#재호의-pr-104-인수-시작) · [담당별 다음 작업](docs/migration-tasks.md#공동-개발-인수와-우선-작업)
- [구조·관측 계약과 이번 검증](docs/structure-contract-refactor.md)
- [이름·경로 이전표](docs/name-mapping.md): 옛 이름은 참고용이며 wrapper를 제공하지 않는다.
- [실행 경로·PR #103 기준선과 제한](docs/pipeline.md)
- [웹 모니터와 단계별 진단](docs/web-monitor.md)
- [카메라 수신·버퍼·보정·투영 API](docs/camera-input.md)
- [전체 이관 작업표](docs/migration-tasks.md), [팀 인계](../../docs/handoffs/lidar-migration.md)
- [다음 PR의 판제 필터 편입표](docs/filter-migration.md) — 계산 품질과 행동 정책 분리, 실제 이전은 미착수
- [HD맵 전달 필터·검증](docs/hd-map-filter-analysis.md#a19-10cm-경계와-실제-발행-필터--2026-10-25) — 기본 filter: 안쪽10cm·점군3점/10%·OBB10%, 상부 객체 제외. 원본 tracker 보존

기본 `lidar.launch`의 `/migration/lidar/objects_local/map` 및 `observations_local/map`에는
HD맵·상부 객체 전달 필터가 적용된다. 동일 scan의 두 좌표계는 같은 ID를 제외한다.
지도/pose 무효에서는 지도 제외를 중단하고, 최근 YOLO vehicle/dynamic 매칭은 모든 구간에서 보호한다.
debug의 원본 tracker/점군은 검수를 위해 남긴다. 기존 Main·판제 입력은 전환하지 않았다.

## 구조

`include/asmc_lidar/`와 `src/`는 같은 책임으로 나눈다. 단계마다 ROS 노드를 만들지 않는다.

| 디렉토리 | 책임 |
|---|---|
| `core/` | 공통 입력·시간·좌표 타입·처리 세대 |
| `geometry/` | 수평화 변환·pose 이력·보정·투영·scan 좌표 |
| `preprocessing/` | ROI·자차 제거·voxel·지면 분리 |
| `detection/` | 군집·검출 관측·OBB |
| `tracking/` | 연결·Kalman·track 상태 |
| `fusion/` | 카메라 관측 버퍼·분류 근거·전방 fusion |
| `filtering/` | ROS 독립 HD맵·상부 전달 필터. 발행 배열만 제외하며 원본 tracker 보존 |
| `pipeline/` | 처리 단계 조합·scan context와 결과 |
| `ros/` | 센서 수신·설정·메시지 변환·진단·실행 |
| `experimental/` | 기본 OFF인 기존 TunnelFilter 원본과 연결 stage |

`DetectionObservation`이 점군과 현재 기하를 한 번 소유하고, `TrackState`와 `CameraClassEvidence`를 조합해 `TrackedObject`를 만든다. 기존 tracker 이력의 복사 정책은 유지한다. `filtering/`은 원본 객체와 분리된 작은 판정 결과만 소유한다.

계산 라이브러리는 ROS 메시지·시간 타입을 사용하지 않는다. 외부 catkin export는 `asmc_lidar_frame_time`, `asmc_lidar_camera_core`, `asmc_lidar_camera_ros` 세 개다. 검출·추적/runtime·HD맵 audit 정적 라이브러리는 패키지 내부에 두어 Main 링크를 바꾸지 않는다.

## 빌드·시험

Noetic 컨테이너의 저장소 루트에서 실행한다. MORAI 메시지를 포함한 의존성이 준비되어 있어야 한다. `PIPELINE_REQUIRED=ON`은 필요한 구현·시험이 빠지면 configure를 실패시킨다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
catkin_make -j2 -l2 -DASMC_LIDAR_PIPELINE_REQUIRED=ON
source devel/setup.bash
catkin_make camera_receiver_ros_test -j2 -l2
(cd build/asmc_lidar && ctest --output-on-failure -T Test --no-compress-output)
python3 src/asmc_lidar/test/pipeline_ci_check.py --devel devel \
  --build build/asmc_lidar --results build/test_results/asmc_lidar
```

별도 build/devel을 지정한다면 **모든** `catkin_make` 호출에 `--build`와 `-DCATKIN_DEVEL_PREFIX=...`을 함께 넣는다. pytest/일반 Python 검사는 계산·ROS 회귀를 대체하지 않는다.

## 실행

기존 bridge와 카메라는 별도로 실행한다. `lidar.launch`는 처리 노드 하나를 시작한다.
`rviz:=true`를 주면 같은 namespace의 관측을 표시하는 Marker 도구와 RViz도 함께 시작한다(기본 OFF).
기본 `fusion:=true`는 카메라 관측을 분류에 사용하며, 카메라 추론을 시작하지 않는다.
`fusion:=false`는 카메라 보정 조회·구독 없이 실행하므로 카메라 패키지가 없는 환경에서도 사용할 수 있다.
일반 실행은 `fusion`, `rviz`, `namespace`, `pipeline_config`를 사용하고 나머지 인자는 내부 대조·배선 조정용이다.
상세 알고리즘 값은 YAML로 관리한다.

재호의 PR #105 웹 모니터를 `view.launch`로 분리했다. 시각화는 결과만 구독하며
처리 노드·드라이버·카메라·판제를 중복 시작하지 않는다. 단계별 점군이 필요할 때는
`lidar.launch debug:=true`로 처리 노드를 시작하고 별도 터미널에서
`roslaunch asmc_lidar view.launch` 후 `http://127.0.0.1:8091/`에 접속한다.
현재 RViz에서는 아래 읽기 전용 Marker 도구로 처리 결과를 확인한다.

```bash
source /root/ws/devel/setup.bash
roslaunch asmc_lidar lidar.launch
# 카메라 없이 LiDAR만 비교하려면 위 명령 대신 사용
roslaunch asmc_lidar lidar.launch fusion:=false
```

출력 기본값은 `/migration/lidar/{objects_local,objects_map,observations_local,observations_map,diagnostics,trace}`다. 입력은 절대 경로 정본을 유지하고 출력 상대 이름은 `namespace` 인자로 결정한다. 운영 출력 차단과 TF 기본 OFF를 유지한다. 기존 Main과 동일 운영 토픽에 동시에 발행하지 않는다.

관측의 `objects.header`가 source stamp·frame의 정본이다. 정상 빈 결과·입력 무효·map pose 무효는 기존 `LidarObjectArray` 상태로 구분한다. 센서가 조용하다고 과거 결과를 새 stamp로 재발행하지 않는다. 현재 속도·운동 validity는 모두 false다. 내부 Kalman·MotionClassifier와 웹 진단이 있어도 판제용 보정 속도가 공용 출력에 연결된 것은 아니다. 구 `/last_lidar`와 메시지가 달라 단순 remap으로 판제 전환할 수 없다([기능 차이](docs/cutover-foundation.md#구-main과-새-패키지의-기능-대조)).

명시적 처리 초기화:

```bash
rosservice call /migration/lidar/lidar_pipeline/reset "{}"
```

이 서비스는 처리 세대를 올리고 대기 scan·추적·pose·trace 이력을 비운다. 입력 공백·GPS 무효·카메라 모드 변경은 이 서비스를 자동 호출하지 않는다. 판제 입력 전환이나 운영 수락 절차가 아니다.

HD맵 판정의 기본값은 YAML `hd_map_audit.mode: "filter"`다. Git에 포함한 작은 gzip bundle을
한 번 로드하고 현재 HD맵 제품 SHA까지 검사한다. 개인 D 경로는 필요 없다. `audit`는 판정만,
`off`는 계산 중단이다. 기본 `filter`는 실제 발행 객체에 제외를 적용하며 기존 Main·판제 입력은 유지한다.
기본 높이 ROI는 IMU 수평화 후 **평지 지면 위2.5m**를 넘는 점을 군집 전에 제외한다.
ROI 기준과 복구·표시 규칙은 [현재 설정](docs/hd-map-filter-analysis.md#a14-기본-hd맵높이-roi와-제외-근거--2026-10-25)을 따른다.

전달 필터는 OBB 겹침과 **전체 cluster support의 도로 점 비율**을 함께 사용한다.
카메라로 연결된 `dynamic_obstacle`·`vehicle`은 모든 구간에서 보호하고, 머리 위 구조물은 최저 점·OBB 하단과
지붕 여유로 실제 발행 제외를 판단한다. [웹 디버거](docs/web-monitor.md#hd맵-필터-디버깅)의 후보 숨김은 화면에만 적용된다.
보호는 최신 실제 YOLO 양성 매칭의 person/dog/deer/bicycle/stroller와 car/bus/truck에 한한다. LiDAR 동적 속도 판정이나
미션 주변10m라는 이유로 unknown을 보호하지 않는다. 현재 `yolo_points_and_obb_subcell_v5` 정책과 원본 YOLO
매칭 분류를 상세 화면에 표시한다. 설정을 바꿨다면 처리 노드를 재시작해야 반영된다.
점군 유지는 도로 면 합집합의 바깥 경계를 연속적으로10cm 줄인 내부에서 **3점 이상 AND 전체10% 이상**이다.
OBB도 같은10cm 안쪽 도로 영역과의 겹침이 **면적10% 이상**이어야 한다. 전달은 **점군 AND OBB**이며
OBB 통과가 점군 미달을 보완하지 않는다. 유효 YOLO 보호와 지도/위치 보류는 별도로 유지한다.
원본 도로 점/OBB 겹침은 비교용으로 함께 표시한다. 측정 OBB·tracker를 바꾸지 않고 발행 배열만 압축한다.
배경 지도는 `asmc_hd_map` v7 정본이며, 판제의 별도 v5 사본과 혼용하지 않는다.
웹은 도로 면과 SHA가 일치하는 물리 차선만 그린다. 링크 중심선·도로 분할 테두리는 표시하지 않는다.

고정 bag 대조는 `lidar_replay --help`와 [실행 안내](docs/pipeline.md)를 따른다. 결과는 저장 규약에 따라 등록한 새 run ID의 D 출력 위치에 보존한다.

## 처리 결과를 RViz에서 보기

새로 처리와 RViz를 함께 시작하려면 다음을 사용한다. 객체/ego 박스를 즉시 표시하며 `debug:=true`는 필요 없다.

```bash
roslaunch asmc_lidar lidar.launch rviz:=true
# 다른 namespace나 터널 필터를 지정해도 RViz가 해당 결과를 자동으로 구독한다.
roslaunch asmc_lidar lidar.launch namespace:=/migration/lidar_tunnel experimental_tunnel_filter:=true rviz:=true
```

두 명령은 대안이며 하나만 실행한다. 이미 같은 처리가 켜져 있으면 두 번째 처리를 시작하지 않는다.
이미 열린 RViz를 사용하려면 `rviz:=false`를 유지하고 아래 수동 연결을 사용한다.
`view.launch`는 별도의 웹 화면이며 이 옵션으로 시작하지 않는다.

bridge와 `lidar.launch`를 유지하고 별도 터미널에서 표시 변환기만 실행한다.

```bash
source /root/ws/devel/setup.bash
rosrun asmc_lidar lidar_markers.py
```

이미 열린 RViz에서는 `File → Open Config`로 `/root/ws/src/asmc_lidar/rviz/lidar.rviz`를 연다.
직접 설정하려면 Fixed Frame=`base_link`, Add=`MarkerArray`, Topic=`/migration/lidar/markers`다.
새 RViz가 필요한 경우에만 `rviz -d /root/ws/src/asmc_lidar/rviz/lidar.rviz`를 실행한다.

- 입력 `/migration/lidar/observations_local`의 실제 8코너를 그린다. yaw로 박스를 재구성하거나 IMU/장착 변환을 다시 적용하지 않는다.
- 라벨은 `ID 소비분류 / 카메라 원본분류 n=voxel 이후 군집 점 수`다. vehicle은 청록, dynamic은 주황, static은 초록, unknown은 노랑이다.
- 속도 validity가 아직 false이므로 운동·속도 화살표를 만들지 않는다. `unknown`은 카메라 분류가 확인되지 않은 상태다.
- 정상 빈 결과·무효·세션/세대 변경에서 이전 박스를 지우고, 입력350ms 중단 시 wall clock으로 제거한다. 중복·이전 세대 관측으로 표시를 갱신하지 않는다.
- 이 도구는 검출·추적·fusion을 계산하지 않고 Marker만 발행한다. 지면 제거/군집 점군을 새로 계산하지 않는다.
- 프리셋의 Raw LiDAR는 기본 OFF다. 이를 켜면 원본 `/velodyne_points`와 최종 처리된 박스를 대조한다. 중간 단계 점군은 이 Marker 도구가 발행하지 않는다. PR #105의 단계별 샘플은 `debug:=true`와 [웹 모니터](docs/web-monitor.md)로 따로 확인한다.
- 같은 Marker 토픽에 ego를 파란 반투명3D 박스·흰12개 모서리·EGO 라벨로 표시한다. 차체 footprint는4.635×1.892m, 높이는 실제 형상에서 확인한1.644m다. 후륜축 기준 x=-0.79~3.845m, 명목 지면 z=-0.36m·상단 z=1.284m다. 입력이 끊겨도 ego 기준선만 유지하며 검출 객체 배열에는 추가하지 않는다.

**2026-10-23 높이 정정:** UDP20개와 `GetVehicleSpec.height`는2.434m지만, `GetActorState(Ego)`
5회의 `bounding_box.extent.z`는 모두0.8220231533m였다. 실제 형상의 높이는 half-extent의2배인
1.6440463066m이므로 표시에는1.644m를 사용한다. 앞서2.434m 메타데이터를 형상 높이로 사용한 것은
잘못이었다. XY는 기존 차체 footprint를 유지하므로 전체 mesh bounding box를 그대로 복제한 것은 아니다.
명목 지면과 높이 snapshot은 표시용이며 실시간 자세/상하 운동 추정이나 인지 GT 입력이 아니다.

기본 출력 namespace를 바꿨다면 `rosrun asmc_lidar lidar_markers.py _input:=/다른경로/observations_local _output:=/다른경로/markers`로 맞춘다.

## 2026-10-23 음영구간 직접 확인

터널 필터 ON 경로에서 RViz의 객체/ego 표시를 확인했으나, 사용자 관찰상 박스가 많이 튀었다.
표시 연결 성공은 검출·OBB·추적 안정성 수락이 아니다. TunnelFilter는 기본 OFF를 유지한다.
같은 점군의 OFF/ON, 구조물 잔존·군집/OBB 변화·ID 전환을 다음 단위에서 분리 검토한다.
카메라 미실행 시 분류는 unknown일 수 있다. 지도 표시/화면 숨김과 실제 전달 필터는 구분한다.
GPS 없는 터널 벽·천장 억제는 미수락이며, 카메라/family를 보조하더라도 바깥 흰선 밖 점군을
일괄 삭제하지 않는다. [손재호의 터널 작업·검증 기준](docs/cutover-foundation.md#손재호-gps-없는-터널-local-인식)을 따른다.

ROS master를 소유한 launch를 종료하면 남은 bridge/RViz가 새 master에 자동 등록되지 않을 수 있다.
반복 대조는 먼저 독립 `roscore`를 실행하고 같은 master를 유지한다. 이미 켜진 RViz를 중복 실행하지 않는다.
