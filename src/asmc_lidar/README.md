# asmc_lidar

ROI → voxel → roll/pitch 수평화 → 지면 분리 → 군집 → OBB → 추적 → 전방 카메라 fusion → local/map 객체를 처리한다. 현재는 **이관 검증 경로**이며 판제는 기존 `lidar+sensorfusion` Main을 사용한다.

안승현은 **HD맵 기반 필터·카메라 연결 지원**, 손재호는 **추적·속도·운동 상태·fusion 보완**을 진행한다. 좌표·시간·공용 계약과 판제 전환은 공동 확인한다.

- [구조·관측 계약과 이번 검증](docs/structure-contract-refactor.md)
- [이름·경로 이전표](docs/name-mapping.md): 옛 이름은 참고용이며 wrapper를 제공하지 않는다.
- [실행 경로·PR #103 기준선과 제한](docs/pipeline.md)
- [카메라 수신·버퍼·보정·투영 API](docs/camera-input.md)
- [전체 이관 작업표](docs/migration-tasks.md), [팀 인계](../../docs/handoffs/lidar-migration.md)

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
| `pipeline/` | 처리 단계 조합·scan context와 결과 |
| `ros/` | 센서 수신·설정·메시지 변환·진단·실행 |
| `experimental/` | 기본 OFF인 기존 TunnelFilter 원본과 연결 stage |

`DetectionObservation`이 점군과 현재 기하를 한 번 소유하고, `TrackState`와 `CameraClassEvidence`를 조합해 `TrackedObject`를 만든다. 기존 tracker 이력의 복사 정책은 유지한다. `filtering/`은 HD맵 필터를 구현할 때 생성한다.

계산 라이브러리는 ROS 메시지·시간 타입을 사용하지 않는다. 외부 catkin export는 `asmc_lidar_frame_time`, `asmc_lidar_camera_core`, `asmc_lidar_camera_ros` 세 개다. 검출·추적/runtime 정적 라이브러리는 패키지 내부에 두어 Main 링크를 바꾸지 않는다.

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
기본 `fusion:=true`는 카메라 관측을 분류에 사용하며, 카메라 추론을 시작하지 않는다.
`fusion:=false`는 카메라 보정 조회·구독 없이 실행하므로 카메라 패키지가 없는 환경에서도 사용할 수 있다.
일반 실행은 `fusion`, `namespace`, `pipeline_config`를 사용하고 나머지 인자는 내부 대조·배선 조정용이다.
상세 알고리즘 값은 YAML로 관리한다.

재호의 localhost 시각화가 올라오면 `view.launch`로 별도 실행한다. 시각화는 결과를 구독하며
처리 노드·드라이버·카메라·판제를 중복 시작하지 않는다. 현재 이 패키지에 웹 시각화 실행기는 없다.
현재 RViz에서는 아래 읽기 전용 Marker 도구로 처리 결과를 확인한다.

```bash
source /root/ws/devel/setup.bash
roslaunch asmc_lidar lidar.launch
# 카메라 없이 LiDAR만 비교하려면 위 명령 대신 사용
roslaunch asmc_lidar lidar.launch fusion:=false
```

출력 기본값은 `/migration/lidar/{objects_local,objects_map,observations_local,observations_map,diagnostics,trace}`다. 입력은 절대 경로 정본을 유지하고 출력 상대 이름은 `namespace` 인자로 결정한다. 운영 출력 차단과 TF 기본 OFF를 유지한다. 기존 Main과 동일 운영 토픽에 동시에 발행하지 않는다.

관측의 `objects.header`가 source stamp·frame의 정본이다. 정상 빈 결과·입력 무효·map pose 무효는 기존 `LidarObjectArray` 상태로 구분한다. 센서가 조용하다고 과거 결과를 새 stamp로 재발행하지 않는다. 현재 속도·운동 validity는 모두 false다.

명시적 처리 초기화:

```bash
rosservice call /migration/lidar/lidar_pipeline/reset "{}"
```

이 서비스는 처리 세대를 올리고 대기 scan·추적·pose·trace 이력을 비운다. 입력 공백·GPS 무효·카메라 모드 변경은 이 서비스를 자동 호출하지 않는다. 판제 입력 전환이나 운영 수락 절차가 아니다.

고정 bag 대조는 `lidar_replay --help`와 [실행 안내](docs/pipeline.md)를 따른다. 결과는 저장 규약에 따라 등록한 새 run ID의 D 출력 위치에 보존한다.

## 처리 결과를 RViz에서 보기

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
- 프리셋의 Raw LiDAR는 기본 OFF다. 이를 켜면 원본 `/velodyne_points`와 최종 처리된 박스를 대조한다. 중간 단계 점군은 현재 공개되지 않는다.

기본 출력 namespace를 바꿨다면 `rosrun asmc_lidar lidar_markers.py _input:=/다른경로/observations_local _output:=/다른경로/markers`로 맞춘다.
