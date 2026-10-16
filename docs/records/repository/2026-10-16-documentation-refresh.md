# 문서 현행화·소스 주석 검토 — 2026-10-16

최신 main의 PR #89까지 반영한 문서/주석 정리입니다. #88의 구조 변경을 포함한 별도 작업이며
PR 병합·실시간 주행·수집·학습·운영 설정 변경은 실행하지 않습니다.
기준 main은 `32ba7e8f`, 구조 PR head는 `f35a1255`, 통합 후 본 작업 시작점은 `4eb061badd2a964cea83db11cb02cb4c87363179`입니다.

## 범위와 검토 수준

[기계 검토 목록](../../../config/docs/source_comment_review_20261016.json)은 팀 소유12개 패키지의
tracked source/config/launch1,081개, 주석/docstring15,548줄의 정적 검색 범위를 기록합니다.
MORAI pinned submodule·nested morai_msgs/Velodyne·외부 SDK/생성 코드는 제외합니다.
source/config의 주석을 검색하고 실행 진입점·단위/좌표/시각·상태/제어 권한·원본/그룹 경계를 수동 대조했습니다.
알고리즘의 모든 주석이 형식적으로 참이거나 모든 주행 동작이 검증됐다는 증명은 아닙니다.
불필요한 줄별 해설을 새로 달지 않고 오해를 일으키는 경계 설명을 보완했습니다.

| 패키지 | 검색한 파일 | 주석/docstring 줄 |
|---|---:|---:|
| Main | 106 | 2108 |
| Msg | 5 | 10 |
| asmc_bridge | 20 | 201 |
| asmc_camera | 293 | 1012 |
| asmc_controller | 55 | 728 |
| asmc_hd_map | 144 | 1915 |
| asmc_lidar | 14 | 49 |
| asmc_msgs | 36 | 492 |
| asmc_planner | 279 | 8236 |
| data_collection | 52 | 177 |
| lane_boundary_adapter | 30 | 401 |
| planning_inputs | 47 | 219 |

## 확인·수정한 설명

- `asmc_planner`는 현재 단일 패키지이며 Behavior/Execution/Frenet은 내부 직접 호출입니다.
  삭제된 ROS package명·옛 `/Object_topic` 실행 안내를 현재 통합 실행과 분리했습니다.
- 통합 판제 traffic_light 기본은 실제 launch의true입니다. 외부 카메라 신호등을 쓰는 권장 조합은false 명시이며
  이 값이 신호 규칙을 끄는 설정이 아님을 설명했습니다. camera STANDBY의 모드 준비도 빠뜨리지 않았습니다.
- `/vehicle/state`와 `/planning/vehicle/state` 추정기를 구분했습니다. MPC x0의 주석은
  고정 토픽명을 단정하지 않고 실제 실행이 선택한 state 입력을 설명합니다.
- #89의 true ENU→UTM yaw 보정·MPC 조향0.15초 지연·변화 제한·29step·조건부58km/h·watchdog을 반영했습니다.
  구조체의false 기본과 main.launch의true override도 구분했습니다.
- 현재 E8/TwinLite TensorRT·전방544×960/측방480×640·15/15/10Hz·confidence0.40/NMS0.50,
  독립 모델 프로세스·수요 기반 BEV와 시각화를 연결했습니다.
- 현재16종/3그룹과 데이터 감사v2의과거4그룹을 구분했습니다. selfcar 숫자/Fence 예외·PE_Firewall·
  CargoBox/WoodBox·traffic_drum·유모차 아기 제외·motorcycle static/bicycle·stroller dynamic을 명시했습니다.
- 카메라 원본 pixels/header·bridge timestamp 출처·빈/무효/stale·unknown geometry·실제 운동 상태를
  msg와 source 경계 주석에 설명했습니다. source 문자열은 실제 API 에셋 ID가 아닙니다.
- data_collection은 Capture trigger/자동 제어/raw bag/snapshot/segmentation cache 절차를 나눴습니다.
  Capture에는 GT streaming bridge가 필수라는 오해와 competition에서 GT socket을 항상 여는 설명을 고쳤습니다.
- `.github/CODEOWNERS`의 삭제 경로를 정리하고 기존 담당자의 실제 asmc_lidar/planning_inputs/scen_runner 경로를 연결했습니다.

### 주석만 수정한 소스

- [src/asmc_camera/scripts/camera_runtime_node.py](../../../src/asmc_camera/scripts/camera_runtime_node.py)
- [src/asmc_controller/src/Control.cpp](../../../src/asmc_controller/src/Control.cpp)
- [src/asmc_hd_map/launch/competition_map.launch](../../../src/asmc_hd_map/launch/competition_map.launch)
- [src/asmc_lidar/include/asmc_lidar/frame_time.hpp](../../../src/asmc_lidar/include/asmc_lidar/frame_time.hpp)
- [src/asmc_msgs/CMakeLists.txt](../../../src/asmc_msgs/CMakeLists.txt)
- [src/asmc_msgs/msg/CameraObjectBox.msg](../../../src/asmc_msgs/msg/CameraObjectBox.msg)
- [src/asmc_msgs/msg/CameraObjectBoxArray.msg](../../../src/asmc_msgs/msg/CameraObjectBoxArray.msg)
- [src/asmc_msgs/msg/DetectedObject.msg](../../../src/asmc_msgs/msg/DetectedObject.msg)
- [src/asmc_msgs/msg/VehicleState.msg](../../../src/asmc_msgs/msg/VehicleState.msg)
- [src/asmc_planner/launch/planner_v2.launch](../../../src/asmc_planner/launch/planner_v2.launch)
- [src/data_collection/scripts/sync_collector_node.py](../../../src/data_collection/scripts/sync_collector_node.py)
- [src/lidar+sensorfusion/src/Main/src/Parser/SensorPoseHistory.hpp](../../../src/lidar+sensorfusion/src/Main/src/Parser/SensorPoseHistory.hpp)
- [src/lidar+sensorfusion/src/Msg/msg/TrackedObject.msg](../../../src/lidar+sensorfusion/src/Msg/msg/TrackedObject.msg)
- [src/planning_inputs/include/planning_inputs/converter.hpp](../../../src/planning_inputs/include/planning_inputs/converter.hpp)

시작점과 비교해 Python AST(docstring 제외), C++ token(comment 제외), XML 구조,
msg/CMake의 비주석 내용이 동일했습니다. 모델/학습 데이터·loss·threshold·launch 인자 값은 바꾸지 않았습니다.
새 실행 문서 검사는 별도 오프라인 도구와 CI step입니다.

## 실행 문서·과거 근거 통합

[명령 안내](../../operations/commands.md)는 수동 개별/통합 구성을 시나리오 통합 구성과 분리하고
제어 생산자·camera/Main source순서·PREPARE/ACTIVE·별도rqt·Capture/Bag 권한을 표시합니다.
[launch 인자 목록](../../operations/launch-reference.md)은 XML23개에서 자동 생성하며
기존 인자 default 변화·삭제된 target·명령 인자 오타를 검사합니다.

아래 긴 문서는 실험값/명령을 기록으로 옮기고 안정된 원래 현행 경로에는 짧은 안내를 남겼습니다.
이는 옛 경로 redirect stub을 다시 만드는 작업이 아닙니다. 원래 경로가 현재 내용의 정본입니다.
기록의 상대 링크만 새 위치로 고치고 코드 블록의 과거 명령은 변경하지 않았습니다.

| 현행 안내/이전 혼합 문서 | 보존한 근거 |
|---|---|
| [docs/operations/data-preparation.md](../../../docs/operations/data-preparation.md) | [docs/records/camera/data-preparation-through-v7.md](../../../docs/records/camera/data-preparation-through-v7.md) |
| [docs/architecture/consolidation-roadmap.md](../../../docs/architecture/consolidation-roadmap.md) | [docs/records/repository/consolidation-roadmap-through-pr87.md](../../../docs/records/repository/consolidation-roadmap-through-pr87.md) |
| [docs/architecture/perception-roadmap.md](../../../docs/architecture/perception-roadmap.md) | [docs/records/camera/perception-roadmap-through-pr85.md](../../../docs/records/camera/perception-roadmap-through-pr85.md) |
| [docs/contracts/dataset-labeling-split.md](../../../docs/contracts/dataset-labeling-split.md) | [docs/records/camera/segmentation-dataset-policy-v6.md](../../../docs/records/camera/segmentation-dataset-policy-v6.md) |
| [docs/competition/obstacle-assets-and-labels.md](../../../docs/competition/obstacle-assets-and-labels.md) | [docs/records/camera/object-label-proposals-through-pr85.md](../../../docs/records/camera/object-label-proposals-through-pr85.md) |
| [docs/handoffs/camera-to-planning.md](../../../docs/handoffs/camera-to-planning.md) | [docs/records/camera/shadow-handoff-v3.md](../../../docs/records/camera/shadow-handoff-v3.md) |
| [src/asmc_camera/training/README.md](../../../src/asmc_camera/training/README.md) | [docs/records/camera/twinlite-training-through-v6.md](../../../docs/records/camera/twinlite-training-through-v6.md) |
| [src/data_collection/README.md](../../../src/data_collection/README.md) | [docs/records/bridge/collection-modes-through-pr82.md](../../../docs/records/bridge/collection-modes-through-pr82.md) |

기존 동결5개 snapshot의 바이트/SHA와 옛 실험 결과를 유지합니다.
문서 이전v1 manifest에는 보존본 연결을 추가하고 원래 historical baseline·source SHA를 바꾸지 않습니다.
#88의 `--verify-relocation`은 내용 보존을 확인한 당시1회 검사이므로 이번 정상 본문 갱신에 실행하지 않습니다.
상시 검사는 링크/anchor·현재 구조·필요 보존 규칙을 검사합니다.

## 현재 데이터와 공유 범위

[로컬 v11 준비본 snapshot](../../../config/docs/dataset_status_20261016.json)을 읽어
train41,345장/154,537양성·234,208ignore, selection205장, heldout116장을 안내에 반영했습니다.
이 작업에서 RGB·대형 manifest를 새로 전수검사하거나 v11 supervision loss를 재구현하지 않았습니다.
원본 v11 전용 본학습 확장과 데이터는 로컬 산출물이며 현재 Git의파일럿 학습기로 대체하지 않습니다.
배포 E8 `.pt`와 학습 재개 checkpoint도 구분합니다.

## 대회 목표와 팀별 다음 작업 검토

[대회 완료 로드맵](../../architecture/consolidation-roadmap.md)에 담당별 결과·수락 증거를 정리했습니다.
우선은 허용 센서로 전 구간 반복 완주·충돌/정체/규정/실제 속도·모드 복구를 확인하는 것입니다.
카메라 모델 추가학습·3뷰 fusion·패키지 이전은 그 기준선을 보존한 독립 변경으로 진행합니다.
기존 ownership을 바꾸거나 팀원에게 새 목표를 임의 배정하지 않았습니다.

남은 항목은 family 순간 공백/freshness, LiDAR 전체 이전/3뷰 fusion·구조물/속도 유효성,
map copy/정본 정합, 판제/제어 회귀와 전체 조건 반복·현장 LAN2LAN/type787 수락입니다.
사용자가 보고한bus/truck·정면bicycle/dog·Dog2/Dog10 약세는 요청 후 추가학습 대상으로 보존합니다.

## 검증 결과

- 전체 Markdown 파일/anchor·7폴더/깊이·12개 옛 안내 제거·동결5개 SHA·역사원문 검사: 통과.
- 현행 launch23개/실행 예제52개·고정16종 ID/3그룹 표 검사: 통과.
- 문서/명령/라벨·P1-B/perception/runtime/topic CPU 회귀52개: 통과.
- 카메라 facade 계약17개·LiDAR 관측 설정 계약4개: 통과.
- Noetic Python3.8에서 문서/명령/P1-B/perception CPU 회귀25개: 통과(새 라벨 검사 추가 전 실행).
- Noetic roslaunch --dump-params23개: 모두 통과. required 경로는 미사용 placeholder만 제공했으며 node/제어/GUI를 시작하지 않음.
- Python3.8 syntax·git diff --check: 통과. msg 비주석 정의·실행 AST/token/XML 내용: 동일.

컨테이너의 /root/ws(repro)·데이터/model mount와 사용자 ROS/rqt는 변경하지 않았습니다.
/data 아래 격리된 검증 사본을 사용했습니다. GPU/MORAI 실제 주행·알고리즘 full regression을 새로 수행한 것은 아닙니다.
Hosted CI는 제출한 commit의 실제 결과를 PR에서 따로 확인합니다. 로컬 통과로 CI 실행을 대신 표시하지 않습니다.

## 기존 실패와 한계

- #89 원본 PR은 `roundabout_execution_test`의 stopline6m 전 candidate commit 실패를 보고합니다.
  직진 안정성·부분 완주 성과, 재밍 이후평균40km/h 미달성, rebase후 새 정량Bag 미측정을 구분합니다.
- 이전 planner 이관의 AVOID/assertion·binding link 실패를 현재 #89의새 실패나 해결로 바꾸지 않았습니다.
- HDmap 전체 rebuild check는 route source provenance가 옛behavior_planner 경로와 현재asmc_planner 경로로
  다르게 기록된 기존 차이가 있습니다. 지도/route 값이나 생성 product를 이 문서 작업에서 재작성하지 않습니다.
- 과거 P1-A baseline test의16checkpoint/provisional 기대는 현재15checkpoint/team_final과 맞지 않습니다.
  동결 기록의 source와 현재 상태를 구분하며 옛 snapshot을 현재 값으로 덮어쓰지 않습니다.
- 저장소 바깥 개인 연구 문서의 찾지 못한 LiDAR 실험 링크3개는 증거를 꾸며 채우지 않았습니다.
  이전 구조 작업의 알려진 별도 범위로 남습니다.
