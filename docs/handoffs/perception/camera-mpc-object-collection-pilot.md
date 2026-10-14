# 다음 구현: 팀 MPC 기반 장애물 자동 수집 파일럿

**2026-10-14 상태:** 아래는 PR #81에서 작성한 초기 수집 설계다. 실제 Capture 수집 도구·PP+P
주행·GT export와 종료 결과는 후속 PR #82에 있다. 이 초기12개 배치·raw 우선 계획을
현재 실행 설정으로 사용하지 않는다. [통합 준비 상태](camera-data-integration-prep.md)를 먼저 확인한다.

기준: 2026-10-11, [v7 기존 자료 채택 결과](camera-training-data-audit.md#기존-자료-채택-확대--2026-10-11-v7).
**이 문서는 다음 구현의 계약이다. Bridge 수정·스폰·주행·녹화·학습은 아직 수행하지 않았다.**
기존 MPC와 bridge 연결 코드를 재사용하고 수집 전용 도구를 추가한다. 운영 추론·ROS 메시지·
대회용 GT 차단 정책을 바꾸지 않는다.

## 부족 자료에 따른 배치

최신 채택본은 `camera_objects_bbox_v7-adoption-r2`, 23,264장·129,630객체다.
train은 22,023장이다. 아래 수치는 **train에서 해당 종류가 등장하는 이미지 수**이며,
객체 개수·증강본 수·독립 장면 수와 다르다.

| 학습 클래스 | Train 등장 이미지 | 수집 풀 안의 비율 |
|---|---:|---:|
| stroller | 0 | 장애물 32.09% |
| plastic_barrier | 172 | 장애물 17.30% |
| steel_barricade | 178 | 장애물 17.01% |
| traffic_cone | 618 | 장애물 9.13% |
| concrete_barrier | 953 | 장애물 7.35% |
| box | 1,109 | 장애물 6.81% |
| traffic_drum | 1,661 | 장애물 5.57% |
| bollard | 2,298 | 장애물 4.73% |
| deer | 61 | 희소 객체 40.73% |
| motorcycle | 69 | 희소 객체 38.30% |
| bicycle | 230 | 희소 객체 20.98% |

12개 배치에서 장애물 10자리, deer/bicycle/motorcycle 2자리를 따로 배분한다.
각 풀의 가중치는 `1 / sqrt(max(train_presence_images, 50))`를 정규화한다.
한 배치에서 반올림해 늘 같은 종류만 선택하지 않도록 고정 seed와 누적 할당 차이를 기록한다.
비율은 여러 배치의 목표이며 매 배치에 모든 종류를 넣는다는 뜻이 아니다.
수집이 늘면 등장 이미지 수·조건별 다양성을 다시 집계한다. 수집 비율을 학습 샘플링 비율로
그대로 사용하지 않는다.

CargoBox/WoodBox, 콘크리트 방벽의 서로 다른 형상, PE_Firewall, 색상별 drum·cone을
하위 유형으로 순환한다. 사용자 표기·gRPC API 이름·학습 클래스·소비 그룹을 별도 보존한다.
정확한 API 에셋과 지면 배치를 확인하지 못한 종류는 미지원 사유와 비어 있는 할당을 남긴다.
다른 물체로 몰래 대체하지 않는다. 특히 stroller는 자료 0장인 우선 확인 대상이며 지원 확인
없이 스폰 가능한 것으로 간주하지 않는다.

새로 스폰하지 않는 종류는 사람·강아지·승용차·버스·트럭·표지판·나무·불·고정 구조물·
입식 전동 킥보드다. 기존 사람·차량 자료는 학습 회귀 방지에 유지한다. 기존 actor와 Ego는
보존하고 이 실행에서 만든 actor만 생성 세대와 소유권 장부를 확인해 제거한다.

## 연결할 기존 코드와 먼저 확인할 사항

| 경계 | 기존 연결 | 다음 단계 확인·구현 |
|---|---|---|
| 전역경로 | [MissionScenarioSpec](../hd-map/mission-scenario-spec.md), [route 좌표 API](../../../src/asmc_hd_map/src/asmc_hd_map/mission_scenario.py) | route station·횡방향 오프셋과 지역 도로 높이로 후보 생성. 지도 원점·MORAI world 변환을 실제 위치로 검증 |
| MPC | [controller](../../../src/asmc_controller/src/main.cpp), [freshness](../../../src/asmc_controller/src/Global/ControlContext.hpp) | `/planning/trajectory`, `/vehicle/state` → `/ctrl_cmd/normal`. trajectory 0.25s·state 0.20s freshness를 유지. preview 후 객체 없는 저속 추종·정지·재개 검증 |
| RGB·semantic·LiDAR | [sensor bridge](../../../src/asmc_bridge/launch/morai_sensor_bridge.launch) | 수집 전용 development profile에서 실제 로드된 sensor ID·종류·Destination port·해상도 전수 조회. 추가 LiDAR의 포트를 ID로 추측하지 않음 |
| Object Info | [UDP bridge](../../../src/asmc_bridge/scripts/morai_udp_bridge.py) | development에서 `/Object_topic` 수신. 현재 `OBJ_N=20` 파싱 한계가 있으므로 실제 패킷 길이·actor 총수·누락 ID 검사. 프로토콜 상한 20으로 단정하지 않음 |
| gRPC actor | [spawn proto](../../../tools/grpc_inha_univ/morai_standard_proto/morai/actor/actor_spawn.proto) | 가능한 에셋 목록과 실제 생성 결과 대조. GetAllActorsState의 vehicle/pedestrian/obstacle 범위·ID/client key·BBox 기준점·timestamp 단위를 확인 |
| 터널 Ego pose | [개발용 Ego state](../../../src/asmc_bridge/launch/vehicle_state_ego_dev.launch) | GPS 무효 시 수집 전용 개발 pose를 사용하고 출처를 저장. GPS/IMU state 생산자와 동시에 실행하지 않음 |
| 에셋 목록 | [기존 스폰 목록](../../../config/morai/object_spawn_catalog_v1.yaml), [16종 의미](../../../config/datasets/camera_object_labels_v2.json) | 차량 목록을 장애물 API 목록으로 오인하지 않음. 정확한 API 이름은 실제 조회 전까지 미확인 |

현재 bridge는 `stream_profile=competition`에서 Object/semantic/instance 입력을 의도적으로
제한한다. 수집용 자동 센서 발견과 수신 옵션은 별도 development 경로에 연결한다.
대회 기본값을 GT 자동 수신으로 바꾸지 않는다. sensor 추가 후 기존 RGB·GPS·IMU·LiDAR가
계속 들어오는지도 검사한다. 현재 설정에 instance **카메라**가 없다면 instance LiDAR를
동일 기능으로 설명하지 않는다.

## Map 장부와 영상 bbox 초안

노이즈 없는 pose와 map에 기록한 스폰 위치는 객체 종류와 위치 연결에 유용하다.
영상 정답의 가시 영역까지 자동으로 정확해지는 것은 아니다. 다음 순서로 구현한다.

1. `layout_id`, seed, 정확한 에셋명, 목표 클래스/하위 유형, actor ID+생성 세대, client key,
   map/world 위치·회전·크기, 기준점, 생성 후 실측 자세, 지면 확인 결과를 스폰 장부에 기록한다.
2. RGB·semantic·actor/Ego state의 원본 timestamp와 callback 수신 monotonic 시간을 따로
   기록한다. ns/µs·clock origin과 bridge stamp의 의미를 실제 패킷으로 확인한다.
   동일 숫자 actor ID의 재사용·세대 변경을 연결하지 않는다.
3. 영상 시점의 유효 Ego pose와 카메라 장착·광학 축·내부 보정으로 map 객체의 8코너를
   투영한다. 카메라 뒤/near plane을 가로지르는 객체를 처리하고 원본 영상 범위로 자른다.
   8코너의 외접 박스는 `projected_amodal_draft`로 저장한다.
4. semantic 가시 영역으로 박스의 가림·잘림을 보완한다. 포괄 `Obstacle`의 세부 종류는
   스폰 장부로 연결하되, 붙어 있는 객체·동일 semantic 종류·기존 actor와의 겹침은
   instance GT 또는 개별 검수를 요구한다. semantic 색상만으로 actor ID를 만들지 않는다.
5. 검수된 `visible_extent` bbox만 기존 16종 YOLO 학습 export에 채택한다. 모호한 가림·
   시각 대응·미대응은 초안에 남긴다. 투영 실패·teacher 미검출을 negative로 바꾸지 않는다.

터널에서는 오래된 GPS pose를 재사용하지 않는다. 개발 Ego pose는 수집의 라벨 생성용이며
운영 perception/planning에서 GT를 사용하도록 바꾸는 작업이 아니다.

## 배치와 높이 검증

전역경로의 각 지점에서 도로 표면 높이·경사와 에셋별 기준점/지지점을 사용한다.
Ego z를 모든 객체에 복사하거나 `중심 z−높이/2`만으로 접지를 판정하지 않는다.
native ground snap은 지원 여부·실제 동작을 확인한 뒤 사용한다.

후보 위치에서는 차량의 주행 swept 영역과 여유 공간을 제외하고 **객체 전체 footprint**가
밖에 있는지 검사한다. 도로 경계·경사, 벽·가드레일·기존 actor, 새 객체끼리의 겹침도
검사한다. 라벨을 많이 얻기 위해 경로를 막는 배치는 사용하지 않는다.

생성 후 회전된 지지점과 주변 표면의 차이, RGB의 떠 있음·지면 관통·잘못된 기울기를
확인한다. 높이·좌표·생성 actor ID 확인 실패는 기록 대상에서 제외해 수정한다.
객체별 허용 높이 오차는 파일럿의 실제 asset/pivot 검증에서 확정하고 수치가 없는 현재를
검증 완료로 표시하지 않는다.

## 구현·실행 순서와 확장 기준

1. **수신·preview:** 로드된 센서/네트워크 snapshot과 SHA, camera calibration, actor/pose
   clocks를 저장한다. Object 총수와 gRPC/UDP 수신 ID를 비교한다. 제어 preview만으로
   주행이 검증됐다고 하지 않는다.
2. **객체 없는 MPC:** 저속 경로 추종·정지·재개, 상태/경로 끊김의 정지 동작을 확인한다.
   기존 제어 mux를 통해 팀 MPC를 사용하고 별도 steering 제어기를 만들지 않는다.
3. **6개 배치:** 종류·생성 세대·지면·투영·전체 센서 수신과 실제 visible bbox 수정량을 확인한다.
4. **최대 12개 짧은 구간:** 배경이 다른 구간, SUNNY/FOGGY 각 13시에서 raw 수집을
   먼저 비교한다. 같은 위치의 연속 사진만 늘리지 않고 거리·뷰·가림 변화별 표본을 확인한다.
5. **방식 판정:** raw+actor/semantic 초안의 수정량·시각 오차가 크면 Capture GT로 전환한다.
   필요한 v3 센서셋·동기 모드는 확인한 설정·이유와 함께 사용자에게 요청한다.
6. **파일럿 수락 후:** 전역경로 전체 주변에 배치해 SUNNY/FOGGY × 11/13/15시의 6회
   주행으로 확대한다. 첫 세트의 부족 유형·조건을 집계한 뒤에만 두 번째 배치와 6회를 결정한다.

동일 배치의 여섯 날씨/시간 주행은 하나의 split 그룹으로 묶는다. 두 번째 배치도 같은 고정
객체 위치·연속 장면이 겹치면 같은 그룹으로 연결한다. train/selection/heldout은 프레임별
무작위 분할하지 않는다. 날씨를 바꿨다는 이유만으로 평가 독립성을 주장하지 않는다.

수락 기록에는 수신 누락·actor ID 누락·stamp 역행/clock 불명·배치 높이 실패·투영 실패·
GT 수정량과 조건별 독립 배치 수를 남긴다. 미확정 clock·actor·접지 문제는 해결 후 확장한다.
기존 actor·날씨·시간·실행 설정은 복구하고 시험 actor만 정리한다.

## 다음 단계 저장 구조

```text
data/collection_runs/mpc_objects/<layout>/<weather-time>/<run>/
  manifest.json        # 센서/네트워크/보정/소스 SHA, 시각 정의, 시작·종료·실패
  spawn-ledger.jsonl   # actor ID+세대, API 이름, pose, footprint/지지점, 접지 검수
  raw/                # RGB·semantic·Object·Ego·actor state 또는 원본 bag
  drafts/             # projected_amodal_draft와 semantic 기반 visible 초안
  reviews/            # 원본 SHA·수정 근거·확정 visible bbox
```

이는 **미구현 저장 계약**이다. 원본은 수정하지 않고 검수 결과를 다음 curated revision으로
연결한다. 독립 배치·클래스·하위 유형·조건별 부족량을 다시 계산한 뒤 샘플링과 비교 재학습으로
이어간다. 현재 운영 모델을 교체하거나 학습을 시작하는 단계는 아니다.
