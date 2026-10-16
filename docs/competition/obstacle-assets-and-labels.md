# MORAI 객체 에셋·학습 라벨·소비 그룹

현재 배포는 **16종 학습 출력 → 3개 소비 그룹**입니다. 에셋명, 원본 라벨, teacher 표현,
학습 클래스, 소비 그룹과 실제 운동 상태를 별도로 기록합니다. 모델 이름만으로 정확한 gRPC 에셋을 알아냈다고 표시하지 않습니다.

## 현재 학습 ID와 소비 그룹

| ID | 학습 클래스 | 소비 그룹 | MORAI 외형·사용자 표기 |
|---:|---|---|---|
| 0 | person | dynamic_obstacle | 성인/어린이/경찰/작업자 등 보이는 사람 |
| 1 | dog | dynamic_obstacle | Dog 하위 종류, Dog2/Dog10 약세 기록 |
| 2 | deer | dynamic_obstacle | 뿔 없는 사슴 포함, doe는 prompt 별칭 |
| 3 | bicycle | dynamic_obstacle | 사람 에셋에 붙어 움직일 수 있는 자전거 |
| 4 | car | vehicle | 승용차/SUV/MPV/wagon 등 확인한 차량 |
| 5 | motorcycle | static_obstacle | Scooter + SportBike; 대회에서 정적 에셋이라는 사용자 확인 |
| 6 | bus | vehicle | 확인된 버스, van 전체를 버스로 일괄 변환하지 않음 |
| 7 | truck | vehicle | 확인된 화물차 |
| 8 | bollard | static_obstacle | bollard 하위 종류 |
| 9 | box | static_obstacle | CargoBox + WoodBox; wooden crate는 prompt 표현 |
| 10 | concrete_barrier | static_obstacle | concrete_barricade + concrete-barrier |
| 11 | steel_barricade | static_obstacle | steel_barricade |
| 12 | traffic_drum | static_obstacle | PE_Drum + redbarrel + trafficbarrel + yellowbarrel |
| 13 | plastic_barrier | static_obstacle | PE_Firewall |
| 14 | traffic_cone | static_obstacle | color cone + rubber cone |
| 15 | stroller | dynamic_obstacle | 사람과 함께 이동할 수 있는 유모차 |

원본 CargoBox/WoodBox·drum 색상·Dog 종류·actor/component 계보는 통합 후에도 보존합니다.
유모차 안의 아기는 별도 person 정답에서 제외하며 미는 성인과 유모차는 각각 유지합니다.
사람+자전거 복합 박스를 동일 좌표의 두 클래스로 복제하지 않습니다.
입식 전동 킥보드는 대상에 추가하지 않습니다.

`source_class_name`은 위 모델 원본 이름이고 `class_name`은 그룹입니다.
그룹은 대회에서의 이동 가능성·판제 소비 목적이며 `is_dynamic`, 속도·충돌 위험을 확정하지 않습니다.
car/bus/truck 혼동은 vehicle 위치 검출과 이름 오류를 별도 평가합니다. motorcycle→car는 그룹 오류입니다.
deer→dog도 원본 종류 오류이며 기존 animal 진단으로 따로 집계할 수 있을 뿐 deer GT를 dog로 바꾸지 않습니다.
미등록 이름은 unknown/진단으로 남기고 static으로 자동 배정하지 않습니다.

## 설정의 권위

- 현재 소비 그룹: [object_classes_mission16_v1.yaml](../../src/asmc_camera/config/camera/object_classes_mission16_v1.yaml),
  [runtime mapping](../../src/asmc_camera/src/asmc_camera/detection/mission.py).
- 고정 학습 순서·구명칭 호환: [camera_object_labels_v2.json](../../config/datasets/camera_object_labels_v2.json)의
  `pilot_target_classes`·`training_map`.
- 소스별 의미: [camera_object_source_labels_v2.json](../../config/datasets/camera_object_source_labels_v2.json).
- 에셋 API 이름: 실제 spawn/catalog 응답·장부로 확인한 값만 사용. 사용자 표기를 추정 API 이름으로 저장하지 않음.

**주의:** 데이터 감사 설정 v2의 `groups`와 `asset_contract.consumer_group`은 과거
person/animal/vehicle/obstacle 의미 분류입니다. 원본 검수의 재현을 위해 유지하며 현재 배포3그룹으로 해석하지 않습니다.
이름 정규화 →16종 학습 클래스 → 현재 runtime3그룹 순서로 연결합니다. 이번 문서 변경은 기존 데이터 설정을 수정하지 않습니다.
`cargo_box`, `wooden_box`, `traffic_barrel` 기록은 각각 box·box·traffic_drum으로 읽습니다.
전역 fence 제외 규칙보다 tooth-decay의 소스별 Fence→steel_barricade 예외를 먼저 적용합니다.

## Roboflow 소스 의미

직접 대응은 원본 박스를 유지하고 이름을 변환합니다. 묶음의 구조·표본 검수와 주석 완전성 판단은
종류 대응과 별개입니다. 숫자의 의미를 다른 프로젝트에 복사하지 않습니다.
혼합 라벨은 개별 종류/연속구간 근거로 분리하고 미확정 부분만 보류합니다.

| 소스 | 직접 대응·검토 사항 |
|---|---|
| Katri 여섯 소스 | 0 car, 1 bus, 2/3 person, 5 traffic_drum, 7 bollard; 4 사람+자전거 복합, 6 dog/deer 혼합 |
| jjj/katri_new_data | Katri 규칙 +8 box; 9 box/concrete_barrier/steel_barricade/plastic_barrier 혼합 |
| aiv5challenge/yeonsoo-sarang | car/person 직접 대응 |
| battery-detection/cone-traffic | cone→traffic_cone; 신호등 제외 |
| bicycle-man/morai-bicycle | 탑승자+자전거 복합 범위 확인, 구성요소 박스 보완 또는 보류 |
| dh-mzmhr/animal | animal의 dog/deer 개별 분리 |
| Enzo | cargo/wood box→box, PE_Drum→traffic_drum, steel barrigate→steel_barricade, barrigate→concrete_barrier, pe barrigate→plastic_barrier; dog 안 deer 반례 개별 수정, speed 제외 |
| hlfma_2/3 | 명확한 person/car/truck/bus 활용; object/others는 개별 의미 확인 |
| mino/vi2re | car/person 직접; obstacle 포괄 혼합, 신호/표지 제외 |
| parkminseo/final | human→person; box는 WoodBox/PE_Firewall 분리, 신호등 제외 |
| s-workspace/morai_obstacle | person 직접 대응 |
| selfcar/morai_test | **0 traffic_drum, 1/2 bollard**; Katri 숫자 규칙 금지 |
| seonhyung/morai-perception | pedestrian/pedstrain 복합, vehicle 포괄, animal 혼합, obstacle 포괄; 이름만 자동 확정하지 않음 |
| tooth-decay/morai_ai_mission | **Fence steel_barricade**, PE traffic_drum, Concrete_1/2 concrete_barrier; Obstacle_Yellow 혼합, sign/tl 제외 |
| yolo-m5lvd/detection | 명확한 기존 클래스 활용; bike/obstacle 혼합, traffic/streetlight 제외 |
| yuna/obstcle | dynamic 사람/차량 포괄·static 장애물 포괄; 개별 재분류 필요 |

프로젝트 전체22개의 정확한 slug·최신 공개 버전 고정은 [source 목록](../../config/datasets/roboflow_sources_v1.json)과
로컬 pins를 사용합니다. 제외 라벨을 뺀 이미지에도 다른 대상이 누락될 수 있으므로 완전한 negative로 간주하지 않습니다.
Teacher의 이름이 틀려도 위치와 실제 종류가 확인되면 검수 후 수정할 수 있습니다. 원본 출력·수정 근거를 남깁니다.

## 대회 범위와 수집

사용자가 전달한2026-10-06 운영진 Q&A는 F2 전체 객체 에셋이 출제 범위이고 정적 에셋이
움직이는 예외는 없으며 OBJ_G80_Open_Door는 제외된다고 설명합니다. 최신 공식 공지가 우선입니다.
전달된102개 집계는 전체 F2 카탈로그를 독립 검수한 결과가 아니므로 완결 목록으로 보장하지 않습니다.
원문 제공 범위와 당시 후보 비교는 [Q&A/과거 라벨 제안](../records/camera/object-label-proposals-through-pr85.md)에 보존합니다.

수집에서 표지판·나무·가로등·맨홀·불을 제외한다는 합의는 자동으로 spawn할 대상의 범위입니다.
지도 구조물을 box/drum으로 잘못 붙이거나 미지원 물체를 static 학습 클래스로 억지 배정하지 않습니다.
높이는 각 지점 지면·에셋 기준점으로 확인하고 F2 저장본과 대조합니다. 동일z 일괄 배치·임의 공중 spawn은 채택하지 않습니다.

## LiDAR 속도 검증에 연결할 기준

운영진 전달 답변의 Object Info 순간 선속도는 후륜축 기준입니다.
현재 bridge의 `/Object_topic` velocity는 km/h이므로 개발 GT 평가에서는 각 성분을3.6으로 나눕니다.
Ego velocity는 이미 m/s인 별도 경로이므로 다시 나누지 않습니다.
부분 군집 평균·OBB 중심과 비교할 때 기준점·좌표·source 시각·자차 보상을 먼저 정렬합니다.
필요한 객체 각속도와 제원이 있으면 `v_P = v_R + omega × r_RP`로 기준점을 변환합니다.
ObjectStatus에 객체 yaw rate가 없으므로 Ego IMU 각속도로 대체하지 않습니다.
GT/Capture/gRPC는 개발용이며 본선 알고리즘 입력으로 사용하지 않습니다.

현재 실제 소비·구형 wire의 정보 한계는 [배포 계약](../operations/camera-deployment.md),
학습 주석·배경·ignore는 [데이터 계약](../contracts/dataset-labeling-split.md)을 따릅니다.
