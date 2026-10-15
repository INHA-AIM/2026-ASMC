# 공용 계약

필드·enum·MD5·source 시각·좌표·상태 전이는 실제 `.msg/.srv`·기계 설정·소비 코드를 함께 확인합니다.
문서 현행화는 wire를 변경하거나 미구현 소비자를 구현 완료로 바꾸는 작업이 아닙니다.

| 경계 | 사람이 읽는 정본 | 기계/실행 정본 |
|---|---|---|
| 카메라 실행·모드·3뷰 관측 | [camera runtime v1](camera-runtime-v1.md) | `camera_object_observation_v1.yaml`, camera launch/runtime YAML |
| 카메라 데이터·라벨·split | [dataset 계약](dataset-labeling-split.md) | 고정16종·소스 대응, versioned policy/manifest/검수 |
| 카메라→LiDAR/판제 | [LiDAR 소비 v1](lidar-planning-consumer-v1.md), [현재 인계](../handoffs/camera-to-lidar.md) | 현재 Main 전방 어댑터·후속3뷰 관측 설정 |
| LiDAR 최종 local/map 출력 | [LiDAR object v2](lidar-object-v2.md) | `lidar_candidate_topic_contract_v2.yaml`, opt-in·판제 cutover 전 |
| typed 신호등·tracked 객체 | [perception runtime v1](perception-runtime-v1.md) | `perception_runtime_topic_contract_v1.yaml` |
| Behavior/Frenet transaction | [FSM/Frenet v2](planning-fsm-frenet-v2.md) | 공용 msg·asmc_planner 내부 직접 호출/관찰 출력 |
| HD Map message/topic | [HD Map topic v1](hd-map-topic-v1.md) | `hd_map_topic_contract_v1.yaml`, asmc_msgs registry |
| HD Map 실행/조회/RViz | [HD Map runtime v1](hd-map-runtime-v1.md) | competition_map launch·runtime pins |
| checkpoint·mission metadata | [K-City metadata v1](competition-kcity2025-metadata-v1.md) | 현재 metadata YAML·출처/authority |
| MORAI wire 호환 | [beta_drive 정책](simulator-morai-msgs-beta-policy.md) | 고정 submodule·bridge decoder |

기계 계약은 저장소 `config/contracts/`에 있습니다. source 설정과 wire 의미가 충돌하면
producer/consumer owner와 영향·호환·검증을 확인합니다. 기존 필드/상수는 동결하며 확장은 additive/versioned로 합니다.
metadata의 official·derived_official·team_final·provisional은 구분하고 team 결정을 공식 공지로 바꾸지 않습니다.

기본 invariants는 SI·source/reference stamp·steady freshness·단일 생산자입니다.
카메라 픽셀, local base_link, map geometry를 구분하며 속도 unknown을0으로 만들지 않습니다.
빈 정상 관측과 무효/미수신/stale, semantic 이동가능 그룹과 실제 운동 판정을 구분합니다.
본선 입력에 GT/oracle/gRPC를 추가하지 않습니다.

## 변경 확인

- message: MD5/source 계약·serialization·영향 producer/consumer build.
- launch/명령: [인자 검사](../operations/launch-reference.md)·ROS 해석·단일 생산자.
- 좌표/시각: pose·extrinsics·기준점/단위·원본 stamp·무효/재시작 회귀.
- 데이터: 원본SHA·실제 라벨·계보 split·배경/ignore/partial·전용 학습기 정합.
- 모델: 동일 입력 품질·전체 스택 공급/지연·실제 주행. 기본 전환과 전체 수락은 별도.

과거 P5-A/P5-C/P5-D/E는 [기록](../records/README.md)의 당시 프로필 근거이며
현재960·TensorRT 전체 수락으로 재사용하지 않습니다. 옛 호환 안내12개는 구조 이전에서 제거했습니다.
package 내부 설명은 `src/<package>/docs/`, 현재 복합 실행은 [operations](../operations/commands.md),
인수 책임은 [handoffs](../handoffs/README.md), 실험은 records에 둡니다.
