# 카메라 → LiDAR 연결 인계

카메라 추론은 `asmc_camera`, 관측과 3D 객체의 matching/fusion은 `asmc_lidar`가 담당한다.
현재 3뷰 관측 수신·버퍼·보정/투영 API와 전방 matching이 구현돼 있다. 3뷰 객체 fusion 확대는 후속이다.

## 현재 연결과 소유권

```text
asmc_camera/camera.launch
 → /perception/camera/objects/observations : asmc_msgs/CameraObjectObservation
 → asmc_lidar의 뷰별 수신·버퍼 → 전방 matching·분류 유지
 → 공용 planning_observations_map → planning_inputs → 판제
 → tracks_local/map (분석·local 기하)
```

`/YOLO/BBox` 정수 wire와 별도 어댑터는 폐기했다. header·원본 영상 크기·subpixel bbox·원본 이름·
그룹·confidence·session/generation·보정/모델/설정 ID를 유지한다. 추론 tensor 크기로 보정을 다시 축소하지 않는다.
정상 빈 결과·무효·미수신/stale·모드/세대 변경을 구분하고, 카메라 미검출만으로 LiDAR 객체를 삭제하지 않는다.

- [수신·버퍼·보정·투영 API](../../src/asmc_lidar/docs/camera-input.md)
- [카메라 실행 계약](../contracts/camera-runtime-v1.md) · [관측 계약](../../config/contracts/camera_object_observation_v1.yaml)
- [실제 LiDAR 소비·제한](../../src/asmc_lidar/docs/planning-runtime.md) · [공동 작업표](../../src/asmc_lidar/docs/migration-tasks.md)

## 모델과 분류

YOLO는 E8 또는 `detector:=pretrained`, 전방544×960 B1·측방480×640 B2, 목표15Hz·confidence0.40·NMS0.50이다.
TwinLite Large E8 TensorRT도 같은 B1/B2 크기·목표15Hz, 신호등은10Hz다. 기본 camera.launch는 세 모델 ACTIVE다.

| 그룹 | 원본 클래스 |
|---|---|
| vehicle | car, bus, truck |
| dynamic_obstacle | person, dog, deer, bicycle, stroller |
| static_obstacle | motorcycle, bollard, box, concrete_barrier, steel_barricade, traffic_drum, plastic_barrier, traffic_cone |

그룹은 소비용 분류이며 실제 운동 상태가 아니다. fresh 양성 matching의 vehicle/dynamic_obstacle은
HD맵 도로 밖에서도 보호한다. 오래된 분류나 LiDAR 추정 속도만으로 이 예외를 적용하지 않는다.
판제 numeric enum은 모든 원본 클래스를 표현하지 못하며 unknown도 충돌 기하를 유지한다.

## F1 연결 검증

전방 실제 matching과 좌우 수신·native 재투영은 [F1 결과](../records/lidar/2026-10-27-front-fusion-verification.md)에 있다.
투영 실패를 미검출로 처리하던 경계와 투영 중 카메라 세대 변경을 수정했다.
[진단 해석·갤러리·재현 방법](../../src/asmc_lidar/docs/fusion-verification.md)을 따른다.
새 진단은 판제 정본 입력이 아니며, 다른 source clock의 원본 시각은 아직 직접 비교할 수 없다.

## 실행과 후속

```bash
roslaunch asmc_camera camera.launch
roslaunch asmc_lidar lidar.launch
```

각 명령은 별도 터미널이다. 모델을 중복 시작하지 않는다. 카메라 없는 비교는 LiDAR의 `fusion:=false`를 사용한다.
3뷰 확대는 뷰별 원본 보정·시각·관측 ID를 사용하며 느린 뷰를 기다리거나 같은 근거를 중복 확인 횟수로 세지 않는다.
안승현은 **3뷰 matching/fusion과 카메라 API·보정·시간·라벨 연결**, 손재호는 **터널·OBB·추적/속도/운동 상태**를 담당한다.
공동 수정 지점과 완료 기준은 [개선 계획](../../src/asmc_lidar/docs/improvement-milestones.md)을 따른다.

bus/truck·차로 앞 bicycle/dog·dog 하위 종류 약세는 가중치 개선 기록이며, 현재 실행 구조와 구분해
장원태가 detect 추가학습을 진행한다. 새 모델은 기존16종/3그룹·원본 좌표·B1/B2 TensorRT와 동일 val을 검증한 뒤 인수한다.
Seg는 mask GT·binding·후처리·3뷰 지연까지 별도 확인할 후보이며 detect 납품의 선행 조건이 아니다.
[인수할 모델·라벨 근거](../../src/asmc_lidar/docs/improvement-milestones.md#yolo-인수--detect-우선-seg는-조건부)를 참고한다. 구 정수 wire 및 이전 인계 수치는
[PR #111까지의 기록](../records/lidar/camera-handoff-through-pr111.md)에 보존한다.
