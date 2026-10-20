# 인지 상시 발행·판제 구간별 소비

## 현재 실행 결정

`roslaunch asmc_camera camera.launch`는 YOLO·TwinLite·신호등 모델을 모두 ACTIVE로
시작한다. 새 입력의 추론과 결과 발행은 구간·GPS 유효 여부와 무관하게 계속한다.
목표는 YOLO15Hz·TwinLite15Hz·ResNet10Hz이며 GUI는 별도다.
처리율 보장이나 음영 밖 family의 주행 유효성을 뜻하지 않는다.

아래 범위는 **판제가 결과를 사용할 권장 범위**다. 이번 변경은 판제에 새 구간 필터나
자동 모드 제어를 구현하지 않는다. 일반 구간은 HD맵, 음영 차선은 유효한 family를 사용하며
객체·신호등의 구체적인 판단과 예외는 판제 담당자가 결정한다.

Frenet V2는 매 주기 새 객체 입력을 요구하고 stale이면 정지한다
([입력 검사](../../src/asmc_planner/src/Planning/V2Cycle.hpp)). 따라서 카메라·LiDAR·
`/planning/objects`를 구간별로 끄지 않는다. 비사용 구간에도 신선한 객체 입력을 유지하며,
객체 없음은 실제 새 관측의 정상 빈 결과로만 표현한다. 미수신을 빈 배열로 위장하지 않는다.
추후 전력/부하 때문에 중단한다면 정책상 비사용과 센서 장애를 구별하는 판제 계약부터 합의한다.

## 판제 적용 확인표

2026-10-20 main `2c94bf11` 코드 대조다. 아래 항목은 판제의 적용·인수를 확인하는 안내이며,
카메라가 판제 정책이나 손재호의 로컬 fusion 구현을 대신 변경한 목록이 아니다.
카메라는 원본 이름·그룹·시각·좌표·상태를 제공하고, 판제는 소비·위험 판단·제어를 소유한다.

| 소비 그룹 | 원본 모델 이름 | 정책에서 구분할 사항 |
|---|---|---|
| `vehicle` | car, bus, truck | vehicle은 차종 묶음. 실제 이동·정지·추종 가능 여부는 위치·추적 근거와 별도 |
| `dynamic_obstacle` | person, dog, deer, bicycle, stroller | 이동 가능한 대회 에셋. 매 관측이 이동 중이라는 뜻은 아님 |
| `static_obstacle` | motorcycle, bollard, box, concrete_barrier, steel_barricade, traffic_drum, plastic_barrier, traffic_cone | SportBike 정적 에셋이라는 사용자 확인 반영. 분류 오류를 실제 운동 정답으로 승격하지 않음 |

`source_class_name`은 원본16종, `class_name`은 위 그룹이다. MORAI actor/API 에셋명은
검출만으로 확인되지 않는다. 자세한 학습 ID·하위 종류는 [에셋/라벨](../competition/obstacle-assets-and-labels.md)을 따른다.

| 연결 경계 | 현재 전달·제한 | 판제가 확인할 것 |
|---|---|---|
| 카메라 관측 | 원본 영상 픽셀 bbox·header와 camera ID, confidence·그룹·원본 이름 | tensor 좌표로 다시 확대하거나 원본 보정을 축소하지 않음. LiDAR의 미터 좌표와 직접 비교하지 않음 |
| Main 전방 호환 | `/YOLO/BBox`는 전방만, 구형 wire에 confidence·그룹·session 없음 | 추가 cutoff는 변환 전 적용. Main은 원본 이름으로 그룹 재구성. 좌우3뷰 fusion은 손재호 작업 |
| 판제 숫자 enum | `vehicle/pedestrian/unknown`. 원본 person은 pedestrian, 다른 dynamic/static은 unknown 가능 | **unknown 형상을 삭제하지 않음**. 세 그룹/원본 종류 정책은 문자열을 보존한 tracked 정보에서 연결 |
| 운동 상태 | semantic 그룹과 `motion_state`·속도 validity는 별개. Main의 opt-in typed 출력은 아직 velocity/dynamic valid=false | unknown·미계산을 정지 속도0으로 해석하지 않음. 실제 운영 출력과 source policy를 확인 |
| 입력 유지 | 새 관측의 빈 결과와 stale/무효/미수신을 구분 | 미션 밖 입력 토픽을 끊거나 빈 배열을 타이머로 만들어 장애를 숨기지 않음 |
| 음영 family | `base_link` xyz·support·freshness, 같은 영상의 IMU 지면 context | 좌/우 역할을 home/opposing으로 고정하지 않음. stale·SHA·세대 불일치와 GPS 복귀 정책 인수 |
| HD 정보 | map pose·station·속도·GPS 유효성은 별도 계약 | [HD 적용 확인표](hd-map-to-planning.md#판제-적용-전-확인--2026-10-20-코드-대조)의 번들·기준점·단위 대조 |
| 신호등 | camera4 ResNet 단일 producer의 typed 결과와 legacy String | legacy latch를 현재 관측으로 믿지 않고 stamp·stale·confidence·정지선/미션을 함께 판단 |

관련 코드의 실제 경계는 [분류 converter](../../src/planning_inputs/src/converter.cpp),
[Frenet 파일 지도 reader](../../src/asmc_planner/src/Frenet/frenet_planner.cpp),
[MGeo 속도 조회](../../src/asmc_planner/src/Behavior/Map/route_manager.cpp)다.
원본 이름·confidence가 빠진 구형 메시지를 소비한 뒤 임의로 복원했다고 표시하지 않는다.
단일 producer·정상 빈 결과·센서 중단·재시작·실제 장애물 회피를 함께 인수한다.
카메라/LiDAR 수신 구현이 빌드됐다는 사실과 판제 폐루프 인수는 구분한다.

## 검토한 소비 범위

표의 `Axxx`는 `A2256W000xxx`의 약칭이다. 정본 경계는
[대회 metadata 원본](../../src/asmc_hd_map/config/kcity2025_competition_metadata_source.yaml)과
[전역경로](../../src/asmc_planner/config/behavior/route_links_1.yaml)에서 확인한다.
객체 소비 단위는 팀 운영 제안이며 새로운 공식 미션 경계가 아니다.

| 결과 | 권장 소비 범위 | 경계 해석 |
|---|---|---|
| 신호등 | A236 시작 → A308 끝 | 지도 정지선·유효 신호와 함께 판단. 상시 추론은 모든 구간에서 신호 정지를 허용한다는 뜻이 아님 |
| 객체 단위 1 | A213 중간 → A315 30% | 정적 장애물 시작부터 동적 장애물 종료까지 연속 사용 |
| 객체 단위 2 | A148 시작 → **A153 끝 / A451 진입** | 로터리·연결로·고주로. A153 주변 차량도 포함 |
| 객체 단위 3 | A448 시작 → A128 진입 | 음영 접근 A448과 공식 팀 음영 A126 포함 |
| TwinLite/family | 공식 팀 음영 **A126**, 대응 차로 A131 | **A126 진입 전에** 새 유효 family·freshness·역할 확인. A448은 접근/준비 구간 |

객체 단위 1의 경로는 A213(50%) → A222 → A220 → A202 → A728 → A599 → A207 → A205 → A315(30%)다.
중간 일반 링크에서도 이 단위의 소비를 연속 유지할 수 있다. 실제 정적 스폰 기대 링크는 A222,
동적 스폰 기대 링크는 A728/A599/A207/A205이며 운영 범위와 구별한다.

객체 단위 2는 A148 → A146 → A151 → A866 → A054 → A846 → A411 → A420 → A408 → A445 → A153이다.
로터리 본 구간은 A148/A146/A151, 고주로는 A411부터 A153 끝이다.
로터리의 판제 접근/준비는 실제 정지선 lookahead 때문에 앞선 A304에서 시작할 수 있다.
합류 관련 주변 링크 A147/A133/A134도 관측 판단에 고려한다.
객체가 첫 미션 링크에 들어온 뒤에야 사용 준비를 시작하지 않는다.

신호등 구간의 전역경로는 A236 → A219 → A215 → A213 → A222 → A220 → A202 → A728 → A599 → A207 → A205 → A315 → A318 → A308이다.
검토한 신호 정지선 링크는 A219/A213/A728/A315/A308이다. 지도 정지선 수를 공식 채점 교차로 수와 동일시하지 않는다.

### 차선변경과 음영 준비

| 고주로 전역경로 링크 | 검토된 같은 방향 인접 링크 |
|---|---|
| A411 | A409, A410, A418 |
| A420 | A430, A431 |
| A408 | A434, A435 |
| A445 | A422, A423 |
| A153 | A432 |

인접 관계는 차선변경 허가가 아니다. A153/A432의 합류·톨게이트 제약과 판제 안전 판단을 유지한다.
차선변경 중 구간 판단은 전역경로 station/corridor와 검토된 주변 링크를 함께 사용한다.
전역경로 link ID에 속하는지만 검사해 인지 소비를 끊지 않는다.
음영 대응 A131은 반대 방향 차로이며 일반적인 동일 방향 좌/우 인접 링크와 구별한다.

공식 팀 음영은 **A448 끝부터 A126 끝**, 즉 A126이다. A448 길이는 약18m라 30km/h에서
약2.2초뿐이다. 향후 STANDBY를 쓰더라도 A448 시작을 고정 cold-start 시점으로 삼지 않고,
실제 준비 시간과 속도를 고려해 A446 등 앞선 접근에서 준비한다. 현재 기본은 상시 ACTIVE지만
모델 로드만으로 인계 준비를 인정하지 않는다. 진입 전에 새 family의 유효성·freshness를 확인한다.

현재 판제는 GPS latitude/longitude/altitude가 모두0이면 JAMMING으로 판단한다.
이는 통신 중단/stale 전체를 검사하는 기능이 아니다. GPS 무효 때 과거 HD맵 station을 현재 위치로
재사용하거나 이를 근거로 A128 진입을 확정하지 않는다. 음영 이탈·복구·fallback은 판제 책임이다.

## 실행·변경 책임

```bash
# 각 터미널에서 같은 workspace 환경을 source. bridge/Main은 별도 한 세트 유지.
roslaunch asmc_camera camera.launch
# 별도 판제 터미널: 카메라가 신호등도 소유하므로 별도 ResNet은 시작하지 않음
roslaunch asmc_planner autonomy_no_bridges_v2.launch
```

**신호등 추론·발행의 단일 소유자는 카메라 담당의 `asmc_camera`다.** 판제 launch는
별도 ResNet을 시작하지 않으며 `traffic_light:=false` 인자도 필요하지 않다.
기존 `traffic_light`, `traffic_light_device`, `traffic_light_checkpoint`, `camera_topic` 인자는
판제 launch에서 제거했으므로 예전 호출에서도 삭제한다. 판제의 신호 판단 규칙과 typed/legacy
소비 정책은 유지한다. 현재 판제는 카메라 모드 서비스를 구간별로 자동 호출하지 않는다.

실행 중 신호등 추론을 잠시 중단하거나 재개하려면 카메라의 기존 모드/주기 API를 사용한다.
이는 ROS parameter를 바꾸는 것과 다르며, 시작 YAML은 실행 초기값을 제공한다.

```bash
# 필요한 경우만: 신호등 모델은 로드한 채 추론 중단
rosservice call /perception/camera/traffic_light/set_mode '{mode: 0, target_hz: 0.0}'
# 현재 주기를 유지하며 재개 (기본10Hz)
rosservice call /perception/camera/traffic_light/set_mode '{mode: 2, target_hz: 0.0}'
```

중단 중 신호는 미검출/무효·stale에 따라 unknown으로 처리한다. 판제 신호 규칙 자체를
끄는 기능은 아니다. 모드 제어 권한은 카메라 담당과 합의해 하나로 유지한다.

`mode:=standby/off`, 모델 조합, 공용 `SetInferenceMode` 서비스는 명시적인 중단·복구·시험에
유지한다. 낮은 수준의 managed component와 구형 `perception.launch profile:=planning`은
기존 STANDBY 계약을 유지하며 공개 `camera.launch`의 ACTIVE 기본과 구별한다.
필요하면 STANDBY → PREPARE → 같은 세대 prepared/pipeline_ready 확인 → ACTIVE로 복구한다.
모드 요청자는 하나만 둔다. 예제 구간 제어기는 기본 미실행이며, 함께 제공하는 설정도
세 모델 상시 ACTIVE를 요청한다. 구간 제어는 판제와 합의 후 별도 설정으로 변경한다.

metadata의 신호등 `runtime_activation_hint`/`disable_outside_zone`은 이전 활성화 제안이다.
구간 기하·규정 기록은 보존하지만 현재 카메라 실행은 이 힌트를 자동 적용하지 않는다.
새 소비자를 연결할 때 이 필드를 생산자 발행 중단 명령으로 사용하지 않는다.

상시 family 출력은 음영 밖 주행 승인과 다르다. 모델 모드·결과 validity·원본 stamp·
freshness·session/generation·실제 ODD를 확인한다. 관측이 끊겼다는 이유로 LiDAR 객체를
삭제하지 않으며, 소비 그룹을 실제 운동 상태로 사용하지 않는다.

현재 상세 계약은 [카메라 실행](../contracts/camera-runtime-v1.md),
[음영 family 인계](camera-to-planning.md), [카메라→LiDAR](camera-to-lidar.md),
[전체 실행 명령](../operations/commands.md)을 따른다.
[기본값 변경 검사](../records/camera/2026-10-16-continuous-inference.md)는 짧은 동작 확인과 전체 수락을 구분한다.
