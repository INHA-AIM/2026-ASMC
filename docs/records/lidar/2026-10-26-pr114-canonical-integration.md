# PR #114와 B4 정본 실행 통합 — 2026-10-26

## 소스와 선택

최신 main `4fa0857d8675fe06e3e8457e7f2b56fdc7e8eba0`(윤태의 PR #114)에 B4를 리베이스했다.
이전 PR #112 head `8d903141511e88b7ebe2ea8a540f420a54bd94e3`는 로컬
`backup/lidar-canonical-before-pr114-20261026`에서 보존한다. 기존 B4 고정 bag run은 동결 상태다.
그 결과를 PR #114의 새 속도 품질 경로와 완전히 동등하다는 근거로 재사용하지 않는다.

사용자 결정대로 B4의 단일 namespace·책임 구조와 개별 실행을 유지한다. 최신 main의 KF 속도
품질, 미확정 속도 점유 보존, 세션별 판제 ID, GPS 복구와 재밍 출구 처리는 보존한다.
판제/controller 계산 소스는 main에서 바꾸지 않았다. 구 Main/Msg·wire/JSON join·중복 인지 필터는
현행 실행에 복원하지 않았다.

## 팀원이 따라야 할 연결

| PR #114 당시 | 이번 정본 |
|---|---|
| `/migration/lidar/planning_observations_map` companion | `/perception/lidar/planning_observations_map` 정본 |
| `asmc_lidar_planning_adapter` 별도 실행 | 같은 gate를 `planning_objects_adapter`에서 사용 |
| `planning_output:=true` | 별도 스위치 없이 판제 구독 수요에 따라 생성·발행 |
| `lidar_source_policy` / `lidar_backend` 선택 | 제거. 시험 입력 topic remap만 허용 |
| `autonomy_asmc_lidar.launch`가 인지+판제 시작 | `lidar.launch`와 `autonomy_no_bridges_v2.launch`를 각각 한 번 실행 |

`LidarPlanningObservation`은 PR #114의 additive 공용 타입이다. 기존 공용 메시지 MD5는 보존한다.
private `DetectedObject.velocity_unavailable` 추가로 전체 재빌드와 소비 노드 재시작이 필요하다.
미확정 속도를 유효한0/LOW_THREAT로 바꾸지 않고 기하와 UNKNOWN·validity를 함께 넘긴다.
일반 `tracks_local/map`은 기존 classifier 속도와 tracking 계약을 유지하므로 판제 KF 출력과 다르다.
둘을 topic 이름만 바꿔 혼용하지 않는다. 최종 필터 kept ID는 두 출력 모두 동일하다.

## 사용자 환경 재빌드와 제한 주행

기존 `asmc-ros-noetic`·마운트·ROS master·UDP bridge를 유지했다. 정차를 확인한 판제만 정상
종료한 뒤 `/root/ws/build`·`/root/ws/devel`로 전체 whitelist/blacklist를 해제해 재빌드했다.
카메라와 기본 `fusion=true debug=false rviz=false` LiDAR를 한 세트씩 시작했다.

| 조건/결과 | 측정 |
|---|---|
| 지도 publisher launch | OFF. `/hd_map/*` publisher 없이 실행 |
| LiDAR debug·별도 웹/trace | OFF |
| 카메라 | 기본 세 모델 ACTIVE |
| 시험 | 팀의 검토된 시작 위치로 Ego만 이동 후 기본 autonomy 실행 |
| 제한 주행 | 10.52초, 경로 station 증가65.75m |
| 최대 판제 속도 | 11.12m/s |
| 판제 위치→전역경로 최근접 표본 거리 최대 | 0.1447m |
| 수신 | private 객체149·공용 관측181·제어272개(준비·종료 포함 전체 구독 기간) |
| 종료 | 시험 autonomy 정상 종료, 원래 Ego 위치에 정차 복원 |

거리는 GPS/IMU 판제 위치와 0.25m 간격 경로 표본의 내부 정합 수치이며 독립 GT 횡오차가 아니다.
짧은 연결 확인을 전체 미션·속도 정확도·터널·동적 객체 수락으로 확대하지 않는다. 날씨·센서·네트워크·
액터는 바꾸거나 새로 생성하지 않았고 bag을 추가 수집하지 않았다. 개발용 RPC는 시험 이동/복원에만 썼다.
자동 생성 ROS 로그는 기존 컨테이너에 남기며 대용량 저장·동결 run을 변경하지 않았다.

판제 웹8088의 `/web/index_v2.html`과 LiDAR 웹8091의 `/`를 실제 재빌드 환경에서 차례로
실행해 HTTP200을 확인하고 각각 소유한 launch만 종료했다. 두 웹의 포트·ROS 이름이 다르며
자기 API를 상대 경로로 조회한다. 주행·인지 생산자를 새로 시작하는 웹이 아니다.

## 사용자 직접 주행 확인

2026-10-26 사용자가 재빌드된 정본 실행으로 직접 주행한 뒤 정상 주행을 확인했다.
이는 사용자 확인이며 별도의 계측 수치·전체 미션 수락으로 확대하지 않는다.
기본 실행 안내는 인자 없는 launch 명령으로 통일하고 모델 선택·수동 모드·디버그 옵션은
[명령 모음](../../operations/commands.md)의 별도 표에서 안내한다.

## 자동 회귀와 시험 실행 문제

- LiDAR CTest34/34, 새 공용 관측 gate5개·기존 변환/그룹6개, 실제 ROS 연결·전환,
  debug OFF 판제 단독 구독·autonomy 경계 smoke·웹 JS68개 통과.
- 카메라 unit/ROS·bridge 그룹 통과. 기존 차선5개 fixture 통과.
- 신규 frame bridge fixture는 첫 출력 구독이 연결되기 전에 첫 입력을 보내 손실됐다.
  출력 subscriber 연결까지 확인한 뒤 시작하도록 시험만 수정했고 동일 사례가 통과했다.
- 시나리오 unit218개 통과, integration112개 중1개 환경 조건 생략. 생략은 통과로 세지 않는다.
- 호스트 Python에는 scipy/protobuf/gRPC가 없어 시나리오 전체 실행이 실패했다.
  기존 의존성이 준비된 ROS 컨테이너에서 위 결과를 확인했고 운영 패키지를 설치/업그레이드하지 않았다.
- 첫 로컬 LiDAR 그룹은 명시적 시험 빌드의 devel 경로가 root로 바뀌어 바이너리를 못 찾았다.
  prefix를 분리 경로로 고정해 재빌드·재실행했다. 그 실패 로그는 삭제하지 않았다.
- 판제 전체 시험의 첫 실행에는 이전 구조의 gtest 실행파일과 새 라이브러리가 섞인 ABI 증상이
  나타났다. 일반 빌드가 제외하는 `tests` 타깃까지 명시적으로 재빌드한 뒤
  판제64/64(309.33초)·제어기18/18(7.51초)이 통과했다. 이전 실패를 동작 수정으로 숨기지 않았다.

CI는 기존141개와 PR #114 신규4개 명령/옵션 조합을 보존 검사한다. GPS 복구와 private 객체
소비 회귀도 명시적으로 빌드·실행한다. 새 hosted CI 시간/수락은 해당 head 결과로 별도 기록한다.
현재 실행 명령과 지도 launch 역할은 [실행 정본](../../../src/asmc_lidar/docs/planning-runtime.md),
[명령 모음](../../operations/commands.md)을 따른다.
