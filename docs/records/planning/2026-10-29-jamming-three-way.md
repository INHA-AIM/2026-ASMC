# camera·LOCAL·localization 비교 통합 기록

## 기준과 보존

비교 브랜치는 `integrate/jamming-three-way-20261029`, 작업트리는
`2026-ASMC-jamming-compare`다. main 또는 기존 연구 작업트리를 수정하지 않는다.
실행 안내는 [세 방식 비교](../../operations/jamming-comparison.md)를 정본으로 한다.

| 입력 | 고정 revision | 의미 |
|---|---|---|
| 최초 main | `69e66d8837f7a35315bc21592456cf3aff16d5df` | camera #137 포함 |
| localization | `711fe7c65eb46ddd474d8eb2b93f538e3cf0f8a2` + 보존한 WIP | 준비된 인계, 위치측위, Frenet 회피 |
| LOCAL | `d7095a00f9175c58ae9ddd5a9ae838db08822822` | 실제 발행 명령 이력, 관측 공간 복구 |
| 재부팅 전 복구 스냅샷 | `9b1fb227d814ea6b61109c2d57d7f50ea14a7021` | 완료 수락이 아닌 별도 archive commit |
| 재개 후 통합한 main | `af911727d2001703ea43a7d6c394d0650e74c489` | #140; 정적 UNKNOWN 정책·실측 모델 필터·runner 개선 보존 |

LOCAL 작업트리에는 이후 별도 변경이 진행됐다. `d7095a00` 이후 변경을 암묵적으로
포함하지 않는다. 원격 LOCAL PR head와 로컬 연구 head를 구분한다. 기존 #138/#139는
이 비교 브랜치의 배포/수락을 대신하지 않는다.

## 통합 책임과 선택

```mermaid
flowchart LR
  I[공통 bridge·camera·LiDAR 입력] --> N[일반 Frenet·MPC]
  I --> M[공통 A448 접근·모드·인계]
  M --> C[camera: 기존 추종기]
  M --> L[LOCAL: 관측 공간 경로·공통 MPC]
  M --> P[localization: map 위치·Frenet 경로·공통 MPC]
  N --> S[공통 selector·250ms watchdog]
  C --> S
  L --> S
  P --> S
  S --> O[/ctrl_cmd 단일 발행]
```

세 재밍 구현 중 선택한 하나만 launch한다. NORMAL 준비와 인계 후보 계산은 제한된
기간에 수행하며, 후보 계산을 실제 발행 명령 이력에 넣지 않는다. 상대이동·운동 근거·
MPC 실행 코드를 공유하되, LOCAL의 GPS 독립 yaw 정책과 localization의 map anchor,
Frenet 시간 매개화는 명시적인 프로필로 구분한다. camera 추종기는 교체하지 않는다.

A448 시작까지 접근 목표 25km/h이며 현재 camera 목표 10km/h, LOCAL/localization 순항 목표 15km/h·상한 20km/h다. GPS 건강 상태와 제어 소유권, 페인트 횡단 허용과 벽 확장 권한은
분리한다. 복구 속도 5km/h는 검증된 경로의 목표이며 막힌 공간에 강제로 적용하지 않는다.

## 재부팅 전 확인한 범위

증거 루트는 D의 `artifacts/jamming-three-way-20261029-r02`다. 최초 미완료 시도의
`r01`과 과거 두 연구 브랜치의 동결 run은 보존한다. 아래 결과는 해당 로그의 범위만
입증하며 실제 통합 주행 수락으로 해석하지 않는다.

| 검사 | 근거 | 결과 |
|---|---|---|
| 통합 runtime와 필수 C++ 타깃 빌드 | `build-r10.log` | 빌드 완료 |
| LOCAL/측위 입력/Frenet 정적 우회·복귀/벽 권한 | `core-tests-r4.log` | 통과 |
| 세 방식 A448 진입과 준비된 인계 | `handoff-tests-r1.log` | ROS 시험 6회 통과 |
| 기존 selector·GPS 복구·측위 초기화·시각 역순 | `controller-regressions-r1.log` | ROS 시험 6회 통과 |
| 계약·문서·dashboard/포트·JS 회귀 | `contracts-r4.log` | 완료; 최종 통합 후 재확인 필요 |
| 세 옵션 실행 그래프 | 재부팅 전 graph 검사 | 선택하지 않은 재밍 노드 없음 |
| 실제 MORAI 세 방식 주행·웹 재접속 | 미실시 | 미검증 |

LOCAL 속도 매개화를 Frenet 경로에 그대로 적용했을 때 기존 정적 우회/복귀가 깨지는
회귀를 발견했다. 확인되지 않은 제동 경로 보완 시도는 제거하고 동일 속도 계획 모듈 안에
기존 Frenet과 LOCAL 프로필을 보존했다. 실패 로그도 남겼다. 측정된 벽 확장과 정지한
person/dog의 우회 허용은 단위 시험을 통과했지만 실제 센서 주행 수락은 별도다.

## 재개와 남은 수락

사용자가 MORAI의 객체 배치 장면을 `test1`로 저장하고 시뮬레이터를 닫았다. 시나리오
파일의 실제 위치와 로드 결과는 아직 독립 검증하지 않았다. 재부팅 전 두 비교 컨테이너만
종료했고 코드, merge/index, 환경, 컨테이너 로그, incremental bundle과 체크섬을 보존했다.
저장 위치 pointer는 워크스페이스의
`.storage/manifests/jamming-compare-integration-20261029-r1/reboot-current.json`이다.

재개 후 순서는 최신 main 차이 통합, 최신 소스 재빌드, camera/LiDAR/공통 전환 회귀,
실행 그래프·MD5·실제 모델 확인, physical lease와 시뮬 상태 확인, 동일 조건 주행이다.
`test1`의 초기 위치·정적 에셋·배치를 확인하기 전 자동 fixture와 같은 장면이라고 보지 않는다.

FOGGY 13시·같은 출발/배치/센서를 고정하고 기본 속도 10/10/15 및 동일 속도 10/10/10을
구분한다. 정지 시간·이유, 열린 경로에서 3초 이상 남는 정지, 인계 공백, 계산 비용,
native/UDP/차량 반응/영상의 충돌 근거를 함께 기록한다. 실제 통과 공간 부재 또는 입력
소실은 강제 주행으로 숨기지 않는다. 아직 집중 튜닝할 한 방식을 선정할 주행 근거는 없다.

## 재개 후 결과

`5423d781`에서 두 연구 이력을 통합하고 `a3a87ea5`에서 main #140을 반영했다.
`4a81c285`는 준비 실패 뒤 재진입에서 남던 selector 소유권 래치를 수정했다. mode가 먼저
도착하고 요청 취소가 나중에 도착하는 순서에서 selector 내부가 NORMAL에 남아 새 JAMMING
명령을 받을 수 없던 문제다. 공표된 target mode를 반영하되 유효 후보 전에는 계속 정지한다.
watchdog250ms와 epoch·시각·생산자 검사는 완화하지 않았다.

재개 후 runtime·평가기 빌드, essential planning/contracts, camera core, 세 옵션 인계와
selector ROS 7회, 지면/터널/점 기반 refinement/자세·source-time/runtime 검사를 통과했다.
근거는 첫 run의 `handoff-selector-fixed-r1.log`, `lidar-regressions-final-r2.log`,
`contracts-final-r1.log`와 재개 빌드·planning 로그다. 실패한 이전 로그도 보존했다.
LiDAR 설정의 origin/main 대비 차이는 `camera_point_refinement.enabled: true` 한 줄이다.
정본의 터널 처리 ON과 지면 far-band fit을 유지하고, 선택적인 좁은 fit은 기본 활성화하지 않았다.

공통 메시지 MD5는 LocalTrajectory `d95a161dd0c92f0250a150ed1bbc40c3`,
ControlHandoff `51dfecdc62c2d50090bf1006f5c885f5`로 유지됐다. #138/#139는 재개 점검에서도
OPEN·미병합이며 원격 head는 각각 `e95dba0e`, `1fdd6c02`다. 이 PR들을 덮어쓰지 않는다.

사용자가 다른 작업의 물리 시험 점유를 확인했으므로 실제 `test1` 주행은 하지 않았다.
초기 읽기 전용 조회에서 15개 native actor와 터널의 정지한 static asset을 확인했다.
차량·객체·환경을 변경하지 않았다. 시험 CLI의 `--preserve-scene-label`은 로드된 정적
배치를 관측만 허용하며 다른 fixture 생성/주차와 함께 사용할 수 없다.

별도 등록 run `jamming-three-way-replay-20261029-r01`에서 원시 GPS/IMU/CAN/LiDAR·4뷰 JPEG
55.24초를 재처리했다. 세 옵션이 같은 컨테이너에서 자기 camera/LiDAR와 단일 selector를
실행하고 선택하지 않은 제어기가 없음을 확인했다. bridge나 SDK 제어 연결은 시작하지 않았다.
`replays/center-raw-three-way-r07`이 입력 생산자·진단 기록 검증까지 마친 회차다. 재생에서만
`/jamming_raw_input`을 명시적으로 허용하고 운영 `/morai_udp_bridge` 계약은 유지했다.

| 재생 결과 | camera | LOCAL | localization |
|---|---|---|---|
| 실행·선택 그래프 | 확인 | 확인 | 확인 |
| 모드 수신 | NORMAL→JAMMING→NORMAL | NORMAL→JAMMING→NORMAL | NORMAL→JAMMING |
| 남은 진단 | 준비된 인계 1회만 관측 | scene/source stale·계획 deadline 다수 | 자세/state stale 뒤 정지 재기준점 대기 |

이 재생은 CPU2·6GiB 제한 및 공유 GPU에서 원래 기록된 자차 운동을 고정했다. 명령으로
자차가 반응하지 않으므로 새 경로의 완주·충돌·주행시간이나 GPS 정지 재초기화의 수락을
판정할 수 없다. 특히 localization의 일반 제어 복귀는 이 회차에서 통과하지 못했다.
stale를 무시해 성공으로 만들지 않았다. 실시간 자원 조건과 닫힌 고리 주행에서 재확인이
필요하며 세 방식의 우열 또는15/20km/h 성능을 이 기록으로 선정하지 않는다.

8089/8097 수동 콘솔은 master11373에 읽기 전용으로 복구했다. Windows HTTP200과 실제
브라우저에서 지도·진단 패널·LiDAR canvas를 확인했다. 전환 상태바의 flex 배치가 본문을
밀어내던 오류(`e22c3831`)를 수정했다. 동일 source/master 콘솔만 재사용한다. 자동 replay는
별도11374·8087/8098이며 role·run·trial과 재접속 identity를 검증했다. 종료·heartbeat 소실 시
이전 현재 위치와 경로를 숨긴다. 수동 콘솔의 WAIT는 입력 없음이며 주행 수락을 뜻하지 않는다.

진단 수집기의 동시 JSON 쓰기 손상을 r05에서 발견했다. 원시 bag과 손상된 진단 원본은
보존하고 callback 쓰기 잠금·회차별 recorder 바인딩을 적용했다. r06의 실행기 이름 충돌도
보존했다. 최종 r07은 모든 UTF-8/JSON 줄과 topic별 수신 개수의 일치를 검증했다.
배포·실주행 지연을 이 CPU 제한 재생의 메시지 개수나 brake 수신 개수로 대신 평가하지 않는다.

선정 가능한 실주행 근거는 아직 없으며, 후속 비교는 같은 `test1` 배치·FOGGY13·센서와
기본10/10/15 및 동일10km/h 조건에서 수행한다. `test1` 객체를 자동 fixture로 덮어쓰지 않는다.
콘솔 복구·제어 명령 그래프·실패 진단은 ASMC 갤러리 **E746**에 게시했다:
http://localhost:8000/jamming-three-way-replay-20261029-r01/review-r4/index.html
그림 생성 코드와 모든 재생·실패 근거는 해당 run에 보존하며 기존 갤러리 번호는 유지했다.

## 공용 구조·설정 정리와 반복 정지 탈출 — 23시 후속 단위

소스 체크포인트는 `929ec998`이다. LOCAL 후속 WIP와 실제 세 방식 비교는 사용자의
완료 통보 이후로 보류했다. 다른 작업트리와 물리 시뮬레이터는 변경하지 않았다.

공통 데이터·상대이동·차체 검사·속도 계산을 `jamming_common`과 `asmc_jamming_common`으로
분리했다. LOCAL 검색과 localization의 map/Frenet 책임을 별도 라이브러리로 유지하고
기존 LOCAL include를 호환 헤더로 보존했다. ROS 메시지와 단일 selector 계약은 유지한다.

LOCAL/localization은 순항15·상한20km/h, 세 방식과 NORMAL의 차체 폭은1.892m다.
LOCAL30cm, localization 최종10cm/native45cm, 별도 도로 외곽 폭2.161m와 벽65cm 여유는
유지했다.3km/h 고정 복구는 후속 사용자 지시로 취소했다. 새 관측이 계속 들어오는
5초 정지가 두 번 누적되면 좌우9개/최대8m 탈출 후보를 우선 검사한다. 동적 전방 충돌은
양보하며, LOCAL raster/OBB 또는 localization map/native 여유·현재/예측 물체 검사를
통과한 후보에만 최대5km/h의 저속 계획을 준다.2m 진행/epoch/입력 공백은 누적을 해제한다.
통과 공간이 실제로 없거나 입력이 무효인 상태를 주행 가능하다고 바꾸지 않는다.

근거는 등록 run `jamming-common-policy-20261029-r02`다. 초기 등록 구성의3km/h 요청과
그 뒤 변경 지시를 구분해 `effective-config.json`을 남긴다. 첫 빌드/실패 로그도 보존한다.

| 검사 | 최종 근거 | 결과 |
|---|---|---|
| 전체 runtime·변경된 두 planner·공용 MPC/시험 빌드 | `build-r4.log`, `build-final.log` | 통과 |
| 계획부 essential CI | `essential-planning-r4.log` | 통과 |
| 세 옵션 준비된 제어 인계 | `handoff-camera/local/localization.log` | 각각 통과 |
| 공용 속도·실측25km/h 인계·차폭/여유·반복정지/동적양보/실제 raster-OBB 우회 | `policy-build-test-r3.log` | 통과 |
| LOCAL/측위/벽 권한 및15km/h 합성 Frenet 장면 | `core-final.log` | 통과, 실제 시뮬 주행 수락 아님 |
| launch/관측 포트 guard Python 검사 | `python-tests-final.log` |13개 통과 |

차폭 변경으로 기존 hard/soft 경계 fixture의 실제 간격이 바뀌어 좌표를 조정했다.
순항15가 새 지배 조건이 된 시험은 상한까지 순항을 요청하는 별도 조건에서 정지 거리
제한을 검증한다. 회복3km/h 고정 assertion도 설정된 탈출 상한 검사로 바꿨다. 입력 유효성,
실제 장애물·노면 veto나 watchdog을 낮춰 통과시킨 결과가 아니다.

현행 설정 설명과 경계는 `docs/operations/jamming-parameters.md`를 따른다.
실제 `test1`·FOGGY13·전환/복귀/정지 회복과15/20km/h 반복 주행은 미검증이다.

## 10월 30일 통합 체크포인트: 선택과 미선택

이 항목은 이전 기록에 이어 수행한 결과다. 새 기준 main은 `af911727`,
이 단위 시작은 `ff9650fb`, LOCAL 완료 원본은 `5f9e1cfa`, #138 원격은
`e95dba0e`, localization 원본은 `711fe7c6`과 별도 WIP다. 원본 worktree는
변경하지 않았다. 변경 전 incremental bundle은 개인 workspace의
`.storage/jamming-pr141-pre-local-20261030-ff9650fb.bundle`이며 원래 base가 필요하다.

| 판단 | 변경과 이유 | 상태 |
|---|---|---|
| LOCAL seed | `8df69a20`의 관련성·거리 정렬, 중복 offset 제거, 6개 제한을 현재 LOCAL 검색에 이식 | 채택, `43627a06` |
| LOCAL 방향 선호 | 검증된 경로의 현재 LiDAR known-class에 대한 비용만 선택적으로 적용 | 부분 채택, `ec430aeb`; 기본 0, 8은 실험값. 원본 20–40m camera preview와 동일 구현 아님 |
| camera guard·quiet bypass | 시야 상실이 통과 증거가 아님 | 미채택; 3회 빈 검출만으로 대상 해제 금지 |
| semantic OBB recovery, KF/static 예외 | 큰 unknown 제거·운동 확정 근거 부족 | 보류, OFF; 시각·속도 validity 유지 |
| 벽·지도 | localization의 이미 통합된 구간 한정 확장과 벽 권한 유지 | 유지; 실제 정체의 원인이라고 단정하지 않음 |
| 차량·여유 | `2.161=1.892+2×.1345` 동등성 검증 및 LOCAL 도로·기동 임계 분리 | 채택, `d529d25c`·`d9244d90`; 도로/물체 veto 별도 |
| 단일 운영 | 최신 main의 11311/8088/8091, camera 기본 및 세 옵션 | launch 기본 복원 `2930c4f4`; 실행 중 타 컨테이너는 변경 안 함 |
| #125/#126 | 관측시각·IMU 단일 투영·sweep·후보 누락·접촉 교훈 | 현 구현/시험과 대응, 오래된 추종기·설정 복원 안 함 |
| #138/#139 | #141이 관련 구현 일부를 통합·대체하지만 전체 기능 등가/완전 검증은 아님 | OPEN 보존; #141 병합 후 누락 검토를 거쳐 별도 종료 여부 결정 |

LOCAL의 rate yaw와 localization의 orientation yaw·anchor/DR/GPS 복구, 서로
다른 Frenet/LOCAL 속도 프로필을 유지한다. LOCAL 후보 시험의 코드는 공통 polygon
검사를 재사용하지만 raster 관측 근거를 지도/Frenet과 동일시하지 않는다.
서로 다른 제원 전체의 단일 canonical loader 이관은 완료하지 않았다. LOCAL 명시
제원 충돌과 native 도로 총폭/reserve 불일치는 오류 처리하지만 모든 옵션에서
제원 정본 충돌을 포괄적으로 검사하는 것은 후속 과제다.

### 격리 검증과 동일 입력 재생

D run `jamming-policy-freeze-20261030-r01`의 계획 CI, CTest 7개,
launch 계약·단일 제어 소유권 검사는 `d9244d90` 실행파일에서 통과했다.
`db48eba0`의 LOCAL 시작 오류 진단만 바뀐 뒤 LOCAL 회귀도 다시 통과했다.
옛 폭 2.160835 설정을 넣으면 기대 폭 1.892를 표시하고 시작을 거절했다.
문서·CI만 이후에 변경한 revision과 실제 빌드 revision은 동결 manifest에 따로 기록한다.

D run `jamming-local-replay-20261030-r01`은 원본 bag
`local3kmh-20261030-drive13/sensors_0.bag`, SHA256
`27b53bb5c7990c46193487a635d816a0363ae4742aa92ded322294a7809b9f91`의
동일 자차 운동 입력을 30초씩 재생했다. weight 0/8 각각 valid 293/291,
LOCAL critical path p95 62.50/46.40ms였다. 각 재생의 타이밍이 다를 수 있고,
동일 scene/lidar stamp로 대응된 유효 221 프레임 중 72개에서 경로 길이가
10cm 초과로 달랐다. 실제 폐루프의 우열이나 독립 시험으로 해석하지 않는다.
기존 원본 LOCAL의 더 큰 차폭과 guard/원거리 예고는 이 A/B에 포함되지 않았다.
비용 8의 운영 승격 근거가 부족하므로 0을 기본으로 남긴다.

### 실제 MORAI 대표 시험

새 run `jamming-compare-drive-20261030-r01`에서 FOGGY 13시, 동일
source·센서·start station 1830·10km/h 상한을 사용했다. 원래 test1 전체를
그대로 재배치하지 않았다. 비정적 actor가 있어 full-preserved 검사는 차량/객체
조작 전에 거절됐고, 기존 배치를 유지하면서 부분 진입(1878)과 정적 장면(1905)
두 구간을 시험했다. dynamic actor가 관측 범위에 들어올 가능성은 배제하지 못한다.
각 셀은 1회이며 시간은 완주 우열을 결정하기에 부족하다.

| 구간 | camera | LOCAL | localization |
|---|---|---|---|
| 진입 station 1878 | 24.00초 완료 | 25.13초 완료 | 23.29초 완료 |
| 진입 제어 공백 최대 | 55.7ms | 110.8ms | 63.4ms |
| 정적 장면 station 1905 | 54.44초 완료, 최장 정지 2.60초 | 42.68초 완료, 최장 정지 0초 | station 약1889, 11.97초 정지 후 중단 |
| 정적 장면 인계 공백 | 51.8ms | 101.1ms | 63.1ms |

LOCAL 정적 장면은 near-field recovery 및 assisted sweep을 사용했으며 기존
4m 근거리의 일부 미관측/내부 비노면 완화가 존재한다. camera의
`creep_clear_distance=0`에서는 해당 정적 객체 creep 확인이 비활성이다.
localization 정지는 fresh 객체 3개, DR, 탈출 요청 이후 후보 0개이며
`shadow_native_blocked`와 `stall_escape_no_observed_opening`이 기록됐다.
벽, 맵, OBB 중 원인은 확정되지 않았다. 100ms 인계 목표는 LOCAL 두 회차에서
미달했고, 이 시험의 측정 공백은 모두 250ms 미만이었다.

UDP 원시 충돌은 각 회차에서 `(-1,0)`만 수신됐고 일부 표본은 동시 자차 위치와
일치했다. native 목록은 비어 있었으나 이를 무충돌 확정으로 취급하지 않는다.
원시 bag·UDP 이벤트·native 상태·scene을 보존했다. 시험 복구 오류는 기록상
없다. 총 주행·터널 출구/GPS 복귀·동적 횡단·25km/h 접근·15/20km/h 수락은
미실시다. 같은 배치에서의 반복과 속도별 수락 전에는 주력 방식 선정을 보류한다.

원시 로그 및 표시(유효 양수 계산 표본의 p95만 포함):
http://localhost:8000/jamming-compare-drive-20261030-r01/review-r2/index.html
조건과 원본 경로는 해당 run의 registration과 각 trial manifest를 따른다.
