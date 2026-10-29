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

A448 시작까지 접근 목표 25km/h, A448 시작부터 camera/LOCAL 10km/h,
localization 15km/h다. GPS 건강 상태와 제어 소유권, 페인트 횡단 허용과 벽 확장 권한은
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
