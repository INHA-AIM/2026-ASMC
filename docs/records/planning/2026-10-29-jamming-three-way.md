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
| 재개 시 확인한 main | `af911727d2001703ea43a7d6c394d0650e74c489` | #140; 후속 통합/검증 대상으로 기록 |

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
