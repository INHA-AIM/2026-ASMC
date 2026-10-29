# LOCAL 옵션 배포 · 2026-10-29

기준 main은 PR136 `fad73f82`다. LOCAL 이력에 이미 포함돼 rebase는 up-to-date였다.
원본 `feat/shadow-local-footprint`, `archive/local2h-before-deployment-20261029`(d4bed659),
CI 미커밋 파일과 문서를 보존했다. 별도 `feat/local-planner-deploy-20261029`에서 배포한다.
기존 원격 브랜치를 force-push하지 않는다. main 병합은 사용자가 결정한다.

## 선택과 공통 변경 경계

- `a374be7d`의 자유 경로, family 비동기 수신 안정화, 실측 속도 계획/복구를 기준으로 삼았다.
- 사용자 요청으로 저속 회복 목표를5km/h로 설정했다. 강제 최저속도가 아니며 검증된 공간·조향·
  실제 감속·충돌은 더 낮은 속도 또는 정지를 요구할 수 있다. LOCAL 목표 상한은10km/h다.
-5개 가이드/centroid/front-only/일시 노면 잡음/6m 복구/이전 경로 유지 후보는 우월성이 없어
  배포 코어에서 제외했다. 원본 연구 branch와 run23–30, 원본 커밋/설정은 보존한다.
- main의 LiDAR 기본 YAML을 유지한다.0.10m 분리 fitting 지원 코드는 과거 재현을 위한 opt-in이며
  전역 활성화는 제거했다. 앞선 실험과 배포 검증을 같은 설정으로 취급하지 않는다.
- 일반 Frenet SpeedProfiler의 Preview/Step은 바꾸지 않았다. LOCAL용 물리 상태 전개 함수만 추가했다.
- mode source30m 사전 준비와 selector100ms 인계는 local 선택 때만 동작한다. camera 기본은 유지한다.
- 공용 LocalTrajectory는 새 타입이다. 기존 타입 필드·MD5·모델/엔진은 변경하지 않았다.
- essential/runtime build graph에 LOCAL 두 실행파일과 핵심 입력/소유권 검사를 포함한다.
  main workflow·캐시·기존 검사·팀 빌드 profile은 과거 버전으로 교체하지 않는다.

## 실행·실제 검증

[명령 정본](../../operations/commands.md#local-배포-선택과-기본-방식-복귀--2026-10-29)을 따른다.
기본과 LOCAL은 같은 camera.launch, lidar.launch를 한 세트 쓰며 autonomy의 선택만 바꾼다.

```bash
roslaunch asmc_camera camera.launch
roslaunch asmc_lidar lidar.launch
roslaunch asmc_planner autonomy_no_bridges_v2.launch jamming_planner:=local
```

기본으로 복귀할 때 현재 autonomy를 종료한 뒤 옵션 없는 autonomy 명령을 실행한다.
두 autonomy를 동시에 실행하지 않는다. LOCAL 선택은 기존 BBox follower를 시작하지 않는다.
이 절의 run34–38은 당시 공유/격리 master 시험 결과다. 후속 단일 컨테이너 실행은 아래 절을 따른다.
자율 시험은 전용 master와 전용 bridge,
공유 physical lease를 사용하고 시험 소유 프로세스만 정리한다.

사용자 기본 컨테이너와 localization worktree는 수정하지 않았다. LOCAL 컨테이너는
`asmc-local2c-20261028`, `/root/ws`는 `2026-ASMC-local-integration`이다. LOCAL 전용 build/devel
자산만 재사용·재구성했다. 실제 rospack 경로·source/binary SHA·대화형11311을 배포 기록에서 검사한다.

run34는 최종5km/h 회복 목표·main LiDAR 설정에서 주차 장애물60.18m/48.04s를 통과했다.
run35는 일반↔음영 LOCAL, run36은 같은 구간의 기본 camera를 검증한다. 완료 여부·거리·
정지·전환 결과는 각 trial-result/events/summary가 정본이다. run31은 개인 SDK 설정이 없어
시뮬 접속 전에 실패했으며 별도 aborted 기록으로 보존한다. 일부 구간 성공을 전 구간 수락으로
표현하지 않는다. 전체 대회 코스와 다양한 동적 장애물 수락은 사용자 추가 시험이 남는다.

판제 웹은 http://localhost:8088/web/index_v2.html, LiDAR 웹은 http://localhost:8091/ 이다.
첫 화면은 자동 보기 또는 LOCAL 경로·노면·fusion이다. 실제 run/master/branch/receive age를
표시하고 종료 때 과거 경로를 비운다. LiDAR 상세 단계 점군은 debug:=true가 필요하다.
웹은 관측용이며 다른 브랜치 포트를 차지하거나 두 제어 스택을 합치지 않는다.

## localization과 같은 조건 비교

현재 선택값은 camera/local만 허용한다. 향후 localization은 같은 launch의 상호 배타 include,
selector 슬롯, watchdog과 모드 인계를 재사용해 추가한다. 이번에는 다른 localization 코드를
가져오거나 GPS 무효를 정상 map 상태로 꾸미지 않았다.

동일 main·센서/네트워크셋·FOGGY13·ego/장애물 배치·상한10·차체 외곽/이격·출발 준비를 고정한다.
거리/접촉, 실제 정지 시간, 재출발·복귀 오차, 유효 경로/실제 명령 최대 공백, 계획/MPC p95,
input age와 실패 사유를 함께 본다. 복구 통과와10km/h 정규 주행을 섞지 않는다.
원복 기준은 archive/local2h-baseline-20261029의430effff와 별도 rollback-3kph.yaml이다.
기본 카메라 방식 복귀에는 소스를 원복할 필요가 없다.

## 배포 재검증 결과

- run34: 최종 설정 주차 차량60.18m/48.04s, 실제 정지3.63s. 지속 정지는 없지만 무정지 회피는 미수락.
- run35: LOCAL 일반→음영→일반236.09m/73.04s, 실제 정지0. ACTIVE LOCAL 유효 경로
  최대 공백120.96ms, 실제 wire 수신 최대102.96ms. 준비 구간·NORMAL 비활성 경로를 섞은
  전체5.35초 공백은 LOCAL 공급 단절로 집계하지 않는다.
- run36: 기본 camera 같은 구간235.98m/42.03s, 실제 정지0. NORMAL→JAMMING→NORMAL
  실제 selector 전환을 확인했고 LOCAL 노드/계획은 없어 선택하지 않은 제어 개입이 없다.
- run37/38: 배포 branch의 웹 연결·재접속·종료 지우기를 재시험. 컨테이너 Git safe.directory
  누락으로 앞선 monitor가 기동 실패한 근거도 남겼고 해당 컨테이너 설정만 수정했다.
  run37은35.20m/14.41s를 통과했다. 이 추가 웹 시험은 CI 부하와 겹쳐 순수 성능 수락에 쓰지 않는다.
- 새 runtime target과 기존 NORMAL/LOCAL planning 검사를 재빌드/실행했다. P1-B43개 메시지
  계약과 기존 MD5 검사를 통과했다. 로그 정본은 validation01/ci02, deployment-build02/core02다.

일반·카메라 기본 방식, 새 LOCAL, 원본 연구를 구분한다. raw bag·원본 revision을 보존하며
전체 대회 코스·동적 장애물 전체 수락은 남는다. 실제 제어 생산자는 selector 하나였으며
자동 시험 후 bridge/인지/autonomy/전용 master는 종료한다.8088 읽기 전용 종료 안내 입구만 남긴다.


## 후속 · 단일 컨테이너와 main 포트

사용자 요청으로 수동 실행 역할을 변경했다. 한 컨테이너가 master11311·bridge·인지·판제와
웹8088/8091을 소유한다. `scripts/drive.sh`가 중복 포트와 공유 simulator lease를 확인하고
`autonomy.launch`로 한 세트를 시작한다. 기본 camera, 명시적 `jamming_planner:=local`을 유지한다.
이 런치는 기존 컴포넌트를 include하며 모델·추적·필터·NORMAL/LOCAL 계산을 복제하지 않는다.
`autonomy_no_bridges_v2.launch`의 기존 분리 실행은 그대로 제공한다.

LOCAL 컨테이너 셸의 과거11348/8090/8092 profile도11311/8088/8091로 정정했다.
Docker 이미지에 남은 과거 ENV11348보다 두 셸 setup과 실행기의 명시적11311이 우선한다.
새 main Compose는11311이며 orphan recorder 수거를 위해 `init: true`를 사용한다.
기존 컨테이너나 다른 worktree를 재생성하지 않는다. 자동 bag master는 매 실행의 명시적 설정이다.

검증: 기본/LOCAL 전체 XML에 중복 노드0, 센서/제어 bridge·카메라·LiDAR·각 웹·selector 각1개,
bridge의 불필요한 vehicle-state 생산자0, LOCAL에서 구 follower0을 확인했다.
포트 점유 시 기존 master·외부 bridge를 보존하며 시작 거부 검사를 통과했다.
다른 localization 실행이 센서 UDP를 사용하던 동안 MORAI를 조작하지 않았다.
이 구성 검사를 새 폐루프 주행 수락으로 표시하지 않는다.
