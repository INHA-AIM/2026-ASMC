# S1 음영구간 시나리오 재현·센서 기록

2026-10-28 KST, `fix/shadow-stop-reproduction`. 운영 비교 기준은 PR #124의 `1ed33fef`다.
원격 main `54f6bb9a`는 당시 PR #124를 아직 포함하지 않는다. 이 단위는 #124 위의 별도 변경이며
모델·LiDAR·family·bbox 판정·차로 변경/제어 정책은 수정하지 않았다.

## 추가한 기반

- 원태의 `tools/scen_runner`에서 기존 네 실행기·판제 재시작·배치/종료/소유권 검사를 재사용한다.
- `--recording-spec`으로 등록된 D run에 센서·판단·명령을 LZ4/60초 분할 bag으로 저장한다.
  recorder 상태·용량 한도·필수 토픽·bag SHA·MD5·부분 파일·overflow를 검사한다.
- 충돌이 없는 장면도 `asmc.section_trial`로 준비 완료/정리 직전에 보존한다.
  이후 충돌·정체 사건을 소모하지 않는다. 원래 collision/stagnation 포맷은 유지한다.
- `--tunnel-profile`로 허용 카탈로그의 정적 모델과 static/dynamic/both·횡단 방향만 고른다.
  기존 도로·차량 크기·배치 가능성 검사를 우회하지 않는다. 프로필 내용/SHA를 보존한다.
- 분석기는 bag을 순차 읽고 `decisions.jsonl`, 사유/공급 요약, 정확히 같은 stamp의 RGB+bbox HTML을 만든다.
  frame/result는 최대64개, 갤러리는 최대32장이다. 운영 기하 수학을 복제하지 않는다.
- `run_ended`/`runs_finished`에 원래 종료 사유를 추가했다. 반복 횟수 완료를 무충돌 통과로 오해하지 않는다.
- CI 기존 작업에 ROS/GPU가 필요 없는 recorder/집계 검사7개만 추가했다. 새 job·대형 bag 의존성은 없다.

실행 안내: [shadow-recording](../../../tools/scen_runner/docs/shadow-recording.md).

## 실제 시험

run 접두사는 `shadow-stop-s1-20261028-`, 위치는 컨테이너 `/artifacts/<run-id>`다.
총 bag 약 **2.717GiB**, 주행 허용 이후 종료/정리 전 시간 합계 **364.782초**다.
이 시간에는 정지 상태도 포함되며 일부 종료 정리 지연이 포함된다. 10분 이내 새 시험 조건을 지켰다.

| Run | 입력 | 실제 환경 | 허용창 | 관찰 결과 |
|---|---|---|---:|---|
| r01 | seed42 무작위 Concrete_Barricade01 | FOGGY11 | 65.366s | far 차로 미확정 지속 정지, 제한시간 종료 |
| r02 | seed42 PE_Drum 지정 | FOGGY11 | 65.201s | far 차로 미확정 지속 정지, 제한시간 종료 |
| r03 | seed9 PE_Firewall_Orange 지정 | FOGGY13 | 58.249s | 차로 변경 상태 관측, 생성기의 구간 종료 조건 도달 |
| r04 | seed17 횡단 Ped_Pet1 | FOGGY13 | 51.352s | 차로 변경 상태 관측, 생성기의 구간 종료 조건 도달 |
| r05 | r03 장면 JSON 재현 | FOGGY13 | 59.212s | 다시 구간 종료 조건 도달 |
| r06 | r03 배치 유지·환경만 파생 변경 | SUNNY13 | 65.402s | far 차로 미확정 지속 정지, 제한시간 종료 |

표의 정확한 값·bag 파일 SHA는 r01의 `matrix-summary.json`과 각 run의 `ros/manifest.json`이 정본이다.
표의 소수점은 기록 시간이며 주행 품질의 정밀도/독립 위치 GT를 의미하지 않는다.
r06은 원본 재현 JSON을 덮어쓰지 않고 부모 SHA와 환경 변경을 기록한 파생 입력이다.
각 run에 사용한 코드 commit은 `scenes/*/*trial*.json` provenance에 있다.
첫 r01의 recorder 코드는 `f2a3dc16`, 후속 모델 지정은 `e3660996`, 마지막 종료 사유 계측은 `0de72105`다.
그 사이 commit은 기록/분석/문서 변화이며 운전 코드는 모두 같은 기준선이다.

공통으로 시뮬에 원래 있던 ID `2/3/29`를 유지했다. 터널 쪽 원래 배치된 차량도 영상에 나타난다.
따라서 단일 생성 객체만 존재하는 격리 시험은 아니다. 이를 숨기거나 검출 오탐으로 단정하지 않는다.
생성기의 구간 종료는 A126에 설정된 도착/종료선 조건이다. A128 이후 복귀·패널티·전체 대회 수락을 뜻하지 않는다.
접촉 감시기는 생성기 소유 객체의 접촉을 판정한다. 이번 결과로 모든 배경 객체에 대한 무충돌을 주장하지 않는다.

## 재현된 판단과 해석

1. r01은 가까운 `concrete_barrier`를 BOTH로 판정했다. 한 실제 표본의 접점은 own38%/adjacent60%,
   지원4행이었다. 동시에 먼 `car`는 `DEFERRED_FAR / beyond_observed_lane_horizon`이었다.
   두 조건이 함께 들어와 `far_obstacle_lane_unresolved`가 반복됐다.
2. r02의 `traffic_drum`은 원본 클래스가 유지되고 contact 판정도 생산됐다. 그와 동시에 far 차량이 남았다.
   정적 그룹 이름을 이유로 차로 변경을 일괄 금지하는 분기를 재현한 것이 아니다.
3. r03/r05에서는 `plastic_barrier`가 실제로 전달되고 `changing_without_bbox_stop` 이후 구간을 끝냈다.
   같은 배치의 SUNNY13 r06에서는 far 정지가 지속됐다. 장면 재현과 센서/검출 결과의 동일성은 다르다.
4. `contact_overlap_too_small`, `projection_source_expired`, `no_continuous_own_lane_to_lookahead` 같은
   일시적인 사유도 별도로 보존했다. 지속 far 정지와 섞어 모두 시간 정합 결함으로 분류하지 않는다.
5. selector 진단은 JAMMING에서 `/ctrl_cmd/jamming` 선택을 보고했다. 최종 `/ctrl_cmd` 발행자는
   `asmc_control_selector` 하나였다. 일반 MPC `INVALID_EGO`와 map 무효 경고만으로 실제 정지 명령의
   원인을 결론내리면 안 된다. selector는1Hz 표본이므로 개별 명령의 정밀한 arbitration 증거와도 구분한다.

사유 샘플 수는 사건 수가 아니다. 예를 들어 r01의 far 정지1105개는20Hz 진단의 반복 표본이다.
녹화된 영상/선택 context 없이 라벨 이름만 비교하거나, unknown/far를 무시하는 정책으로 바꾸지 않았다.

## 기록 품질·검증·복구

- 6회 모두 recorder manifest `complete`, 당시 필수 토픽 누락0·overflow 검출0·미완료 bag0.
  이것은 녹화 완결성이지 센서의 모든 프레임 전달·공급률 수락이 아니다.
- 당시에는 `/CollisionData`와 bridge diagnostics를 bag에 넣지 않았다. 접촉은 generator gRPC 기록과
  rosout에 남겼다. 후속 기록 목록에는 세 토픽을 추가했으며 이번 bag에 소급 존재한다고 표시하지 않는다.
- r01 첫 두 준비 시도는 protobuf import 순서, 불완전한 run-counts 시험 YAML로 시작 전에 실패했다.
  별도 로그를 보존하고 수정했다. 주행 성공에 포함하지 않는다.
- 시스템 Python3.14에서 matplotlib 미설치로 integration 검사가 실행되지 않아 ROS 컨테이너에서 재검사했다.
  archive/replay27, episode45, GUI36, tunnel13, recorder5, analyzer2: **128개 통과**.
  UI headless 검사에는 한글 폰트 경고가 남았지만 assertion 실패는 없었다.
- gRPC 연결 상태에서 Python subprocess를 만들 때 fork 지원 경고가 발생했다. 실제6개 recorder 생성·종료·
  bag 해시/읽기를 확인했다. 경고 자체를 없앴다고 표시하지 않는다.
- ROS master·기존 bridge는 유지했다. 시험의 인지·판제 supervisor와 생성 객체만 정리했다.
  before/after actor ID와 Ego XY를 확인했고 날씨/시간을 원래 값으로 되돌렸다.
  물리 엔진의 정차 settling으로 roll/pitch/z는 순간적으로 동일하지 않다. 차량은 원래 위치에 정차시켰다.
- 새 실행은 Python 도구이므로 catkin 메시지/실행파일 재빌드는 필요하지 않았다. 마운트된 `/root/ws`에는
  해당 작업 commit을 적용했다. 판제·인지 정책의 배포 기본값 변경은 없다.
- 문서 링크/보존 검사와 CI coverage 검사는 로컬 결과다. hosted CI 결과와 구분한다.

## 다음 한 단위

S1의 후반으로 **같은 source stamp의 bbox·family·ground projection을 native 분류기에 재입력하는 작은 회귀**를
만든다. 먼 차량과 가까운 장애물을 분리한 대조 장면에서 아래를 구분한다.

- 관측 범위 밖 far 객체가 안전하게 회피 가능성을 판단할 만큼 근거를 얻었는가.
- bbox 하단과 family 차로 접점의 실제 기하 불일치인가, 객체가 두 차로에 실제 걸쳤는가.
- 차량이 정지했는데 진행 시간만 누적되는 timeout 문제인가.
- 설정/세대/정확한 source context가 어긋나는 연결 결함인가.

명확한 연결 결함만 해당 회귀로 수정한다. 현재 근거가 부족한 far/UNKNOWN을 단순히 무시하거나
라벨별 예외를 늘려 통과시키지 않는다. 그 다음 S2 구조 정리, S3 family+local LiDAR 통로 계획으로 연결한다.
S1 전체 정지 해결·local LiDAR 회피 연결·튜닝 수락은 아직 완료하지 않았다.
