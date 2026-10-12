# Capture 장애물 자동 수집 — 실행 및 인계

기준: 2026-10-12. [기존 자료 v7](camera-training-data-audit.md#기존-자료-채택-확대--2026-10-11-v7)를 보존한다.
수집 전용 구현이며 운영 모델·공개 ROS 메시지·대회용 GT 정책은 변경하지 않는다.

## 이번에 구현한 경로

Capture UI를 켜고 **gRPC Capture 1Hz·단일 요청**으로 RGB·Semantic·Instance·native BBox를 저장한다.
GT 센서 UDP 수신이나 bridge profile 확장은 사용하지 않는다. 현재 MORAI의 Capture 상태 조회
API는 UNIMPLEMENTED라 UI 설정과 실제 파일 생성으로 확인했다. actor가 없으면 BBox 파일이
생성되지 않는 경우가 있으므로 RPC 성공을 파일 완성 또는 빈 정답으로 해석하지 않는다.

- [주행 coordinator](../../../src/data_collection/scripts/capture_route_collector_node.py):
  `/collection/vehicle/state`의 개발 Ego pose, 기존 팀 MPC 또는 기존 PP 계산, 단일 제어 gate.
- [수집 정책](../../../src/data_collection/src/data_collection/capture_collection.py):
  배분·전체 footprint·속도 단위·Capture 후 freshness·전진/달성 속도 검증.
- [반복 실행기](../../../tools/perception/run_capture_route_collection.py):
  파일럿, 검토 전용 1회 주행, 수락된 배치의 SUNNY/FOGGY × 11/13/15시 여섯 주행.
- [배치 도구](../../../tools/perception/capture_route_layout.py):
  plan/create/cleanup. 기존 actor를 일괄 삭제하거나 시나리오를 리셋하지 않는다.
- [동기화](../../../scripts/sync_capture_mode_data.py),
  [GT export](../../../tools/perception/export_capture_route_bbox.py): 원본 SHA·native ID·instance 색상과
  실제 가시 픽셀을 연결하고 고정 16종 YOLO bbox로 내보낸다.

`capture_route_collection.launch`는 수집 전용이다. 기본 actuator는 **gRPC**다.
기존 ROS selector의 출력을 `/collection/actuator_cmd`로 격리해 UDP와 중복 구동하지 않는다.
ROS type2 velocity의 km/h를 gRPC m/s로 명시 변환하며 조향 rad를 ±40° 기준 정규화한다.
`actuator_transport:=udp`는 명시적 대조 옵션이며 이번 환경에서 실제 구동 수락을 받지 않았다.
PP는 기존 `UrbanRouteDriveCase`의 Pure Pursuit 계산과 P 속도 제어를 재사용한다. PID가 아니다.
수집 경로의 급곡선에서는 lookahead를 8m에서 4m로 줄이며 기존 곡선 감속을 유지한다.

## 실행 게이트와 속도

1. 정차 Capture 3회로 원본 RGB/GT 및 파일 완성을 확인한다.
2. 객체 없이 MPC 5km/h를 시험한다. 유효한 양의 속도 trajectory가 없으면 별도 실패로 기록한다.
3. Capture 완료 이후 새 state/trajectory가 공급되고도 2초 내 재개되지 않으면 MPC run을 종료한다.
   PP는 새 run ID로 5→20→30km/h, 속도별 Capture 30회를 독립 시험한다.
4. 횡오차 p95≤0.5m·최대≤1m, 접촉 없음, Capture 오류 0을 요구한다.
   전진 거리>5m 및 요청 속도의 80% 이상을 누적 제어 간격 3초 이상 달성해야 한다.
   목표 속도 파라미터만 보고 주행 통과로 표시하지 않는다.
5. 30km/h가 실패하고 20km/h가 통과하면 20km/h로 수집한다. 곡선에서는 기존 PP의 감속을 적용한다.
6. 배치는 6→12→60→120 순으로 검사하며 120 수락 후에만 240을 대조한다.

Capture 정지 중 제어 상태를 적분하거나 명령을 밀어 넣지 않는다. 완료 후 새 입력으로 재개한다.
MPC의 state 0.20s·trajectory 0.25s freshness와 기존 timeout은 완화하지 않는다.
MPC 경로는 수집 목표 속도 이하로만 제한하며, 이를 변경 없는 MPC 성능 시험으로 설명하지 않는다.

## 배분 및 접지

최신 v7 train 등장 이미지 수는 stroller 0, plastic_barrier 172, steel_barricade 178,
traffic_cone 618, concrete_barrier 953, box 1,109, traffic_drum 1,661, bollard 2,298,
deer 61, motorcycle 69, bicycle 230이다. 120개 중 장애물 100개·희소 객체 20개를
`1/sqrt(max(등장 이미지 수,50))`로 배분하고 하위 에셋을 순환한다.
사람·일반 차량·강아지·표지판·나무·불·입식 전동 킥보드는 새로 스폰하지 않는다.
유모차 에셋에 포함된 사람/어린이는 실제 GT에서 person으로 보존한다.

장부에 정확한 API 이름, 목표 클래스, native ID, client key, 생성 세대, 위치·회전·크기를 기록한다.
GetAllActorsState는 actor 수를 20개로 자르지 않는다. 순차 기록도 120/240개 전부 보존한다.
UDP Object Info의 20개 파서는 수정하지 않으며 이 수집의 라벨 ID 연결에 사용하지 않는다.

각 route 지점의 높이는 **도로 표면 추정**이다. 옆 지면과 동일하다고 보장하지 않는다.
전체 footprint가 전체 route 및 Ego 여유 폭 밖에 있어야 하며 기존/new actor와 겹치지 않아야 한다.
에셋별 native 원점과 실제 형상 접지를 정차 RGB로 교정한다.
[capture_asset_ground_v1.json](../../../config/morai/capture_asset_ground_v1.json)에 교정 근거와 원점 오프셋을 고정한다.
BBox 하단은 진단으로만 보존한다. Scooter1은 BBox 아래에 약 1.1m 빈 공간이 있어
BBox 하단을 지면에 맞추면 실제 형상이 뜬다. 이 자동 보정은 제거했다.
물리 정착 후 native ID/위치를 다시 확인한다. 떠 있음·관통·벽 충돌이 의심되는 배치는 보류한다.

`inspection_only:=true`는 수치 기하·ID 검사를 통과한 배치의 RGB 접지 검토용이다.
이 run은 `training_eligible=false`이며 시각 검수가 끝나기 전 학습용 수락으로 표시하지 않는다.
정식 여섯 주행은 모든 활성 actor의 placement/identity 승인이 필요하다.

## 저장 및 라벨 규칙

```text
data/capture_layouts/<layout>/spawn-ledger.jsonl
  # 계획, 생성/정착 상태, native ID, 접지 승인, 소유권 기반 제거 이력
data/capture_runs/<run>/
  capture_manifest.jsonl   # Capture 단일 요청, source time 의미, wall/monotonic 분리
  control.jsonl            # state·횡오차·속도·명령·Capture 활성 상태
  collection.json         # layout·부모 run·제어기·transport·train 그룹
  validation.json         # 추종·종료 이유·대체 필요 여부·초기 Ego 자세
  summary.json
  spawn-ledger.jsonl
data/datasets/<run>/
  frames/{intensity,semantic,instance,bbox}/<view>/
  manifest.jsonl          # 원본 경로·복사 SHA
  lineage/               # 실행·검증·복원·스폰 장부 사본
  _SUCCESS
data/dataset_versions/camera_objects_bbox_capture_<revision>/
  data.yaml              # 고정 16종; train 전용 보강 부분집합
  images/train/          # 보존 원본 RGB를 가리키는 심볼릭 링크
  labels/train/
  manifest.jsonl         # native 박스·가시 박스·원본 종류·SHA·layout 계보
  verification.json
  quarantine.json
```

동일 layout의 모든 날씨·시간은 같은 train 그룹이다. 기존 selection/heldout을 이동하지 않는다.
구성요소 ID는 parent 숫자로 추측하지 않고 실제 BBox·instance와 검수 근거로 연결한다.
BabyStroller1/2에는 person·stroller·person 구성요소가 있어 각 native ID의 종류를 따로 보존한다.
완전히 가려져 instance 픽셀이 없는 native 박스는 원본에 남기고 가시 박스로 export하지 않는다.
보이는 target semantic 픽셀이 미연결이면 해당 영상을 보류한다.
BBox 누락·미검수 빈 결과를 negative로 만들지 않는다. 작은 양성 GT는 유지한다.
숫자 ID와 원본 종류가 같아도 Instance RGBA가 여러 개일 수 있다. 독립 검수 영상의
객체와 색상을 함께 확인하며, actor 제거 뒤 색상이 바뀌면 해당 run의 새 검수 snapshot으로
다시 연결한다. 이전 snapshot의 색상이 없는데 임의의 다른 색상으로 대체하지 않는다.
주행 중 특정 뷰에 BBox 파일이 없을 수 있어 `bbox3_partial` 동기화는 원본 RGB/GT를 보존하고
`missing_bbox_views`를 명시한다. 이 뷰는 export에서 보류하며 빈 label 파일로 만들지 않는다.
`_SUCCESS`는 선택한 복사 profile의 완료를 뜻하며 전체 뷰 GT 수락과 별개다.
이 부분집합은 기존 v7과 병합·샘플링하기 전 `training_ready=false`다.

## 실행 명령

`asmc-ros-noetic`에서 해당 checkout이 ROS overlay로 빌드되고 source된 상태에서 실행한다.
현재 `/root/ws` 마운트는 유지했고 검증 overlay는 `/data/runtime/capture-route-collection`이다.
기존 MPC 판제 입력이 이미 실행 중이면 중복 launch하지 않는다. PP fallback에는 별도 판제 실행이 필요 없다.

```bash
roslaunch data_collection capture_collection_planner.launch
```

```bash
roslaunch data_collection capture_route_collection.launch controller:=pp target_kph:=20 max_captures:=30 run_id:=unique_run_id
```

수락된 pilot와 검수된 ledger로만 자동 여섯 주행을 요청한다.

```bash
python3 tools/perception/run_capture_route_collection.py collect --pilot-results /data/capture-pilot-preflight-20261011/pilot-pp-corner-r1.json --ledger /data/capture_layouts/APPROVED_LAYOUT/spawn-ledger.jsonl --layout-id APPROVED_LAYOUT --output /data/capture_runs/unique_collection_report.json
```

2026-10-12 추가 요청에 따라 방향 다양화에는 `--rotation-step-deg 60`을 사용한다.
같은 배치의 여섯 조건에서 상대 방향은 0/60/120/180/240/300°다. 정차한 회차 사이에만
소유 actor를 회전하며, 실제 위치·방향·높이와 전체 footprint 간격을 다시 검사한다.
변경 전 자세를 mutation journal에 저장하고 실패하면 원래 자세로 되돌린 뒤 주행을 시작하지 않는다.
회전별 ledger를 불변 revision으로 남기며, 새 방향의 RGB 구조물 겹침은 라벨 채택 전에 검수한다.
동일 배치의 방향·날씨 변형은 계속 같은 train split 그룹이다.

이 빌드에서는 `SetPhysics(false)`가 native bbox의 pivot offset을 잃는 반례가 있어
수집 객체의 물리를 끄지 않는다. `--pause-static --restore-reviewed-static`으로
검수한 native 자세를 복원하고 수집 actor만 pause한다. Ego는 이 고정 대상에 포함하지 않는다.
Pedestrian `SetTransform`의 z는 ground-relative이고 조회/스폰 z는 map 높이인
차이도 실제 Deer1 시험으로 확인했다. 회전 도구는 이 차이를 변환하고 native 조회에서 5° 이내 yaw
편차에만 1회 보정한 뒤 원래 2°/0.2m 허용값으로 검증한다. 물리 OFF 시험과 실패
rollback 기록은 삭제하지 않으며, 복원 후 새 ledger로만 수집을 재개한다.


`--stop-after-current-file PATH`는 해당 파일이 생기면 현재 바퀴의 정상 종료 후 멈춘다.
`--deadline-utc 2026-10-11T23:00:00Z`처럼 시각을 지정하면 30분 주행/복원 여유가
없는 새 바퀴를 시작하지 않는다. 진행 중단 시 소유 자식 제어기도 함께 종료하고 환경을 복원한다.

터널 오른쪽의 기존 5m 횡방향 배치는 벽 바깥에서 하단이 가려진 반례가 있었다.
안쪽 약 3.35m 후보를 별도로 생성해 전체 footprint·접지·RGB를 검사한다. 유효한 가림은
수집하되 벽 관통·지지점 불명확을 유용한 가림으로 간주하지 않는다.
다음 세트는 새로 확보한 클래스 등장 이미지 수를 반영한 `--allocation-policy`와
`--station-phase`, `--side-phase`로 종류·위치·좌우를 바꾼다. 기존 세트의 반복 사진을
독립 장면 증가로 보고하지 않는다. 확인된 접지 실패 RedBarrel은 새 계획의 에셋 풀에서 명시 제외한다.

회차 사이에 Ego만 정차 확인 후 저장 시작 자세로 복귀시키고 위치/방향을 검증한다.
이 MORAI 빌드의 작은 SetTransform yaw 편차에는 5° 이내 오차에 한해 1회 보정하며 허용 오차를 늘리지 않는다.
종료 시 정지를 확인한 뒤 원래 gear/control mode를 복원한다. 날씨·시간은 반복 실행기가 복원한다.
Capture Mode UI와 원본 파일은 유지한다.

## 현재 결과와 미완료 범위

[실행 증거](../../../config/baselines/evidence/camera_capture_collection_v1/summary.json)를 정본으로 확인한다.
정차 6개 배치에서 3 Capture × 3뷰 = **9장·57객체**, 미해결 영상 0인 부분집합을 생성했다.
이는 다양한 주행 데이터나 신규 클래스 전체 지원의 근거가 아니다.

첫 UDP 파일럿은 후진에 가까운 밀림과 낮은 속도를 실제 주행으로 잘못 판정했다.
`pilot-correction.json`에 기존 결과를 보존하고 무효 사유를 남겼다. 실제 속도·전진 게이트를 추가했다.
수집 전용 Main의 legacy `/ctrl_cmd_0`는 격리했다. 기존 runtime/사용자 컨테이너 마운트는 바꾸지 않았다.
MPC는 정지 trajectory만 받아 출발하지 못했으며 Capture 재개 실패와 구분했다.
첫 gRPC PP 20km/h 파일럿은 통과했고 30km/h는 p95 횡오차 0.573m로 보류했다.
급곡선 lookahead와 Capture 후 freshness 대기를 보완한 재시험에서는 5/20/30km/h 모두
30 Capture, 횡오차·실제 달성 속도·전진 기준을 통과했다. 30km/h 실제 최대 29.82km/h,
횡오차 p95 0.200m·최대 0.212m다. 전체 경로 수락은 이 짧은 파일럿과 별개다.
12개 단계(활성 11개)의 SUNNY 13시 전체 경로 검토 주행도 완료했다: 약 2.18km,
583 Capture 성공·오류 0, 횡오차 p95 0.245m·최대 0.448m. 날씨/시간 복원을 조회해 확인했다.
다만 원본 종료 기록의 `stopped=false`가 재확인돼 실행 수락은 보류했다. 생성한 native GT는
진단 부분집합으로 보존하며, 그 박스 정확도와 주행 종료 수락을 구분한다.

120개 계획, 구현된 여섯 조건 반복 실행, 실제 여섯 주행 완료는 서로 다른 상태다.
시각 접지 검수·전체 경로 추종 수락·60/120개 확장과 실제 조건별 수집량은 최신 실행 증거에 남긴다.
이번에는 학습·운영 모델 교체를 하지 않는다.

### 원본 완전성 검사와 접지 교정 기록

에셋 28종의 native 원점을 정차 RGB로 검수했다. `RedBarrel`은 원점과 +0.65m 모두
지면 아래로 떨어져 자동 배치 대상에서 보류한다. YellowBarrel은 +0.65m 생성 후 정착을 확인했다.
교정 설정의 ledger/snapshot SHA는 불변 evidence 사본을 가리킨다. 에셋 접지가 맞아도
각 경로 지점의 경사·기존 구조물과의 충돌은 별도로 검수해야 한다.

`resolve_capture_route_components.py`는 검수된 에셋 schema와 **실제 BBox에 존재하는**
ID·원본 종류가 일치할 때만 구성요소를 연결한다. 계산만으로 존재하지 않는 ID를 추가하지 않는다.
다른 actor root와 충돌하거나 원본 종류가 다르면 변환을 중단한다.

전체 경로 첫 원본 583회 중 Instance 누락 2회(421,556)를 명시적으로 제외한 581회를 복사했다.
BBox 누락은 722뷰이며 빈 정답으로 만들지 않는다. `route12_r1` export는 902장·1,811객체다.
정확한 작은 양성 GT는 유지하며, 부양 Scooter가 보이는 영상은 학습에 넣지 않았다.
Windows 원본 복사는 제한된 256KB 버퍼를 사용해 DrvFs 대량 복사의 ENOMEM을 피하고,
재개 시 크기뿐 아니라 SHA를 비교한다.

완료된 주행은 다음 명령으로 원본 완전성 검사 → 누락 기록 → SHA 복사 → component 연결 →
가시 instance 기반 bbox 변환을 재현한다. 출력 버전은 기존 결과와 다른 새 이름을 쓴다.

```bash
python3 tools/perception/finalize_capture_route_data.py --run-id COMPLETED_RUN --ledger REVIEWED_LEDGER --layout-id LAYOUT --output ../data/dataset_versions/NEW_VERSION --gallery ../artifacts/NEW_GALLERY
```

Capture RPC 성공, 파일 완전성, 배치 검수, 학습 부분집합 수락은 각각 다른 단계다.
부분집합을 v7과 병합하고 클래스·장면 샘플링을 조정하기 전에는 학습 준비 완료로 표시하지 않는다.

### GT ID 및 종료 게이트 재검증

60개 단계에서 BBox의 같은 숫자 ID에 Obstacle/Pedestrian이 함께 기록된 반례가 확인됐다.
숫자 ID만으로 class를 연결하지 않는다. 에셋 schema의 원본 종류와 실행별 Instance RGBA를
함께 검사하며, 같은 ID·종류의 색상이 여러 개이면 해당 연결은 보류한다. 구성요소 source 종류가
다르면 그 박스를 다른 actor의 정답으로 만들지 않는다. 작은 잘못된 이름도 export에서 제외한다.
이전 `route12_r1`은 보존하고 정정본 `route12_r2`를 별도로 생성한다.

첫 60개 단계의 702 Capture는 추종 p95 0.248m·최대 0.422m였으나 복원 직후
`stopped=false`였다. 이전 실행기가 이를 통과 판정에 반영하지 않은 결함을 수정했다.
원본 보고서는 덮어쓰지 않고 `route60-inspection-correction.json`으로 무효 사유를 남겼다.
현재 차량의 별도 P단/정지 확인을 원래 실패의 면제로 사용하지 않는다. 수정 경로는 5초 안에
정지·기어·모드가 연속 0.5초 일치해야 한다. selector는 복원 오류가 나더라도 종료한다.
원본 GT 품질과 주행 실행 수락은 분리하고, 진단 run의 subset은 복원 실패 상태를 명시한다.

수정 후 49개 승인 배치의 전체 경로 재시험은 678 Capture, 횡오차 p95 0.249m·최대
0.438m와 연속 정지/기어/모드 복원 검사를 통과했다. 60개 계획에서 실제 수치 검사 통과는
57개, 구조물 접촉·접지 확인이 어려운 8개를 제거한 승인 배치는 49개다.
다음 120개 계획에서는 118개 생성, 115개 수치 검사 통과이며 시각 검수 전에는 수락하지 않는다.

`verify_capture_route_dataset.py`로 실제 YOLO 파일의 클래스·좌표·객체 수, RGB 및 주석
SHA와 디코딩 픽셀을 다시 확인한다. 고정 v7 selection/heldout의 파일·픽셀 해시와 비교하고
평가 manifest가 바뀌지 않았음을 기록한다. 같은 배경의 유사 장면 독립성은 해시 검사만으로
보장하지 않으며, 동일 배치의 모든 조건은 하나의 train layout 그룹으로 유지한다.

```bash
python3 tools/perception/verify_capture_route_dataset.py --dataset ../data/dataset_versions/CAPTURE_SUBSET --frozen-manifest ../data/dataset_versions/camera_objects_bbox_v7-adoption-r2/manifest.jsonl --output ../data/collection-evidence-20261012/NEW_VERIFICATION.json
```

### 2026-10-12 회전·터널 보완과 수집 중단

회차 사이 owned actor를 60°씩 회전하고 실제 native 자세를 새 ledger에 기록했다.
첫 회전 배치의 114개 actor·11개 목표 종류·26개 에셋을 RGB로 확인했다.
터널 오른쪽은 기존 횡간격 5m 대신 3.35m 후보를 시험해 10개를 유지했다.
주행 여유 기준을 충족하지 못한 PE 방벽 1개는 제거했으며 기준을 완화하지 않았다.
벽 인접 부분 가림은 보존한다. 실제 벽 교차 여부가 불명확한 반례를 완전히 검증했다고
안내하지 않는다. 구조물이 가리는 부분을 포함한 native 투영 박스와 실제 가시 instance
박스는 다르며, 학습 export에는 후자를 사용하고 원본 박스를 별도로 보존한다.

SUNNY 11/13/15시와 FOGGY 11시의 **주행·복원 검사**는 완료했다. 성공 Capture RPC는
3,892회, 세 뷰 필수 파일이 존재하는 기록은 3,789회다. 그러나 영상 검사에서 SUNNY
15시의 229개 기록에 적어도 한 뷰의 상수 RGB가 있고, FOGGY 11시는 모든 기록에서
한 뷰 이상이 검은 화면이다. 주행 통과와 데이터 수락을 분리한다. 세 뷰 모두 상수 이미지가
아닌 SUNNY 기록은 2,593회이며, 이 수 역시 전체 주석 검수 완료를 뜻하지 않는다.

첫 60° SUNNY 11시 조건만 별도로 검수·변환해 **2,404장·29,871객체**를 생성했다.
실제 YOLO 재읽기, SHA·좌표·고정 ID·중복 및 고정 평가본 누출 검사는 통과했다.
v7에 병합하거나 학습 샘플링을 조정한 상태는 아니다. 나머지 회전 조건은 원본과 실패
기록을 보존하며 검수 대기다. 검은 이미지를 학습용 negative로 사용하지 않는다.

2026-10-12 03:40 KST FOGGY 13시 Capture 중 Simulator가 종료됐다.
Unity crash 기록은 물리 메모리 99%, 시스템 commit 130,835/130,835MB와
`ImageConversion.EncodeToPNG` 스택을 남겼다. 직전에는 graphics resource ID 초과와
CopyTexture 오류가 있었다. 반복 Capture 자원 누적은 근거 있는 가설이며 분리 대조로
확정한 원인은 아니다. 종료 후 차량·환경 복원은 gRPC 단절로 **검증하지 못했다**.
FOGGY 13/15시, FOGGY 11시 재수집과 SUNNY 15시 손상 구간 보완은 남아 있다.
두 번째 균형 배치 120개는 계획만 만들었으며 생성·수집하지 않았다.

재실행 후에는 actor ID를 그대로 신뢰하지 않고 새 세대·native 상태·센서와 정차 Capture를
확인한다. 수집 실행은 `--max-laps-per-session 2`로 두 바퀴 이하씩 나눈다.
`--conditions foggy:11 foggy:13`처럼 남은 조건만 새 보고서에서 실행할 수 있다.
각 세션의 첫 ledger 방향은 별도로 준비하고, 이후 회차 사이에만 60° 추가 회전한다.
기존 정상/실패 보고서를 수정해 재시도를 성공으로 바꾸지 않는다.

호스트의 `monitor_capture_memory.py`는 Windows 메모리를 읽어 JSONL로 기록한다.
Simulator private memory 16GiB 또는 시스템 commit 65%에서 다음 회차를 막고,
32GiB·85% 또는 가용 물리 메모리 512MB 미만에서는 `--abort-file`을 생성한다.
동일 파일을 전달받은 실행기는 자신이 만든 제어 자식만 SIGINT로 종료하고 복원을 시도한다.
이 임계값은 이번 고갈 이후의 보수적 예방 설정이며 장시간 실환경 수락 결과는 아니다.
센서 경로와 capture_runs 경로를 주면 성공 RPC의 세 뷰 RGB도 검사한다. 상수 이미지나
연속된 telemetry 실패에서는 수집을 중단한다. 사용자 Windows 프로세스는 종료하지 않는다.
모니터와 실행기에는 호스트·컨테이너에서 **같은 파일**을 가리키는 경로를 전달한다.

```bash
../data/roboflow/.venv/bin/python3 tools/perception/monitor_capture_memory.py --output ../data/collection-evidence-20261012/NEW_SESSION_MEMORY.jsonl --stop-after-current-file ../data/collection-evidence-20261012/NEW_SESSION.stop --abort-file ../data/collection-evidence-20261012/NEW_SESSION.abort --sensor-root /mnt/c/MoraiLauncher_Win/SensorData --capture-runs-root ../data/capture_runs --max-seconds 3600
```

상태·실패 영상은 `http://localhost:8768/capture-collection-20261012/`와
`http://localhost:8768/capture-render-integrity-20261012/`에서 확인한다.
로컬 메모리 조회와 중단 신호 CPU 시험은 통과했지만 재시작 후 실제 주행 중단·복원 시험은
아직 수행하지 않았다. Hosted CI의 결제 제한과 함께 미완료 항목으로 남긴다.

### 09:03 KST 재개

사용자가 Simulator를 재실행하고 추가 4시간 수집을 승인했다. 기한은 13:03 KST다.
기존 actor가 사라진 것을 조회로 확인하고, 이전 SUNNY15 배치 위치·방향을 기준으로
새 generation의 114개를 생성했다. 정차 Capture에서 세 뷰 RGB·semantic·instance와
실제 BBox 생성을 확인했다. 새 ID에 이전 ID 검수 결과를 그대로 부여하지 않는다.
`--inspection-only`로 지정한 조건을 먼저 수집하고 실제 새 RGB 검수를 거친다.

SUNNY15 재수집은 871 Capture·오류 0, 횡오차 p95 0.248m·최대 0.432m와
주행·정지·환경 복원 검사를 통과했다. 새 114개를 13개 상세 페이지에서 확인한 뒤
60° 회전해 FOGGY11을 시작했다. 첫 주행 동안 private memory는 약 3.3–3.7GiB로
안정적이었다. 두 바퀴마다 정차·메모리 이력·RGB를 확인하고 다음 묶음을 판단한다.
경고·상승·렌더링 실패가 있으면 재시작 전까지 계속하지 않는다. 충분한 여유가 실제로
확인되면 동일 Simulator에서 다음 묶음을 진행할 수 있으며 process PID와 그 판단을 기록한다.
두 바퀴 제한을 Simulator가 반드시 재시작됐다는 증거로 사용하지 않는다.

원본 복사는 완료된 run만 처리하며 상수 RGB·필수 파일 누락을 사유와 함께 제외한다.
새 주행과 회전 영상의 학습 채택은 별도 검수·GT 변환 단계다. 추가학습·운영 변경은 하지 않는다.
