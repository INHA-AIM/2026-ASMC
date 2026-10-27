# 관측 연결·위치·유효성 개선 마일스톤

**2026-10-28 우선순위 갱신:** M1 3뷰 기본·최적화 구현을 유지하며, 승현의 다음 단위는
[음영 정지 재현·local 통로/객체 판제](../../../docs/architecture/shadow-planning-refactoring.md)다.
현재 PR은 클래스/그룹 전달·시간 경계·판제 콘솔 기반까지 구현한다. 인지 품질 후속은
[보류 과제 인수 단위](#보류-과제-인수-단위)에 정리했다. 구현된 기능의 미수락과 새 개선의 미착수를 구분한다.
M3/M4와 터널 인지 개선을 완료한 것으로 표시하지 않으며 새 역할 분배는 팀 확인 뒤 반영한다.

기존 2026-10-27 인수안은 **안승현: 3뷰 fusion, 손재호: 터널 통합 뒤 M3 관측 위치·OBB와
M4 추적·속도·운동 상태**였다. 이는 기술 문의·교차 검토의 기준으로 보존하고,
후순위 과제를 누가 언제 재개할지는 착수 시 확인한다. 장원태의 YOLO detect 추가학습은 별도다. 모델 교체보다
현재 관측을 정확히 연결하고, 관측 위치와 유효성을 안정화하는 작업을 우선한다.
이 문서는 승현의 제안을 반영한 인수안이다. 재호·원태의 일정과 인수 확인은 PR 리뷰에서 확정하며,
CODEOWNERS는 변경하지 않는다. 아래 후보를 이미 운영 적용한 것으로 읽지 않는다.

## 출발점과 이번 PR 경계

- PR #118의 F1이 main `22577363`에 병합됐다. 되돌리지 않고 후속 PR에 연구 근거·역할·작업표를 정리한다.
- [F1 결과](../../../docs/records/lidar/2026-10-27-front-fusion-verification.md)는 당시 전방 실제 matching,
  좌우 수신/native 재투영, 관측 식별 결함 수정까지였다. 이후 **PR #122에서 3뷰 matching과 터널 최적화를 병합**했다.
- 별도 연구 브랜치의 [선별 통합 기록](../../../docs/records/lidar/2026-10-27-research-integration.md)은
  과거 결과와 실행 소스를 보존한다. 원본 clock 대응·독립 위치/속도 GT는 여전히 미확보다.
- [재호의 PR #119](https://github.com/INHAautonav/2026-ASMC/pull/119)는 `52017fe2`에 병합됐다.
  터널 구현과 오프라인 대조를 마쳤고, head `90415a11`에서 F1 main을 병합했다.
  병합 후 회귀·실시간 공동 수락은 남아 있다. 과거 base `93666315`로 현재 main을 되돌리지 않는다.
- 연구 정리 PR 당시 운영 소스 변경은 없었다. 후속 M1은 최적화·사용자 승인 후3뷰 기본으로 승격했다. 기존 운동 판단과 공개 계약은 유지한다.

## 작업 순서와 담당

번호는 작업 식별자이며 착수 순번이나 현재 실행 중인 작업을 뜻하지 않는다.
현재 S1–S4 음영 판제 개선을 우선하고 아래 인지 후속은 보류한다. M3→M4는 같은 관측 기준선을 사용한다.

| 단위 | 담당·현재 상태 | 변경 범위·선행 조건 | 완료 산출물 |
|---|---|---|---|
| M0 · 실패 사례 연결 | 보류·미착수, 착수자/검토자 확인 필요 | 기존 C0/F1·터널 replay/StageObserver 재사용. 빠진 단계만 선택 사례에 추가 | 군집 혼입 / 위치 변화 / 연관·miss / 전달 필터 문제를 구분한 사례 장부 |
| **M1 · 3뷰 fusion** | **승현 · 기본 승격/공동 수락 잔여** | F1 유지, 뷰별 비차단 선택·동일 scan context·중복 근거 통합. tracker/속도는 유지 | 전방 회귀 + 좌우 연결·뷰 중단/복귀·중첩/충돌의 검증 가능한 분류 근거 |
| M2 · 터널 통합 수락 | 보류·구현됨/검증 잔여, 기존 검토 창구 재호/승현 | PR #119·#122 구현·오프라인 결과 재사용 | GPS0/중단/복귀·실제 장애물 보존·확인 지연·local 공급 검증 |
| **M3 · 관측 위치·OBB 안정화** | **보류·미착수**, 기존 인수안 재호/승현 | M0 사례·M2 통합 기준선. 우선 기존 L-shape와 관측 기준점 대조 | 가시 면/부분 가림에서 중심·yaw·치수 변화 원인과 한 가지 개선 |
| **M4 · 연관·속도·운동 상태** | **보류·미착수**, 기존 인수안 재호 | M3 관측 입력 고정. innovation/공분산·miss·재초기화 중 한 원인씩 대조 | ID 전환·정적 오동적·속도 튐·유효성/반응 지연 개선 및 한계 |
| R1 · 얇은 장애물 복구 | 보류·설계만, 재개 담당 미정 | M0로 최초 소실 단계 확인. 실제 점 근거가 있을 때만 복구 후보 | 필요한 장애물 보존·배경 오복구·중복 검출/track 비교 |
| V1 · semantic 정적 보조 | 보류·구현됨/기본 OFF, 재개 담당 미정 | M3/M4의 기하·속도 기준선 고정 후 OFF/ON | 순간 튐 억제와 실제 저속 이동 해제 검증 |
| Q1 · 공급 공백·비용 | 보류·실패 근거 있음, 인지/실행 공동 검토 | 동일 scan의 입력·처리·발행·수신 단계 구분 | 473.47ms 기존 실패의 원인·수정 또는 잔여 제한 |
| M5 · 판제 공동 수락 | 보류·미착수, 인지·판제 공동 | 채택된 변경만 개별 검증 후 통합. 모든 후보 완료를 요구하지 않음 | 일반/정적/동적/로터리/고주로/터널 주행과 공급·장애 대응 결과 |

M0는 C0/F1을 다시 수행하는 큰 진단 프로젝트가 아니다. 터널 scan64/112/118 및 일반 구간
대표 사례에서 **현재 코어**를 bag 처음부터 replay하고, anchor 주변만 상세 표시한다. 오래된
track ID76을 현재 같은 물체의 ID로 가정하지 않는다. 당시 raw 점 실험과 현재 voxel 평균점의
연결을 일대일로 꾸미지 않으며, 확정할 수 없는 detection→track 연결은 unknown으로 남긴다.
기존 trace로 설명되는 항목에는 새 계측을 만들지 않는다.

## M1 — 승현의 구현과 수락 범위

현재 구현·복구 설정과 semantic 보조 경계는 [M1 정본](three-view-fusion.md)을 따른다.
[터널 연산 최적화](../../../docs/records/lidar/2026-10-27-m1-tunnel-optimization.md)는 같은 결과로 계산 증가 기준을 통과했다. 2026-10-28 사용자 승인으로3뷰 기본 승격했다. 실제 물체 보존·전체 공급 공동 수락은 별도로 남아 있다.
승현의 객체 인지 후보는 [얇은 장애물 미연결 점 지지 복구](fusion-recovery.md)다. 다만 최근 음영 정지 기록을 반영한 아래 우선순위 제안을 먼저 검토한다. 재호의 M3/M4와 벽 폭 검토는 별도다.
아래는 유지하는 수락 경계다.

1. 전방의 receive header ±50ms·최대 대기200ms·unique 관측 사용과 분류 확인 정책을 회귀 기준으로
   고정한다. 뷰별 후보 큐를 독립시키고 한 뷰 중단이 다른 뷰를 기다리게 하지 않는다.
2. scan context·점군은 한 번 확보하고 세 뷰가 참조한다. 투영은 현 native 코어와 원본 크기
   전방1280×720·측방640×480을 사용한다. 좌우 카메라를 전방 보정으로 투영하지 않는다.
3. 근거 키는 카메라 session/generation/view/stamp와 bbox index, LiDAR session/generation/scan으로
   구분한다. 같은 영상 재조회는 확인 횟수를 늘리지 않는다. 같은 scan의 여러 뷰가 한 track을
   볼 때 확인 횟수를 세 배로 늘리지 않는 통합 단위를 먼저 확정한다.
4. 여러 뷰의 class가 충돌하면 근거·가시성·시각을 남기고 unknown/기존 유효 분류 유지 정책을
   명시한다. 마지막 도착 뷰가 덮어쓰거나 confidence만으로 항상 결정하지 않는다.
5. fresh 양성 vehicle/dynamic_obstacle 근거만 HD맵 보호로 전달한다. 카메라 빈 결과·미수신은
   LiDAR 객체 삭제 사유가 아니며 LiDAR 속도와 과거 분류를 신규 카메라 확인으로 승격하지 않는다.
6. 2D box만 있고 LiDAR 지지가 없는 경우 3D 위치·속도를 만들어내지 않는다. 점 지지 분리는
   혼입 사례가 남을 때 별도 후보로 대조하며, 3뷰 확장과 tracker 교체를 한 번에 하지 않는다.

전방 단독 결과 동등성, 중첩 뷰의 중복 확인 방지, 한 뷰 중단/빈 결과, 재시작·세대 전환,
GPS 무효 local 투영을 먼저 검사한다. 기존 trace/갤러리로 전후 연결 근거와 비용을 제시한다.
서로 다른 원본 clock을 직접 차감하지 않는 F1 receive 기준 구현은 가능하다. 정확한 촬영 시각
정렬이나 ego motion 보정을 완료했다고 주장하려면 별도 clock 근거가 필요하다.

## M2–M4 — 기존 인수 근거와 교차 리뷰

PR #119가 보고한 40개 CTest·47회 replay, 실제 차량138/138·시설물97/97 보존은 해당 고정 자료의
결과다. GPS 정상4,230프레임의 전처리 일치와 전체 track/속도/상태 메시지 일치는 다르다.
후방 잔여 후보와 GPS 강제 무효 스트레스의 전방 후보가 남아 있어 모든 터널 해결로 표시하지 않는다.

통합 시 F1 진단·관측 epoch 검사·fresh camera 보호를 유지한다. 정상 GPS3회 복구와 후보별
3/5/6회 확인의 **최초 전달 지연**, GPS0 상태의 local 공급, 지도 무효 상태, 실물 돌출부·작은
시설물·흰선 밖 여유 공간 객체 보존을 확인한다. 벽 추적용 signed CAN/IMU 운동 보상과 객체의
상대/절대속도 유효성을 혼동하지 않는다. 자세한 구현은 PR #119 정본을 따른다.

M3에서는 관측 point centroid·OBB center·KF 기준점을 분리해 기록한다. 가시 면 변화에 따른
중심 이동, yaw의 π/2 또는 π 모호성, 치수 변화·벽 혼입을 먼저 구분한다. 참조 yaw/크기로
큰 OBB를 강제 유지하거나 현재 장애물을 사라지게 만드는 후보는 채택하지 않는다.

M4는 안정된 M3 관측에 대해 연관 오류면 covariance 기반 gate, miss/재관측 문제면 lifecycle을
먼저 고른다. Q/R·연관 비용·운동 모델·확인 횟수를 동시에 바꾸지 않는다. GT가 없으면 속도
정확도 대신 내부 일관성만 보고하고, UNKNOWN/무효를 유효한0속도나 static으로 바꾸지 않는다.

## 보류 과제 인수 단위

2026-10-28 사용자 결정: 음영 판제 개선을 먼저 진행하며 아래 과제는 삭제하지 않고 후순위로 둔다.
**보류는 이 작업 계획의 우선순위이며, 팀원의 별도 진행을 중단하거나 새 담당을 배정한 지시가 아니다.**
다른 팀원도 각 단위를 인수할 수 있다. 착수자는 기존 담당과 겹치는 파일·입력 계약을 확인하고
작업표에 이름/검토자·상태·기준 revision·결과 링크를 남긴다.
기존 M0–M5 번호는 유지하고 R1/V1/Q1만 추가한다.

### 공통 착수·종료 규칙

1. S1에서 실제 장애물 소실·벽 오검출·local 입력 공백이 정지 원인으로 재현되면
   **그 원인에 해당하는 M2/R1/Q1만** 앞당긴다. 그렇지 않으면 음영 주행 기반 뒤에 선택한다.
2. 먼저 M0 장부에 run/scan·관측 ID, RGB/점군·단계별 결과, 최초 실패 단계와 미확정 사항을 연결한다.
   원본 자료가 없으면 필요한 최소 기록 조건을 남긴다. 새 수집은 저장 규약과 새 run 등록을 따른다.
3. 후보는 하나씩 비교한다. 같은 입력·모델·보정·지도·설정/소스 SHA와 원시 출력, 차이 원인,
   실패 사례, 처리 p50/p95/p99·RSS·공급 공백을 남긴다. 임계값 변경은 실행 전에 근거와 수락값을 고정한다.
4. 각 단위의 결과는 **채택 / 보류 / 불필요** 중 하나다. 담당·검토자·검증 범위·복구 revision/설정과
   남은 실패를 기록한다. 구현 존재·일부 replay 통과·운영 기본 적용·전체 수락을 한 상태로 묶지 않는다.
5. 필요한 장애물 신규 소실, 현재 근거 없는 형상/속도 생성, 좌표·시각·세대 위반 또는
   설명되지 않는 공급 악화가 생기면 기본 승격을 보류한다. 새 모델·전면 재설계는 별도 계획이다.

### M2 — 터널 처리 검증과 필요한 보완

- **시작점:** [최적화 경로와 수락 한계](three-view-fusion.md#터널-연산-최적화),
  [기존 인수·실패 기록](../../../docs/records/lidar/cutover-foundation-through-pr111.md),
  `src/experimental/`의 tunnel core/route와 기존 tunnel 시험.
- **착수 근거:** 지면/벽/잔여 구조물 단계의 RANSAC 실패, 천장/벽 잔류 또는 실제 장애물 소실.
  콘솔의 RANSAC 두 줄만으로 실패 단계나 주행 정지 인과를 확정하지 않는다.
- **작업 범위:** scan·단계·후보 점/평면 지지·실패 후 출력·확인 지연을 연결한다. GPS0/복귀,
  입구/곡선/출구, 양쪽/한쪽 벽, 흰선 밖 여유 공간의 물체, 카메라 중단을 대조한다.
  실패한 단계 하나만 보완하며 3뷰 보호·원본 index/context 재사용을 유지한다.
- **완료 근거:** 필요한 물체 보존·벽/천장 잔류와 최초 전달 지연의 전후표,
  local 공급 유지·map 무효 계약·실패 fixture. 벽 폭은 부드러운 제약 후보이며
  흰선/명목 폭 밖 점을 통째로 제거하지 않는다.
- **경계:** 인지는 검증 가능한 벽 지지/객체를 제공한다. 통로 선택·출구 복귀·회피 정책은 S3/S4 판제 책임이다.
  벽 출력 API가 필요하면 별도 계약으로 검토하며 debug JSON을 제어 입력으로 사용하지 않는다.

### R1 — 얇은 장애물 소실 진단과 제한된 복구

- **시작점:** [steel barricade 단계별 원인·복구 설계](fusion-recovery.md).
  EagerMOT/Autoware/LCF3D의 참고 범위·고정 revision은 그 문서에 유지한다.
- **착수 근거:** 새 YOLO 양성이 있는데 필요한 얇은 물체의 LiDAR 관측이 소실된 같은 입력 사례.
  raw→ROI/voxel→ground/tunnel→cluster/OBB→track→delivery 순서로 최초 손실을 찾는다.
- **작업 범위:** 실제 점 지지가 있고 검출 생성 단계의 손실일 때만 미연결 bbox의 깊이/연결 성분을
  제한적으로 대조한다. scan 투영/index를 재사용하고 기존 검출과 중복 제거 후 한 번만 tracker에 넣는다.
  검출은 있는데 연관/miss에서 사라졌다면 복구기를 만들지 않고 M3/M4로 넘긴다.
- **완료 근거:** 필요한 물체의 검출/최초 전달/유지율, 배경·벽 오복구, 중복 OBB·ID, 비용 비교.
  LiDAR 점이 없으면 복구 불가로 남긴다. 2D만으로 거리·크기·속도를 만들거나 static 전체를 터널 삭제에서 보호하지 않는다.
- **복구:** 새 관측 생성은 기존 분류 연결과 별도 후보로 격리한다. 채택 전 기존 detection/tracker 경로를 기본으로 유지한다.

### M3 — 관측 위치·OBB 안정화

- **시작점:** `src/detection/`, `src/tracking/`의 관측 인수 경계,
  위 M3 인수 근거와 L-shape/Autoware shape estimation 참고 목록.
- **착수 근거:** 같은 물체가 계속 관측되는데 가시 면·가림·벽 혼입으로 centroid/OBB 중심·yaw·치수가 튀는 사례.
  먼저 측정 중심과 KF 중심, 관측/예측 기하를 분리한다.
- **작업 범위:** 고정 cluster sequence에 fitting 목적함수·가시 면 기준점·yaw 모호성 처리 중
  원인이 확인된 하나만 대조한다. tracker 파라미터와 새 복구 검출을 동시에 바꾸지 않는다.
- **완료 근거:** 중심/코너/방향/치수 변화와 현재 실제 점의 포함 범위, 회전·부분 가림·얇은 물체 사례.
  GT가 없으면 절대 정확도 대신 내부 일관성으로 보고한다. 박스가 부드러워져도
  실제 크기 변화에 늦게 반응하거나 충돌 기하를 축소하면 채택하지 않는다.
- **인계:** M4가 동일 관측을 재사용할 수 있는 입력 sequence와 관측 기준점 계약, 변경 전후 replay를 제공한다.

### M4 — 추적·속도·운동 상태 개선

- **시작점:** `src/tracking/`, MotionClassifier와 [속도·판제 계약](planning-runtime.md).
  이미 있는 CA KF/Hungarian을 새로 도입하는 작업으로 설명하지 않는다.
- **선행:** M3 결과 또는 현재 관측을 그대로 고정해도 되는 근거. 연관 오류와 측정 위치 오류를 섞지 않는다.
- **작업 범위:** M4a 연관/gating·ID/miss·재관측 → M4b 속도/공분산·유효성 →
  M4c 운동 확인/해제의 순서로 필요한 부분만 별도 커밋·대조한다. Q/R·비용·확인 횟수 일괄 탐색은 하지 않는다.
- **완료 근거:** ID 교체·단절, 정지/출발/저속/회전/부분 가림의 반응 지연과 오동적 판정,
  독립 GT가 있는 구간의 속도 오차. GT 부재는 비교 불가로 표시한다.
- **경계:** local 상대 정보·map 절대 정보와 자차 보상 횟수를 명시한다. GPS 무효/시간 역행/reset에서
  오래된 map 근거를 재사용하지 않는다. UNKNOWN/무효를 측정된0으로 바꾸지 않으며 TTC·행동 선택은 판제에 둔다.

### V1 — semantic 정적 보조 검증

- **현재 상태:** [M1 semantic 보조](three-view-fusion.md#semantic-정적-보조와-판제-인수)는 구현돼 있으나
  `motion.semantic_static_enabled: false`다. 다시 구현하는 과제가 아니라 OFF/ON 수락 검증이다.
- **선행:** M3/M4의 기하·속도 기준선과 fresh 양성 클래스 근거를 고정한다. semantic으로 측정 결함을 감추지 않는다.
- **작업 범위:** 정지 cone/box/barrier의 속도 점프와 실제 저속 이동, 물체를 미는 경우,
  vehicle/person 오분류, 그룹 충돌·만료·GPS 무효를 같은 입력으로 대조한다.
  기존 후보값은 M1 문서를 따르고 검증 중 자동 탐색하지 않는다.
- **완료 근거:** 불필요한 동적 전환 감소와 실제 이동 근거가 생긴 첫 scan의 보조 해제,
  기존 확인 정책의 동적 전환 지연. 원시 KF·속도·ID·OBB가 보조 ON/OFF로 바뀌지 않는지 검사한다.
- **채택 제한:** motorcycle을 포함한 의미 그룹이 물리적 정지를 보장하지 않으며 vehicle/dynamic도 정지할 수 있다.
  실제 움직임 억제·오래된 라벨 사용·무효 속도 정상화가 생기거나 근거가 부족하면 기본 OFF를 유지한다.

### Q1 — 공급 공백·진단 비용의 잔여 검증

- **시작점:** [F1의473.47ms 공급 실패](../../../docs/records/lidar/2026-10-27-front-fusion-verification.md)와
  [터널 최적화 대조](../../../docs/records/lidar/2026-10-27-m1-tunnel-optimization.md).
  과거 실패 수치를 현재 원인이나 해결 완료로 바꾸지 않는다.
- **작업 범위:** 같은 scan의 입력 수신→계산→발행→소비 수신을 연결해 센서 공백·큐·연산·직렬화·
  계측 지연을 분리한다. debug OFF/ON·웹 유무를 같은 조건에서 비교하고 bounded 기록을 유지한다.
- **완료 근거:** 단계별 p95/p99·최대 공백·누락·RSS/큐 추세와 객체 출력 동등성.
  정상 공급≥10Hz·발행/수신 공백≤250ms의 기존 기준으로 남은 실패를 보고한다.
  평균 Hz·관측기간 단축·freshness 확대·과거 결과 재발행으로 통과시키지 않는다.
- **재개 조건:** 음영 주행의 실제 입력 지연 원인이면 우선 재개하고, 그렇지 않으면 M2/R1/M3/M4/V1
  채택 후보의 공통 성능 검사로 수행한다. 모든 과제에서 동일한 대규모 수집을 반복하지 않는다.

### 담당자가 남길 최소 인계

`단위 ID / 상태 / 착수자·검토자 / 기준 revision·입력 run / 확인한 실패 단계 /
선택 후보와 제외 범위 / 검증·실패 링크 / 채택·보류·불필요 판단 / 복구 방법 / 다음 한 단위`를
이 작업표와 결과 문서에 남긴다. 미확보 자료와 추가 기록 조건도 기록한다.
외부 코드를 참고했다면 revision·라이선스·읽은 파일·실제 적용 부분을 구분한다.
개인 경로나 채팅 이력만 있어야 이어갈 수 있는 상태로 인계하지 않는다.

## 우선 참고할 기준선과 적용 조건

이 절은 팀이 공유할 **선별 참고 목록의 정본**이다. 개인 연구 목록 전체를 의존성이나 필수 과제로
옮기지 않는다. 논문은 방법의 근거, Autoware/PCL은 구현 참고, 우리 갤러리는 제한된 로컬 실측으로
구분한다. 아래 문헌·공식 구현은 2026-10-27 확인했으며, 외부 전체 알고리즘을 우리 bag에 새로
이식·검증한 상태가 아니다. 실제 이식 시 사용한 외부 revision·라이선스·변경 지점을 기록한다.

### 먼저 읽고 부분 대조할 논문

| 연결 단위 | 논문·발표 / 원문·공식 구현 | 우리 코드에서 대조할 부분 |
|---|---|---|
| M1 | **EagerMOT: 3D Multi-Object Tracking via Sensor Fusion**, ICRA2021. [논문](https://arxiv.org/abs/2104.14682) · [공식 코드](https://github.com/aleksandrkim61/EagerMOT) | 2D/3D 관측 연결·한 센서만 있을 때의 근거 관리. 거대 군집의 점 분리 기법은 아니며 전체 tracker 이식은 보류 |
| M3 | **Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners**, IV2017. [저자 기관 논문](https://publications.ri.cmu.edu/efficient-l-shape-fitting-for-vehicle-detection-using-laser-scanners) | 이미 있는 L-shape의 fitting 기준·가시 면·위치 기준점 대조. L-shape 신규 도입으로 설명하지 않음 |
| M4 | **Probabilistic 3D Multi-Object Tracking for Autonomous Driving**, arXiv2020. [논문](https://arxiv.org/abs/2001.05673) · [공식 코드](https://github.com/eddyhkchiu/mahalanobis_3d_multi_object_tracking) | residual·covariance 검증 후 Mahalanobis gate 대조. 학습 통계·좌표를 확인하지 않은 gate 강화는 금지 |
| M4 | **SimpleTrack: Understanding and Rethinking 3D Multi-object Tracking**, ECCVW2022. [논문](https://arxiv.org/abs/2111.09621) · [공식 코드](https://github.com/tusen-ai/SimpleTrack) | 관측 확인·miss 유지/종료·재관측을 분리. 재현된 lifecycle 규칙 하나씩 대조 |

SimpleTrack는 arXiv2021 공개·ECCVW2022 발표이며 proceedings 출판 시점과 구분한다.
Probabilistic 3D MOT는 NeurIPS2019 워크숍 nuScenes 챌린지 방법을 기술한2020 논문으로,
학습 feature를 사용하는 Multi-Modal 후속과 혼동하지 않는다.

### Autoware와 PCL — 공식 구현 참고

**Autoware는 제외하지 않는다. M3의 우선 구현 참고이며 M4에서도 필요한 부분을 읽는다.**
Autoware 전체를 하나의 논문 또는 이미 도입한 baseline으로 표시하지 않고 아래 모듈을 지정한다.

| 연결 단위 | 공식 구현 | 적용 후보·제한 |
|---|---|---|
| **M3 · 우선** | **[Autoware shape estimation](https://github.com/autowarefoundation/autoware_universe/blob/main/perception/autoware_shape_estimation/README.md)** | 규칙 기반 L-shape, 참조 yaw/size를 이용한 형상 추정·보정·유효성 검사. 현재 관측 중심·OBB 중심·KF 기준점과 대조하며 PointNet/ROS2 스택을 함께 도입하지 않음 |
| **M4 · 참고** | **[Autoware multi-object tracker](https://github.com/autowarefoundation/autoware_universe/blob/main/perception/autoware_multi_object_tracker/README.md)** | class별 연관 gate·운동 모델·크기 관리 설계. 현 CA/Hungarian을 즉시 교체하거나 상대/절대속도 계약을 바꾸지 않음 |
| M2 이후 혼입이 남을 때 | [PCL Conditional Euclidean Clustering](https://pointclouds.org/documentation/tutorials/conditional_euclidean_clustering.html) · [평면 추정](https://pointclouds.org/documentation/tutorials/planar_segmentation.html) | 연결 조건·실제 지지 범위의 표면 분리. 다른 점을 통한 우회 연결·법선 비용·실물 손실 검사. 평면 전체를 무조건 삭제하지 않음 |

Autoware의 클래스별 형상 거부·크기 기본값을 우리16종/3그룹에 그대로 복사하지 않는다.
미확정 객체·부분 관측의 충돌 기하 보존, yaw 모호성, 참조 형상을 오래 유지하는 오류를 따로
검사한다. shape/연관 정책의 부분 대조이지 공용 메시지나 Noetic 실행 구조 교체가 아니다.

### 실패 원인이 확인될 때만 볼 후보

| 후보 / 원문·공식 구현 | 착수 조건과 우선순위 |
|---|---|
| 기존 E8 box + 깊이/연결 지지점 후보 · [과거 실측](../../../docs/records/lidar/2026-10-27-research-integration.md#수치를-해석할-때) | M1 이후 배경 혼입이 남을 때. 최근접 성분이 항상 객체가 아니므로 점/이웃 탐색 상한·원본 index·unknown 보존부터 구현 |
| **depth_clustering**, IROS2016/PFG2017. [공식 코드·논문 목록](https://github.com/PRBonn/depth_clustering#related-publications) | range-image 각도/깊이 연결 대조. MORAI VLP-16 투영·raw index·과분할 확인 후 사용 |
| **TRAVEL / AOS**, RA-L2022. [논문](https://arxiv.org/abs/2206.03190) · [공식 코드](https://github.com/url-kaist/TRAVEL) | 지면 전체 교체 대신 객체 군집 부분의 혼입/분리 대조. 과거 로컬 검토 기록을 현 runtime 수락으로 승격하지 않음 |
| **AB3DMOT**, IROS2020/ECCVW2020. [공식 코드·논문](https://github.com/xinshuoweng/AB3DMOT) | 동일 OBB sequence에 대한 단순 KF/Hungarian 대조군. 현재와 중복이 많아 전면 교체는 후순위 |
| **FEC: Fast Euclidean Clustering**, Drones2022. [저자 페이지·논문](https://yizhenlao.github.io/FEC/) · [공식 코드](https://github.com/YizhenLAO/FEC) | 군집 계산이 실측 병목일 때. 빨라지는 것과 벽/객체가 정확히 분리되는 것은 별도 |
| **Poly-MOT**, IROS2023. [논문](https://arxiv.org/abs/2307.16675) · [공식 코드](https://github.com/lixiaoyu2000/Poly-MOT) | 관측은 정상인데 회전·class별 운동 모델이 원인일 때. 불안정한 class/yaw를 강제 확정하지 않음 |
| **Fast-Poly**, RA-L2024. [논문](https://arxiv.org/abs/2403.13443) · [공식 코드](https://github.com/lixiaoyu2000/FastPoly) | 추적·연관 계산이 병목일 때. 저자 tracker FPS를 전체 인지 처리율로 사용하지 않음 |
| **Patchwork++**, IROS2022. [논문](https://arxiv.org/abs/2207.11919) · [공식 코드](https://github.com/url-kaist/patchwork-plusplus) | 실제 지면 분리 실패가 확인될 때. 벽/천장 semantic 분리기의 대체재로 취급하지 않음 |

위 조건과 비용·위험은 현재 코드에 대한 **적용 판단**이다. 논문 benchmark 성능이 MORAI에서
재현됐다는 뜻이 아니다. 지면·군집·tracker 전체 교체를 필수 과제로 두지 않는다.
학습 Plan B(CenterPoint-Pillar)와 Plan A(HARP 계열+영상 semantic)는 기존 구조 개선 뒤에도
한계가 남고 도메인 GT·GPU 예산을 확보할 때 별도 실험한다. 관련 실행·라벨 근거는
[동결 연구 갤러리와 한계](../../../docs/records/lidar/2026-10-27-research-integration.md)에 유지한다.

## YOLO 인수 — detect 우선, seg는 조건부

장원태에게 요청할 우선 산출물은 **기존16종/3그룹의 detect 추가학습 모델**이다. 원본 클래스 순서,
데이터/split SHA·ignore 규칙·epoch/EMA 출처·동일 val 비교·실패 사례·배포 export를 함께 인수한다.
현재 B1/B2·mask·confidence/NMS 설정에서 재검증하며 새 모델과 3뷰 알고리즘의 효과를 분리한다.

[공식 segmentation 출력](https://docs.ultralytics.com/tasks/segment/)에는 box·class·confidence와
instance mask가 있지만 현재 detect engine의 binding·후처리에 그대로 끼울 수 있다는 뜻은 아니다.
2D mask도 가림·배경·시각 불일치를 없애거나 LiDAR가 못 본 깊이를 제공하지 않는다.

과거 라벨 감사에서 train의 mask 메타데이터 양성은 native RGBA64,966 + packed5,162개였으나
box-only80,485개도 있었다. 완전 양성 프레임에서 모두 mask 메타데이터가 있는 train은186장,
실제 mask 점검은48개 표본이었다. 이를 전체 mask GT 검증 완료나 현재 데이터 재감사로 해석하지 않는다.
Partial GT를 배경으로 학습하거나 box를 사각 mask GT로 바꾸지 않는다.
우선 detect 개선을 납품하고 mask 원본/index를 보존한다. seg는 작은 고정 pilot에서 16종 mask
품질·box recall·원본 좌표 복원·TensorRT binding·3뷰 전체 지연을 검증한 뒤 별도 채택한다.

## 병행 개발의 경계와 수락

- 기존 기술 검토 경계는 승현: `fusion/`, 카메라 `geometry/`·수신·HD맵 보호 연결,
  재호: `experimental/`/터널 전처리, `detection/obb`, `tracking/`·운동 상태다.
  다른 팀원이 인수할 때 겹치는 변경을 먼저 확인한다. `pipeline/`, `ros/runtime`, trace와 공용 타입은 공동 리뷰한다.
- 점군·변환 context를 재사용하고 모델/뷰마다 깊은 복사를 추가하지 않는다. 같은 observation의
  재사용 가능성과 분류 확인 횟수는 서로 다른 개념이다.
- 세 작업 모두 `local`은 GPS와 독립, `map`은 유효 scan pose에서만 제공한다. 판제 입력·TTC·추종·
  정지·회피 정책은 판제 소유다. 진단 JSON을 판단 입력으로 연결하지 않는다.
- 새 알고리즘은 고정 입력·출력 차이·실패 사례·p50/p95/p99·RSS·공급 공백을 남긴다. 새 추론 모델은
  별도 회귀한다. 정상 공급≥10Hz·공백≤250ms는 기존 수락 목표이며 F1 정차473.47ms 실패를 면제하지 않는다.
- 동시 원본 시각/독립 GT가 없으면 비교 불가로 남긴다. 갤러리가 매끄럽다는 이유로 정확도·운영
  수락을 선언하지 않는다. 개발용 Ego GT는 평가에만 사용한다.

현재 실행과 복구는 [planning-runtime](planning-runtime.md), 상태 요약은 [공동 작업표](migration-tasks.md),
F1 재현은 [fusion-verification](fusion-verification.md)이 정본이다.

## PR #122 이후 실행 기반과 다음 단위 제안 — 2026-10-28

[기본 빌드 일치 작업](../../../docs/records/lidar/2026-10-28-canonical-runtime-build.md)은
알고리즘 개선과 분리한다. 병합된 소스·YAML·실행파일·웹을 `/root/ws`로 통일하고,
실행 전 소스 SHA 대조 및 실행 중 1Hz 빌드/모드 진단을 제공한다. 기존 터널 최적화와
3뷰 기본값을 실제 실행에 적용하는 작업이며 RANSAC 실패나 음영 주행 전체 해결을 뜻하지 않는다.

다음은 **제안/미착수**이며 새 구현 착수나 담당 변경을 확정한 목록이 아니다.

1. **음영 정지 경계 확인·복구 우선:** 최신 빌드 기준으로 GPS 무효 진입에서 선택된 제어원,
   local 객체 공급, family/BBox 유효성, 차선 변경 상태를 같은 scan/시각으로 확인한다.
   최근 로그에는 접점 겹침1%의 차로 불명 → 변경 중 원거리 객체 차로 불명 → 30초 정지 고착이
   기록됐다. 정상 MPC의 `INVALID_EGO`와 실제 선택된 음영 명령을 구분한다.
   불확실한 장애물을 무조건 무시하거나 timeout을 늘리는 대신, 분류/변경 상태가 다시 유효해졌을 때의
   재평가·복귀 조건을 판제와 정한다. `tracks_local`은 아직 현재 BBox 음영 제어기의 직접 입력이 아니다.
2. **터널 인지 결함이 확인되면 M2로 연결:** 지면/벽/잔여 구조물 중 RANSAC 실패 위치와 후보 점·평면 조건,
   실패 시 보존/대체 결과·local 공급 시간을 대조한다. 콘솔의 두 줄만으로 3뷰 지연이나 차량 정지를
   확정하지 않는다. 필요한 객체 소실·벽 오검출이 직접 원인이면 재호의 터널 처리와 공동으로 다룬다.
3. **객체 인지 개선:** 위 실행/정지 경계가 설명된 후 얇은 barricade의 실제 점 지지·군집·관측 확인·miss를
   먼저 분리하고 camera-guided 복구 후보 하나를 대조한다. 2D 검출만으로 3D 장애물을 생성하지 않는다.
   OBB 기준점/연관/속도는 M3/M4와 경계가 겹치지 않도록 한다.

통로 기반 새 local planner, ego motion, 모델 교체는 이 기반 작업에 포함하지 않는다.
기존 제어 경계의 결함을 확인한 다음 별도 계획으로 비교한다.
