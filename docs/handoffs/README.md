# 파트 간 인수인계 문서

> **문서 역할:** Active Handoff Index — 생산자·소비자 사이의 현재 인수 조건과 legacy 구분

Handoff 문서는 공용 message의 정본을 새로 정의하지 않는다. producer가 무엇을 내보내고
consumer가 어떤 freshness·quality·fallback을 검사해야 하는지, 어느 팀이 남은 결정을
소유하는지를 기록한다. message/type 자체는 `asmc_msgs`와 `contracts/`가 우선한다.

## 현재 handoff

| 생산자→소비자 | 문서 | 상태 |
|---|---|---|
| Camera 16종→LiDAR·판제 | [E8 배포·모델 선택·세 그룹](./perception/camera-mission16-deployment.md) | 현재 개발 기본; 전 구간 강건성·전체 ACTIVE 수락은 후속 |
| Camera 배포 이후 | [최적화·문서·E드라이브 수집·후속 학습](./perception/camera-deployment-roadmap-20261014.md) | 순차 후속 계획; 이번에 수집/학습 미실행 |
| Camera lane/BEV→Planning | [음영 camera v3](./perception/shadow-camera.md) | Active, read-only adapter 인수 완료; deployment 미승인 |
| Camera runtime 단계별 계획 | [카메라 런타임 계획](./perception/camera-runtime-plan.md) | Active Plan, 구간 활성화·10Hz·3뷰·모델별 최적화 후속 단계 |
| 판제→Camera 모델 실행 | [독립 추론 모드와 직접 실행](./perception/camera-inference-control.md) | PR #69 병합; 독립 준비·주기 수락 및 분류 차이 2건 승인. 연속 주행은 판제 통합 검증 |
| Camera objects P2a | [직접 launch와 검증](./perception/camera-objects-p2a.md) | Opt-in YOLO11m 생산자와 기존 LiDAR 호환 어댑터 |
| Camera detector→LiDAR fusion | [객체 검출](./perception/camera-object-detection.md) | 이전 YOLO11m/P2a 기록; 현재 계약은 E8 배포 문서 |
| Camera mask·LiDAR 장애물 융합 연구 | [YOLO26과 융합 후보 분석](./perception/camera-lidar-fusion-study.md) | Working Proposal, 공개 모델·데이터·소스 검수와 격리 비교 계획. 운영 미적용 |
| Camera·LiDAR 통합 부하 | [B0 기준선 프로토콜](./perception/camera-lidar-b0.md) | 현행 모델 6조합·fusion·GPS 진단, 운영 기본값 변경 없음 |
| Behavior→Frenet v1 | [legacy 3-package 설계](./planning/behavior-legacy.md) | Legacy Reference, 현재 실행 금지 |
| 규정집 v1.2→전 파트 | [v1.2 영향과 확인 요청](./competition/rule-v1.2-impacts.md) | Active Handoff, 판정 전용 채널·cmd type·속도 상한 |
| 장애물 출제 범위→Camera·LiDAR·판제 | [MORAI 객체·분류·속도 GT](./competition/obstacle-assets-and-labels.md) | 10/06 전달 Q&A 요약; 원본 목록·과거 후보 보존; 현재16종/3그룹은 E8 계약 |
| HD맵→판제 | [전역 지도·미션 경계·GPS 상태](./hd-map/planning-consumption-v1.md) | Active Handoff, v6 지도 bundle·후륜축·station·음영 fallback 반영 항목 |
| HD맵 미션 구간→시나리오 생성기·판제 | [미션 시나리오 spec](./hd-map/mission-scenario-spec.md) | Active Handoff, 구간별 회피 차로·actor 후보·샘플링 API, 값은 계속 수정 가능 |

현재 Behavior↔Frenet production 계약은 handoff 문서가 아니라
[FSM↔Frenet v2 frozen contract](../contracts/planning/fsm-frenet-v2.md)가 소유한다.
Planning 전체 경계와 실행은 [개발 가이드](../packages/planning-control/README.md)와
[운영 규약](../operations/planning-control.md)을 따른다.

## Handoff에 반드시 남길 것

- producer topic, message type, frame, units와 source timestamp
- freshness, invalid/partial/degraded 상태와 consumer reject 조건
- artifact/model/config 식별자와 재현 가능한 bag 또는 test
- producer가 소유하는 판단과 consumer가 소유하는 정책
- 본선 허용 여부와 privileged 개발 입력 사용 여부
- rollback 경로와 closed-loop 승격 gate

성능 숫자는 데이터·checkpoint·환경이 명시된 record를 연결한다. 특정 날짜의 측정값을
영구 contract로 복사하지 않는다.

## 상태 해석

- **Active Handoff:** 현재 producer와 consumer가 검토할 수 있는 인수 조건이다.
- **Deployment approved:** 별도 full-stack/field gate를 통과한 경우에만 명시한다.
- **Legacy Reference:** 과거 설계 근거이며 현재 launch나 topic에 적용하지 않는다.

따라서 `Active Handoff`라고 적혀 있어도 `deployment_allowed=false`, read-only 또는 미연결
상태일 수 있다. 각 문서의 제한을 함께 읽는다.
