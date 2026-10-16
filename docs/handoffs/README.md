# 파트 간 인수인계 문서
<!-- documentation-structure-v2:start -->

## 연결별 바로가기

- [카메라→Main/LiDAR](camera-to-lidar.md)
- [카메라 음영 차선→판제](camera-to-planning.md)
- [HD맵→판제](hd-map-to-planning.md)
- [HD맵 미션 시나리오](hd-map-mission-scenario.md)
- [손재호 LiDAR 이전 책임·계획](lidar-migration.md)

아래 기존 목차의 상태 설명은 이번 경로 이전에서 갱신하지 않았다.
실험·단계별 계획의 링크는 records로 이동했으며 내용 현행화는 다음 단계에서 진행한다.

<!-- documentation-structure-v2:end -->

카메라→기존 Main→판제의 현재 실행/소비 규칙과 손재호 이전 경계는 [LiDAR 카메라 인계](camera-to-lidar.md)가 정본이다.


> **문서 역할:** Active Handoff Index — 생산자·소비자 사이의 현재 인수 조건과 legacy 구분

Handoff 문서는 공용 message의 정본을 새로 정의하지 않는다. producer가 무엇을 내보내고
consumer가 어떤 freshness·quality·fallback을 검사해야 하는지, 어느 팀이 남은 결정을
소유하는지를 기록한다. message/type 자체는 `asmc_msgs`와 `contracts/`가 우선한다.

## 현재 handoff

| 생산자→소비자 | 문서 | 상태 |
|---|---|---|
| Camera 자료·수집 도구 통합 | [PR81/82 도구 통합·데이터 구분](../records/camera/camera-data-integration-prep.md) | v6/v7 자료 도구·Capture 수집 통합; 원본/학습 재고/배포 모델 구분 |
| Camera 16종→LiDAR·판제 | [E8 배포·모델 선택·세 그룹](../operations/camera-deployment.md) | 현재 개발 기본; 전 구간 강건성·전체 ACTIVE 수락은 후속 |
| Camera 배포 이후 | [최적화·문서·E드라이브 수집·후속 학습](../records/camera/camera-deployment-roadmap-20261014.md) | 순차 후속 계획; 이번에 수집/학습 미실행 |
| Camera lane/BEV→Planning | [음영 camera v3](./camera-to-planning.md) | Active, read-only adapter 인수 완료; deployment 미승인 |
| Camera runtime 실행 정본 | [카메라 실행 계약](../contracts/camera-runtime-v1.md) | 3뷰·YOLO/TwinLite 목표15Hz·최소10Hz·모드와 관측 계약 |
| TwinLite TensorRT 검증 | [변환·부하·보류 근거](../records/camera/camera-twinlite-tensorrt.md) | PR #85 후보; 품질 통과와 후단 readiness 수락을 구분 |
| 음영 lane 실행 최적화 | [family 전용·BEV 수요·GC 진단](../records/camera/camera-shadow-family-optimization.md) | PR #85 후속; GPU/음영 반복·장기 수락 대기 |
| Camera runtime 이전 단계별 계획 | [카메라 런타임 계획](../records/camera/camera-runtime-plan.md) | 이전10Hz 단계 계획; 최신 실행은 위 계약을 따름 |
| 판제→Camera 모델 실행 | [독립 추론 모드와 직접 실행](../records/camera/camera-inference-control.md) | PR #69 병합; 독립 준비·주기 수락 및 분류 차이 2건 승인. 연속 주행은 판제 통합 검증 |
| Camera objects P2a | [직접 launch와 검증](../records/camera/camera-objects-p2a.md) | Opt-in YOLO11m 생산자와 기존 LiDAR 호환 어댑터 |
| Camera detector→LiDAR fusion | [객체 검출](../records/camera/camera-object-detection.md) | 이전 YOLO11m/P2a 기록; 현재 계약은 E8 배포 문서 |
| Camera mask·LiDAR 장애물 융합 연구 | [YOLO26과 융합 후보 분석](../records/camera/camera-lidar-fusion-study.md) | Working Proposal, 공개 모델·데이터·소스 검수와 격리 비교 계획. 운영 미적용 |
| Camera·LiDAR 통합 부하 | [B0 기준선 프로토콜](../records/camera/camera-lidar-b0.md) | 현행 모델 6조합·fusion·GPS 진단, 운영 기본값 변경 없음 |
| Behavior→Frenet v1 | [legacy 3-package 설계](../records/archive/planning-behavior-legacy.md) | Legacy Reference, 현재 실행 금지 |
| 규정집 v1.2→전 파트 | [v1.2 영향과 확인 요청](../competition/rule-v1.2-impacts.md) | Active Handoff, 판정 전용 채널·cmd type·속도 상한 |
| 장애물 출제 범위→Camera·LiDAR·판제 | [MORAI 객체·분류·속도 GT](../competition/obstacle-assets-and-labels.md) | 10/06 전달 Q&A 요약; 원본 목록·과거 후보 보존; 현재16종/3그룹은 E8 계약 |
| HD맵→판제 | [전역 지도·미션 경계·GPS 상태](./hd-map-to-planning.md) | Active Handoff, v6 지도 bundle·후륜축·station·음영 fallback 반영 항목 |
| HD맵 미션 구간→시나리오 생성기·판제 | [미션 시나리오 spec](./hd-map-mission-scenario.md) | Active Handoff, 구간별 회피 차로·actor 후보·샘플링 API, 값은 계속 수정 가능 |

현재 Behavior↔Frenet production 계약은 handoff 문서가 아니라
[FSM↔Frenet v2 frozen contract](../contracts/planning-fsm-frenet-v2.md)가 소유한다.
Planning 전체 경계와 실행은 [개발 가이드](../architecture/planning-control.md)와
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
