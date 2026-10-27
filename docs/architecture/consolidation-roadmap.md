# 대회 완료 로드맵

목표는 허용 센서로 전체 미션을 반복 완주하고, 충돌·규정 위반·인지/제어 공백을 줄이면서
주행 시간을 개선하는 것입니다. 모델 점수나 패키지 이전 완료만으로 대회 준비를 판정하지 않습니다.
담당은 [현재 ownership](../onboarding/ownership.md)을 유지하며 아래는 기존 역할에 따른 다음 작업 제안입니다.
LiDAR 후속 책임은 [M0–M5 인수안](../../src/asmc_lidar/docs/improvement-milestones.md)를 반영하며 수락 완료와 구분합니다.

## 현재 기준선

- PR #85: 전방960·TwinLite TensorRT·family 공급 최적화, 사용자 직접 시각화 확인 후 병합.
- PR #87: YOLO를 카메라로 단일화, 기존 Main의 전방 어댑터·판제 wire 유지.
- PR #86: `tools/scen_runner` 전체 씬/구간 검증·판제 재시작·충돌/정체 저장·재현.
- PR #89: UTM 방향 정합·고속 추종·조건부 순항/곡선 속도·명령 watchdog.
- 문서 구조 변경 #88과 이번 현행화는 코드 완료 상태와 별도입니다. 병합 전 문서를 main 적용으로 오인하지 않습니다.

- B4 정본화·구 Main/Msg 폐기와 C0 좌표/시각 수정, PR #118 F1 전방 연결 검증은 병합됐다.
  [현행 인계](../handoffs/lidar-migration.md)의 제한을 함께 본다. 위 초기 PR은 역사적 기준선이다.

## 담당별 다음 결과와 완료 증거

| 우선순위·담당 | 다음 결과 | 확인할 기준 |
|---|---|---|
| 1 · 판제/제어 정윤태·양서준·이강태, 통합 공동 | 현재 단일 planner + asmc_lidar + 카메라 기준선의 전체 미션 반복 | SUNNY/FOGGY×11/13/15, 실제 최고속도·충돌·정체·완주시간, STOP/재밍 진입·복귀·명령 단절 |
| 1 · 안승현 + 소비 파트 | 관측·원본 이름/그룹·상태·pose 인계 확인 | 단일 producer, source stamp·좌표·세대·freshness, unknown/무효/빈 결과를 구분 |
| 1 · 손재호 + 판제 | LiDAR 객체/구조물·속도·분류의 소비 신뢰성 | 객체 보존·벽 혼입·추적 ID·GPS 무효/복구, LiDAR·판제 UTM yaw 일치, 미확정 속도를 0으로 치환하지 않음 |
| 1 · 안승현 M1, 손재호 M2–M4 | [3뷰 fusion·터널·관측 위치·추적 개선](../../src/asmc_lidar/docs/improvement-milestones.md) | 관측 식별/중복·위치/속도 유효성·실물 보존·공급. 동시 전면 교체 없이 한 단위씩 대조 |
| 2 · 장원태 + 판제 | 시나리오 검증·실패 재현을 반복 가능한 팀 절차로 고정 | 객체/환경/seed/지도 SHA와 충돌·정체 이력; CP14→START 조건은 공식 모든 체크포인트 검사와 별도 |
| 2 · 안승현 + 지도 소비 파트 | 지도·차량 상태 정본과 판제 map copy 정합 | 링크/route/원점·yaw·checkpoint parity, 출처 경로 기록 오류와 실제 지도 값 변경 구분 |
| 병행 · 장원태, 승현 모델 인수 | YOLO detect 추가학습·고정 val/배포 회귀 | bus/truck, 차로 앞 bicycle/dog, dog2/dog10, 터널·근거리; 기존 객체 회귀와 실행 지연 확인 |
| 최종 · 전체 | 현장 배포·복구·연속 주행 수락 | 주최 환경·LAN2LAN·허용 입력·cold start·재시작·센서 중단·디스크/GPU 공급 확인 |

## 지금 닫지 않은 항목

L0–L8/A1·B4의 이전 기록과 현재 개선 계획을 분리한다. `asmc_lidar`는 판제·수집·시나리오 정본이며
구 Main/Msg는 폐기됐다. C0/F1은 병합됐지만 원본 clock 대응·독립 위치/속도 GT·전체 공급 수락은
완료되지 않았다. [공동 작업표](../../src/asmc_lidar/docs/migration-tasks.md)를 따른다.
재호의 PR #119 터널 처리는 10/27 확인 시점에 오프라인 검증 후 미병합이다. F1과 통합·실시간
검증을 남기며 승현의 M1 3뷰 fusion과 병행한다. TTC·추종·정지·회피는 판제 정책으로 유지한다.

카메라 TensorRT가 기본이어도 family 순간 공백250ms 기준과 freshness350ms의 남은 실패는 추적합니다.
YOLO는 전 조건 모든 에셋 지원을 검증한 상태가 아닙니다. 장원태가 detect 추가학습을 진행하며 seg는 별도 라벨/실행 pilot 이후 판단합니다.
`asmc_lidar`는 검출·추적·전방 fusion·공용 관측·전달 필터와 기본 판제 입력을 제공합니다. 기본 `/perception/lidar`, 병행 비교 `/migration/lidar`이며 3뷰 YOLO 출력이 3뷰 LiDAR fusion 완료를 뜻하지 않습니다. 손재호는 터널 통합 뒤 관측 위치/OBB·추적/속도/운동 상태를, 안승현은 3뷰 fusion·HD맵·카메라 연결을 맡습니다. 흰선 밖의 여유 공간에 놓인 객체를 일괄 제거하지 않습니다.

판제의 최신 주행 개선과 전체 회귀 통과는 다릅니다. 이전 이관 기록의 AVOID/link 실패와
#89가 보고한 `roundabout_execution_test` 실패는 시점별로 구분해 해결·재검증합니다.
회전교차로 이후 평균40km/h는 보고됐으나 재밍 이후40km/h와 전체 이동 평균의 뚜렷한 상승은 미달성입니다.
신호/장애물 정지를 포함한 평균과 구간 이동 평균을 분리해 평가합니다.

현재 지도 products와 판제 local map은 자동 일치하지 않습니다. 전체 지도 재생성 검사에 남은
route source 경로 provenance 차이는 [현행화 검토 기록](../records/repository/2026-10-16-documentation-refresh.md)에 남깁니다.
현재 CompetitionInfoPublisher는9099→`/Ego_topic`으로 실측 확인했습니다([정정](../records/bridge/2026-10-23-competition-channel-audit.md)). 주행 부호·본선 LAN2LAN 수락은 남았습니다.

## 단계별 변경 규칙

먼저 현재 기준선의 실패를 저장·재현하고, 원인에 맞는 작은 변경 하나를 대조합니다.
LiDAR 이전·모델 재학습·속도 튜닝·공용 계약 변경을 한 실험에 섞지 않습니다.
출력 동등성·source 시각·구조적 유효성, full-stack 부하, 전체 미션/조건 반복을 차례로 확인합니다.
평균Hz만으로 순간 공백을 면제하거나 그룹 F1으로 차량 위치 누락을 숨기지 않습니다.

필수 수락이 끝나기 전에는 추가 seg 학습, 공통 BEV 재설계, RF-DETR 재비교, LBC 연구를 우선하지 않습니다.
GPU·디스크 최적화는 계측으로 확인한 병목에 적용합니다. E드라이브 이전은 복사·해시·링크·컨테이너 검증 뒤 원본을 정리합니다.

[현재 실행](../operations/commands.md) · [카메라 로드맵](perception-roadmap.md) ·
[LiDAR 이전 상세](../handoffs/lidar-migration.md) · [이전 단계별 계획](../records/repository/consolidation-roadmap-through-pr87.md)
