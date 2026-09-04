# drivable_bev (placeholder)

> **역할:** camera/MGeo/LiDAR를 같은 ego-centric grid에 놓기 위한 공통 BEV 후보
> **상태:** Working Placeholder — live camera와 map crop pilot 전에는 인터페이스 미확정
> **정본 제안:** [인지 융합 로드맵](../../../docs/perception-map-fusion-roadmap.md)

## 예정 범위

- 현재 sensor set의 calibration과 letterbox 역변환 검증
- 카메라별 lane/drivable probability의 IPM 투영
- MGeo drivable/lane/route/stopline local crop 수용
- LiDAR occupancy를 별도 layer로 유지
- coverage/confidence를 보존한 camera overlap 결합
- ego lane, 인접 차로와 `safe_drivable` 구조화

## 예정하지 않는 범위

- RGB 세 장을 합친 뒤 모델을 다시 추론하는 방식
- LiDAR 장애물을 정적 map에 미리 bake하는 방식
- GPS pose가 불확실한데 map crop을 확정 GT로 사용하는 방식
- behavior/planner의 최종 차로변경 결정

## 공통 계약 전 최소 확인

1. 카메라 front/3-view live 품질과 p50/p95 latency
2. 현재 sensor set의 실제 내·외부 파라미터
3. MGeo local crop의 좌표·차로폭·route 연결 정확도
4. `base_link` 축, ego origin, BEV 범위와 meter/pixel 후보
5. planning과 LiDAR가 실제 소비할 raster/vector 필드

임시 토픽과 열린 결정사항은 [로드맵 §6](../../../docs/perception-map-fusion-roadmap.md#6-working-topicinterface-proposal)에
기록한다. 합의가 끝난 값만 [`src/interfaces`](../../interfaces/README.md)로 옮긴다.

초기 Loc-Aid/agentic 평가 아이디어는
[perception-agentic-loop.md](../../../docs/perception-agentic-loop.md)에 연구 기록으로 보존한다.
