# 2026-ASMC 팀 문서

> **최종 수정:** 2026-07-25  
> **대상:** 팀원 전원  
> **통합 리드:** 안승현  

## 읽는 순서

1. [getting-started.md](./getting-started.md) — 클론·첫 실행
2. [collaboration.md](./collaboration.md) — **★ Phase0/1 · WIP · Path A/B (필독)**
3. [network-udp-ros.md](./network-udp-ros.md) — **UDP↔ROS (26.R1.h3 · `127.0.0.1` Docker)**
   · 치트시트 [config/morai_udp_sim_cheat_sheet.md](../config/morai_udp_sim_cheat_sheet.md)
4. [team-dev-guide.md](./team-dev-guide.md) — 팀원별 경로 (Phase 0 개인 레포 표)
5. [planning-control-udp.md](./planning-control-udp.md) — **Planning/Control UDP-only 정본**
6. [integration-plan.md](./integration-plan.md) — Wave 1/2
7. [perception-agentic-loop.md](./perception-agentic-loop.md) — Loc-Aid/bake eval
8. [architecture-overview.md](./architecture-overview.md)
9. [competition-environment.md](./competition-environment.md)
10. [git-workflow.md](./git-workflow.md)
11. [sim-verification-checklist.md](./sim-verification-checklist.md)
12. [morai-msgs-beta-policy.md](./morai-msgs-beta-policy.md)
13. [morai-msgs-versions.md](./morai-msgs-versions.md) — msg 브랜치 · **대회 ROS 실측 요약**
14. (로컬) [morai-competition-ros-topics.md](../../docs/competition/morai-competition-ros-topics.md) — 대회 UI 토픽 전부

## 본 repo에 포함되는 것

| 포함 | 미포함 (로컬·별도 관리) |
|------|-------------------------|
| catkin 패키지·통합 코드 | 맵·대용량 데이터 (`ASMC_DATA`) |
| Docker·빌드 스크립트 | 학습 체크포인트 |
| 팀 작업 규약·아키텍처 | 논문 PDF, 베이스라인 코드 clone |

베이스라인·MORAI 예제 코드는 필요 시 각자 GitHub에서 clone하여 **참조만** 합니다. 본 repo에 복사하지 않습니다.
