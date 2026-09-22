# learning (논리 그룹)

> **역할:** Research/Legacy Index — 기존 LBC 학습·로깅 진입점
> **현재 카메라 인지:** [`src/perception/`](../perception/README.md)
> **문서 허브:** [docs/README.md](../../docs/README.md)

실제 LBC 코드는 `src/learning_by_cheating/`에 있고 이 폴더에는 README만 있다. 현재
lane/drivable 학습·live 추론은 `src/perception/twinlite_morai/`와 이후
`camera_semantic_perception/`에서 진행한다.

| 경로 | 담당 | 상태 |
|------|------|------|
| `src/learning_by_cheating/` | 안승현 | ✅ PR #2 |
| `mgeo_toolkit/` | 안승현 | ✅ |
| `R_KR_PG_KATRI/` | — | 맵 JSON |
| `scripts/rosbridge.sh` | 안승현 | 종료된 direct ROS 실험 재현용, 본선/현재 개발 금지 |
| `scripts/udp_bridge.sh` | 이강태·안승현 | 현재 센서 ROS 변환 경로 |

상세: [docs/onboarding/team-dev-guide.md](../../docs/onboarding/team-dev-guide.md) §4

## LBC 재현용 시작

```bash
./scripts/udp_bridge.sh --simulator 26r1
source devel/setup.bash
./src/learning_by_cheating/scripts/start_lbc_morai.sh
```

과거 WebSocket 실패 조건을 재현해야 할 때만 `rosbridge.sh`를 사용한다. ROS GT transport
비교는 종료됐고 대회판과 현재 카메라 live 검증의 기본 경로는 UDP bridge다.

## 주의

- beta: `wheel_angle`, `CtrlCmd.steering`
- jang gRPC multi-TL visualizer **미포함** (D-09)
- NPZ·체크포인트 git 금지
- LBC의 192×192 BEV를 새 camera/map/LiDAR 공통 BEV 규격으로 간주하지 않음
