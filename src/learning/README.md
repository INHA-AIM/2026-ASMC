# learning (논리 그룹)

학습·로깅·LBC. **실제 코드는 `src/learning_by_cheating/`** (이 폴더는 README만).

| 경로 | 담당 | 상태 |
|------|------|------|
| `src/learning_by_cheating/` | 안승현 | ✅ PR #2 |
| `mgeo_toolkit/` | 안승현 | ✅ |
| `R_KR_PG_KATRI/` | — | 맵 JSON |
| `scripts/rosbridge.sh` (WebSocket; UDP는 `udp_bridge.sh`) | 안승현 | ✅ |

상세: [docs/team-dev-guide.md](../../docs/team-dev-guide.md) §4

## 빠른 시작

```bash
./scripts/rosbridge.sh
source devel/setup.bash
./src/learning_by_cheating/scripts/start_lbc_morai.sh
```

## 주의

- beta: `wheel_angle`, `CtrlCmd.steering`
- jang gRPC multi-TL visualizer **미포함** (D-09)
- NPZ·체크포인트 git 금지
