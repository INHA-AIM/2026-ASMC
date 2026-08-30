# integration (논리 그룹)

MORAI bridge·UDP·런타임 연동. **실제 스크립트는 repo 루트 `scripts/`** (이 폴더는 README만).

| 경로 | 내용 | 담당 | 상태 |
|------|------|------|------|
| `scripts/rosbridge.sh` | roscore + **rosbridge** (WebSocket) | 안승현 | ✅ |
| `scripts/udp_bridge.sh` | **UDP** bridges (대회 경로) | 이강태·안승현 | ✅ |
| `scripts/wait_morai_topic.sh` | 토픽 대기 | 안승현 | ✅ |
| `scripts/diagnose_morai_rosbridge.sh` | rosbridge 연결 진단 | 안승현 | ✅ |
| `config/morai_rosbridge.env` | rosbridge 포트·ROS env | 안승현 | ✅ |
| `tools/grpc_inha_univ/` | gRPC 제어 브릿지 | 장원태 | ✅ |
| UDP `Ctrl_cmd` | planning | 정윤태 | ⬜ `feature/yuntae-*` |

상세: [docs/team-dev-guide.md](../../docs/team-dev-guide.md)

```bash
# 개발용 rosbridge (WebSocket) — 당일/규정 경로 아님
./scripts/rosbridge.sh
./scripts/diagnose_morai_rosbridge.sh

# 대회 UDP 경로
./scripts/udp_bridge.sh
```
