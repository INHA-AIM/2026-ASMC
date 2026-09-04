# integration (논리 그룹)

> **역할:** Active Index — MORAI UDP bridge와 내부 ROS 런타임 진입점
> **네트워크 정본:** [network-udp-ros.md](../../docs/network-udp-ros.md)
> **문서 허브:** [docs/README.md](../../docs/README.md)

실제 스크립트는 repo 루트 `scripts/`에 있고 이 폴더에는 README만 있다. 기본 경로는
MORAI UDP → bridge → 팀 PC 내부 ROS다.

| 경로 | 내용 | 담당 | 상태 |
|------|------|------|------|
| `scripts/udp_bridge.sh` | 허용 UDP → 내부 ROS sensor/status bridge | 이강태·안승현 | 현재 기본 |
| `scripts/rosbridge.sh` | roscore + WebSocket rosbridge | 안승현 | 개발용 레거시, 본선 금지 |
| `scripts/wait_morai_topic.sh` | 토픽 대기 | 안승현 | ✅ |
| `scripts/diagnose_morai_rosbridge.sh` | rosbridge 연결 진단 | 안승현 | ✅ |
| `config/morai_rosbridge.env` | rosbridge 포트·ROS env | 안승현 | ✅ |
| `tools/grpc_inha_univ/` | gRPC 제어 브릿지 | 장원태 | ✅ |
| UDP `Ctrl_cmd` | standalone planning/control | 정윤태·양서준 | 현재 경로는 planning 규약 참고 |

상세: [docs/team-dev-guide.md](../../docs/team-dev-guide.md)

```bash
# 기본 개발·대회 경로
./scripts/udp_bridge.sh

# WebSocket 기능을 별도로 재현할 때만 사용 — 본선 금지
./scripts/rosbridge.sh
./scripts/diagnose_morai_rosbridge.sh
```

Capture Mode gRPC와 시나리오 gRPC는 개발·데이터 수집 도구이며 본선 runtime에 포함하지
않는다. 본선 launch에는 규정 외 oracle/GT subscriber가 없는지 별도 검사한다.
