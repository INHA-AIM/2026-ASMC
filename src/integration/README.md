# integration (논리 그룹)

> **역할:** Active Index — MORAI UDP bridge와 내부 ROS 런타임 진입점
> **운영 정본:** [개발·시뮬레이터 운영 가이드](../../docs/operations/README.md)
> **구현·통합:** [MORAI bridge guide](../../docs/packages/asmc_bridge/README.md)
> **문서 허브:** [docs/README.md](../../docs/README.md)

실제 스크립트는 repo 루트 `scripts/`에 있고 이 폴더에는 README만 있다. 기본 경로는
MORAI UDP → bridge → 팀 PC 내부 ROS다.

| 경로 | 내용 | 담당 | 상태 |
|------|------|------|------|
| `scripts/udp_bridge.sh` | 허용 UDP → 내부 ROS sensor/status bridge | 이강태·안승현 | 현재 기본 |
| `scripts/rosbridge.sh` | roscore + WebSocket rosbridge | 안승현 | 종료된 direct ROS 실험 재현용, 본선/현재 개발 금지 |
| `scripts/wait_morai_topic.sh` | 토픽 대기 | 안승현 | ✅ |
| `scripts/diagnose_morai_rosbridge.sh` | rosbridge 연결 진단 | 안승현 | ✅ |
| `config/morai_rosbridge.env` | rosbridge 포트·ROS env | 안승현 | ✅ |
| `tools/grpc_inha_univ/` | gRPC 제어 브릿지 | 장원태 | ✅ |
| UDP `Ctrl_cmd` | standalone planning/control | 정윤태·양서준 | 현재 경로는 planning 규약 참고 |

상세: [docs/onboarding/ownership.md](../../docs/onboarding/ownership.md)

```bash
# 현재 개발 경로
./scripts/udp_bridge.sh --simulator 26r1

# packet provenance label만 competition으로 지정
./scripts/udp_bridge.sh --simulator competition

# 과거 direct ROS 실패를 재현할 때만 사용 — 현재 개발·본선 금지
./scripts/rosbridge.sh
./scripts/diagnose_morai_rosbridge.sh
```

두 `--simulator` 값은 현재 포트·IP·allowlist를 바꾸지 않는다. 따라서 두 번째 명령은 아직
현장 배포 profile이 아니며, Object/Semantic/Instance가 빠진 본선 launch는 Phase 3에 구현한다.

`asmc-ros-noetic` 컨테이너는 이 레거시 rosbridge와 같은 뜻이 아니다. UDP bridge가 MORAI
packet을 내부 ROS topic으로 정규화하고 인지 노드를 실행하는 현재 필수 환경이다.

Capture Mode gRPC와 시나리오 gRPC는 개발·데이터 수집 도구이며 본선 runtime에 포함하지
않는다. 본선 launch에는 규정 외 oracle/GT subscriber가 없는지 별도 검사한다.
