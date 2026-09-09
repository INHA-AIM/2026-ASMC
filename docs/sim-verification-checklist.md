# MORAI 시뮬 검증 체크리스트

> **문서 역할:** Active Checklist — 공통/파트별 시뮬 검증
> **주의:** `Legacy/개발 전용`으로 표시된 rosbridge·gRPC 항목은 본선 gate가 아니다.
> **정본 연결:** 허용 통신과 포트는 [network-udp-ros.md](./network-udp-ros.md), Planning은 [planning-control-udp.md](./planning-control-udp.md)
> **문서 허브:** [README.md](./README.md)
> **목적:** 코드 통합(catkin build) 이후, **실제 MORAI 시뮬**에서 담당 파트별로 확인할 항목을 기록합니다.  
> **원칙:** 통합 PR은 build green으로 merge 가능. 아래 항목은 **merge 후·대회 전** 팀원이 로컬에서 수행하고 결과를 PR 코멘트·노션·이슈에 남깁니다.

**최종 수정:** 2026-09-05

---

## 0. 공통 사전 조건 (모든 파트)

| # | 확인 | 담당 | 통과 기준 |
|---|------|------|-----------|
| 0.1 | WSL clone 위치 | 전원 | `~/...` ( `/mnt/c` 아님 ) |
| 0.2 | Docker WSL 연동 | 전원 | `docker ps` 동작 |
| 0.3 | `git submodule update --init` | 전원 | `src/MORAI-ROS_morai_msgs` @ `45c6baf` |
| 0.4 | catkin build | 전원 | `./scripts/build_ws.sh` green |
| 0.5 | MORAI SIM 실행 | 전원 | Windows 26.R1.h3에서 K-City 2025와 대회 sensor set 로드 |
| 0.6 | UDP Bridge IP/PORT | 전원 | Network/Sensor Host·Dest와 [UDP 치트시트](../config/morai_udp_sim_cheat_sheet.md) 일치 |
| 0.7 | 허용 센서 ROS 변환 | 전원 | Ego, camera, LiDAR, GPS, IMU 핵심 토픽 수신 |

**공통 명령**

```bash
./scripts/udp_bridge.sh --simulator 26r1        # 26.R1.H3 개발
# 또는: ./scripts/udp_bridge.sh --simulator competition
docker exec -it asmc-ros-noetic bash         # 또는 네이티브 Noetic
cd /root/ws && source devel/setup.bash
rostopic hz /Ego_topic /image_jpeg/compressed /velodyne_points /gps /imu
```

---

## 1. Planning/Control (standalone UDP)

| # | 확인 | 담당 | 통과 기준 |
|---|------|------|-----------|
| 1.1 | standalone build | 정윤태·이강태 | `./scripts/build_planning_control.sh` 성공 |
| 1.2 | global path 로드 | 이강태 | Planner 로그에 본선 global path 로드 성공 |
| 1.3 | GPS/IMU UDP | 정윤태 | `9301/9303` 수신 후 localization 유효 |
| 1.4 | 임시 객체 입력 | 정윤태 | 센서 연동 전 ObjectInfo `7505` 수신·장애물 반응 확인 |
| 1.5 | PlannedPath UDP | 이강태 | Planner 송신과 MPC `127.0.0.1:9300` 수신 로그 |
| 1.6 | CtrlCmd UDP | 정윤태·양서준 | MPC가 MORAI `9111`로 송신하고 차량 이동 |
| 1.7 | MPC tracking | 정윤태·양서준 | 경로 이탈·조향 진동 없이 추종 |
| 1.8 | 프로세스 종료 | 정윤태 | `Ctrl+C` 후 두 프로세스와 UDP socket 모두 종료 |
| 1.9 | 저속·곡률 | 정윤태 | 급커브·저속 구간에서 이탈·진동 없음 |

**비고:** Behavior Planner는 이 파이프라인에 연결하지 않는다. 상세 규약은 [planning-control-udp.md](./planning-control-udp.md)를 따른다.

---

## 2. Learning / LBC (Legacy/선택)

이 절은 기존 LBC 실험 재현용이다. 현재 카메라 lane/drivable 우선순위는
[인지 융합 로드맵](./perception-map-fusion-roadmap.md) §7을 따른다.

| # | 확인 | 담당 | 통과 기준 |
|---|------|------|-----------|
| 2.1 | beta msg | 안승현 | `rosmsg show morai_msgs/EgoVehicleStatus` → `wheel_angle` |
| 2.2 | BEV spec (오프라인) | 안승현 | `python3 src/learning_by_cheating/scripts/verify_lbc_spec.py` ALL PASSED |
| 2.3 | LBC imshow (선택) | 안승현 | `start_lbc_imshow.sh` — ego·신호등·동적 객체 시각적 정합 |
| 2.4 | LBC collector | 안승현 | `start_lbc_morai.sh` — NPZ 저장, 채널 7개 shape |
| 2.5 | 신호등 ROS | 안승현 | ego 진행 방향 그룹 색이 시뮬과 **대체로** 일치 (완벽 불필요, 한계 문서화) |
| 2.6 | autonomous_driving | 안승현 | 학습 체크포인트 + `/ctrl_cmd` `steering` publish |

**의도적 미검증 (D-09):** jang gRPC multi-TL visualizer — 장원태·안승현 페어로 추후.

---

## 3. Integration / Bridge

| # | 확인 | 담당 | 통과 기준 |
|---|------|------|-----------|
| 3.1 | UDP bridge | 안승현 | `morai_udp_bridge`, `morai_sensor_bridge` 모두 실행 |
| 3.2 | UDP 수신 | 안승현 | bridge의 Ego/camera/LiDAR/GPS/IMU rx 증가 |
| 3.3 | 토픽 rate·age | 안승현 | 핵심 토픽이 설정 Hz 부근이고 장시간 정지하지 않음 |

`rosbridge.sh`와 WebSocket 9090 검증은 레거시 개발 편의 항목이며 본선 경로가 아니다.

---

## 4. Scenario runner (gRPC)

이 절은 26.R1.h3의 개발·데이터 수집 자동화 검증용이다. 본선 실행 profile에는 포함하지 않는다.

| # | 확인 | 담당 | 통과 기준 |
|---|------|------|-----------|
| 4.1 | gRPC 연결 | 장원태 | `config/runtime.yaml` host (WSL→Windows IP) |
| 4.2 | `run.sh` smoke | 장원태 | `./tools/aim_scenario_runner/run.sh` — 시나리오 시작·종료 |
| 4.3 | random_route_drive | 장원태 | ego route·cruise 또는 pure pursuit 동작 |
| 4.4 | NPC spawn | 장원태 | `npc.enabled: true` 시 주변 차량 유지 |
| 4.5 | save_ego_pose | 장원태 | `tools/save_ego_pose.py` — yaml·preview 저장 |
| 4.6 | route BEV viz | 장원태 | **기본 off.** 켤 때만 va LBC visualizer 연동 확인 |

**gRPC host 확인**

```bash
ip route | grep default | awk '{print $3}'   # WSL → Windows
# config/local_override.yaml 에 반영
```

---

## 5. Perception

### 5.1 Camera lane/drivable

| # | 확인 | 담당 | 통과 기준 |
|---|---|---|---|
| 5.C1 | frozen checkpoint | 안승현 | EMA/checkpoint SHA와 model source commit 일치 |
| 5.C2 | front live | 안승현 | drivable/lane probability와 overlay 정상 |
| 5.C3 | three-view live | 안승현 | front/left/right batch, timestamp skew와 stale 기록 |
| 5.C4 | runtime profile | 안승현 | FPS, p50/p95 latency, drop과 VRAM 기록 |
| 5.C5 | condition matrix | 안승현 | Sunny/Foggy × 11am/1pm/3pm 육안·독립 test 기록 |

현재 5.C1~C5는 다음 구현/실증 gate이며 완료 표시가 아니다.

### 5.2 3D detection

| # | 확인 | 담당 | 통과 기준 |
|---|------|------|-----------|
| 5.1 | 학습 smoke | 손재호 | `docker/perception-train`에서 `train.py` 1 epoch (소형 subset) |
| 5.2 | morai_3d_live | 손재호 | ROS 토픽 구독·CSV 라벨 1프레임 저장 |
| 5.3 | 카메라 sync | 손재호 | 3cam + lidar 타임스탬프 gap < 설정값 |
| 5.4 | inference | 손재호 | `inference.py` + 체크포인트 로드 (NAS에서 복사) |
| 5.5 | sparsedrive_ops | 손재호 | CUDA ext 빌드 필요 시 문서화 |

**데이터:** `dataset/`, `*.pth`는 repo 밖 — `$ASMC_DATA` 또는 NAS.

---

## 6. 통합 E2E (대회 전)

| # | 확인 | 담당 | 통과 기준 |
|---|------|------|-----------|
| 6.1 | 전 모듈 동시 기동 | 통합 리드 | bridge + planning + (perception\|learning) 충돌 없음 |
| 6.2 | 토픽명 합의 | 전원 | [architecture-overview.md](./architecture-overview.md) §5 반영 |
| 6.3 | 대회 PC 네이티브 | 전원 | Ubuntu 20.04 + Noetic 동일 빌드 |
| 6.4 | morai_msgs 단일 | 전원 | submodule `beta_drive` only |

---

## 7. 결과 기록 템플릿

담당자가 검증 후 아래 형식으로 이슈/PR에 남깁니다.

```markdown
## Sim verify — <파트> — <이름> — <날짜>

- 환경: WSL / Docker / 네이티브, MORAI 버전, 맵
- 항목: 2.3, 2.4 (예)
- 결과: PASS / FAIL
- 로그·스크린샷: (링크)
- 이슈: (없음 / 설명)
```

---

## 8. 관련 문서

- [team-dev-guide.md](./team-dev-guide.md) — **팀원별 개발 경로·주의사항**
- [integration-plan.md](./integration-plan.md) — 통합 단계
- [morai-msgs-beta-policy.md](./morai-msgs-beta-policy.md) — msg 정본
- [getting-started.md](./getting-started.md) — clone·Docker
- [docker-guide.md](./docker-guide.md) — 컨테이너 사용
- [planning-behavior-handoff.md](./planning-behavior-handoff.md) — planning 토픽 계약
