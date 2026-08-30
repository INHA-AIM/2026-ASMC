# data_collection — Sync 데이터 수집

> Phase 0 · 안승현  
> 설계: [`docs/competition/sync-collection-design.md`](../../../docs/competition/sync-collection-design.md)

## 방침 (전부 수집?)

**시뮬에는 규정 센서를 다 달아도 되고**, 디스크에는 **profile로 켠 채널만** 쓴다.

| Profile | 용도 |
|---------|------|
| `dry_run` | `/Ego_topic`만 — 첫 연동 |
| **`perception_core`** | **기본** — cam×3 + lidar + ego + objects + TL(optional) |
| `lidar_gt` | Det3D |
| `state_only` | GPS/IMU + ego |
| `full` | 전채널 · **짧은 런만** (`max_frames` 기본 300) |

“나중에 뭐 쓸지 몰라서 항상 full” → I/O·디스크로 수집이 죽음.

## 환경변수

```bash
export AIM_PROJECT=~/projects/2026-ai-sw-mobility-competition
export ASMC=$AIM_PROJECT/2026-ASMC
export ASMC_DATA=${ASMC_DATA:-$AIM_PROJECT/data/asmc_datasets}
```

선택: `ASMC_COLLECT_ENABLE=cam_extra,gps` / `ASMC_COLLECT_DISABLE=tl_status`

## 실행

```bash
cd "$ASMC"
source devel/setup.bash   # catkin build data_collection 후

# 1) ego dry-run
roslaunch data_collection sync_collector.launch profile:=dry_run

# 2) 기본 인지 수집
roslaunch data_collection sync_collector.launch profile:=perception_core

# 3) full 쇼트런
roslaunch data_collection sync_collector.launch profile:=full max_frames:=100
```

저장: `$ASMC_DATA/datasets/<run_id>/` (`meta.yaml`, `manifest.jsonl`, `sensors/`, `state/`, `gt/`).

## MORAI Capture Mode 자동 수집

`capture_mode_collector_node.py`는 차량 제어 명령을 발행하지 않는다. 따라서
`mpc_node`가 `/ctrl_cmd`를 발행하는 자동주행과 MORAI Game Wheel/Keyboard 수동주행에서
동일하게 사용할 수 있다.

### 준비

1. MORAI 26.R1.H3에서 K-City 2025와 `sensor_set_v1`을 로드한다.
2. Sensor Capture Mode를 활성화한다.
3. 아래 둘 중 하나를 연결한다.
   - 권장: MORAI gRPC server `7789` — 성공/실패 응답과 sim time을 받을 수 있다.
   - 폴백: MORAI ROS Network에 `/SaveSensorData` subscriber를 추가한다.
4. ROS 컨테이너에서 UDP bridge를 실행한다. 휠 주행에서도 `/Ego_topic` 기록을 위해
   bridge는 유지하되 MPC 등 `/ctrl_cmd` 발행 노드는 끈다.

### 실행

```bash
cd "$ASMC"
source devel/setup.bash

# 권장 시작값: 1 Hz, Ctrl-C 전까지 계속 수집
roslaunch data_collection capture_mode_collector.launch capture_hz:=1.0

# 첫 QA: 정확히 100 capture 후 종료
roslaunch data_collection capture_mode_collector.launch \
  capture_hz:=1.0 max_captures:=100 run_id:=kcity_capture_001

# 정차 중 중복 프레임 제외: /Ego_topic이 반드시 필요
roslaunch data_collection capture_mode_collector.launch \
  capture_hz:=1.0 only_when_moving:=true require_ego:=true min_speed_mps:=0.2
```

Docker 안에서 MORAI가 Windows host에 있으면 다음처럼 주소를 덮어쓴다.

```bash
roslaunch data_collection capture_mode_collector.launch \
  grpc_address:=host.docker.internal capture_hz:=1.0
```

기본 `backend:=auto`는 gRPC에 먼저 연결하고, 열려 있지 않으면 연결된
`/SaveSensorData` ROS subscriber로 전환한다. 특정 방식을 강제하려면
`backend:=grpc` 또는 `backend:=ros_topic`을 지정한다. ROS 방식은 publish 이후의
저장 완료 응답이 없으므로 100-frame SensorData 파일 QA가 특히 중요하다.

MORAI 이미지·GT는 Windows의 `SensorData`에 저장된다. Linux에는 다음 lineage만
`$ASMC_DATA/capture_runs/<run_id>/` 아래 기록한다.

- `meta.json`: Hz, gRPC 대상, motion gate 등 run 설정
- `capture_manifest.jsonl`: custom filename, wall/ROS/MORAI sim time, Ego snapshot,
  최근 `/ctrl_cmd`, RPC 결과와 latency
- `summary.json`: 성공·실패·motion skip 집계

최근 `/ctrl_cmd`가 있으면 manifest의 `control_source=ros_ctrl`, 없으면
`manual_or_unknown`으로 기록한다. 이는 제어권을 변경하지 않고 사후에 MPC/휠 run을
구분하기 위한 메타데이터다.

### Hz 정책

- 기본값은 **1 Hz**다. 현재 4 Camera + LiDAR + GPS + IMU 1세트 23파일의 실측 쓰기
  완료가 약 0.50~0.57초였기 때문에 안전 여유를 둔 값이다.
- 2 Hz 이상은 `1 → 2 → 5 → 10 Hz`, 각 100회 순서로 파일 누락·부분 파일·SIM FPS를
  검사한 뒤 사용한다. 2 Hz를 넘기면 노드도 미검증 경고를 출력한다.
- 스케줄이 밀려도 catch-up capture를 연속 호출하지 않는다.
- gRPC 실패가 기본 3회 연속 발생하면 자동 중단한다.

실제 저장 없이 manifest·motion gate만 확인하려면 `dry_run:=true`를 사용한다.

## 선행 조건

1. MORAI + bridge → ROS 토픽 (`aim_ws-va` `morai_*_bridge` 또는 개발용 rosbridge)
2. 카메라 토픽 이름이 브릿지와 다르면 `config/profiles/*.yaml`의 `topics:` 수정
3. 대회 ROS UI에는 TL Get·센서 ROS가 없을 수 있음 → `missing[]`로 기록되고 계속 진행

## 모드

현재: `latest_timer` (최신 메시지 래치 후 `collect_hz` 스냅샷).  
다음: Time Manager sync tick ([mentoring](../../../docs/competition/mentoring-morai.md) §6).

## lane/drivable 수집 구현 상태

현재 코드는 TwinLiteNet+ 학습 수집기로 **완성된 상태가 아니다.**

- RGB 카메라는 저장하지만 Semantic 카메라 구독·paired timestamp가 없음
- writer는 카메라별 원본 header stamp/shape/camera_id를 manifest에 기록하지 않음
- `latest_timer`는 RGB↔Semantic 정합을 보장하지 않음
- 카메라 blob을 `.jpg`로 저장하므로 Semantic 무손실 PNG/ID map writer가 별도로 필요
- `lane_capture` / `lane_stream_gt` profile은 아직 없음

구현 순서는 [데이터 로드맵 WP-01~09](../../../docs/competition/yolopv3-data-development-roadmap.md)와 [수집 계약](../../../docs/competition/sync-collection-design.md)을 따른다. Bronze/Silver에는 native 해상도를 보존하며 TwinLite H×W=384×640 변환은 Silver 이후 adapter/Gold에서 수행한다.
