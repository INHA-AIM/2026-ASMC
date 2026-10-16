# Capture 파일 수집·복사

`capture_mode_collector_node.py`는 차량 제어 명령을 발행하지 않는다. 따라서
MPC가 `/ctrl_cmd/normal`을 발행하고 선택기가 `/ctrl_cmd`로 전달하는 자동주행과 MORAI Game Wheel/Keyboard 수동주행에서
동일하게 사용할 수 있다.

### 준비

1. MORAI의 검증된 v3 주행 센서셋을 로드하고 Capture Mode를 켠다.
   정차3회에서 RGB·semantic·instance·BBox 파일 생성을 확인한다.
   별도 GT 스트리밍 센서셋은 이 Capture 방식의 필수 조건이 아니다.
2. Sensor Capture Mode를 활성화한다.
3. 아래 둘 중 하나를 연결한다.
   - 권장: MORAI gRPC server `7789` — 성공/실패 응답과 sim time을 받을 수 있다.
   - 폴백: MORAI ROS Network에 `/SaveSensorData` subscriber를 추가한다.
4. ROS 컨테이너에서 UDP bridge를 실행한다. 휠 주행에서도 `/Ego_topic` 기록을 위해
   bridge는 유지하되 MPC 등 `/ctrl_cmd` 발행 노드는 끈다.

### 센서셋 정본과 LiDAR GT

| 파일 | 용도 |
|---|---|
| `config/morai/sensors/ASMC_sensor_set_v1.json` | 과거 run 재현용 |
| `config/morai/sensors/ASMC_sensor_set_v2.json` | 과거10Hz 회귀용 |
| `config/morai/sensors/ASMC_sensor_set_v3.json` | 현재 주행 센서셋15Hz; 현장 수락은 별도 |
| `config/morai/sensors/ASMC_sensor_set_GT.json` | v2 + Semantic Camera×3 + Instance LiDAR, 개발·학습 전용 |

v1→v2의 유효 변경은 rear camera pitch `340°→328°`다. GT 센서셋의 LiDAR는 다음처럼
동일 위치·주기에서 입력과 정답을 분리한다.

| 용도 | Sensor ID | 위치 | 주기 | 타입 | UDP Dest |
|---|---:|---|---:|---|---:|
| runtime/학습 입력 | 5 | `(0.8, 0, 1.45)m` | 10 Hz | Intensity (`0`) | 9299 |
| 학습 정답 | 11 | `(0.8, 0, 1.45)m` | 10 Hz | Instance (`2`) | 9311 |

ID 5의 실제 intensity만 detector 입력으로 사용하고 ID 11의 instance point와
`_instance.txt`/`_instance_8Points.txt`는 class, 3D box, 상대속도와 unique ID 정답으로만
사용한다. GT value를 입력 feature로 사용하지 않는다. 공용 bridge는 competition 기본에서9311 socket/publisher를 만들지 않는다.
명시적인 local development profile에서만 instance_id uint32 개발 GT를 발행한다. 과거 Streaming GT에는
5–11 pairing gap이 있었으므로 detector 학습 정답은 Capture 파일을 사용한다. 일반 Real
Time raw bag은 clustering/tracking과 통합 성능 재현에 사용한다.

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
  grpc_address:=127.0.0.1 capture_hz:=1.0
```

### `run_id` 규칙

한 run은 주행 도중 `scenario/variant`, 날씨, simulator time, controller,
sensor profile이 바뀌지 않는 연속 수집 단위로 잡는다. 하나라도 바뀌거나 수집을
종료했다가 다시 시작하면 새 `run_id`를 사용한다.

```text
<YYYYMMDD>_<HHMMSS>_kcity_<scenario>_<weather>_sim<hh><am|pm>_<controller>
```

맨 앞 시각은 **수집 시작 KST**이고 `sim<hh><am|pm>`은 MORAI 환경 시각이다.
수집 시작 시각을 초 단위 고유 키로 사용하므로 별도 `runNNN`은 붙이지 않는다.
환경 시각을 확인하지 못한 run만 `sim_unknown`으로 기록한다.

예시:

```text
20260903_213829_kcity_comp_sample_sunny_sim11am_wheel
20260904_000940_kcity_comp_sample_foggy_sim1pm_wheel
20260904_101530_kcity_jam_static_block_sunny_sim3pm_mpc
20260904_111500_kcity_merge_base_foggy_sim5pm_mpc
```

- 영문 소문자, 숫자, `_`만 사용한다. 구현상 `-`도 허용하지만 `_`로 통일한다.
- `train/val/test`는 나중에 run 단위로 배정하므로 이름에 넣지 않는다.
- 중단 후 이어 찍더라도 기존 ID를 재사용하지 않고 새 수집 시작 시각으로 생성한다.
- rejected run을 포함해 과거 ID를 재사용하지 않는다. Windows `SensorData`의 같은
  custom filename과 충돌하거나 원본 lineage를 혼동할 수 있다.
- 현재 collector의 `meta.json`은 Hz/backend/controller snapshot은 남기지만 MORAI
  scenario 파일명, 날씨, NPC 구성은 자동 수집하지 않는다. 자동화 전까지는 run ID와
  별도 수집 대장에 scenario 파일/hash, weather/time, actor 구성, 특이사항을 기록한다.

동일한 `run_id`의 `$ASMC_DATA/capture_runs/<run_id>`가 이미 있으면 안전을 위해 시작이
실패한다. 디렉터리를 지우고 재사용하지 말고 새 수집 시작 시각을 사용한다.

기본 `backend:=auto`는 gRPC에 먼저 연결하고, 열려 있지 않으면 연결된
`/SaveSensorData` ROS subscriber로 전환한다. 특정 방식을 강제하려면
`backend:=grpc` 또는 `backend:=ros_topic`을 지정한다. ROS 방식은 publish 이후의
저장 완료 응답이 없으므로 100-frame SensorData 파일 QA가 특히 중요하다.

MORAI 이미지·GT는 Windows의 `SensorData`에 저장되는데 **루트가 두 개로 갈린다.**
`is_custom_file_name = true`인 캡처에서 카메라·GPS·IMU는
`C:\MoraiLauncher_Win\SensorData`로, 3D LiDAR는
`C:\MoraiLauncher_Win\MoraiLauncher_Win_Data\SaveFile\SensorData\LIDAR_5`로 간다.
모라이 자체 동작이며 센서셋에 저장 경로 필드가 없어 설정으로 바꿀 수 없다. 아래
`sync_capture_mode_data.py`의 `--sensor-root` 기본값은 앞의 루트만 가리키므로
**LiDAR는 동기화 대상이 아니다.** LiDAR Capture 형식과 과거 실측 상세는 repository 밖의
개인 분석 기록으로 분리돼 있으며, 팀 공용 파이프라인이 이를 사용하게 될 때 별도 versioned
contract와 importer를 이 저장소에 추가한다.

Linux에는 다음 lineage만 `$ASMC_DATA/capture_runs/<run_id>/` 아래 기록한다.

- `meta.json`: Hz, gRPC 대상, motion gate 등 run 설정
- `capture_manifest.jsonl`: custom filename, wall/ROS/MORAI sim time, Ego snapshot,
  최근 `/ctrl_cmd`, RPC 결과와 latency
- `summary.json`: 성공·실패·motion skip 집계

최근 `/ctrl_cmd`가 있으면 manifest의 `control_source=ros_ctrl`, 없으면
`manual_or_unknown`으로 기록한다. 이는 제어권을 변경하지 않고 사후에 MPC/휠 run을
구분하기 위한 메타데이터다.

### 주행 종료 후 Windows → WSL 동기화

동기화는 주행과 MORAI를 종료한 뒤 **WSL 호스트에서** 실행한다. 원본은 삭제하지 않는다. 기본legacy18은Camera4×4종+GPS+IMU18파일이며
bbox3은세뷰RGB·semantic·instance·BBox를 필수 검사한다. RPC 성공과 실제 파일 완성을 구분한다.

```bash
cd "$ASMC"

# 복사 없이 전체 무결성 검사
python3 scripts/sync_capture_mode_data.py \
  --run-id kcity_auto_sunny_003 \
  --dry-run

# 검사 통과 후 복사. 중단되면 같은 명령으로 이어받는다.
python3 scripts/sync_capture_mode_data.py \
  --run-id kcity_auto_sunny_003 \
  --jobs 4
```

여러 run은 `--run-id`를 반복한다. 완료 데이터는
`$ASMC_DATA/datasets/<run_id>/`에 저장되고 `_SUCCESS`, `dataset.json`,
`manifest.jsonl`이 생성된다.

성공으로 기록됐지만 실제 파일 일부가 누락된 capture는 원본 manifest를 고치거나 파일을
삭제하지 않는다. 확인된 sequence를 명시적으로 제외하면 `dataset.json`에 원래 성공 수,
제외 수·sequence·custom name이 남는다.

```bash
python3 scripts/sync_capture_mode_data.py \
  --run-id <run_id> \
  --exclude-sequence <run_id>:128 \
  --exclude-sequence <run_id>:129
```

존재하지 않거나 성공하지 않은 sequence를 지정하면 동기화를 중단한다. 같은 옵션으로
`--dry-run`을 먼저 실행하고, 실제 복사에도 동일한 제외 목록을 사용한다.

```text
datasets/<run_id>/
  frames/{intensity,semantic,instance,depth}/{front,left,right,traffic_light}/
  state/{gps,imu}/
  lineage/{meta.json,summary.json,capture_manifest.jsonl}
  manifest.jsonl
  dataset.json
  _SUCCESS
```

TwinLiteNet+ 입력은 `intensity/{front,left,right}`, 라벨 원본은 대응하는
`semantic/{front,left,right}`다. `traffic_light`는 별도 신호등 모델용이며,
`instance`·`depth`는 향후 작업을 위해 원본으로 보존한다.

라벨 팔레트·기존384×640 학습cache·run-based split과 QA 계약은
[중앙 데이터 계약](../../../docs/contracts/dataset-labeling-split.md)을
따른다.

객체 포함 run의 Semantic 전수 감사는 `scripts/audit_semantic_capture.py`로 수행한다.
run/view별 공식 class pixel 수와 양성 sample 수, 미등록 색, actor-Instance 대응률 및
대표 QA overlay를 함께 기록한다. 대회 학습 클래스에서 빠지는 색은
`--require-zero-class blue_lane`처럼 명시해, 이후 데이터에 다시 나타나면 감사 명령이
실패하도록 한다.


객체 bbox용 복사는 `--profile bbox3` 또는 누락을 명시하는 bbox3_partial을 사용합니다.
BBox 파일 누락은 negative가 아니며 해당 뷰/관측의 export를 보류합니다.
