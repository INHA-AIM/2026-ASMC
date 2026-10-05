# LBC teacher BEV research tool

> **경계:** MORAI simulator의 privileged state로 GT/teacher BEV를 만드는 연구·디버그 도구다.
> `asmc_camera` sensor runtime이나 본선 판제 입력이 아니다.

기존 `src/learning_by_cheating` 중 실제로 사용되던 7-channel renderer, scenario
visualizer, NPZ collector, 진단·규격 검사만 이관한 경로다. 외부 연결은
`tools/aim_scenario_runner` 1곳이며 `route_bev_visualizer_enabled: false`가 기본값이다.
원본 규격은 [dotchen/LearningByCheating](https://github.com/dotchen/LearningByCheating)의
`carla_utils.get_birdview`, `models.common.crop_birdview`를 기준으로 한다.

## 어떤 BEV인가

공식 Learning by Cheating의 bird-view 규격을 MORAI KATRI map에 맞춰 재현한다.

| 항목 | 값 |
|---|---|
| full raster | `320 x 320 x 7` |
| network crop | `192 x 192 x 7` |
| resolution | `5 px/m` |
| channel | road, lane, TL red/yellow/green, vehicle, pedestrian |
| static source | KATRI road mesh·lane boundary |
| dynamic source | MORAI Ego/Object/traffic-light state |

카메라 추론으로 만드는 `asmc_camera.bev`와는 용도와 입력이 다르다.
이 결과는 학습 teacher, scenario 검수, 디버깅에만 사용한다.

## 구조

```text
lbc_teacher_bev/
├── lbc_bev/       # renderer, map bake, MORAI adapter
├── scripts/       # visualization, collection, diagnostics, verification
├── config/        # optional RViz preset
├── requirements.txt
└── README.md
```

생성 자산은 Git 소스 하위에 저장하지 않는다.

```text
$ASMC_DATA/lbc_teacher_bev/
├── bev_map/
└── diag/
```

`ASMC_DATA`가 없으면 repository와 나란한 `data/lbc_teacher_bev` 경로를 사용한다.
ROS container에서는 기본적으로 `/data/lbc_teacher_bev`다.

## 실행

`asmc_bridge`가 `/Ego_topic`, `/Object_topic`과 사용 가능한 traffic-light 정보를
발행하는 상태에서 실행한다. 구 direct ROS bridge `9090` 설정은 필요하지 않다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# OpenCV viewer
tools/research/lbc_teacher_bev/scripts/visualize_lbc_bev.sh

# RViz/rqt publisher and optional viewer
VIEWER=rviz tools/research/lbc_teacher_bev/scripts/visualize_lbc_bev_rviz.sh

# 7-channel NPZ collection
tools/research/lbc_teacher_bev/scripts/collect_lbc_bev.sh

# shareable diagnostic report
tools/research/lbc_teacher_bev/scripts/diagnose_lbc_bev.sh
```

발행 topic:

- `/lbc_bev/image_full`
- `/lbc_bev/image_cropped`

사용 가능한 MORAI traffic-light message를 자동으로 구독하며,
`/Service_MoraiEventCmd`에서 turn signal을 확인해 신호 방향을 해석한다.

## 검증

```bash
python3 tools/research/lbc_teacher_bev/scripts/verify_lbc_colors.py
python3 tools/research/lbc_teacher_bev/scripts/verify_lbc_spec.py
python3 tools/research/lbc_teacher_bev/test/test_layout.py
```

Python API:

```python
from lbc_bev import LBCRenderer

renderer = LBCRenderer("/root/ws")
result = renderer.render(x, y, yaw_deg)
```

API로 사용할 때는 `tools/research/lbc_teacher_bev`를 `PYTHONPATH`에 추가한다.

## 제거한 구 코드

- 3-channel 입력을 고정해 현재 7-channel collector와 호환되지 않던 PA 학습 v2
- 존재하지 않는 `run_experiment.sh`와 구 map topic을 전제한 collector wrapper
- `/ctrl_cmd`를 직접 발행하던 미검증 autonomous-driving prototype
- 빈 `dataset.py`, `sa_model.py`와 deprecated `hd_map_crop_ver2.py`

필요하면 이들은 Git history의 `src/learning_by_cheating` 이전에서만 복원한다.
