# 신호등 인지 — P4-C 호환 기준 구현

> **역할:** Compatibility Reference Package Guide
>
> 통합 경계는 [Perception 개발·통합 가이드](../../docs/packages/asmc_perception/README.md)를
> 따른다. HD Map의 신호등 필요 구간 metadata와 이 추론 process를 연결하는 구간별
> orchestration은 아직 구현된 기능으로 간주하지 않는다.

P4-C1부터 운영 producer는 `src/asmc_perception/scripts/traffic_light_node.py`이고 P4-C3 판제
consumer는 `/perception/traffic_light/detection`을 우선 사용한다. 이 디렉터리의
`live_infer.py`는 parity 기준 구현, `best.pt` artifact 위치와 구 String consumer 호환을 위해
보존한다. `/traffic_light/state`, `/traffic_light/detail` 이름은 변경되지 않았으므로 기존 도구는
계속 연결되지만 새 판제 코드는
[perception runtime 계약](../../docs/contracts/perception/runtime-v1.md)을 따라야 한다.

MORAI의 ROS compressed image를 받아 신호등 상태를 추론하고 결과를 다시 ROS
토픽으로 발행한다. 기본 입력은 `/cam_extra/image_jpeg/compressed`이다.

현재 통합 배포에서는 `asmc_perception/traffic_light_node.py` 프로세스 하나가 모델 로드, 이미지
구독, GPU 추론과 typed·legacy 결과 병행 발행을 담당한다. 아래 standalone 명령은 기준 구현
재현과 호환 점검용으로 남긴다.

## 데이터 흐름

```text
/cam_extra/image_jpeg/compressed (sensor_msgs/CompressedImage)
                  │
                  ▼
          live_infer.py / run.sh
    JPEG decode → resize → ResNet18 → 판정
                  │
                  ├──> /traffic_light/state  (std_msgs/String)
                  └──> /traffic_light/detail (std_msgs/String JSON)
                                      │
                                      ▼
                         판단·제어 코드 / traffic_light.py
```

## 구성

| 파일 | 역할 |
|---|---|
| `live_infer.py` | ROS 이미지 구독, 모델 추론, 결과 토픽 발행을 한 프로세스에서 수행 |
| `run.sh` | ROS 환경 source와 `live_infer.py` 실행을 묶은 편의 스크립트 |
| `model.py` | ResNet18 정의, 전처리, 체크포인트 로드, 최종 판정 |
| `traffic_light.py` | 판제가 최신 결과를 `state()`로 읽는 비동기 구독 API |
| `best.pt` | 학습된 모델(size 384, 저장 당시 val 약 99.8%) |

## 실행

ROS master와 카메라 토픽이 먼저 떠 있어야 한다.

```bash
cd /home/jang/2026-ASMC/src/traffic_light
./run.sh
```

ROS 환경이 이미 source돼 있다면 Python 파일을 직접 실행해도 된다.

```bash
python3 live_infer.py
```

`--ckpt` 기본값이 같은 폴더의 `best.pt`이므로 체크포인트 인자도 생략할 수 있다.

주요 옵션:

```bash
./run.sh --image-topic /cam_extra/image_jpeg/compressed
./run.sh --hz 15 --min-detect 0.5 --print-topk
./run.sh --stale-sec 1.0
./run.sh --device cpu
./run.sh --state-topic /traffic_light/state
```

확인:

```bash
rostopic hz /cam_extra/image_jpeg/compressed
rostopic echo /traffic_light/state
rostopic echo /traffic_light/detail
```

## 출력

`/traffic_light/state`에는 다음 문자열 중 하나가 발행된다.

```text
red yellow green red_yellow red_left green_left left not_detected
```

`unknown`은 모델 내부 학습 클래스지만 최종 출력 후보에서는 제외한다. `unknown`을
제외한 신호 클래스의 최고 확률이 `--min-detect` 이하일 때 `not_detected`가 된다.
기본 임계값은 `0.40`이다.

`/traffic_light/detail`은 디버깅 및 확률 확인용 JSON 문자열이다.

```json
{"label":"green_left","prob":0.9421,"top":[["green_left",0.9421],["green",0.0312]],"ms":3.1}
```

두 출력 publisher는 latch가 켜져 있어 판제 노드가 나중에 시작해도 마지막 결과를
즉시 받는다. 새 이미지 결과가 `--stale-sec`(기본 1초) 동안 없으면 추론 노드
자체가 `not_detected`를 발행하므로, 토픽을 직접 구독하는 판제도 과거 신호를
계속 사용하지 않는다.

## 판제에서 결과 읽기

판제가 이미 ROS 노드라면 `/traffic_light/state`를 직접 구독하는 것이 가장 단순하다.

```python
import rospy
from std_msgs.msg import String

latest_light = "not_detected"

def traffic_light_cb(msg):
    global latest_light
    latest_light = msg.data

rospy.Subscriber("/traffic_light/state", String, traffic_light_cb, queue_size=1)
```

기존처럼 함수 호출 형태가 편하면 `traffic_light.py`를 import한다.

```python
from traffic_light import TrafficLightSubscriber

tl = TrafficLightSubscriber(stale_sec=1.0).start()

while control_loop:
    light = tl.state()
    label, prob = tl.detail()

    if light == "red":
        brake()
    elif light == "not_detected":
        keep_previous()

tl.stop()
```

`state()`와 `detail()`은 ROS나 GPU를 기다리지 않고 마지막 수신값을 즉시 반환한다.
새 결과가 `stale_sec`(기본 1초) 동안 들어오지 않으면 자동으로 `not_detected`를
반환해 오래된 신호를 계속 사용하지 않는다.

## 설치

ROS Noetic의 `rospy`, `sensor_msgs`, `std_msgs`가 필요하다. Python 추론 의존성은:

```bash
pip install -r requirements.txt
```

GPU 환경에 맞는 PyTorch 설치가 별도로 필요할 수 있다.

```bash
# RTX 40 계열 예시
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121

# RTX 50 계열(Blackwell) 예시
pip install --upgrade torch torchvision --index-url https://download.pytorch.org/whl/cu128
```

기본 `--device auto`는 설치된 PyTorch wheel이 현재 GPU 아키텍처를 지원하지 않으면
경고 후 CPU로 폴백한다. 실시간 운용에는 해당 GPU를 지원하는 PyTorch를 설치해야
하며, `--device cuda`를 지정하면 미지원 환경에서 즉시 오류로 종료한다.

카메라는 `sensor_msgs/CompressedImage`를 발행해야 하며, JPEG 데이터는
`cv_bridge`나 OpenCV 변환 없이 메시지의 `data`를 직접 사용한다.
