# 음영구간 반복 시험과 센서·판단 기록

원태의 구간 생성기·배치 검사·판제 재시작·소유 객체 정리·재현 코드를 재사용한다.
새 경로 추종기나 차량 제어 발행자를 추가하지 않는다. 목적은 정지·실패를 같은 장면에서
다시 확인하고, 센서 bag으로 당시 입력과 판정을 분석하는 것이다.

## 실행 준비

일반 실행은 [README](../README.md)의 네 터미널 구성을 따른다. 이미 브릿지가 실행 중이면
새 브릿지를 켜지 않는다. `run_perception.sh`와 `run_planning.sh`를 시작한 뒤 아래 명령을 쓴다.
외부 판제·카메라·LiDAR와 중복 실행하지 않는다. 각 실행기는 자신이 시작한 프로세스만 정리한다.

새 수집 전 호스트의 저장 관리 도구로 검사·경로 확인·run 등록을 완료한다.
컨테이너의 `/artifacts/<run-id>`는 등록된 D 출력 마운트여야 한다. 이 도구는 D 여유 공간
50GiB 규칙을 면제하지 않는다. 특정 실험에 승인된 예외는 해당 run의 provenance에만 기록한다.

등록한 run 폴더에 호스트 registry의 `run.json`을 `registration.json`으로 복사하고
다음 `recording-spec.json`을 둔다. 실행 중인/완료한 run과 `ros/`를 재사용하지 않는다.

```json
{
  "run_id": "shadow-example-r01",
  "run_root": "/artifacts/shadow-example-r01",
  "max_bytes": 4294967296,
  "minimum_remaining_bytes": 2147483648
}
```

4GiB는 이 기록의 상한, 2GiB는 수집 중 비상 중단 여유다. 등록 전 저장 정책의 대체값이 아니다.
수집기는 256MiB 버퍼·LZ4·60초 분할 bag을 사용한다. recorder 종료·용량 초과·장면 writer
실패는 시험 오류로 드러내며 부분 bag을 삭제하거나 정상 완료로 취급하지 않는다.
끝날 때 필수 토픽 수신량, 타입/MD5, 파일 SHA와 recorder overflow를 검사한다.
필수 토픽이 한 번이라도 왔다는 검사는 지속 공급·동기화 수락을 의미하지 않는다.

```bash
cd /root/ws
source devel/setup.bash
./tools/scen_runner/run_scenario.sh --mode section --section tunnel --seed 42 \
  --recording-spec /artifacts/shadow-example-r01/recording-spec.json
```

반복 횟수는 `config/run_counts.yaml` 또는 `--run-counts <YAML>`로 정한다.
별도 YAML에도 `scene_runs`와 다섯 구간의 `section_runs` 키가 모두 필요하다.
GUI 종료/Ctrl+C는 소유 판제를 정리하고 bag을 닫는다. 실험 종료 후 writer가 없음을 확인하고
호스트 저장 관리자에서 run을 close한다. 장면 재현은 센서 프레임·추적 ID·실시간 부하까지
동일하게 재현하지 않는다. 정확한 당시 입력 대조는 bag을 사용한다.

## 대상 모델을 지정한 반복

무작위 전체 카탈로그 대신 특정 실패 객체를 반복하려면 별도 YAML을 만든다.

```yaml
scene_type: static
static_models: [PE_Drum]
```

`--tunnel-profile <YAML>`을 같은 명령에 추가한다. `scene_type`은 `static/dynamic/both`,
`cross_pattern`은 `horizontal/forward`, `static_models`는 기존 허용 카탈로그의 모델 이름이다.
예: `PE_Firewall_Orange`, `CargoBox`. 카메라 클래스 이름 `plastic_barrier`와 시뮬 에셋 이름은 다르다.
모델 선택 외의 도로·차량 크기·배치 가능성·간격 검사는 유지한다. 프로필 내용과 SHA를 장면에 저장한다.
`dynamic`에 정적 모델을 지정하거나 재현 JSON과 새 프로필을 섞으면 시작을 거부한다.
날씨·시간은 기존 seed 기반 환경 선택기를 사용하고 실제 값을 저장한다.

## 보존하는 근거

| 산출물 | 내용 |
|---|---|
| `ros/*.bag` | 3뷰 RGB·YOLO 관측/상태·family·ground projection·LiDAR local/map·GPS/IMU·frame timing·판제·제어·rosout |
| `ros/manifest.json` | 기록 명령·용량 설정·필수 토픽 누락·메시지 타입/MD5·bag SHA·종료 오류 |
| `scenes/<시각>/events.jsonl` | 생성/삭제·준비·재시작·종료 및 접촉 판정 |
| `scenes/<시각>/*_trial_*.json` | 충돌 전에도 준비 완료 및 정리 직전 장면 저장; seed·환경·에셋·pose·코드/설정 SHA |
| 기존 collision/stagnation JSON | 충돌·정체가 실제 발생한 경우만 기존 사건으로 저장 |

`trial`은 정상 통과의 증거가 아니다. 실패하지 않은 장면도 복원할 수 있도록 만든 별도 기록이며
이후 충돌/정체 저장을 소모하지 않는다. 기존 객체는 생성기 소유 객체와 구분한다.
시험자가 원래 배치한 객체·날씨·Ego 위치는 별도 before/after snapshot으로 기록·복구 여부를 확인한다.
실험 시 실행된 ROS parameter dump와 launch 로그도 해당 run에 보관한다.

## 분석과 재현

```bash
cd /root/ws
source devel/setup.bash
export PYTHONPATH="$PWD/tools/scen_runner:${PYTHONPATH:-}"
python3 -m scen_runner.records.analyze_shadow /artifacts/shadow-example-r01
./tools/scen_runner/run_scenario.sh --replay /artifacts/shadow-example-r01/scenes/<시각>/001_trial_tunnel.json
```

재현을 새로 녹화하려면 **새 등록 run**의 recording spec을 지정한다. `--replay-browser`는
저장된 trial/collision/stagnation을 구분해 보여준다. 맵 SHA가 다르면 기존 검사를 계속 적용한다.

분석 결과 `analysis/index.html`에는 정확히 같은 stamp의 RGB와 실제 bbox 판정을 표시한다.
`summary.json`은 토픽 공급·source stamp 중복/역행·정지 사유 샘플 수,
`decisions.jsonl`은 bbox·추종기·selector·경고의 기록이다. 재분석은
`--output-name analysis-v2`처럼 새 하위 폴더를 사용한다. 메모리에는 최근 최대64개 frame/result만 보관하고
갤러리는 최대32장을 추출한다. 실제 투영·차로 판단 수학을 Python으로 재구현하지 않는다.

selector는 1Hz 진단이므로 마지막 selector 표본만으로 명령 한 개의 선택 시점을 확정하지 않는다.
일반 MPC의 `INVALID_EGO`, map 관측 무효와 **실제 선택된 음영 제어기의 정지 사유**를 분리한다.
전체 bag의 공급 공백에는 준비·재시작도 포함된다. 정상 ACTIVE 공급률은 별도 시간창으로 검증해야 한다.
Source/header는 원문대로 보존하며 ROS receipt와 monotonic 시간을 직접 빼지 않는다.

## 현재 적용 범위

이 기능은 시험·재현 도구다. 인지 모델, bbox confidence/contact 기준, family freshness,
차로 변경 timeout과 판제 행동은 변경하지 않는다. local LiDAR는 함께 녹화하지만
현재 bbox 음영 제어기가 local OBB를 회피 입력으로 소비하게 바뀐 것은 아니다.
실제 실패와 후속 단위는 [음영 판제 계획](../../../docs/architecture/shadow-planning-refactoring.md)을 따른다.
