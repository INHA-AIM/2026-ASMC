# 세 모델 상시 추론 기본값 확인 — 2026-10-16

사용자 결정에 따라 공개 `camera.launch`와 내부 `_runtime.launch`의 기본을 ACTIVE로 변경했다.
YOLO/TwinLite/ResNet 목표15/15/10Hz, E8·TensorRT·입력 크기·confidence/NMS·mask는 유지했다.
구간은 [판제 소비 안내](../../handoffs/perception-to-planning.md)로 정리했고 생산자 발행을 제한하지 않는다.
A153 끝/A451 진입까지 차량을 사용하며 공식 팀 음영 A126에 들어가기 전에 family를 확인한다.

## 검사

- 카메라 runtime/launch CPU 회귀85개 통과: 새 기본 ACTIVE, 명시적 STANDBY/OFF, 예제 설정의 GPS/구간 독립성 포함.
- 문서·공용 계약 회귀52개, 판제 입력 launch 계약2개 통과. 기존 메시지·서비스 정의는 변경하지 않았다.
- 문서 구조/링크·launch 인자 생성 검사와 인지 계약 검사 통과. 판제 behavior YAML은 주석만 바뀌었고 파싱한 값은 원본과 동일하다.

## 현재 PC의 짧은 실행 확인

`asmc-ros-noetic`의 기존 main `09ee363c9712d507807788301c77135981709029`에서
`camera.launch mode:=active`를 명시해 새 기본값과 같은 실행을 적용했다.
변경된 launch 자체는 별도 격리 workspace의 ROS 회귀에서 검사했다. 실행 중 main 소스는 수정하지 않았다.
기존 master/Main을 유지하고 bridge·카메라를 한 세트만 실행했다. GUI와 판제/차량 제어는 시작하지 않았다.
기존 Main의 시험용 제어 remap은 유지하며 `/ctrl_cmd`·`/ctrl_cmd_0` 공개 발행자는 없었다.

| 20.03초 수신 확인 | 새 출력률 |
|---|---:|
| YOLO 전방/좌/우 | 각15.03Hz |
| family 발행 | 14.98Hz |
| 전방 `/YOLO/BBox` | 14.98Hz, 어댑터 단일 발행자 |
| 신호등 typed 결과 | 9.93Hz |

세 모델은 ACTIVE·loaded·warmed_up, errors0이었다. 위 결과 토픽의 중복/역행 stamp는0이었다.
신호등의 결과 `last_output_valid=false`는 실행 오류 카운터와 구분한다. 신호가 보이지 않는 결과를
유효 신호로 해석하지 않는다. 측정 종료 시 GPU 메모리는5554/8188MiB였다.

증거는 로컬 `data/runtime/pr91-main-apply-20261016/`의
`camera-active-runtime-tests.log`, `camera-always-active-smoke.json`,
`camera-always-active.log`, `camera-always-active-bridge.log`에 보존했다.
이는 단일20초 동작 확인이다. 최악 지연·최대 공백·GUI/날씨 반복·음영 유효 family 공급·
LiDAR 최종 출력·공동 자율주행·장시간 수락을 새로 검증했다고 표시하지 않는다.
과거 family 순간 공백과 품질 제한은 [기존 공급 기록](camera-family-supply-optimization.md)에 유지한다.
