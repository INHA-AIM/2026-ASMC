# Bridge 콘솔 줄바꿈과 센서별 Hz 표시

PR #93의 [이전 출력 기록](2026-10-17-bridge-log-readability.md)을 보존하면서
사용자 확인을 반영해 콘솔 배치를 다시 정리했다. 아래 숫자는 형식 설명용 예시다.

```text
[NETWORK]
  RX ego=38.2Hz  col=38.4Hz  obj=38.2Hz
  TX cmd=idle  | link=A126
  state=OK  | err +0  | drop +0
[SENSORS]
  cam1=19.4Hz  cam2=19.4Hz  cam3=19.4Hz  cam4=10.0Hz
  gps=5.0Hz  imu=34.0Hz  lidar=12.4Hz  lidar_instance=12.4Hz
  sem_front=19.4Hz  sem_left=19.4Hz  sem_right=19.4Hz
  state=OK  | err +0  | drop +0
```

- Network와 Sensor가 각각5초마다 한 번의 `rospy.loginfo`로 여러 줄을 출력한다.
  `[NET 5.0s]`처럼 제목에 측정 구간을 표시하지 않는다. Hz는 여전히 실제 경과 시간으로 계산한다.
- `RX`(receive)는 MORAI에서 받는 상태, `TX`(transmit)는 MORAI로 보내는 제어 명령이다.
  제어 미송신은 `cmd=idle`로 표시한다.
- 등록된 센서만 이름과 Hz로 표시한다. 일반 모드는 `obj`·GT 센서가 없고,
  사용자 정의 센서는 이름을 유지해 별도 줄에 표시한다.
- `state`는 현재 문제, `err +N`·`drop +N`은 최근5초 증가량이다.
  선택 local Competition의 알려진 미지원 envelope 처리와 실제 오류 표시는 변경하지 않는다.
- 1초는 두 diagnostics 토픽의 상태 발행 주기다. 오류 로그의 고정1초 주기가 아니다.
  상태 전환 경고와 파싱·송신 오류의5초 throttle은 유지한다.
- 계측 도구는 여러 줄을 하나의 로그 이벤트로 세며 과거 출력 형식도 읽는다.
  센서 수와 줄 수가 바뀌어도 보고 주기 계산은 블록 단위다.

검증은 ROS Python3.8의 브릿지 회귀34개와 CLI·설정·문서 회귀23개를 통과했다.
실제 MORAI 입력으로 GT·일반 모드를 독립 master에서 각각20초 검사했다.
각 실행에서 Network/Sensor 요약이 각각4블록, diagnostics가 각각20회 나왔다.
콘솔 간격은4.999–5.002초, diagnostics 간격은0.998–1.003초였고,
보고 예외·parse/header 오류·socket drop·LiDAR stamp 중복/역행은0이었다.
선택 Competition 안내는 실행마다1회였으며 `UNSUPPORTED` 기록은 유지됐다.
공개 제어와 planner UDP forwarding을 차단했고 시험 bridge와 master는 종료했다.
ROS clock 정지 시 stale/복구 검사는 기존 PR #93 결과를 재사용한다.
이번 변경은 출력 배치만 바꾸며 보고 주기·health 판정·센서 조립은 유지한다.

검증 결과와 원시 로그 SHA는
[집계](../../../config/baselines/evidence/bridge_console_layout_20261017/summary.json)에 기록한다.
전체 실행 로그는 workspace 공통 `data/runtime/bridge-logs-20261017/`에 별도 이름으로 보존한다.
실제 실행 checkout을 재빌드하되 사용자 bridge를 자동 실행하거나 PR을 병합하지 않는다.
