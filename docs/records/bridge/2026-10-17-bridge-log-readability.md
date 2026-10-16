# Bridge 로그 가독성과 선택 Competition 형식 구분

PR #93의5초 요약을 RX/TX·카메라 뷰·센서 역할별로 묶었다. 이전
[통계 안정화 기록](2026-10-17-lidar-cadence-diagnostics.md)의 실측과 실패 증거는 유지한다.

## 26.R1과 참가팀 대회판의 차이

저장소의26.R1 network export에는 Competition Vehicle Status(type787)가 없다.
공통 bridge는9798을 선택 입력으로 열어 두지만 입력이 없으면 `IDLE`이다.
참가팀 대회판을 실행한 현재9798에서는 다음 envelope를 실제 관측했다.

| 항목 | 현재 수신 | 기존 검증 decoder |
|---|---|---|
| Header | `#DetailEgoInfo$` | `#MoraiInfo$` |
| Body | 105B | 152B |
| Datagram | 138B | 181B |
| Tail | CRLF | CRLF |

이름만 다른 동일 데이터로 간주하지 않는다. 일반 제품의
[공식 UDP 안내](https://help-morai-sim-en.scrollhelp.site/morai-sim-standard-en/udp-network-settings)는
참고 자료이며, 이105B 대회판 본문의 필드 계약을 승인한 근거로 사용하지 않았다.
이번 변경은 decoder 추가가 아닌 **선택 입력의 알려진 미지원 envelope 분류**다.

- `competition` 수신 채널·`transport=local`·선택 입력일 때만 적용한다.
- 헤더, 전체138B, declared105B, 실제105B, CRLF가 모두 일치해야 한다.
- 1회 INFO로 안내하고 diagnostics에 `health=UNSUPPORTED`,
  `unsupported_protocol=#DetailEgoInfo$`, `unsupported_packets`, `data_available=false`를 남긴다.
- `messages`와 수신 성공Hz를 올리거나 `/Competition_topic`으로 변환하지 않는다.
- 이 envelope는 반복 콘솔의 문제·오류 합계에서 제외한다. 누적 관측은 diagnostics에 남는다.
- 알 수 없는 헤더·손상된 길이/tail·파싱 실패는 계속 경고한다. `--require-competition-status`
  또는 LAN2LAN에는 예외를 적용하지 않는다. 필수 수락 gate는 유효 decode가 없으면 통과하지 않는다.

`UNSUPPORTED`의 진단 level0은 선택 입력을 운영 필수 장애로 취급하지 않는다는 뜻이다.
Competition 데이터가 정상 해석되어 사용 가능하다는 뜻으로 읽지 않는다. 건강 상태는 전송·decode
경계의 진단이며 GPS fix·IMU 자세와 차량 상태/fusion 유효성 판단은 기존 소비자가 담당한다.

## 5초 콘솔과1초 diagnostics

일반 모드 출력 예시는 다음과 같다. 아래 수치는 형식을 설명하는 예시다.

```text
[NET 5.0s] RX ego=38.2 col=38.4 Hz | TX cmd=idle | link=A126 | state=OK | err +0 / drop +0
[SENSOR 5.0s] | RGB F/L/R/4=19.4/19.4/19.4/10.0 Hz | GPS/IMU=5.0/34.0 Hz | LiDAR=12.4 Hz | state=OK | err +0 / drop +0
```

GT 모드에서는 Network의`obj`, Sensor의`GT SEM F/L/R`와`GT LiDAR`가 추가된다.
`F/L/R/4`는 전방/좌/우/4번 카메라다. 등록되지 않은 뷰는`-`, 등록됐지만 최초 미수신인
필수 뷰는0.0Hz와`WAITING`으로 구분한다. 사용자 정의 센서도 별도 이름으로 출력한다.
K-City의`A2256W000126`은 콘솔에서`A126`으로 줄이며 원본은 Network diagnostics의
`last_link_id`에 보존한다. 다른 형식의 링크는 임의로 줄이지 않는다.

- `state`: 필수 입력과 실제 오류의 현재 문제. Stale에는 수신 age도 표시한다.
- `err +N`: 최근5초의 parse/header/send 오류 증가량.
- `drop +N`: 최근5초의 Linux 소켓 drop 증가량. 계측 미지원이면`?`다.
- 누적 메시지·오류·미지원 패킷, age, 지연·최대 공백·회전 상세는 기존 diagnostics에 남긴다.
- 주기5초/1초와 monotonic clock, bounded snapshot, callback 재시도는 유지한다.

## 검증

알려진 형식의 local 선택 예외, 필수·LAN2LAN 예외 금지, 잘못된 길이·tail·알 수 없는 헤더,
원본 decoder 거부, 정상 메시지0·사용 가능false, 오류/미지원 구분과 카메라 순서를 CPU로 검사한다.
기존 protocol·동시 접근·paused ROS clock의 stale/복구 검사도 유지한다.

실제 MORAI 입력은 독립 master에서 GT/일반/필수 Competition 조건을 직렬 검사한다.
공개 제어와 planner raw forwarding을 차단하고 매 실행 뒤 시험 프로세스를 정리한다.
결과·SHA는 [집계](../../../config/baselines/evidence/bridge_logging_20261017/summary.json),
원본은 workspace 공통`data/runtime/bridge-logs-20261017/`에 보존한다.
병합은 이번 범위에서 제외한다.
