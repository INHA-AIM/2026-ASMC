# GT 수신 옵션과 두 LiDAR 15Hz 설정 — 2026-10-17

사용자 요청으로 공개 bridge CLI의 `--streams`를 `--gt`로 교체했다.
일반 실행은 그대로이며 `--gt`만 Object·Semantic·Instance LiDAR를 추가한다.
내부 `competition/development` 프로필과 공용 `morai_udp` 구현은 유지한다.
이 이름은 MORAI 버전 선택이 아니다. GT는 local 전용이며 이전 `ASMC_STREAMS`는 제거했다.

GT 센서셋의 ID5 Intensity·ID11 Instance LiDAR를 모두15Hz로 변경했다.
각 `sensorPeriod=0.06666667014360428`, `frequencyIndex=2`이며 포트·장착 자세·
카메라·GPS/IMU·나머지 설정은 보존했다. 이전10Hz SHA는 bridge 계약과 Git에 남긴다.
v1/v2 회귀본은10Hz, v3는15Hz를 유지하며 generated transport 설정도 그대로다.

Bridge 조립 참고값은 v3/GT에 맞춰15Hz로 정리했다. Azimuth 기반 실제 회전 조립은 유지한다.
이 값은 MORAI 송신 주기를 바꾸거나 빈 scan을 보간하는 기능이 아니다.
GT JSON을 MORAI UI에서 다시 로드해야 새 주기가 적용된다. 기존 v2 비교에는
`lidar_rotation_rate:=10.0`을 명시할 수 있다.

## 제공된 실행 로그의 별도 문제

기존10Hz GT 실행 로그는 Intensity 약8.5–8.8Hz, Instance 약8.3–9.0Hz를 보였다.
이는 새15Hz 설정의 실측 결과가 아니다. 재로드 후 새 수신률을 확인해야 한다.

UDP9798에는 `#DetailEgoInfo$` 헤더가 들어왔지만 현재 Competition Status decoder의
검증 형식은 `#MoraiInfo$`·152-byte body다. 따라서 Competition 수신 수가0으로 남는다.
이는 GT 옵션이나 LiDAR Hz 문제가 아니다. 로드한 Network 설정·실행판과 실제 datagram을
확인해야 하며, 이번에는 미확인 형식을 정상 Competition Status로 변환하지 않았다.
기존 Ego 입력과 GPS/IMU 차량 상태는 이 문제와 구별한다.

## 검증 범위

실제 CLI parser의 기본/GT·transport·field gate·폐기 인자 검사와 문서 회귀23개가 통과했다.
두 LiDAR의 주기·frequencyIndex·ID·포트를 검사하고 v2/v3·생성 파일과 현재 SHA를 대조했다.
JSON 의미 비교에서 두 LiDAR의 주기 관련4개 필드 외에는 변경이 없고 v1/v2/v3 바이트도 동일했다.
별도 소스 환경의 decoder·센서·검증기·launch ROS 회귀15개가 통과했다. 사용자 bridge·
시뮬레이터는 재시작하지 않았다. 격리 환경에 기존 topic verifier 스크립트가 빠진 첫 시도는
환경 보완 후 재검증했다.
원본 로그를 새로운15Hz 실측이나 현장 Competition Status 수락으로 표시하지 않는다.
