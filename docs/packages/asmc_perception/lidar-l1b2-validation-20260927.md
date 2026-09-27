# L1-B2 GPS 전환·출력 선택 — 격리 검증

**상태: 격리 후보 검증, 운영 미연결.** 로컬 검토용이며 토픽·메시지·판제 안전 동작을
교체하지 않는다. [재실행](../../../tools/lidar/gps_transition/README.md) ·
[고정 manifest](../../../config/baselines/lidar_l1b2_20260927.yaml) ·
[이전 B1 좌표·시간](./lidar-l1b1-validation-20260927.md) ·
[판제 공통 소비 계약](../../contracts/perception/lidar-planning-consumer-v1.md).

## 무엇을 분리했나

raw GPS fix가 유효한지와 LiDAR **관측 시각**의 map pose가 존재하는지는 다르다.
또 map 객체가 있더라도 판제 `/vehicle/state` 및 trajectory가 안전하게 준비됐다는
뜻은 아니다. 후보 selector는 이 세 gate를 분리한다. local 객체의 `base_link`
중심/3D 코너/ID/stamp는 GPS 상태 변화로 덮어쓰거나 재생성하지 않는다. map 객체는
GPS ready, 명시적으로 신뢰한 관측 pose, B1 map geometry가 모두 있을 때만 선택한다.
LiDAR 내부 tracker를 mode마다 초기화하지 않는 **계약을 위한 selector**이지,
현재 tracker의 ID 연속성이 실주행에서 개선됐다는 결과는 아니다.

`status>0`만으로는 유효 fix가 아니다. 정차 bag의 shadow entry/middle 각 176개
GPS는 전부 **status=1이지만 위도·경도가 0**이었다. 후보는 이를 invalid로 판정한다.
stale(0.5초), 영점/역행/중복 source stamp, NaN, 좌표범위, clock 불일치,
recovering(3회), 점프를 서로 다른 사유로 남긴다. 점프 `40 m/s + 1.5 m`은
규정 속도나 배포값이 아니라 비교용 보수 gate 가설이다. 최초 거부 시 허용 거리를
고정해 시간이 흐른 것만으로 해제되지 않게 했다. 다른 독립 pose가 동시각·2 m
이내임을 증명한 경우만 reanchor 후보가 된다. 이 수치는 연속 주행에서 조정해야 한다.

## 입력·결과

기준 commit은 main `1ce434d`, 손재호 `6435af1`, 서준 `8b3f0b1`이고,
B1 입력 `l1b1_v3/results.jsonl`의 동일 `roll_pitch_mount` 객체 snapshot을
11가지 상태에 넣었다. 산출물은 `$ASMC_DATA/lidar_assessment/20260927/l1b2_v4/`;
원본 bag과 B1 출력은 복사·수정하지 않았다. v1은 점프 거부 뒤 시간이 지나면
자동 재수용되는 실패를 발견한 실행이다. v2에서 이를 고쳤고, v3에서 scan과 GPS의
도착 순서 검사, v4에서 잘못된 관측 stamp 취급을 강화했다. v2~v4의 결과 JSON은
byte-for-byte 동일하다.

| 동일 snapshot 상태 | local | map / planner |
|---|---|---|
| 정상 GPS·관측 pose·차량 상태·trajectory | ID 42, 원본 stamp·3D geometry 유지 | 객체 사용 가능 / ready |
| status=1, 위도·경도 0 | 같은 local 객체 | `gps_invalid` / 불가 |
| GPS 미수신, 복구 2회째, 위치 점프 | 같은 local 객체 | 각각 `gps_stale`, `gps_recovering`, `gps_jump_rejected` / 불가 |
| pose 불신, 차량 상태 불가, trajectory 불가 | 같은 local 객체 | map pose 불가 또는 map 객체만 존재; planner는 불가 |
| detector 미실행, 정상 빈 관측, 센서 누락 | 상태를 각각 보존 | 미실행/`observed_empty`/누락을 같은 빈 배열로 취급하지 않음 |

합성 6개 test method와 11개 snapshot case가 통과했다. 동일 snapshot의
ID·관측 stamp·geometry가 GPS gate 때문에 바뀌지 않는지 검사했다. B1의 실제
시간·좌표 변환과 이 selector를 **운영 producer에서 결합한 것은 아니다**.

고정 정차 bag에서 GPS 962개, raw cloud 2,562개, `/last_lidar`·`/lidar_nogps`
각 2,559개 메시지를 읽었다. 두 음영 measurement window의 GPS 352/352개는
status=1/zero 좌표였고, raw cloud 924개는 gate invalid로 분류됐다.
고주로 measurement window의 GPS 176개/scan 467개는 텔레포트 때문에
점프 거부 상태로 남았다. 이것은 진짜 고주로 주행의 GPS 실패율이 아니라,
독립 reanchor 증거가 없는 격리 gate가 **의도된 텔레포트를 거부한 결과**다.
복귀 initial에서는 다시 ready가 됐지만 이 역시 실제 음영 탈출 연속 주행이 아니다.

## 아직 수락하지 않은 것

정차 bag은 rear-axle의 관측시각 pose와 실제 적용 수평화 행렬이 없고,
`/vehicle/state`도 기록하지 않았다. GPS 5 Hz 수준의 표본만으로 B1의
100 ms bracket 조건을 충족하는 map pose를 소급 생성하지 않는다. scan의
`map_pose_not_assessed`가 모든 scene에서 기록된 이유다. 따라서 map OBB의
실자료 오차, 추적 ID 연속성, 벽 근접 차량, 판제의 정상→음영→복귀 제어,
연속 주행 jump 허용/오탐, 차량 상태·trajectory 전환은 미검증이다.
두 LiDAR output topic 메시지 수는 **객체 유효성이나 활성 여부를 뜻하지 않는다**.

운영 연결 전에는 관측별 pose/twist/leveling provenance를 producer가 전달하고,
상태가 있는 versioned object interface 또는 호환 adapter를 판제 담당자와
합의해야 한다. 서준 브랜치의 `Msg`/`asmc_msgs` ROS1 MD5는 main과 달라 섞어
실행할 수 없다. 판제의 fail-safe·ID 재연결·대상 소실 유예를 먼저 제거하지 않는다.
다음 **한 단위**는 실제 camera 50 ms 경계·지연·누락에서 터널 벽 잔여점이 생기는
현상을 동일 scan으로 평가하는 것이다. 운영 전환/PR은 별도 검토 단계다.
