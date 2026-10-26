# Debug OFF와 CI 회귀 실행 정리

2026-10-26. 비교 기준은 PR #112의 `e1301d6ae9f919dae1dfdc82e492ca8e1e0a1254`다.
인지 알고리즘·필터·판제 행동·메시지·운영 노드는 변경하지 않는다. 판제 웹의 관측 기본값은 정본으로 맞춘다.

## Debug OFF

`lidar_pipeline_node::Process`의 `PublishTracking/PublishObservation`은 `debug_`와 독립이다.
debug OFF는 stage observer와 점군 표현을 끄며 전달 필터·공용 객체 생산을 끄지 않는다.

`common_lidar_ros_test.py`에 `debug:=false rviz:=false`를 명시하고 다음을 추가 확인했다.

- 웹·trace·객체 배열 소비자와 디버그 stage 발행자가 없음.
- 노드 진단의 `debug_enabled=false`.
- 시험기의 local/map 구독을 끊고 판제 어댑터만 남긴 상태에서 새 결과10개 이상 전달.
- 원본 stamp·ID·8코너·속도·운동 상태 보존과 GPS/IMU 중단·복귀·reset/생산자 재시작 처리 유지.

사용자가 켠 운영 구성은 읽기 전용으로10초 확인했다. local/map 각각112개, 수신률11.23Hz,
최대 공백103.8ms, `OBSERVED_EMPTY`112개, input→publish p95 진단2.42ms, debug stage 토픽0개였다.
당시 판제 어댑터는 실행 중이 아니었고 모든 관측이 비어 있었다. **이는 새 폐루프 주행이나 실제
장애물/전체 미션 수락이 아니다.** 비어 있지 않은 객체의 판제 전달은 합성 점군을 실제 처리하는
독립 ROS 시험으로 확인했다. 기존 사용자 주행 기준선은 유지한다.

## 이전 CI의 비용

[이전 전체 CI](https://github.com/INHAautonav/2026-ASMC/actions/runs/40277001624)의 job은12분48초였다.
주요 단계는 전체 빌드149초, LiDAR 회귀85초, 공용 연결+음영 어댑터219초, CPU runtime 설치36초,
Python26초, 카메라 제어/시각화55초였다. 공용 연결 단계 중 차선변경22개 검사는 약123초였다.
실제 pause·timeout·안전 전환을 시험하는 시간을 잘라내지 않고 독립 검사 사이의 직렬 대기를 줄인다.

## 변경과 보존

빌드 완료 후 시간 민감 LiDAR/HTTP 그룹을 단독 실행하고 나머지 그룹을 최대2개씩 실행한다.
모든 그룹을 실행하고 하나라도 실패하면 실패한다. 새 그룹 실행기의 실패 집계·실제 중첩 실행·timeout·
누락/빈 그룹·덮어쓰기 방지·단독 실행 순서8개 단위검사를
통과했다. 이전141개 고유 명령/옵션을 정적 대조하고 LiDAR34개 필수 gate도 유지한다.
Bridge는 기존7개 catkin 검사에 원래 정의돼 있던 C++ vehicle-state 검사까지 포함해 CTest8개를 실행한다.

중복 ROS fixture 세 호출은 같은 옵션당 한 번으로 모았다. 독립 판제 전용 빌드, safety 회귀,
정적 소유권 검사, 전체 PR/main 검사, 이전 PR 실행 취소는 유지한다. 변경 경로를 이유로 새 검사를
건너뛰거나 오래된 성공 결과를 cache로 복원하지 않는다. cache key와 로그 정책은 [운영 안내](../../operations/ci.md)를 따른다.

## 로컬 병렬 실행

분리 빌드 `/tmp/asmc-b4-build`·devel에서 모든 그룹이 통과했다. 합성 CI 결과는
`/tmp/asmc-ci-parallel-r1/summary.json`과 그룹 로그에 보존했다. 대용량 자료·기존 D run은 변경하지 않았다.

| 그룹 | 실행시간 |
|---|---:|
| 음영 어댑터 | 186.677s |
| LiDAR·실제 판제 연결 | 103.412s |
| 카메라 Python | 39.122s |
| 카메라 제어·시각화 ROS | 60.836s |
| Bridge | 12.098s |
| 그룹 시간 합계 | 402.145s |
| 병렬 실행 실제 경과 | 203.373s |

합계는 이번 병렬 실행에서 각 그룹이 사용한 wall 시간의 합이며 실제 workers1 측정이 아니다.
GitHub job 전체 시간에는 checkout·설치·빌드·cache·artifact도 포함되므로 로컬 중첩 시간만으로
Actions 단축률을 주장하지 않는다. 최종 hosted 결과와 이전12분48초의 비교는 PR #112에 기록한다.

## Hosted에서 확인한 보강

[첫 병렬 CI](https://github.com/INHAautonav/2026-ASMC/actions/runs/40281664164)는 HTTP 슬롯 시험이
0.5초 헤더 제한 안에 두 연결을 모두 관측하지 못해 실패했다. 나머지 네 그룹은 통과했다.
서버/시험 timeout을 늘리지 않고 해당 시험을 포함한 LiDAR 그룹을 먼저 단독 실행하도록 바꿨다.
동일 조건을 병렬 외부 부하와 분리한다. 이 단독 그룹이 실패해도 나머지는 검사하며 PR은 실패한다.

설치 로그에는 오래된 pip20 resolver가 NumPy1.17/Pillow7의 dependency conflict를 경고하면서
성공 처리한 점도 있었다. CI만 pip24.3.1로 준비하고 NumPy/Pillow 최소 요구를 명시하며
설치 후 `pip check`로 호환성을 확인한다. 실행 중인 GPU 모델·운영 Python은 재설치하지 않았다.

## 두 웹과 실행 안내

판제 웹8088과 LiDAR 웹8091의 기본 포트 및 ROS 이름이 다름을 확인했다. 새 독립 master/임시
포트에서 두 서버의 HTML·API·정본 state/객체 구독과 차량 제어 미발행을 확인했다. 합성 typed 객체가
판제 `/api/snapshot`에 나타나며 motion JSON은 기본 구독하지 않는다. 운영 서버는 종료하지 않았다.
사용자는 [실행 명령](../../operations/commands.md)에 따라 한 웹씩 켜서 직접 주행을 확인한다.

최종 로컬 재검증은 `/tmp/asmc-ci-exclusive-r2/summary.json`에 있다. LiDAR 단독100.624s 후
나머지 그룹을 병렬로 실행해 전체285.638s, 모든 그룹이 통과했다. 그룹 시간 합계는393.376s다.
최초 전면 병렬화203.373s보다 느리지만 시간 민감 검사의 외부 부하를 줄이는 구성을 택했다.
서버 헤더0.5s/전체 요청1.5s와 시험 내부 기준은 유지한다.
CI pip bootstrap wheel은 Noetic Python3.8에서 다운로드 호환성을 확인했으며 운영 pip에는 설치하지 않았다.
