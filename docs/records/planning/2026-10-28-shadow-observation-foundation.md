# 음영 판제 관측·표시 기반 검증

2026-10-28. 기준 main `54f6bb9a`, 작업 시작 `4932b7d6`.
설계와 다음 단위는 [음영 판제 개선](../../architecture/shadow-planning-refactoring.md)을 따른다.
이 기록은 인터페이스·표시·기존 제어 회귀이며 새 MORAI 주행 수락이 아니다.

## 변경과 유지 범위

- 공용 LiDAR 관측의 원본 클래스·그룹과 session/generation/scan/config/calibration 식별자를
  private planning 입력까지 보존했다. Behavior/Frenet 객체도 클래스·그룹을 유지한다.
- 숫자 type, 운동 상태, 속도 validity와 행동 정책은 유지했다. 16종의 전달을 검사했고
  static 의미 그룹으로 강제 정지 상태를 만들지 않았다.
- source-age 검사를 공용 정수 ns 함수로 모았다. 기존300ms/미래50ms 제한과 사용자 설정의
  교집합을 유지한다. source·수신 clock을 혼합하거나 새 pose를 생성하지 않는다.
- 정본 LiDAR launch가 활성 fusion의 front 설정 또는 비최적화 터널 YAML을 조용히 실행하지
  못하게 했다. 명시적인 migration namespace 비교는 가능하다.
- 판제 웹에 실제 선택 제어·음영 정지 사유·bbox 개별 사유·family support·실제 생성 경로와
  typed local 객체를 연결했다. GPS/map·LiDAR debug 입력 없이도 진단 가능하다.
- 웹의 local 객체는 현재 음영 회피 입력이 아니다. 표시를 위해 재피팅·재투영·차량 제어를 추가하지 않았다.

## 메시지와 배포

공용 `asmc_msgs` 정의는 변경하지 않았다. 다음 **private** 메시지는 새 필드로 MD5가 바뀐다.

| 메시지 | 이전 MD5 | 변경 후 MD5 |
|---|---|---|
| planning_inputs/DetectedObject | `5e47469ca3513469843668044f6c2ecc` | `57732f7f96117cfea4793870ce3de16d` |
| planning_inputs/DetectedObjectArray | `510be3b5a25a92d2ee02a16740b031d1` | `05c44986409fa9c0b11ed96dbe830b0e` |

소스를 적용한 workspace 전체를 빌드하고 입력 어댑터·Behavior·Frenet·dashboard를 함께 재시작한다.
실행 중 메시지 생성 파일만 바꾸지 않는다. 기존 private bag은 당시 revision 또는 명시적인
오프라인 변환이 필요하다. 컨테이너가 마운트한 checkout에 변경이 반영됐는지 먼저 확인한다.

```bash
cd /root/ws
./scripts/build_ws.sh -j2 -l2 -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch asmc_planner dashboard.launch
```

주소는 `http://127.0.0.1:8088/web/index_v2.html`이다. 생산자·autonomy 실행은
[현재 명령 정본](../../operations/commands.md)을 따른다.

## 로컬 검증 결과

운영 master·노드·컨테이너 마운트를 바꾸지 않고 컨테이너의
`/tmp/asmc-shadow-foundation`에서 변경 패키지를 빌드했다. 나머지 패키지는 현재 workspace
소스를 참조한다. 아래 결과는 hosted CI와 별개다.

| 검사 | 결과 |
|---|---|
| Release workspace 빌드·planner 설치 | 통과. devel/install 양쪽 dashboard helper import 확인 |
| LiDAR CTest | 고유41개 통과. 정본 설정 거부·3뷰/터널·출력·ROS 회귀 포함 |
| geometry CTest | 1개 통과. ns 경계 포함 |
| planning 입력 gtest | 16개 통과. 16종 그룹, 정상 빈 결과/무효, epoch, 시각 경계 포함 |
| planning 입력 실제 ROS | 1개 통과. 문자열·동일 관측 ID 전달 및 무효/stale 제외 |
| planner CTest | 선택한 고유61개 통과. 명령·회피·회전교차로·차량 여유·메시지 변환 등 |
| ControlSelector 실제 ROS | 3개 통과. 제어권·신규 선택 명령 freshness·복귀 |
| dashboard 실제 ROS→HTTP | 1개 통과. GPS/debug 없는 local 표시, invalid 즉시 제거와 freshness 만료 |
| dashboard 순수 Python/JS | Python9개·JS 검사2개 통과 |
| headless Chromium | 실제 웹 자산+합성 snapshot으로 local/map 전환·정지 사유·콘솔 오류 검사 통과 |
| 문서·이전 기록·실행 명령·정본 의존성 | 검사 통과. 과거 실험 수치·동결 자료 보존 |

초기 격리 시험에서 필수 gtest 실행파일의 별도 빌드와 지도/tools 경로 연결이 빠져 실패했다.
격리 환경을 보완한 뒤 실패 항목을 **이름 기준으로** 다시 실행해 위 통과 집계를 만들었다.
이를 알고리즘 결함 수정 또는 첫 실행 전부 통과로 표시하지 않는다.
ROS용 dashboard 모듈은 실행 relay로 설치하면 import가 실패하는 문제를 실제 빌드에서
확인해 일반 module 파일로 devel/install 양쪽에 설치하도록 고쳤다.

작은 ROS fixture와 브라우저 합성 검증은 실제 터널 주행·프레임 공급 성능 측정을 대체하지 않는다.
새 bag·학습·주행 수집은 하지 않았고 현재 사용자의 운영 프로세스도 재시작하지 않았다.

## 남은 한계와 다음 작업

- plastic_barrier/drum의 contact overlap·far obstacle·timeout 정지는 아직 해결하지 않았다.
- GPS 무효의 map unavailable/일반 MPC INVALID_EGO와 실제 선택된 음영 제어 정지를 구분한다.
- 473.47ms 공급 실패, 얇은 물체 소실, 실제 속도 정확도는 기존 미해결 목록에 남긴다.
- S1에서 같은 입력의 정지 원인을 고정한 뒤 S2 구조 흡수, S3 local 통로·객체 계획,
  S4 단일 제어 연결을 순차 검증한다. IMU 위치 적분과 HD맵 재활성화는 이번에 구현하지 않았다.

## 후속 사용자 컨테이너 적용·한글 표시 확인

같은 날 사용자 요청으로 코드 revision `a7e21f16`을 컨테이너가 마운트한
`2026-ASMC` checkout에 적용하고 `/root/ws` 전체를 Release 재빌드했다.
작업 브랜치는 `2026-ASMC-work`에 유지하며 main 브랜치를 병합한 것은 아니다.
확인 당시 실행 중인 운영 ROS master/판제/카메라/LiDAR 노드는 없었다.

- 새 private 메시지 MD5가 위 변경 후 값과 일치했다. 새 interactive 셸은 기본
  `/root/ws/devel/setup.bash`를 사용하며 개인 overlay가 필요 없다.
- LiDAR 빌드 점검의 `source_matches=true`, package source SHA는
  `a2e59ee7597f715d0b6b304745dcc3a20fc1320a2455ac1ac6570db034d7c908`다.
- 실제 배포 빌드로 LiDAR CTest **41/41**, 56.38초; launch 선택4개;
  planning 입력 ROS와 dashboard ROS→HTTP 각1개 통과.
- 웹 HTML/CSS/JS는 UTF-8이고 HTTP에도 charset을 명시했다. 버튼/선택창·진단 고정폭 글꼴에
  한글 fallback을 적용했다. Chromium에서 실제 CSS와 OS의 Malgun Gothic 글리프 사용을 확인하고
  한글 화면을 검수했다. 외부 폰트 다운로드나 Windows 폰트 파일의 Git 배포는 하지 않는다.
- source age·경로 등 화면은 합성 ROS/브라우저 fixture로 검사했다. 실제 차량 주행 검증은 아니다.
- 마지막에는 기본8088의 읽기 전용 `dashboard.launch`만 시작했다.
  `/behavior_dashboard_server`와 `/rosout`만 등록됐고 제어 발행자는0이었다.
  센서·인지·autonomy 실행 전에는 정상적으로 데이터 대기 상태다.

구 `/migration/lidar` 운영 전환은 이미 완료됐다. 현행 실행 안내·회귀 시험은
운영 `/perception/lidar`, 명시적인 격리 시험 예시 `/test/lidar`로 구분한다.
새 코드 경로나 호환 별칭을 만든 것이 아니며 namespace 인자의 일반 기능을 유지한다.
과거 bag·A1/F1 재현 도구의 원래 토픽 문자열은 당시 기록을 읽기 위해 보존한다.
새 시험 없이 이 문자열을 현재 운영 출력으로 사용하지 않는다.
