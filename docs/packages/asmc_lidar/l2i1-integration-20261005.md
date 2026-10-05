# L2-I1 단일 LiDAR 후보 패키지 — 검증 기록

## 범위와 결론

최신 origin/main 252f83a에서 새 feat/asmc-lidar-integration worktree를 분리했다.
기존 feat/asmc-lidar-runtime 실험 worktree와 미커밋 사용자 변경은 수정하지 않았다.
손재호 6435af1의 재현 가능한 지면/OBB 코드와 V4 보완 지면을 선별 이식했고
구조물·군집·추적의 향후 소유 경계를 [인계 문서](./README.md)에 적었다.

후보 단일 경로는 LiDAR+IMU만 구독한다. GPS·미션 번호로 일반/터널 모드를 바꾸지
않는다. 지면 후보의 source stamp·원본점 계보를 유지하고 공개 점군과 OBB는
후륜축 base_link다. 입력 점군의 velodyne frame을 검사해 장착 변환 중복을 막는다.
구조물 의심 지지점과 프레임별 미분류 객체를 **둘 다 보존**한다. 구조물 판정에 의한
자동 삭제는 OFF이며, 현재 OBB에는 지속 ID·분류·속도가 없다.

이 결과는 **실행 가능한 인계 기준선**이지 운영 판제 입력으로 채택한 성능 결과가
아니다. 기존 Main/Msg, bridge, 판제, 센서 설정 및 기존 토픽·MD5를 바꾸지 않았다.

## 수행한 검증

| 검증 | 결과 | 해석 한계 |
|---|---|---|
| Noetic 독립 컨테이너 build, CTest | 전체 빌드 및 8/8 통과 | 합성 좌표/원본점/문맥 회귀 포함 |
| V4 동기 GT 1,439 scan | 기존 jaeho_support와 원본점 계보, 지면/구조물 마스크, 이벤트·평면 진단 0불일치 | 처리시간 필드는 비결정적이라 비교 제외 |
| V4 터널 2,360 scan | 동일 필드 0불일치 | 객체 GT 없는 터널 주행의 새 객체 성능 승인 아님 |
| Capture 105장면 × 2 카메라 상태 × 5 정차 예열 | 1,050행 처리, 프레임 후보 OBB 4,580개, 구조물 가설 2,810개; 반복 결과 SHA 완전 일치 | 반복 관측을 1,050 독립 상황으로 세지 않음. 클래스·속도·정답 정확도 미평가 |
| 격리 ROS bag 터널 접근→진입 35초 | 365수신/364처리, IMU stale 1, queue drop 0. p50/p95/p99 37.79/77.14/78.72ms | 약 10.4Hz 기록 입력 |
| 격리 ROS bag 내부→복귀 35초 | 384수신/384처리, 무효·drop 0. p50/p95/p99 29.80/71.73/75.16ms | 약 11.0Hz 기록 입력 |

두 replay는 같은 launch·설정·좌표 계약을 사용했고 GPS 토픽을 재생하지 않았다.
관측 stamp는 중복·역행 없이 유지됐다. 진입의 IMU stale 한 건은 빈 장애물 출력으로
위장하지 않고 경고로 기록했다. 전체 bag의 모든 223초를 한 번에 연속 검증한 것은
아니다. p95가 15Hz 간격 약 66.7ms를 넘으므로 **15Hz 동시 실행 gate는 미통과**다.
실제 시뮬→브릿지→LiDAR의 전체 지연으로 이 수치를 해석하지 않는다.

고정 입력·소스·산출물 해시는
[L2-I1 manifest](../../../config/baselines/lidar_l2i1_20261005.yaml)에 있다.
대용량 JSONL과 로그는 ASMC_DATA/lidar_assessment/20261005_l2i1에 보존했다.
첫 live_smoke 기록은 rosbag의 BAG 경로를 --topics 뒤에 둔 CLI 오류로 입력 0건이었다.
그 로그는 남겼고, 이후 BAG 경로를 옵션 앞에 둔 live_smoke2/live_entry/live_exit만
계측 결과로 사용했다.

## 재실행

기본 빌드·단위 시험:

~~~bash
source /opt/ros/noetic/setup.bash
cd /root/ws
catkin_make --only-pkg-with-deps asmc_lidar -j2
source devel/setup.bash
cd build/asmc_lidar && ctest --output-on-failure
~~~

동일 코어의 고정 snapshot 재처리(출력은 없는 새 파일을 지정):

~~~bash
cd /root/ws
devel/lib/asmc_lidar/asmc_lidar_ground_live_offline \
  /data/lidar_assessment/20261004_gt_dual_ground/sync/snapshots.jsonl \
  src/asmc_lidar/config/autonav_v3.yaml /data/lidar_assessment/<새_실험명>/jaeho_sync.jsonl \
  guard25 jaeho src/asmc_lidar/config/ground_v4/jaeho_support.yaml
devel/lib/asmc_lidar/asmc_lidar_unified_offline \
  /data/lidar_assessment/20260929/l1b5a_capture_v1/snapshots.jsonl \
  src/asmc_lidar/config/autonav_v3.yaml /data/lidar_assessment/<새_실험명>/unified_capture.jsonl
~~~

격리 ROS 재생에는 기존 master/bridge 대신 별도 Noetic 컨테이너에서
ROS_MASTER_URI=http://127.0.0.1:11431, ROS_IP=127.0.0.1로 roscore -p 11431을
실행한다. 두 번째 shell에서 unified.launch rviz:=false, 세 번째에서
tools/lidar/l2i1/monitor.py 출력.json 43을 실행한다. 그다음 bag 파일을
**--topics 앞에** 둔다.

~~~bash
rosbag play /data/lidar_assessment/20261005_ground_v3_drive/tunnel_path_all/drive_0.bag \
  --clock -s 80 -u 35 --topics /velodyne_points /imu
# 복귀 검증은 -s 165 -u 35
~~~

## 남은 승인 게이트와 rollback

1. 손재호는 현재 미확정 구조물 후보가 차량 측면·가드레일·벽 가까운 낮은 장애물을
   구별하는지 GT로 검증한 뒤에만 억제 조건을 별도로 승격한다. 군집 분할·합침과
   지속 ID/속도도 다른 회귀 단위다.
2. 15Hz 실제 센서셋, camera/HD맵/planning 동시 실행에서 p95/p99·queue drop·CPU
   부하를 다시 측정한다. 이번 약 10~11Hz bag 결과로 통과시키지 않는다.
3. GPS 정상→무효→복구와 optional HD맵/카메라 실제 토픽 어댑터, 판제 typed 객체
   메시지 및 추적 출력은 별도 계약 후 연결한다. 현재 local 단일 경로는 GPS 없이
   실행되지만 map 출력 가용성 검증을 끝낸 것은 아니다.

문제가 생기면 unified.launch만 종료한다. 기존 Main과 판제 입력은 손대지 않았으므로
운영 rollback은 이미 보존돼 있다. [과거 V4 지면 선택 근거](./l2a2-v4-ground-selection-20261005.md)의
객체 하부 삭제·터널 잔여 한계를 다음 성능 작업에서 숨기지 않는다.
