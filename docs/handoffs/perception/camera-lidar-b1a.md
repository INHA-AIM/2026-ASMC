# B1a 재호 터널 구조물 필터 검증

> B0 PR #70은 `07cc51a`에 merge commit으로 병합했다. 부하 조합의 운영 승인은 아니다.
> 이번 비교는 팀원 브랜치 `8dde9b691bebd4f9d5172d0c43bd6b16d2052c2b`를 고정한다.
> 운영 소스·메시지·기본 설정은 변경하지 않는다. 팀원 브랜치를 main에 복사하지 않는다.

## 실험 범위

재호의 실제 `TunnelFilterCore`와 ROS 보호 경로, `Main` 실행기를 별도 catkin workspace에서
빌드한다. 197개 원본 파일의 SHA를 기록하고, 시험 launch와 별도 합성 probe만 추가한다.
시험 launch는 UDP 드라이버·YOLO 노드·GUI를 제외하고 planner UDP를 끈다. legacy 제어
발행은 `/b1a/discarded_ctrl_cmd*`로 격리한다. 공유 bridge·master에는 연결하지 않는다.

B0 C1 첫 실행의 YOLO 결과를 재사용한다. 원래 소수 좌표를 기존 `int()` 방식으로
변환하며 원본 라벨·순서·source stamp·frame_id와 빈 결과를 유지한다. 도착 가용 시각은
B0 관측기의 `ros_ns`이며 원래 monotonic 수신 시각도 sidecar에 보존한다. 이 ROS 시각은
bag `/clock`의 해상도에 제한되므로 나노초 단위 실제 GPU 완료 시각으로 해석하지 않는다.
기존 YOLO header seq는 B0 관측에 없으므로 복원했다고 주장하지 않는다.

| 조건 | 실험 필터 | YOLO 보호 | 터널 dynamic ROI |
|---|---|---|---|
| T1 | OFF | 원래 설정 | 원래 설정 |
| T2 | ON | OFF | ON |
| T3 | ON | ON | ON |
| T4 | ON | ON | OFF |

오프라인은 두 bag × 네 조건 = 8회다. 각 실행은 전체 scan을 연속 처리하며 GPS 무효
샘플을 주입한다. 원본 평가기의 130ms 처리 지연을 고정하고 시간 상태를 매 프레임
초기화하지 않는다. 평가기에 포함된 관측 대기·중간 결과 재계산 시간은 실제 처리율이 아니다.
`A`는 해당 설정의 실제 운영 처리 경로이고, 필터 ON에서 항상 기존 OFF 결과라는 뜻은 아니다.
조건 간 비교는 같은 scan의 T1/T2/T3/T4 기록으로 한다.

1배속 재생은 T1/T3 × 두 bag × 세 GPS 조건 × 3회 = 36회다. 반복마다 순서를 바꾸고
새 master와 노드를 시작한다. 정상 GPS, 무효 샘플, 전달 중단을 구분한다.

| bag | 무효/중단 구간 | LiDAR 입력 | 자연 GPS 단절·객체 GT |
|---|---|---|---|
| `lidar_manual_20260929_130623/raw_0.bag` | 20–40초 | 649 | 없음 |
| `20260912_003803…shadow_static_avoid…_0.bag` | 6–20초 | 208 | 없음 |

무효 샘플은 `/gps`의 status·좌표만 0으로 바꾸고 header는 유지한다. 존재하는 `/gps/fix`도
무효로 바꾼다. 전달 중단에서는 두 GPS 토픽만 생략한다. IMU·Ego·카메라·LiDAR는
계속 공급한다. 원본 bag은 읽기 전용이며 주 bag에는 `/gps/fix`가 없다.

## 재현 환경과 실행

MORAI가 종료된 상태에서 `asmc-camera-lidar-b1a` 전용 network-none 컨테이너와
master 11314를 사용한다. 공유 `asmc-ros-noetic`와 master 11311은 유지한다.
측정은 GUI·새 모델 추론 없이 직렬 수행한다. 200ms CPU/RSS·전체 GPU/VRAM 표본에는
측정 관측기 비용과 Windows desktop GPU 사용도 포함된다. full camera stack 성능이 아니다.
호스트 감시기는 15초마다 Windows Simulator 프로세스를 확인한다. 실행 중·확인 실패·
60초 이상 오래된 확인이면 행렬을 중단한다. 예비 실행 `protocol-79448bc`는 시작 검사 결과를
늦게 확인하여 Simulator 실행 중에 일부 진행됐으므로 고정 수락 측정에서 제외하고 보존한다.

호스트의 이 브랜치에서 원본 소스를 준비한다. 출력 디렉터리는 새 경로여야 한다.

```bash
python3 tools/perception/prepare_b1a_source.py --output /tmp/asmc-b1a/source
cp -r tools/perception/b1a_probe /tmp/asmc-b1a/source/src/b1a_probe
```

위 소스를 컨테이너 `/root/filter_ws`, 증거 디렉터리를 `/evidence`, 원본 자료를 읽기 전용
`/data`로 마운트한다. `/root/ws`는 현재 검증 브랜치이며 바깥 메시지를 먼저 source한다.

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
cd /root/filter_ws
catkin_make -j4
catkin_make run_tests_Main_gtest_test_tunnel_filter_core -j4
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11314
export ROS_HOSTNAME=localhost
```

시험 입력 생성과 전체 실험 실행은 Python 검사 도구로 한다. 기존 디렉터리를 덮어쓰지 않는다.

측정 중에는 호스트 WSL 터미널에서 상태 감시를 유지한다. `/evidence`의 호스트 마운트 경로를 사용한다.

```bash
python3 tools/perception/watch_b1a_host.py --state /tmp/asmc-b1a/evidence/host-state.json
```

```bash
python3 /root/ws/tools/perception/make_b1a_fixtures.py --output /evidence/fixtures-final
python3 /root/ws/tools/perception/run_b1a_matrix.py \
  --fixtures /evidence/fixtures-final --output /evidence/protocol-final \
  --revision <ホストで確認した実行コミットSHA> --host-state /evidence/host-state.json
python3 /root/ws/tools/perception/analyze_b1a_matrix.py /evidence/protocol-final
```

## 직접 launch·시각화

전체 측정이 종료된 후에만 별도 시각화를 실행한다. 각 터미널에서 위 ROS 환경과 master
11314를 설정한다. GUI에는 DISPLAY와 X11 마운트가 필요하며 GUI 컨테이너 진입은 별도로 한다.
원본 Main launch는 드라이버·구 YOLO를 실행하므로 아래 생성된 **시험 launch**를 사용한다.

master 터미널:

```bash
roscore -p 11314
```

필터 터미널:

```bash
rosparam set /use_sim_time true
roslaunch /root/filter_ws/b1a_main.launch \
  enable_tunnel_experimental_filter:=true tunnel_protect_enable:=true
```

재생 터미널:

```bash
rosbag play --clock --rate 1.0 /evidence/fixtures-final/shadow-invalid.bag
```

GUI는 별도 터미널에서 연다.

```bash
rqt_image_view --clear-config /b1a/front_forwarded
rviz -d /root/ws/tools/perception/b1a.rviz
```

RViz는 수평화된 터널 단계 점군만 비교한다. legacy `velodyne` frame_id가 원본 센서·
후륜축 `base_link` 좌표를 보장하지 않는다. 이 설정에 raw 점군·map 객체를 겹쳐 넣지 않는다.
GUI의 최신 영상/점군 조합은 동기 정답이 아니다. 같은 관측의 제거 전후 비교는 저장된
대표 프레임과 수평화 역변환·장착 변환을 검증한 투영 자료를 사용한다.

## 판단 범위와 후속 단계

벽·천장 제거 수와 track ID 개수는 정답 정확도·recall이 아니다. 원본 영상·점군과 같은
stamp에서 제거·유지 점, 객체 잘림·합침을 검토한다. 제거된 구조물을 자유 공간으로 보지 않는다.
실제 객체 GT 부족·소실·좌표 계약 미확정이 남으면 운영 채택은 보류한다.

1. **local/map 공동 설계:** local 처리·추적을 GPS/map 변환에서 분리한다. LiDAR local-only와
   판제의 관측 pose 변환, 또는 LiDAR local+독립 map 어댑터를 소비자 요구로 비교한다.
   pose 공급 주체·관측 stamp·변환 유효성·좌표·변환 중복 방지를 먼저 합의한다.
2. **COCO 확대:** YOLO11m 전용 시험 프로필에서 네 클래스/전체 클래스 후보를 비교한다.
   source 라벨을 보존하고 person/vehicle/obstacle 대응·인프라 제외·미지원 장애물을 평가한다.
   legacy LiDAR 호환 네 클래스 정책은 별도로 유지한다. 카메라 only 운용은 아직 확정하지 않는다.
3. **B1b 기하 개선:** B1a 잔여 벽 혼입·객체 합침에 같은 box·raw 점군의 깊이·연결성 연관을
   추가한다. 기존 box 보호와 대조하며 이번 PR에 새 알고리즘을 넣지 않는다.
4. **모델·성능:** YOLO26 detect → seg boxes-only → 같은 seg의 masks 사용으로 효과를 분리한다.
   첫 1초 차선 공급, 카메라 수·10Hz 조합, 데이터 감사·학습, FP16·batch·공통 처리·TensorRT와
   RTX 4070 SUPER 인수는 각각 독립 검증한다.

판제가 구간 활성화와 결과 소비를 결정한다. 차선 피팅 소비는 음영 ODD로 한정하며,
기존 jamming 어댑터와 21% crop 기본 설정은 유지한다.
