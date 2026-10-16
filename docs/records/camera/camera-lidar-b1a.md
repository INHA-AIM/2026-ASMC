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
  --revision 19a0b29 --host-state /evidence/host-state.json
python3 /root/ws/tools/perception/analyze_b1a_matrix.py /evidence/protocol-final
```

`--revision`에는 호스트에서 확인한 실제 측정 도구 커밋 SHA를 기록한다. 위 값은 이 검증의
도구 고정 revision이며 다른 코드로 재현할 때는 그 코드의 SHA로 교체한다.

## 직접 launch·시각화

전체 측정이 종료된 후에만 별도 시각화를 실행한다. 각 터미널에서 위 ROS 환경과 master
11314를 설정한다. GUI에는 DISPLAY와 X11 마운트가 필요하며 GUI 컨테이너 진입은 별도로 한다.
현재 `asmc-camera-lidar-b1a`는 측정 종료 후 로그를 보존하고 X11을 연결한 GUI 확인용
프로필로 재생성했다. Qt 화면 연결을 확인했고 master·필터·GUI는 자동 실행하지 않았다.
GUI 프로필의 software GL 설정은 측정 프로필과 구분한다. 각 터미널에서 이 컨테이너에
직접 진입하고 ROS 환경을 읽은 뒤 아래 명령을 실행한다. 공유 실시간 master 11311과 격리돼 있다.
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

2026-10-08 추가 코드 검토로 확인한 카메라 클래스 fusion의 역수평화 경계와
local/map 연관·출력 역할은 [LiDAR 개선 과제](../../../src/asmc_lidar/docs/migration-tasks.md)에
연결했다. 이 문서 보강은 운영 코드를 변경하지 않는다.

### 확인할 출력 경계

이 브랜치의 `/lidar_nogps`는 일반 구간에서 항상 비어 있지 않은 공통 local 생산자가 아니다.
무효 GPS·터널 latch에 따라 local 객체를 채우며 `/last_lidar`는 그때 비운다. 동시에
`/lidar/tracked_objects_map`은 별도로 발행된다. 무효 좌표와 GPS 메시지 중단에서 pose의
유효성·최신성을 표시하는 계약이 없어 map 발행을 소비 가능 판정으로 간주하지 않는다.

GPS status=0 또는 좌표 모두 0이면 즉시 jamming 상태로 들어간다. 터널 latch는 무효
샘플 3회로 확인하고 유효 샘플 3회로 해제한다. 정상 상태에서 메시지만 끊는 경우는
이 무효 샘플 판정이 발생하지 않는다. 현재 main의 stale GPS gate로 local 처리도 멈췄던
B0 결과와 다른 실행 경로이며, 어느 쪽의 운영 수정도 이번 PR에서 수행하지 않는다.

터널 점군의 수평화는 `Ry(pitch) Rx(roll)`인 반면 기존 map 어댑터는 pitch 역회전만
적용한다. 원본 코드에 대한 별도 합성 벡터 probe로 이 좌표 경계를 확인한다. 이 문제의
수정과 frame_id·관측 pose·유효성 계약은 local/map 공동 설계로 분리한다.

터널 실험 플래그 ON은 GPS 정상 구간에서도 진입 전 천장 절단과 대형 OBB 방어 규칙을
적용한다. 따라서 정상 GPS의 T1/T3가 항상 동일하다는 가설은 두지 않는다. OFF 보존은
T1의 실제 RANSAC과 기존 함수 재계산 결과의 비트 단위 대응으로 검사한다.
실험 ON의 `replica_mismatch`는 변경된 최종 점군과 원래 RANSAC의 비교이며, 자체로
처리 결함·누락 정답을 뜻하지 않는다.

투영은 원본 수평화 공식의 역행렬과 원본 장착 변환·광학 축을 합성 시험으로 확인한다.
평가기의 roll/pitch 기록은 소수 5자리이므로 대표 투영은 진단 그림이다. 같은 raw scan과
카메라 stamp 차이, 사용한 보정 SHA를 기록하며 물리적 센서 보정의 독립 수락이나 객체 GT
검증으로 확대하지 않는다. 단계별 keep 점에는 최종 OBB 방어 규칙으로 제외되는 군집도 있다.

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


## 완료 결과 — 2026-10-08

고정 조건의 오프라인 8회와 1배속 36회를 모두 완료했다. **검증 기준선 완료이며 필터 운영 채택은 보류한다.**
측정 도구는 `19a0b29`, 팀원 소스는 `8dde9b6`, 후처리·보존 도구는 `df1a20c`로 고정했다.
197개 원본 파일과 실제 측정 바이너리 SHA는 일치한다. 운영 카메라·LiDAR 소스·메시지·모델·
HD맵·기본 설정을 바꾸지 않았으므로 기존 B0 회귀와 3,514프레임 동등성 증거를 재사용한다.
기존 facade 실패 1건·승인된 분류 차이 2건은 B0 증거에 그대로 남으며 새 예외로 확대하지 않는다.

### 처리 scan과 GPS 조건

다음은 전체 입력 중 실제 처리한 서로 다른 LiDAR source stamp 수다. 객체 수·recall이 아니다.
입력은 주 bag 649 scan, 음영 bag 208 scan이다. 생략 stamp·지연·리소스·GPS 복구·출력 좌표는
[전체 지표](../../../config/baselines/evidence/camera_lidar_b1a_v1/summary.json)에 반복별로 보존했다.

| bag | GPS | T1 OFF — 1/2/3회 | T3 ON+보호 — 1/2/3회 |
|---|---|---|---|
| 주 | 정상 | 570 / 567 / 567 | 580 / 574 / 573 |
| 주 | 무효 | 571 / 566 / 570 | 621 / 617 / 618 |
| 주 | 전달 중단 | 570 / 566 / 568 | 578 / 577 / 578 |
| 음영 | 정상 | 202 / 204 / 204 | 205 / 206 / 206 |
| 음영 | 무효 | 203 / 204 / 204 | 207 / 206 / 207 |
| 음영 | 전달 중단 | 204 / 204 / 204 | 204 / 206 / 206 |

무효 주입 구간의 T3 처리량은 주 bag 265개 입력 중 매번 264개, 음영 bag 118개 중 매번
118개였다. 정상 또는 전달 중단에서는 실험 터널 필터가 활성화되지 않았다. 무효 샘플 후
local 처리는 이어지지만 무효 map 좌표도 계속 발행돼 최대 절댓값이 약 **4,124,175m**에
이른다. map 발행 자체를 유효 위치로 판단하면 안 된다. 정상 GPS 3회로 latch가 해제되는
과정도 기록했다. `/lidar_nogps`는 정상 구간의 항상 활성인 local 출력이 아니다.

29개 실행에서 초기 빈 출력의 ROS-now fallback stamp 다음에 실제 첫 LiDAR stamp가
나오며 역행했다. map·nogps·last 각각 29건, 총 87건이다. 모두 첫 실제 source 출력으로
넘어가는 시점이며 처리된 ROI stamp에는 역행·중복이 없었다. 전역 역행 0이라고 판정하지
않는다. [원본 쌍](../../../config/baselines/evidence/camera_lidar_b1a_v1/support/startup-stamp-reversals.json)을
local/map 출력 계약의 초기 무관측 정책 검토에 넘긴다.

### 제거 효과와 객체 보존 한계

오프라인 T3 활성 구간에서 주 bag은 천장 77,066점·벽 1,737,188점을 제거하고 42,227점을
유지했다. 음영 bag은 천장 38,403점·벽 775,457점을 제거하고 24,927점을 유지했다.
모두 voxel 입력 이후 단계 합계이며 raw→ROI 축소로 사라진 점과 구분한다. 단계 합계
불일치는 없었다. T1 OFF의 실제 RANSAC과 원본 함수 재현은 주 649·음영 208 scan 모두
일치했다. 필터 ON은 정상 구간의 진입 전 천장 처리도 바꾸므로 OFF 동일성을 요구하지 않는다.

T2는 보호 입력을 사용할 수 없어 애매한 벽 점을 보수적으로 남긴다. T3의 정상 빈 YOLO
결과는 사용 가능한 보호 입력이므로 이 점들을 더 제거한다. 따라서 T2/T3 차이를 모두
검출 객체 보호량으로 해석하지 않는다. 주 T3에는 벽에 붙은 보호 후보 **79개 관측**이 남았다.
이는 79개의 독립 차량이라는 뜻이 아니다. 잔여 벽 혼입은 B1b 깊이·연결성 비교 대상이다.

원본 C++ core 시험 28개, 실제 ROS 보호의 ±50ms 포함·초과·빈 결과·미수신·늦은 도착
7개 경우를 확인했다. 높은 차량의 보호점 30개가 보호 판정보다 앞선 천장 절단에서 전부
사라지는 합성 반례도 재현했다. 이 반례 재현 성공을 객체 보존 성공으로 표시하지 않는다.
보호 경로는 그 처리 시점에 도착한 최근접 결과를 즉시 사용하며 늦은 결과로 이전 점군을
다시 보호하지 않는다. 기존 fusion matcher의 200ms 대기와 다른 동작이다.
별도 map probe는 roll 0에서 벡터 오차 0.00000017m, roll 0.15rad에서 **0.335096m**를
보였다. 수평화의 roll 역변환이 map 어댑터에서 빠지는 경계 문제는 운영 수정 없이 남겼다.

같은 scan의 원본 영상·필터 전후 점군을 6개 사례로 비교했다. 주 bag의 흰 차량 점은
대표 사례에서 유지되고 벽·천장 점은 제거됐다. 음영의 상자는 기존 YOLO 네 클래스에서
검출되지 않으며, 일부 사례에서 LiDAR 군집도 없어 카메라 class 확대·미지원 장애물 평가가
필요하다. T1에도 해당 객체의 독립 대응이 없으므로 이를 T3가 새로 놓친 정답으로 단정하지
않는다. GT가 없어 차량·보행자·상자 recall이나 충돌 안전성은 수락하지 않았다.

- [대표 그림 모음](../../../config/baselines/evidence/camera_lidar_b1a_v1/cases/index.html)
- [주 bag 차량 사례](../../../config/baselines/evidence/camera_lidar_b1a_v1/cases/primary-case-01.png)
- [음영 상자 사례](../../../config/baselines/evidence/camera_lidar_b1a_v1/cases/shadow-case-02.png)
- [baseline](../../../config/baselines/perception_camera_lidar_b1a_v1.yaml)

### 증거 보존과 PR 범위

Git에는 모든 고정·예비 실행의 프레임 지표·관측·로그·CPU/RSS·전체 GPU 표본과 6개 그림,
18개 무손실 대표 점군 NPZ, 원본 replay source를 넣었다. 원본 영상과 입력 bag은 SHA와
생성 recipe로 연결한다. 전체 scan 점군이 든 대용량 원본 archive는 다음 호스트 경로에
별도 보존했다. 일부 성공 실행만 선택하거나 실패 사례를 삭제하지 않았다.

```text
data/perception_baselines/camera_lidar_b1a_v1/full-evidence/
```

이 경로는 workspace 공통 루트 기준이다. [provenance](../../../config/baselines/evidence/camera_lidar_b1a_v1/provenance.json)에
절대 경로·바이트 수·SHA를 기록했다. Git archive에서 제외한 것은 전체 `clouds.bin`뿐이며
모든 scalar 기록은 남겼다. replay source에서 제외한 두 legacy weight는 새 추론에 쓰지
않으며 기존 저장 위치와 원본 SHA는 보존한다. `/tmp` fixture는 원본 bag·고정 YOLO events로
재생성할 수 있다. Simulator 실행 중이었던 예비 측정은 고정 수락 측정에서 제외했다.

PR #71은 검증 도구·실패 기준선·후속 과제만 다룬다. 실제 필터 도입, GPS 의존성 수정,
map 계약, COCO 확대, B1b 알고리즘은 별도 PR로 검증한다. 다음 우선순위는 **local/map
출력 계약 검토와 독립 YOLO11m COCO 확대 시험**이다. 이 결과를 바탕으로 남은 벽 혼입·
객체 합침을 B1b에서 비교한다. 카메라 only 회피는 아직 결정하지 않았다.
