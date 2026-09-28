# L1-B4-A 격리 벽·장애물 분리 평가

손재호 `6435af1`을 읽기 전용 기준선으로 사용한다. 운영 ROS publisher나 planner 연결이 없는
C++/PCL 평가기이며, 좌표는 기존 **수평화된 센서 원점**이다. base_link/map 변환을 동시에
적용하지 않는다. [기준 manifest](../../../config/baselines/lidar_l1b4_20260929.yaml)와
[비교 보고서](../../../docs/packages/asmc_perception/lidar-l1b4-validation-20260929.md)를 함께 읽는다.

## 비교와 점 계보

`baseline`, `policy_only`(빈 배열만 미수신처럼 취급), `geometry`, `candidate_off`가 같은
ROI·voxel·수평화·지면·천장·벽 fit/persistence 결과를 쓴다. 기하 후보는 벽 제거 직전 label을
복원하고 새 삭제 마스크를 적용한 뒤 **원래 ClusterAndFilter와 L-shape OBB**를 다시 호출한다.
`prepare.py`는 build 디렉터리의 헤더에 friend 선언 하나만 추가하며 원본 algorithm body를
바꾸지 않는다. 이 절차는 삭제 정책 비교이며 독립적인 전체 벽 추정기를 평가한 결과가 아니다.

기하 허용대는 `median(abs(residual)) + 3 × 1.4826 × MAD`를 5~15cm로 제한한다.
법선은 센서 쪽이 양수이고, 지지 구간 안의 전경점만 20cm 연결 반경으로 묶는다.
30cm 확장은 원래 전경점에서 한 번만 적용하며 `signed residual >= band/2`를 요구해
벽 표면으로 전파하지 않는다. singleton도 보호 마스크에 남기지만 최종 10점 cluster 조건과
OBB 제거 규칙은 그대로다. 가설값은 실행 중 수정하지 않는다.

카메라 보호 class와 기하 보호 mask는 따로 기록한다. YOLO `unknown`은 보호 class가 아니며
기하 mask를 car/person class로 넣어 OBB 필터를 우회하지 않는다. 빈 결과는 free-space 근거가 아니다.

JSONL은 point label/reason, camera/geometry mask, wall model/support, cluster index, OBB와
제거 사유를 담는다. `clouds/<input_id>.bin`은 수평화된 float4 XYZI이며
`<input_id>.indices.bin`은 원본 raw point별 **little-endian int32 voxel index**다.
`-1`은 ROI 제외, 같은 voxel index는 voxel에서 합쳐진 raw 점이다. 이미 전처리한 fixture는
identity mapping이다. 점 개수·intensity 보존은 **voxel 이후 변환 단계**에서 검사한다.
voxel 자체가 원본 점 개수와 intensity를 그대로 보존한다는 의미가 아니다.

## 빌드와 고정 입력

저장소 루트에서 host 시험:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B -m unittest discover -s tools/lidar -p 'test_*.py'
```

ROS/PCL 환경은 기존 컨테이너를 사용한다. ROS runtime library를 찾으려면 setup source가 필요하다.

```bash
docker exec -it asmc-ros-noetic bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
source /data/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/devel/setup.bash --extend
cmake -S /root/ws/tools/lidar/tunnel_geometry -B /tmp/asmc-l1b4-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DLEGACY_MAIN_SOURCE=/data/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/src/Main
cmake --build /tmp/asmc-l1b4-build --parallel 2
cmake -E chdir /tmp/asmc-l1b4-build ctest --output-on-failure
```

입력 생성은 기존 36조건/180 frame을 원본 내용과 대조하여 보존하고, 80조건/400 frame을
추가한다. first-return 입력은 16 elevation과 0.25° azimuth 광선의 첫 교차면만 남긴
analytic fixture이며 MORAI/VLP16 재현이 아니다. GT는 별도 truth.jsonl에만 있고 후보에는 전달하지 않는다.

```bash
python3 -B /root/ws/tools/lidar/tunnel_geometry/fixtures.py \
  --legacy-snapshots /data/lidar_assessment/20260928/l1b3g_synthetic_v6/snapshots.jsonl \
  --legacy-summary /data/lidar_assessment/20260928/l1b3g_synthetic_v6/summary.json \
  --setting /data/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/src/Main/setting.yaml \
  --output /data/lidar_assessment/20260929/<new-fixture-run>
```

## 재실행

모든 output은 신규 경로를 요구한다. source revision/config/raw cloud/tool SHA는 실행 시작에
고정하고 입력·소스가 도중 바뀌면 보고서 생성을 실패시킨다. bag이나 AIM 원본을 복사하지 않는다.

```bash
python3 -B /root/ws/tools/lidar/tunnel_geometry/run.py \
  --snapshots /data/lidar_assessment/20260929/l1b4_fixtures_v1/snapshots.jsonl \
  --truth /data/lidar_assessment/20260929/l1b4_fixtures_v1/truth.jsonl \
  --setting /data/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/src/Main/setting.yaml \
  --params /data/lidar_assessment/20260926/offline_v2/resolved_params.yaml \
  --config /root/ws/config/baselines/lidar_l1b4_20260929.yaml \
  --executable /tmp/asmc-l1b4-build/tunnel_policy_eval \
  --output /data/lidar_assessment/20260929/<new-synthetic-run>
```

실제 bag의 이미 동결한 snapshot은 `--snapshots`를
`/data/lidar_assessment/20260928/l1b3g_geometry_v7/snapshots.jsonl`로 바꾸고 `--truth`를
제거한다. `--reference-geometry /data/lidar_assessment/20260928/l1b3g_geometry_v7/geometry.jsonl`을
추가하면 이전 evaluator와 frame별 baseline parity를 검사한다.
`--decision baseline`이 기본(즉시 매칭)이며 `--decision candidate`는 기록된 25ms 선택을
사용하는 보조 비교다. 새 pose를 대기 중 입력에 소급 적용하지 않는다.

```bash
# host에서; repeat에는 같은 입력으로 다시 실행한 신규 결과를 지정
python3 -B tools/lidar/tunnel_geometry/verify.py --data-root ../data \
  --run ../data/lidar_assessment/20260929/<synthetic-run> \
  --run ../data/lidar_assessment/20260929/<autonav-run> \
  --repeat ../data/lidar_assessment/20260929/<repeat-synthetic-run>

# 컨테이너에서 standalone BEV/side PNG와 SVG 생성
python3 -B /root/ws/tools/lidar/tunnel_geometry/plots.py \
  --run /data/lidar_assessment/20260929/<synthetic-run> \
  --truth /data/lidar_assessment/20260929/l1b4_fixtures_v1/truth.jsonl \
  --output /data/lidar_assessment/20260929/<new-figure-dir>
```

## AIM 보조 입력과 한계

host에서 `ASMC_AIM_DATA`를 사용한다. 원본 FLU를 FRU로 다시 뒤집지 않고 해당 세션의
장착 변환을 한 번만 적용하여 roundtrip을 확인한다. AIM declared base와 rear axle의
일치나 선택 점군의 attitude는 기존 감사만으로 승인되지 않아 filter replay는 보류한다.
카메라4·미검증 GT·우리 HD맵 station은 이 도구에서 사용하지 않는다.

```bash
ASMC_AIM_DATA=/mnt/d/asmc_aim/dataset python3 -B tools/lidar/tunnel_geometry/aim_support.py \
  --audit ../data/lidar_assessment/20260928/aim_audit_v3 \
  --output /tmp/<new-aim-support-run>
```

성공 판정은 wall-stage 객체 점 보존 95%, 최종 completeness 80%/purity 90%, 미수신 기준보다
잔여 벽 50% 감소, 기존 성공 객체 추가 소실 0 및 반복 결정성을 함께 검사한다.
정답 없는 실제 bag에서 filter의 wall label이나 객체 count를 정확도 점수로 바꾸지 않는다.
시간은 보호 계산·공통 전처리·copy·군집/OBB를 분리한 offline 값으로 tracker/ROS/GPU 통합 지연이 아니다.
운영 적용은 Autonav 실제 장애물 배치와 연속 주행 검토 뒤 별도 단위로 진행한다.
