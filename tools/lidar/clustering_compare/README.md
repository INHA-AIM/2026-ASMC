# L1-B5-A — 격리 TRAVEL AOS / 원본 점 군집화 비교

[결과·한계](../../../docs/packages/asmc_perception/lidar-l1b5a-clustering-validation-20260929.md) ·
[고정 설정](../../../config/baselines/lidar_l1b5a_clustering_20260929.yaml).
ROS publisher/subscriber가 없으며 `src/`, 운영 토픽/MD5, 센서, 판제는 변경하지 않는다.

현재 Capture의 투영 감사가 **97/105 통과, 전체 승인 실패**이므로 AOS는 모든 Capture에서
`blocked_projection_admission`이다. 통과한 97개만 골라 AOS를 실행하지 않는다.
기준선과 raw Euclidean은 105장면 모두 비교하고, AOS는 검증 가능한 합성 입력만 실행한다.
합성에서도 회귀가 있어 운영 미채택이다. `passed_preservation_and_parity`는 실험 무결성이지
검출 성능·후속 진급·배포 수락이 아니다.

## 빌드

호스트 저장소 루트에서 읽기 전용 external 사본만 컨테이너 `/tmp`로 복사한다.
기존 checkout을 수정하지 않으며 ROS2 스택을 설치하지 않는다.

```bash
git -C ../external/baselines/TRAVEL rev-parse HEAD
# 반드시 95dc2fbd66a343efd9060c45a5711b6307a950a4
docker cp ../external/baselines/TRAVEL/cpp/travel/core/travel \
  asmc-ros-noetic:/tmp/asmc-travel-reference
docker exec -it asmc-ros-noetic bash
```

이후 컨테이너 안에서 실행한다. 소스 11개와 AOS/types/logging 3개 SHA가 다르면 configure를
거부한다. GPL-3.0 checkout이며 운영 이식·배포 전에 라이선스 범위를 별도 검토한다.
상위 `getRowIdx` 두 overload의 end 경계, 인스턴스별 column resolution, 계측 접근자만
**생성된 빌드 사본**에 적용한다. 연결 거리·군집 그래프는 바꾸지 않는다.

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
source /data/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/devel/setup.bash --extend
cd /root/ws
cmake -S tools/lidar/clustering_compare -B /tmp/asmc-l1b5a-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DLEGACY_MAIN_SOURCE=/data/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/src/Main \
  -DTRAVEL_CORE_SOURCE=/tmp/asmc-travel-reference
cmake --build /tmp/asmc-l1b5a-build -j2
cmake -E chdir /tmp/asmc-l1b5a-build ctest --output-on-failure
```

## 고정 데이터 재실행

```bash
# 최초 projection audit. 현재 데이터에서는 보고서를 보존하고 exit 1로 차단한다.
PYTHONDONTWRITEBYTECODE=1 python3 -B tools/lidar/clustering_compare/projection.py \
  --config config/baselines/lidar_l1b5a_clustering_20260929.yaml \
  --admission /data/lidar_assessment/20260929/l1b4b_capture_gt/inputs_v4 \
  --output /data/lidar_assessment/20260929/<new-projection-audit>

# campaign은 이미 동결된 l1b5a_projection_v1에 묶여 있다.
# --kind: capture, synthetic(기존 580호출), extra(추가 80호출), stationary(정차 bag).
# 모든 출력 폴더는 새 이름이어야 한다. 원본 bag/Capture를 복사하거나 변경하지 않는다.
PYTHONDONTWRITEBYTECODE=1 python3 -B tools/lidar/clustering_compare/campaign.py \
  --kind capture --data-root /data --build /tmp/asmc-l1b5a-build \
  --repeat --output /data/lidar_assessment/20260929/<new-capture-campaign>

PYTHONDONTWRITEBYTECODE=1 python3 -B tools/lidar/clustering_compare/campaign.py \
  --kind synthetic --repeat --output /data/lidar_assessment/20260929/<new-synthetic-campaign>

# 추가 fixture는 이미 l1b5a_extra_fixtures_v1에 생성되어 있다.
PYTHONDONTWRITEBYTECODE=1 python3 -B tools/lidar/clustering_compare/campaign.py \
  --kind extra --repeat --output /data/lidar_assessment/20260929/<new-extra-campaign>

PYTHONDONTWRITEBYTECODE=1 python3 -B tools/lidar/clustering_compare/campaign.py \
  --kind stationary --output /data/lidar_assessment/20260929/<new-stationary-campaign>
```

`run.py`는 개별 경로도 받는다: `--snapshots --setting --params --config --executable --output`,
`--kind`, `--decision`(기본 baseline), 선택 `--reference-geometry`.
Capture는 `--source-manifest --projection-audit`가 필수이며 SHA와 전체 승인을 검사한다.
`score.py --run --truth --output --kind capture|synthetic`는 GT를 **후처리에만** 사용한다.
`fixtures.py --output <new-directory>`는 추가 광선 합성을 재생성한다. 이는 MORAI/VLP16
물리 시뮬레이션, 측정 ring/time, 실제 YOLO 결과가 아니다.

## 결과 검증·그림

```bash
# 원본 artifact SHA와 geometry/OBB/계보의 반복성을 검사한다(호스트도 가능).
python3 -B tools/lidar/clustering_compare/verify.py \
  --run /data/lidar_assessment/20260929/l1b5a_capture_final_v1/run \
  --repeat /data/lidar_assessment/20260929/l1b5a_capture_final_v1/repeat

python3 -B tools/lidar/clustering_compare/verify.py \
  --run /data/lidar_assessment/20260929/l1b5a_stationary_final_v1/run \
  --historical /data/lidar_assessment/20260928/l1b3g_geometry_v7/geometry.jsonl

python3 -B tools/lidar/clustering_compare/plot.py \
  --run /data/lidar_assessment/20260929/l1b5a_extra_final_v1/run \
  --input-id l1b5_azimuth_seam_empty_frame4 --xmin -18 --xmax -5 \
  --output /data/lidar_assessment/20260929/<new-figure>.png
```

점과 주황색 OBB는 기존 평가 좌표다. 센서 원점의 입력으로 AOS 투영 후, 원본 인덱스로
수평화된 점을 찾아 기존 L-shape에 넘긴다. 원본 점→voxel·투영 셀·pre-gate cluster·최종
cluster/OBB를 모두 보존한다. 중복 셀 점은 첫 점 이후 제외된 상태로 채점하며 강제로
군집 ID를 복원하지 않는다. 최소 관측량은 raw 점 수가 아니라 **기존 voxel 10개**다.

`projection_reasons`: 0 입력 아님/기존 제거, 1 코어 반환, 2 비유한 좌표, 3 범위 밖,
4 채널 모호, 5 셀 중복 제외, 6 코어 미할당, 7 기존 voxel 관측량 gate 탈락.
`labels`는 동결 wall-stage 마스크이므로 `labels=kept`만으로 최종 군집 포함을 판단하면 안 된다.
`target_projection_reasons`와 `cluster_id`, OBB removed flags를 함께 본다.

manifest는 입력/설정/원본 점군·generated source·실행 파일/라이브러리·도구의 SHA를 담고,
geometry JSONL은 원본 ID/관측 stamp/수평화 행렬을 보존한다. 처리 계측만 repeat digest에서
제외한다. 정차 replay·동시 평가 프로세스의 CPU 비용을 전체 시스템 latency로 해석하지 않는다.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B -m unittest discover -s tools/lidar -p 'test_*.py'
python3 -B tools/lidar/check_l1b5a_baseline.py --data-root ../data
```
