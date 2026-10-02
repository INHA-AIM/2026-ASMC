# Patchwork++ 높이 갱신 격리 평가

[H1 결과와 한계](../../../docs/packages/asmc_lidar/l2a2-h1-height-review-20261003.md)를 먼저
읽는다. 높이 안정화는 확인했지만 원거리 차량 소실이 남아 **운영 미채택**이다.
이 도구는 ROS 노드·기본값·토픽을 바꾸지 않으며 bridge·YOLO·시뮬레이터가 필요 없다.

## 구성과 비교 의미

- `prepare.py`: bag 원본 SHA를 확인하고 과거와 같은 644/129 입력을 재추출한다.
  기존 선택의 보수성을 유지하며 Capture62는 IMU 없는 과거 자료라고 표시한다.
- `audit_inputs.py`: 파일 SHA·IMU 인과성·첫 ring의 아스팔트 GT 관측량을 확인한다.
- `generate.py`: SHA가 일치하는 vendor에서 빌드 폴더에만 파생 코어를 만든다.
  vendor/external은 수정하지 않고 BSD 라이선스를 함께 복사한다.
- `height_review`: original, singleton, fixed, jaeho를 한 실행당 하나씩 평가한다.
  fixed는 높이만 1.80m로 유지하고 지역별 threshold 학습은 유지한다.
- `run.py`: raw/voxel과 pitch/full-RP를 분리하고 각 조합을 두 번 재생한다.
- `score.py`: GT 객체점 보존과 무라벨 bag의 기하 proxy를 별도 채점하고 그림을 만든다.
- `test.cpp`, `test_replay.py`: 빈/단일 표본, 합성 기하와 잘못된 입력을 검사한다.

`height_review INPUT raw|voxel pitch|full POLICY trace|quiet OUTPUT`가 단독 실행 인터페이스다.
재호는 pitch만 허용한다. 공통 ROI는 기존 평가와 같고 범위 1~60m, voxel10cm, RNR OFF다.
기존 명령 형태의 호환용 `none`도 평가기에서 인식하지만 H1 실험에는 사용하지 않았다.
출력 파일이 있으면 중단하며 기존 결과를 덮어쓰지 않는다. `mask`는 원본 점 순서대로
0=ROI 밖/미분류, 1=ground, 2=retained, 3=재호의 수직 평면 제거다.
PP patch의 `ring`은 zone 내 번호가 아닌 전체 concentric index이고,
`plane_variance_m2`는 PCA 최소 고유값이다. 실측 지면 높이·GT 잔차가 아니다.
`history_count`는 누적 갱신 history 수이며 현재 프레임 표본 수와 다르다.
재호 실행의 height 필드는 공통 runner의 미사용 PP 초기값이므로 의미가 없고 summary에서는 null이다.

## 재실행

호스트에서 이 독립 Git worktree 루트로 이동한 뒤 실행한다. 동일 image ID는
`sha256:1ecd9f50491565cad32e32980bb5df5b48487f470440cb66c9ec3b8dc5c11400`였다.
다른 이미지라면 먼저 원본 parity부터 확인한다. CPU 전용이며 GPU와 host networking을 쓰지 않는다.

```bash
ASMC="$(git rev-parse --show-toplevel)"
ASMC_DATA="${ASMC_DATA:-$(dirname "$ASMC")/data}"
H1_OUT="$(mktemp -d "$ASMC_DATA/lidar_assessment/h1-replay-XXXXXX")"
docker run --rm --network none --user "$(id -u):$(id -g)" \
  --entrypoint bash \
  -v "$ASMC:/work:ro" -v "$ASMC_DATA:/data:ro" -v "$H1_OUT:/out:rw" \
  asmc/ros-noetic:dev -lc '
    set -e
    source /opt/ros/noetic/setup.bash
    cmake -S /work/tools/lidar/height_review -B /out/build -DCMAKE_BUILD_TYPE=Release
    cmake --build /out/build -j2
    cmake -E chdir /out/build ctest --output-on-failure
    python3 /work/tools/lidar/height_review/prepare.py --data /data --output /out/inputs
    cmp /out/inputs/manifest.json /data/lidar_assessment/20261003_l2a2_h1/inputs/manifest.json
    python3 /work/tools/lidar/height_review/audit_inputs.py /out/inputs /out/input_audit.json
    python3 /work/tools/lidar/height_review/test_replay.py /out/build/height_review /out/synthetic.json
    python3 /work/tools/lidar/height_review/run.py --binary /out/build/height_review --inputs /out/inputs --output /out/results
    python3 /work/tools/lidar/height_review/score.py /out/inputs /out/results /out/report_v1
  '
```

이 명령은 data의 기존 20261003 IMU 평가 extractor, 20261002 일반도로 selection,
20260926 Capture stage, H1의 영구 oracle을 사용한다. bag 두 개를 복사하지 않고
선택 snapshot만 재생성한다. `/tmp`의 옛 실험 폴더는 더 이상 필요하지 않다.
최초 보존 때만 사용한 `--legacy`는 일반 재실행에서 주지 않는다.

## 결과 해석과 회귀

`results/runs.json`은 실행 인자·binary SHA·반복 일치를 기록한다. `report_v1/summary.json`은
원본 마스크 parity, 클래스/거리/낮은 높이대, 객체별 소실, 자세 민감도와 시간을 담는다.
분류가 늦게 끝나도 source stamp를 바꾸지 않는다. trace OFF 결과는 trace ON과 분류가 같고
별도 timing으로 보고한다. 재호의 segment 시간에는 기존 군집·OBB가 포함되어 있어
PP 분할 시간과 직접적인 속도 우열 비교를 하면 안 된다.

보존된 정본을 다시 계산하지 않고 검증하려면 다음을 실행한다.

```bash
python3 -B tools/lidar/height_review/verify.py --repo-root . --data-root "$ASMC_DATA"
```

도구 소스·입력·결과 SHA와 반복 일치, 미채택 gate를 검사한다. 새 재실행의 처리시간은
비결정적이므로 정본 결과 파일의 전체 SHA 대신 `run.py`의 시간 제외 비교로 판단한다.

원본 ROS 패키지의 회귀는 별도 기존 `asmc_lidar_core_test`, `asmc_lidar_frame_contract_test`,
`asmc_lidar_imu_selection_test`, B1 frame_time 시험, `tools/lidar/l2a1/verify.py`와
`tools/lidar/l2a2/check_manifest.py`를 사용했다. 운영 `src/`에 이 파생 코어를 연결하지 않는다.
