# 객체 보존 우선 지면 설정 비교

L2 A2 V2는 V1 분류를 유지한 채 linefit 작업 누락 수정과 12개 단일 변수
변경을 분리한다. 운영 토픽이나 판제 입력을 교체하지 않는다. 현재 자료는
이미 분석한 개발 자료이며 독립 시험 세트가 아니다.

## 설정과 재실행

`src/asmc_lidar/config/ground_v2`의 14개 설정은 V1, linefit 누락 수정만,
Patchwork++ 6개, linefit 6개다. 각 linefit 튜닝 설정은 누락 수정도 포함하므로
수정만 적용한 결과와 V1 양쪽에 비교한다. `comparison_config`를 생략하면
V1 동작을 유지한다. 빈 경로나 `v1.yaml`은 같은 SHA를 갖는다.

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
cd /root/ws
# 기존 브릿지를 그대로 사용한다. 후보는 명시적으로 선택한다.
roslaunch asmc_lidar ground_compare_live.launch \
  comparison_config:=/root/ws/src/asmc_lidar/config/ground_v2/lf_covered.yaml

# 출력이 이미 있으면 거부한다. raw와 sync는 서로 다른 출력에 순서대로 처리.
python3 tools/lidar/ground_tuning/run.py \
  /data/lidar_assessment/20261004_gt_dual_ground/sync \
  /data/lidar_assessment/20261004_ground_v2/sync
python3 tools/lidar/ground_tuning/score.py \
  /data/lidar_assessment/20261004_gt_dual_ground/sync \
  /data/lidar_assessment/20261004_ground_v2/sync \
  /data/lidar_assessment/20261004_ground_v2/sync_metrics
```

`run.py --profiles <id> ...`는 고정 설정의 일부를 실행한다. 결과와 입력의 SHA,
실행 명령, 바이너리 SHA를 각 manifest에 남긴다. `score.py`의 정식 결과는
14개 설정 모두 있어야 생성한다. GT는 채점기에만 들어간다.

## 원인 진단

원인 비트는 원본점 인덱스로 복원한다. 여러 분기를 거친 점에는 여러 비트가
있으므로 원인별 점 수를 합하면 전체 점 수보다 클 수 있다. 분석 보고서의
`retained_reason_bits`는 최종 비지면 점에 한정한 집계다.

| Patchwork++ 비트 | 의미 |
|---:|---|
| 1 | patch 점 부족 |
| 2 | 지역 수직 평면 처리에서 비지면 |
| 4 | 최종 지역 평면 거리 초과 |
| 8 | 평면 uprightness 거부 |
| 16 | near patch heading 거부 |
| 32 | elevation와 flatness를 통과하지 못해 TGR 후보 |
| 64 | 원거리 upright patch 지면 승인 |
| 128, 256 | 각각 elevation, flatness 승인 |
| 512, 1024 | TGR 승인, 거부 |
| 2048 | 25cm 고정 높이 보호로 지면에서 비지면으로 복원 |
| 4096 | 위 분기에 미할당된 점. 범위 제외 등은 분류 마스크와 함께 확인 |

`pre_guard_mask`는 25cm 보호 이전 결과다. 보호 규칙으로 남은 경사 노면을
평면 거리 임계값 문제로 해석하지 않는다.

| linefit 비트 | 의미 |
|---:|---|
| 1, 2 | 자기 sector 또는 이웃 sector 선으로 지면 승인 |
| 4 | 자기 sector에 해당 거리의 유효 선 없음 |
| 8 | 이웃 탐색 이후에도 유효 선 없음 |
| 16 | 유효 선은 있으나 허용 거리 초과 |
| 32 | 범위 밖 등 유효 bin 미할당 |
| 64 | 분류 스레드가 방문하지 않음 |
| 128 | 해당 sector의 피팅 스레드가 방문하지 않음 |

linefit의 `fit_events`는 잔차·경사·긴 간격의 높이 차·초기 연결 간격·시작
높이 조건을 거부한 횟수다. **sector 피팅 과정의 사건 수**이며 특정 원본점을
삭제한 직접 원인이나 GT 오류 수가 아니다. 점별 유효 선 부재와 구분한다.

## 판정과 검증

raw와 sync 각각에서 10점 이상 객체의 추가 완전 소실, 클래스별 전체·하부
보존 악화, 미분류 증가를 검사한다. `object_regressions.jsonl`에는 평균에
가려질 수 있는 개별 객체 점 감소·하부 감소·추가 분절을 보존한다.

구조 진급 조건은 아스팔트·보도 각각, 0.2/0.4/0.8m 각각에서 추가 분절과
새 고립점이 모두 증가하지 않고 적어도 하나가 감소하는 것이다. 다른 도색·구조물의
결과도 보고하며, 최종 운영 채택은 이 점수로 자동 결정하지 않는다.
전체 바닥 삭제 또는 객체 삭제로 조각 수만 낮춘 결과는 객체 통과 조건으로 차단한다.

```bash
python3 tools/lidar/ground_tuning/test_score.py
python3 tools/lidar/ground_live_review/test_metrics.py
cd build/asmc_lidar && ctest --output-on-failure
```

진단은 분류 분기 변경 없이 build 파생본에 추가한다. 원본 vendor·external을
수정하지 않는다. 반복 검사는 `compare.py --diagnostics`로 마스크·점 대응·
stamp·원인·피팅 사건이 동일한지 검사한다. 실행시간은 비교에서 제외한다.
