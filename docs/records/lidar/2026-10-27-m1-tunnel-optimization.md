# M1 터널 군집화 병목 분리와 같은 결과 최적화

**3뷰 터널 처리 p95를 51.637 → 8.519ms로 줄였다.** 전방 단독 대비 증가분은
최적화 후4.605ms로 같은 기록의 +5ms 기준을 통과한다. 벽/천장 제거·카메라 보호·추적·
분류·운동·HD맵 전달 판단은 비교 전후 동일하다. 전체 실시간 공급 수락이나 3뷰 기본 승격은 아니다.

기준 코드 `97d2e4ca93608933f36b31aff52d7bda69e8600d`, 구현 `9d461d3e`.
앞선 [M1 결과](2026-10-27-m1-three-view-fusion.md)의42.729ms는 당시 환경의 측정값으로 보존한다.
이번51.637ms와의 차이를 코드 회귀로 해석하지 않는다. 이번에는 같은 환경에서 전후를 교차 측정했다.

## 판제의 5개 선과 LiDAR 터널 경로는 별개

현재 `jamming_lane_boundary_adapter`의 `BuildBoundaryFamilyStripContext`는
왼쪽/중앙/오른쪽 **3개 경계**로2개 차로 strip을 만들고 각 쌍의 중점으로 중앙선2개를 계산한다.
3개 경계와2개 중앙선이라는5곡선 자료 구조이지, 새로운 경계5개를 별도 피팅하는 경로가 아니다.
`camera_bbox_lane_node`는 이 strip을 영상 좌표로 투영해 YOLO bbox의 차로를 판정한다.

이 판제 소비 노드·카메라 모델·family 생산자를 실행하지 않은 native LiDAR replay에서도
터널 지연이 재현됐다. `asmc_lidar`의 터널 처리에는 lane_family 입력이 없다.
따라서 이번 증가분의 원인은 그5곡선 계산이 아니다. 실제 동시 실행에서 CPU 경합이 생길
가능성까지 부정하는 것은 아니며 판제 처리시간 자체를 측정·개선했다고 주장하지 않는다.
판제 행동 코드와 family 경계는 이번 작업에서 변경하지 않았다.

## 확인한 병목

3뷰 터널 전처리는 이미 도착한 ±50ms 관측만 사용하며 기다리지 않는다. 관측이 없는 뷰의
시야에 있는 점은 보수적으로 남긴다. 기존 optical 해석 수정과 이 ambiguity로 조밀한 벽 점이
남아 `RemoveResidualWalls`와 `ClusterAndFilter`에서 동일 점들의 근방 탐색을 반복했다.
최종 발행 객체가 적어도 중간 군집 계산 비용은 크다. 기존422scan의 ambiguity·후단 후보
증가는 이 근거 보존 경로와 연결되며, 최적화를 위해 보호 조건을 느슨하게 만들지 않았다.

3뷰 터널, 단계별 p95의3회 중앙값(ms):

| 단계 | 전 | 후 |
|---|---:|---:|
| 카메라 보호 투영 | 0.302 | 0.289 |
| 천장 높이 제거 | 0.012 | 0.012 |
| 벽 평면 fit | 1.652 | 1.556 |
| 잔여 벽 군집화 | **22.808** | **3.322** |
| 잔여 벽 OBB | 1.720 | 1.640 |
| 최종 군집화 | **23.421** | **0.005** |
| 최종 OBB | 1.663 | 0.0001 |
| 전체 pipeline | **51.637** | **8.519** |

각 단계 p95는 같은 scan일 필요가 없으므로 더해서 전체 p95로 사용하지 않는다.
`wall_remove`에는 residual 군집·OBB가 포함된다. `tunnel.timings_ms`에서 중첩 관계를 명시하며,
기존 공개 메시지/MD5는 유지한다. 최종 OBB가 매우 짧은 것은 피팅을 없앤 것이 아니라
동일 scan에서 이미 구한 결과를 옮겨 사용했기 때문이다.

## 적용한 변경

1. **정확한 Euclidean 연결 성분 탐색.** 공간 bucket에는 아직 방문하지 않은 점만 남긴다.
   동일 float XYZ 거리·strict radius·min/max 점 수를 적용한다. seed 순서, 군집 내부 index 정렬과
   PCL의 군집 크기/동률 정렬을 유지한다. 점 downsampling·근사 거리·연결 반경 변경은 없다.
   일반 도로의 기존 adaptive clustering은 변경하지 않고 터널의 두 호출에만 적용한다.
   비정상 수치 스케일은 원래 PCL 탐색으로 처리하며, 잘못된/중복 index와 비유한 점은 거부한다.
2. **같은 scan 내부 재사용.** 잔여 벽 단계에서 점 라벨이 하나도 변하지 않고 두 단계의
   군집 조건·OBB 개수가 같을 때만 군집 index와 OBB를 move한다. 점이 제거되거나 최소 점 수가
   다르면 재계산한다. scan 시작/Reset에서 비우고 이전 프레임으로 넘기지 않는다.
   터널712scan 중643scan에서 재사용했다. 점군 사본·새 대기열을 추가하지 않았다.
3. **진단.** 보호 투영, residual 군집/OBB와 기존 core 단계 시간을 trace에 제공한다.
   시간과 재사용 여부는 진단이며 판제 입력·객체 판단에 사용하지 않는다.

OBB는 기존 L-shape 각도 탐색과 closeness score를 유지한다. 벽/천장 판정 기준, 카메라
미수신 보호,3/5/6회 확인, 추적 ID·속도·분류·HD맵 기준도 그대로다.

## 참고 근거와 적용 범위

- [PCL Euclidean Cluster Extraction](https://pointclouds.org/documentation/tutorials/cluster_extraction.html):
  공간 분할과 radius flood-fill의 연결 성분 정의를 참고했다. 실제 대조 기준은 컨테이너
  **PCL1.10/FLANN**의 float 거리·출력 정렬 구현이다. 외부 소스를 새 운영 코드에 복사하지 않았다.
- [Zhang 등, Efficient L-Shape Fitting, IV2017](https://publications.ri.cmu.edu/efficient-l-shape-fitting-for-vehicle-detection-using-laser-scanners):
  이미 사용하는 각도 탐색형 OBB의 근거다. 이번에는 다른 fitting 목적함수로 교체하지 않고
  중복 실행만 줄였다. 논문 성능을 ASMC 측정값으로 사용하지 않는다.
- [EagerMOT, ICRA2021](https://arxiv.org/abs/2104.14682)과
  [Autoware ROI–cluster fusion](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/docs/roi-cluster-fusion/):
  2D 관측 연결과3D 추적의 분리, 영상 투영 기반 연결 경계를 참고한다. 기존
  [고정 checkout·라이선스](../../../src/asmc_lidar/docs/three-view-fusion.md#참고-구현과-적용-범위)를 유지한다.
  모든 뷰를 기다리거나 BEV raster를 새로 만드는 경로는 도입하지 않았다.

## 검증과 재현

등록 run은 `lidar-m1-tunnel-opt-20261027-r1`이며 결과는
`D:\morai dataset\shahn03\artifacts\lidar-m1-tunnel-opt-20261027-r1`에 있다.
기존 F1의4개 bag SHA를 다시 검증하고 새 수집 없이 수행했다. 사용자 승인된 M1 한정
저장 예외를 기록했으며 저장 정책 파일은 바꾸지 않았다.

`tools/perception/m1/tunnel_benchmark.py`가 native binary/대응 라이브러리로
4bag × 전방/3뷰 × 전후 ×3회 = **48회** 교차 실행한다. 임의 Python matching 구현은 없다.
전체 객체·점군 digest·fusion·벽 판단을 비교하고 생성 session/설정 identity, 시간과 캐시
진단만 제외한다. **모든 반복에서 행동 필드 해시가 일치**했다.

전체 pipeline p95의3회 중앙값(ms):

| 자료 | 전방 전 | 전방 후 | 3뷰 전 | 3뷰 후 |
|---|---:|---:|---:|---:|
| 정차 SUNNY | 7.048 | 6.440 | 6.923 | 7.018 |
| 직진·가감속 | 4.859 | 4.732 | 4.465 | 4.478 |
| 터널 | 4.003 | 3.914 | 51.637 | 8.519 |
| FOGGY | 6.899 | 6.475 | 6.546 | 6.413 |

최대 RSS는 전157.38MiB/후156.93MiB다. 전체 처리 p95는 native CPU 처리시간이며
실제 수신→발행 지연·15Hz 모델 공급률이 아니다. 단기 반복 RSS를 장기 누수 검증으로 해석하지 않는다.

- CTest **40/40** 통과: 기존 ROS/수신/추적/웹 포함.
- PCL과 난수60장, 조밀한 벽, 음수 셀, 정확한 반경 경계, 크기 제한/동률 순서 대조.
- cache의 다음 scan/Reset, 최소 점 수 불일치, 실제 잔여 벽 제거 후 무효화 검사.
- [기계 판독 요약](../../../config/baselines/evidence/lidar-m1-20261027/tunnel-optimization.json).

## 독립 ROS와 웹 실행

같은 터널 bag을 각각60초, 별도 master11339/웹8099에서 재생했다. 운영 master11311과
기존 카메라·LiDAR·판제·bridge는 종료하지 않았다. 제어 토픽을 재생하지 않았다.

| 구성 | 수신/유효 scan | 최소 완전5초 창 | 최대 수신 공백 | debug capture p95 |
|---|---:|---:|---:|---:|
| 3뷰 debug OFF | 711/711 | 11.6Hz | 123.33ms | 해당 없음 |
| 3뷰 debug ON + 웹 | 712/711 | 11.6Hz | 127.86ms | 0.959ms |

debug ON의 초기 IMU 준비 전 무효1개를 숨기지 않았다. scan/debug 덮어씀0, 브라우저
JS 오류0이다. 웹에서 GPS 무효 점은 노랑이고 map은 사용 불가로 표시된다. 두 구성의
차이를 통계적 overhead 측정으로 확대하지 않는다. 기존 운영 프로세스는 계속 실행됐으므로
이 시험은 부하가 통제된 MORAI/GPU 동시 수락이 아니다. `execution-context.json`에
기존 도구 summary의 일반적인 no-contention 문구를 정정했다.

이번 터널 bag의 공급 기준은 통과했지만 이전 정차 bag의310ms 입력 공백, F1의473.47ms
실패를 없앤 것으로 보지 않는다. 입력 정상성과 실제 새3뷰 장애물 보존은 별도 검수한다.

## 적용 경계와 다음 작업

최적화 코드는 기존 전방 및3뷰 후보의 터널 경로에 공통 적용한다. 기본값은
`fusion.views: front`, `motion.semantic_static_enabled: false`다.3뷰는 내부 `all` 프로필로
실행할 수 있다. 순수 계산 증가 기준을 통과했지만 실제8종 장애물 보존·중첩 오매칭과
MORAI/GPU 동시 부하 수락이 남았으므로 이를 자동 기본 승격하지 않는다.
기존 입력 공백 및 F1의473.47ms 공급 실패를 해결했다고 표시하지 않는다.

다음 우선 작업은 **3뷰에서 보존한 실제 물체와 벽 오보호의 구분·연속 공급 검수**다.
단순 속도를 위해 미도착 카메라를 정상으로 간주하거나 ±50ms/freshness를 넓히지 않는다.
재호의 벽/천장 품질·OBB 위치·추적/속도 개선은 이 동등한 군집 결과 위에서 이어간다.
