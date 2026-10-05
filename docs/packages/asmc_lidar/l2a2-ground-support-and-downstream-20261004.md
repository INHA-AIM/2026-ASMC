# L2-A2 지면 지지 후보와 후단 군집·OBB 재검증 (2026-10-04)

결론: **Patchwork++ 고정 높이 지면분할을 운영에 채택하지 않는다.** 지면점의 국소
지지 근거를 요구한 첫 후보는 차량 구제에 실패했다. 단순 높이 veto는 같은 입력에서
차량 보존과 후단 객체 후보 수를 개선했지만, 재사용한 GT 자료에 대한 탐색이며
보도·도로 잔여점과 새 환경 검증이 남았다. 재호 기준선도 여전히 보존한다.

## 고정 입력과 두 평가

원본·동기 v3 GT 수집본의 동일한 ROI, IMU pitch 수평화, 10cm voxel 계보와
`sensor_height=1.80m` Patchwork++ v1.4.1 (`3e6903a1d5`) 마스크를 사용했다.
raw 4,048프레임과 sync 1,439프레임 전체에서 지면점·객체점·GT instance
완전 소실을 채점했다. 다음 네 방법의 점 표현은 모두 동일한 voxel 출력이며,
재호의 지면+수직 평면 제거 결과를 기준선으로 유지했다.

1. 재호 기준선, 변경 없는 Patchwork++, 고정 `z_level > -1.45m` veto.
2. 신규 지역 지지 후보: Patchwork++가 지면으로 분류한 높은 점을 되살리되,
   반경 2m 안에 `z_level∈[-2.2,-1.55]m`의 낮은 Patchwork++ 지면점 3개와
   35cm 이상의 높이 차이가 있어야 한다. GT는 이 판단에 쓰지 않았다.

후단 영향은 라벨을 보지 않고 raw의 매 20번째 프레임(203개), sync의 매
10번째 프레임(144개)을 골랐다. 각 마스크의 잔존 voxel에 **기존 일반도로의
1.5m Euclidean 연결, 10~10,000 voxel cluster 조건, 1도 간격 L-shape OBB**를
동일하게 적용했다. GT instance마다 한 cluster에 관측 객체점 80% 이상이 있고
그 cluster 원본점의 90% 이상이 그 객체일 때 `80/90`으로 집계했다. 이는
검출기·tracker·판제 출력의 정확도가 아니다. 특히 기존 Main의 객체 제거·분류
후단과 동일한 운영 ROS 실행이라고 부르지 않는다.

### 평가 1: 지면 단계

표의 %는 공통 ROI의 해당 GT class 원점 중 지면으로 분류·제거된 비율이다.
차량·보행자·장애물은 **낮을수록**, 아스팔트는 **높을수록** 일반적으로 좋지만
보도점 제거 여부는 주행 관련성·경계 인식 정책에 따라 따로 판단해야 한다.

| 입력 | 방법 | 아스팔트 | 차량 | 보행자 | 장애물 | 보도 | 차량 완전 소실 |
|---|---|---:|---:|---:|---:|---:|---:|
| raw | 재호 | 100.00% | 4.87% | 8.50% | 13.68% | ≈100% | 2/249 |
| raw | Patchwork++ | 95.51% | 26.52% | 3.17% | 11.28% | 61.23% | 30/249 |
| raw | 고정 높이 veto | 95.51% | 2.72% | 2.91% | 7.82% | 61.23% | 2/249 |
| raw | 지역 지지 | 95.51% | 25.61% | 3.09% | 10.98% | 61.23% | 30/249 |
| sync | 재호 | 100.00% | 6.16% | 18.02% | 29.45% | ≈100% | 0/1,027 |
| sync | Patchwork++ | 95.93% | 18.46% | 7.63% | 17.53% | 61.14% | 13/1,027 |
| sync | 고정 높이 veto | 95.93% | 2.52% | 7.39% | 14.70% | 61.08% | 0/1,027 |
| sync | 지역 지지 | 95.93% | 17.45% | 7.45% | 16.65% | 61.14% | 13/1,027 |

즉 지역 지지 조건에서는 구조상 근처 낮은 지면점이 확인되지 않는 높은 차량점이
대부분 구제되지 않았다. 가림, 원거리 점 희소성, Patchwork++가 낮은 노면을
지면으로 먼저 잡지 못하는 현상 중 무엇이 지배적인지는 점별 사례 검토가 더
필요하다. 이 실패를 threshold만 완화해 해결됐다고 선언하지 않는다.

### 평가 2: 동일 후단 군집·OBB

`80/90`을 충족한 관측 수다. 반복 프레임 관측은 독립 객체 수가 아니다.

| 입력 | 방법 | 차량 | 보행자 | 장애물 | 전체 cluster 수 |
|---|---|---:|---:|---:|---:|
| raw 203프레임 | 재호 | 9/13 | 15/22 | 12/27 | 1,051 |
| raw 203프레임 | Patchwork++ | 4/13 | 15/22 | 12/27 | 2,156 |
| raw 203프레임 | 고정 높이 veto | 10/13 | 15/22 | 14/27 | 2,196 |
| raw 203프레임 | 지역 지지 | 4/13 | 15/22 | 12/27 | 2,157 |
| sync 144프레임 | 재호 | 78/104 | 25/46 | 23/87 | 965 |
| sync 144프레임 | Patchwork++ | 33/104 | 31/46 | 28/87 | 1,786 |
| sync 144프레임 | 고정 높이 veto | 86/104 | 31/46 | 36/87 | 1,825 |
| sync 144프레임 | 지역 지지 | 36/104 | 31/46 | 30/87 | 1,789 |

고정 높이 veto로 차량 cluster 완전성·순도가 회복되어도 비지면 후보 cluster가
재호 대비 거의 두 배다. GT class가 불완전하므로 이 모두를 오검출이라고 할 수는
없다. 보도점 잔여와 큰 OBB 후보는 후속 판제 부담 위험이다. `obb_examples`에는
각 후보의 L-shape 길이·폭·높이·yaw가 있고, 검증된 GT 3D box가 없어 OBB
위치/크기 오차는 점수화하지 않았다.

첫 동기 프레임의 Python 재생과 실제 PCL 코어 모두 `ROI 7196, voxel 4872,
kept 164, cluster 8, 크기 [44,32,18,16,16,16,11,10]`으로 같았다. 차량이 있는
`sync_001160`도 재호는 12개, Patchwork++는 15개 cluster의 **정렬된 크기 전체**가
PCL과 일치했다. 이는 두 프레임의 군집 parity이지 모든 프레임의 bitwise Main
parity는 아니다. 동기 전체 재실행에서 클래스·객체·cluster·OBB 산출물은 동일했다.

## 해석과 다음 수락 경계

- 지역 지지 첫 후보는 **미채택**. 같은 자료의 높이 veto는 원인 분리를 위한
  연구 기준이며 운영 default로 올리지 않는다. 새 홀드아웃에서 차량 소실 0,
  저상 장애물 보존, 아스팔트 잔여·보도 clutter, 후단 객체 후보 품질을 함께 봐야 한다.
- 두 수집본은 이미 지면 탐색에 사용한 자료다. 동기 수집에서 이전에 제외된
  92프레임에는 actor GT가 없어서 도로점 회귀에는 쓸 수 있지만 차량 수락자료는
  아니다. 이 실험에서 별도로 재처리하지 않았다.
- 이 평가는 일반도로 지면·군집 경로에 한정된다. 터널 벽/천장, YOLO 보호,
  tracking, GPS 전환, map 차로 귀속을 검증하지 않았다.
- Python의 scipy 군집 시간은 ROS/PCL 처리시간이 아니다. 기존 실제 Main
  raw p50 1.861ms·sync p50 2.313ms와 Patchwork++ **분할만**의 시간을
  직접 비교하지 않는다. 후보 운영 부하는 동등한 C++ pipeline에서 재계측한다.

재실행: ROS 컨테이너에 `tools/lidar/ground_support_review/evaluate.py`를
`/tmp/asmc_ground_support_evaluate.py`로 복사한 후 아래 명령을 사용한다.
입력은 `$ASMC_DATA/lidar_assessment/20261004_gt_dual_ground/{raw,sync}`이고
출력 파일이 이미 있으면 다른 이름을 지정한다.

```bash
python3 /tmp/asmc_ground_support_evaluate.py INPUT_DIR OUTPUT.json --cluster-stride 20  # raw
python3 /tmp/asmc_ground_support_evaluate.py INPUT_DIR OUTPUT.json --cluster-stride 10  # sync
```

기계 판독 원본은 `$ASMC_DATA/lidar_assessment/20261004_gt_dual_ground/reports/
{raw,sync}_support_coupled_v1.json`에 있다. 코드 SHA-256은
`43346b9f45dfb4affb8f176cfb7d261ac32fcecb31e1e4f0c8f1f157f3043c72`.
운영 LiDAR·메시지·판제·센서 설정은 수정하지 않았다.

## 이전 external 지면 제거 후보

| checkout | 고정 SHA | 현재 위치/판단 |
|---|---|---|
| [Patchwork++](https://github.com/url-kaist/patchwork-plusplus) | `3e6903a1d5` (v1.4.1) | 현재 검사 대상. 2022 논문 기법 자체와 2026 checkout 버전의 코드를 구분한다. |
| [linefit](https://github.com/lorenwel/linefit_ground_segmentation) | `e6b9418cc5` | 가벼운 polar line fitting CPU 대조군. 아직 동일 GT 실행 전. |
| [TRAVEL](https://github.com/url-kaist/TRAVEL) | `95dc2fbd66` | graph 기반 지면 분할과 AOS 객체 군집이 분리돼 있다. AOS 선행 실험만 있고 지면 부분 동일 GT 비교 전. |
| [GroundGrid](https://github.com/dcmlr/groundgrid) | `7d6a5d7dff` | odometry/map TF 의존성이 있어 GPS 음영 전제를 먼저 검토해야 한다. |
| [DipG-Seg](https://github.com/EEPT-LAB/DipG-Seg) | `182e348430` | 16-beam 지원을 표방하지만 우리 VLP16의 ring/투영 계약 검증 전. |

이들은 외부 checkout으로 **클론·주요 입력 조건 검토**와 **우리 GT 성능 검증**을
구분해야 한다. Patchwork++만 이번 자료로 비교 완료했다. 논문의 발표 연도,
저자 국적, 타 데이터셋 SOTA 숫자만으로 우리 15Hz VLP16·GT·후단 객체 품질의
우열을 정하지 않는다. 다음 비교는 같은 ROI/voxel/자세/GT에서 linefit을
CPU 기준으로 먼저 추가하고, TRAVEL 지면 부분을 별도 후보로 붙이는 순서가
현실적이다. 라이선스는 core 이식 전에 각각 재확인한다.
