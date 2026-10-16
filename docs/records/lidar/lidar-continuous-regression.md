# PR #72 — 연속 지면·군집·추적 회귀와 재호 인계

> 2026-10-16 현재 실행은 [카메라→Main/LiDAR 인계](../../handoffs/camera-to-lidar.md)를 따른다. 아래 날짜별 실험 명령은 당시 checkout 재현용이며, 현재 Main에서는 start_yolo 옵션이 제거됐다. 원본 수치·설정·실패 증거는 보존한다.

시작점은 [LiDAR 인계 README](../../handoffs/lidar-migration.md)다. 이 문서는
공통 좌표·시간 변경과 roll 추가를 실제 처리 함수로 분리 검증하는 근거를 연결한다.
공통 경계와 roll 기본값은 각각 OFF이며, 속도·동정적 validity는 false다.

## 현재 구현과 확인한 문제

| 항목 | 이번 변경 | 검증 범위 |
|---|---|---|
| 관측 시간 | 원본 ROS source ns, callback ROS receipt, steady receipt/processing 분리 | 같은 ms 안의 다른 source stamp, 미래 입력·늦은 GPS 배제 |
| GPS/map | strict 100ms 보간 유지, bounded 300ms 보정은 별도 API | 무효 barrier·정상 3회 복구·receipt 이전 표본만 선택 |
| 지면 실패 | common 경로는 근거 없는 이전 평면을 재사용하지 않고 유한 점을 보존 | 합성 unknown 24점의 삭제/보존 정책 차이 재현; 실제 장애물 소실의 정답은 아님 |
| 연속 처리 | 실제 ROI/voxel·지면·군집·OBB·CA/Hungarian·fusion 함수 재사용 | 이전 상태를 이어가는 고정 입력 CPU 대조; 실시간 부하 시험과 구분 |
| 추적 진단 | source dt·할당 ID/비용·OBB·miss·KF 초기화/상태·legacy 속도 기록 | 속도 정확도나 객체 정답으로 사용하지 않음 |
| 점 대응 | voxel index, 원 센서 centroid, 수평화 centroid, 제거/보존 기록 | raw beam 대응·scan 내부 deskew를 증명하지 않음 |

`ground_fallback_frames`와 기존 임계값은 변경하지 않았다. opt-out legacy는 기존
평면 fallback을 유지한다. 공통 경로는 이동·자세에 따른 평면 운반 근거가 없으므로
현재 scan fit이 실패하면 unknown으로 보존한다. 이로 인해 남는 지면/군집도 실패
분석에 포함하며, 단순히 점 보존이 늘었다고 검출 성능 개선으로 해석하지 않는다.

## 연결 코드와 재현 도구

- `Main/src/Geometry/FrameBoundary.*`: 관측별 행렬·원본 stamp·dual clock context.
- `Main/src/Parser/SensorPoseHistory.*`: ns local 선택, 실제 receipt 기반 bounded map.
- `Main/src/Lidar/ImuRansac/Ransac.cpp`: 현재 지면 fit 실패의 common fallback 제한.
- `Main/src/Lidar/LidarProcess/PipelineTrace.hpp`: trace ON에서만 점/상태 직렬화.
- `Main/src/Lidar/Tracking/Tracking.cpp`: 기존 Hungarian 할당을 관측 기록으로 남김.
- `Main/test/lidar_continuous_replay.cpp`: 실제 설정·함수·이전 상태를 사용하는 CPU 도구.
- `tools/perception/run_lidar_continuous_matrix.py`, `analyze_lidar_continuous_matrix.py`:
  반복/GPS 대조와 종료·누락 검증. 원본과 모든 실패 시도는 보존한다.
- `tools/perception/make_lidar_motion_fixture.py`: 검출 결과와 독립된 합성 형상·속도 truth.

CPU 도구는 별도 master에서 bag의 기록 순서를 논리적 도착 시각으로 고정한다. ROS
worker·UDP·차량 제어를 시작하지 않는다. 이는 실제 callback 스케줄이나 처리율 측정이
아니다. queue wait 판정에는 고정된 decision clock을 전달하고 운영 호출은 실제 wall clock을
유지한다. 원본 YOLO/JPEG 가용 순서도 고정한다. 동일 raw의 반복과 GPS 변경 대조를
실시간 재생의 서로 다른 처리 scan/입력 선택과 혼동하지 않는다.

trace OFF에서는 점 대응 set·추가 점군 복사·파일 기록을 만들지 않는다. trace ON의
큰 점군 기록은 성능 수락에 사용하지 않는다. 저장량도 관측 비용이므로 raw 증거는
검증한 압축본으로 보존한다.

## 결과와 수락 상태

최종 결과는 [연속 회귀 baseline](../../../config/baselines/perception_lidar_continuous_regression_v1.yaml)에
실행별로 기록한다. 기존 78회 결과와 실패는
[이전 경계 baseline](../../../config/baselines/perception_lidar_camera_frame_boundaries_v1.yaml)에 그대로 남긴다.

- 지면 실패 fixture 수정 전: ROS 시험 13개 중 1개 실패. 합성 unknown 24점이 이전 평면으로 삭제됐다. 이 점은 이전 지면 높이에 놓인 일직선 입력이며 실제 장애물 정답이 아니다.
- 수정 후: ROS 14개 통과. **별도** 실제 PCL 지면 시험에서 roll 0/±5/±10° × pitch 0/±3°의 15조건을 검사해 지면 256점 제거와 높은 점 24개 보존을 확인했다. 이 별도 elevated-point 정답을 지면 fit 실패 사례에 적용하지 않는다. 전체 장애물 보존 수락과도 다르다.
- 첫 CPU 시도는 tmpfs 용량 부족으로 중단됐다. 해당 실패 로그와 점군 trace를 보존하고
  별도 디스크 및 압축을 사용하는 후속 실행과 구분한다. 일부 출력만으로 완료 판정하지 않는다.
- 실제 bag에는 객체/속도 독립 GT가 없다. 합성 truth의 정확도와 실제 bag 안정성·미대응
  관측을 별도 보고하며 신규 소실/잘림/합침/오연결을 자동 면제하지 않는다.
- 회귀 통과 전 기본값을 승격하지 않는다. 기존 분류 차이 2건·facade 실패의 승인은
  이번 좌표/roll/추적 변경의 신규 실패에 적용하지 않는다.

## 세 구성 비교 결과 — 기본 적용 보류

CPU 전체 bag 재현 20회에서 반복 6쌍, GPS 무효/중단 대조 8쌍의 local 형상·ID·OBB·
분류 카운터·KF 상태가 정확히 같았다. 같은 입력의 결과를 재사용하는 시험이며 실제
callback 도착 순서를 같게 만드는 성능 시험은 아니다.

| 비교 | 주 bag | 음영 bag |
|---|---|---|
| legacy → common pitch 대응 분류 차이 | 0 | 0 |
| common pitch → roll 대응 분류 차이 | 11 | 0 |
| pitch → roll CA 속도 변화 >2m/s | 657 → 599 | 527 → 555 |
| pitch → roll 미대응 관측(기준/후보) | 946 / 932 | 351 / 358 |

공간 대응은 같은 source stamp에서 상호 유일 최근접 중심 0.25m 이내로 제한했다.
분류 11건은 신규 차이이며 기존 승인 2건으로 면제하지 않는다. 속도 변화에는 실제 운동,
군집 변화와 KF 초기화가 섞여 있다. 초기화 카운터가 2에 고정돼 있는 객체를 모두 초기화
오류로 분류하던 첫 분석 표시는 수정 분석과 구분해 보존했다. 원인을 확정한 GT 판정이 아니다.
legacy 첫 주 scan은 처리 준비가 없어 대응 행렬도 없었으며, 미대응 목록에 남겼다.

정상 GPS 전체 bag 18회(2 bag × 3구성 × 3반복)는 모든 구성에서 주 bag matched 646,
음영 bag 207이었다. 주 bag은 dropped 1·종료 pending LiDAR 1/YOLO 1, 음영은 dropped 0·
pending LiDAR 1/YOLO 2까지 포함한다. 각 구간 offset p95의 최댓값은 30.7ms다.
이는 전체를 합친 p95라고 표현하지 않는다. typed stamp 중복·역행은 0이고 단일 frozen
YOLO 발행자를 확인했다. 새 GPU 추론을 실행한 카메라 전체 성능 증거는 아니다.

공통 local 관측의 observer 수신→출력 p95는 주 bag 약 44.9–45.5ms, 음영 약 46.9–48.4ms다.
최초 report의 latency 표본 0은 ms/ns 비교 오류였다. 원본 report·observations를 유지하고
원본 ns로 분석을 수정했다. 이 지연은 Main의 실제 callback receipt와 별개다.

합성 80프레임은 첫 입력 동시 도착의 준비 실패 1개를 포함해 79프레임을 처리했다.
미대응은 legacy/pitch 99개, roll 106개였다. 각각 낮은 물체 70/70/75, 보행자 27/27/28,
차량 1/1/1, 벽 1/1/2다. 낮은 물체는 노면에 붙은 30cm 형상으로 기존 지면 삭제 대역을
검사하며, 통과시키려고 위치나 임계값을 바꾸지 않았다. 중심 대응 실패를 모두 완전 소실로
단정하지 않고 부분 관측·군집 합침을 검토할 후보로 남긴다.

직진·자세 변화 없는 첫 16프레임의 독립 속도 truth에서 legacy scalar 속도의 오차 p95는
약 0.0051m/s, common pitch는 0.0026m/s였다. roll 효과를 검증하는 구간은 아니며,
후반 yaw/roll 입력은 완전한 자차 운동 GT가 아닌 자세 스트레스 시험이다. 실제 bag 속도
정확도나 일반 회전 보상의 근거로 확대하지 않는다.

사전 선정한 6개 raw scan의 점 대응·그림도 보존했다. shadow case-01은 유지점이
7037→7031개인데 실측 객체 후보 수가 23→13으로 바뀌었다. 이는 군집/연관 민감성을
추적할 사례이며 실제 차량 10대 소실의 정답이 아니다. 독립 객체/지면 라벨이 없으므로
신규 소실·잘림·합침·오연결 수락은 보류한다. 지면 fit은 두 bag 모두 유효했지만 그 사실만으로
올바른 지면을 골랐다고 판단하지 않는다.

## 증거 위치와 재현

[portable 증거](../../../config/baselines/evidence/lidar_continuous_regression_v1/README.md)에
20회 전체 상태·18회 원본 관측·합성 truth/실패·고정 scan과 SHA 목록을 연결했다.
큰 전체 point trace는 호스트 `artifacts/lidar-pr72`에 보존하고 파일별 SHA를 저장했다.
실행 CLI의 잘못된 `73-working-v3` 라벨은 지우지 않고 runtime 08dbb79의 124개 소스·설정
SHA 대조로 정정했다. 분석 코드·runtime 코드와 trace ON/OFF 범위를 구분한다.

바깥 workspace를 빌드/source한 뒤 nested workspace를 빌드한다. 공통 경로를 직접
확인할 때는 다음 launch를 명시적으로 실행한다. 기존 shared 컨테이너의 옛 devel을
source하는 것만으로 새 바이너리가 설치되는 것은 아니다.

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/lidar+sensorfusion/devel/setup.bash
roslaunch Main main.launch start_yolo:=false rviz:=false \
  enable_common_geometry:=true lidar_level_roll:=true
```

이 명령은 미수락 후보를 시험하는 opt-in이다. 카메라 생산자와 GUI는 별도 실행한다.
roll을 분리 비교하려면 `lidar_level_roll:=false`, 기존 경로는 `enable_common_geometry:=false`다.
카메라 모델·보넷 mask·클래스·confidence·메시지 MD5와 jamming 21% crop은 이번에 변경하지 않았다.

## 사용자 직접 검토 — 24점·신규 분류·속도 변화

[비교 갤러리](../../../config/baselines/evidence/lidar_continuous_regression_v1/user-review/index.html)는
브라우저로 열면 24점 그림, 11개 관측의 원본 JPEG·확대·점군/OBB, 세 구간 전후 연속
영상을 제공한다. GitHub에서는 HTML을 렌더링하지 않으므로 로컬 checkout의 파일을 연다.
자료 생성은 오프라인이며 공유 bridge/master/GUI·운영 설정을 바꾸지 않았다.

**24점의 의미를 정정한다.** 지면 fit 실패 fixture의 점은 `x=8..8.46m, y=0.2m,
z=-1.5m`에 놓인 합성 일직선 입력이다. 이전 64점 평면과 높이가 같고, 현재 seed가
최소 30개에 못 미친다. roll 0→10°는 context이며 이 24점에 `CorrectingPoints`를
호출하지 않는다. 실제 장애물이나 raw scan을 자세 변화로 변환한 정답 시험이 아니다.
실제 `NormalVectorFiltering`을 별도 프로세스에서 실행해 동일 입력이 이전 fallback에서는
0점, common에서는 24점 남는 것을 확인했다. 이전 정책은 변경 전 common과 같은
평면 재사용 동작을 유지하는 opt-out 분기로 재현했다. 지면일 수도 있어 단순 삭제를
장애물 손실 결함으로 판정할 수 없다. 현재 처리는 근거 부족 시 unknown 보존 정책이며,
남은 지면의 오검출/연산 비용은 재호의 후속 검토 대상이다. 두 실제 bag에서는 모든 처리
관측의 현재 plane fit이 유효했으므로 이 fallback 사례가 실제 bag의 차이를 설명하지 않는다.

분류 11관측은 10개 `car→unknown`, 1개 `unknown→car`다. 같은 객체의 연속 관측을
포함하므로 차량 11대 소실로 계산하지 않는다. 각 그림은 발행에 전달한 실제 post-fusion
상태와 해당 관측의 수평화 역행렬을 사용한다. raw voxel centroid에는 역변환을 다시
적용하지 않는다. 원본 JPEG는 가장 가까운 source stamp(50ms 이내)이며 실제 YOLO
매칭 stamp와 같다고 가정하지 않는다. YOLO box를 다른 시점 영상에 겹치지 않았다.
전방 밖의 10번 관측은 같은 시점 우측 원본 영상과 고정 camera snapshot의 투영을 함께
제공한다. 분류 카운터·이력이 다른 상태에서 실측 OBB는 남아 있다는 점과 객체 자체
보존을 구분해 검토한다. 영상 대응도 독립 객체 GT를 대신하지 않는다.

속도 지표는 같은 ID의 내부 CA 상대속도 벡터 갱신량 `|v_now-v_previous|>2m/s`다.
실제 차량 속도·가속도·속도 오차가 아니며, 초기화·군집·연관·자세·실제 운동이 섞인다.
아래 분위수는 **2m/s 초과로 센 이벤트들의 변화량**이다.

| bag / 구성 | 횟수 | 변화량 p50 (m/s) | p95 (m/s) | 최대 (m/s) |
|---|---:|---:|---:|---:|
| 주 / legacy | 668 | 4.72 | 24.80 | 36.42 |
| 주 / common pitch | 657 | 4.32 | 24.74 | 45.75 |
| 주 / common roll+pitch | 599 | 4.32 | 24.60 | 36.20 |
| 음영 / legacy | 557 | 4.32 | 20.83 | 31.26 |
| 음영 / common pitch | 527 | 3.77 | 15.10 | 27.02 |
| 음영 / common roll+pitch | 555 | 3.51 | 15.64 | 27.01 |

roll만 분리하면 주 bag 횟수는 58회(-8.83%), 최대는 9.55m/s(-20.88%) 감소했지만
p95는 약 0.57%만 감소했다. 음영은 28회(+5.31%) 증가, p50은 약 6.98% 감소,
p95는 약 3.58% 증가했고 최대는 거의 같다. legacy→roll은 좌표 변경 효과까지 섞여
주 668→599(-10.33%), 음영 557→555(-0.36%)다. 주 bag 자체도 터널을 포함하므로
이 집계를 순수 일반도로/음영 외 성능으로 확대하지 않는다. 초기화 후보가 주 bag
281→282개로 거의 같은데 전체 급변은 감소했다. 원인 확정이나 속도 정확도 근거는 아니다.

사용자는 **주 bag에서 감소한 조건에서 음영 급변 증가를 허용**했다. 이 승인만 기록하고
신규 분류·객체 보존·map 품질·공개 속도 validity는 자동 수락하지 않는다. PR은 Draft,
공통/roll 기본값은 OFF로 유지한다. 다음 카메라 작업은 이 검토 이후 별도 단계다.

재현 도구는 `export_ground_failure_review.py/.cpp`(기존 실제 Main test 라이브러리
링크, 전용 master 11314), `render_lidar_user_review.py`(기록된 행렬/점과 원본 이미지)다.
중간 렌더링 실패와 보완 전 갤러리는 호스트 `artifacts/lidar-pr72`에 보존했다.
최종 갤러리는 11관측 전체, 전방 밖 사례, ns 문자열 표시와 파일 SHA를 검사했다.

### 사용자 영상 검토 결과와 후속 담당

사용자가 갤러리의 원본 영상·OBB·연속 영상 세 구간을 직접 검토했다.
아래는 영상 판독 의견이며 독립 GT 기반 정량 제거율/recall로 계산하지 않는다.

| 검토 대상 | 사용자 관찰 | 후속 작업 |
|---|---|---|
| 합성 unknown 24점 | 지면인지 사진으로 확인하려 했으나 대응 사진이 없음 | 좌표에서 만든 fixture임을 유지. 실제 지면 fit 실패의 객체 보존/오검출은 별도 실제 입력으로 평가 |
| 속도 튐 | 변화량까지 보면 종합적으로 개선된 것으로 판단 | 기존 음영 증가 허용과 함께 안정성 평가로 기록. 공개 속도 정확도/validity는 별도 |
| 신규 분류 차이 | 카메라 YOLO 보완 과제로 진행할 수 있다고 판단 | 다음 카메라 단계에서 클래스 범위·라벨 안정성과 fusion 확인 이력을 함께 분석 |
| 분류 10번 | 차량을 제대로 표현하지 못하는 작은 OBB | LiDAR raw→제거 후 점→군집 지지점→OBB와 연관/분류 이력 추적. 차의 부분 관측인지 구조물 점의 잘못된 연결인지 구분 |
| 연속 구간 1·2·3 | 벽을 조각난 객체 OBB로 잡는 문제가 공통으로 보임 | 재호의 구조물 경계/객체 분리·연결성·군집 과제에 고정 사례로 사용 |
| 바닥 OBB 노이즈 | 구간 1은 pitch-only가 낫고 구간 2·3은 roll+pitch가 나음 | 구간별 평가를 유지. roll이 모든 프레임의 노이즈를 줄인다고 일반화하지 않음 |

YOLO는 원본 라벨·검출 범위·분류 안정성을 보완한다. 차량 3D 형상과 벽 조각 OBB는
LiDAR의 점 분리·군집·형상/연관 개선 과제다. 분류 차이가 YOLO 모델만의 원인이라고
확정한 것은 아니므로 관측별 투영·IoU·확인 카운터도 함께 확인한다. 부분 관측에 보이지
않는 차량 전체 치수를 임의로 채우거나 작은 OBB를 일괄 삭제하지 않는다.

24점은 bag에서 선택한 점이 아니라 좌표로 생성했으므로 사진이 존재하지 않는다.
실제 bag의 사진을 그 합성점의 정답처럼 연결하지 않는다. 현재 두 bag의 지면 fit은 모두
유효했으며 실제 fit 실패 시 객체 손실을 증명한 이미지 사례는 이번 자료에 없다.
이번 기록은 사용자 시각 검토를 마무리하며 운영 코드·기본값은 유지한다.

## 최종 회귀

Main aggregate 296개는 오류/실패 0, 바깥 aggregate 533개는 기존 camera facade 실패 1개만
남았다. HD맵은 올바른 `ASMC_DATA=/data`로 재실행해 앞선 환경 오류 2개를 해소했다.
카메라 detection 42·runtime 29·직렬화 1·계약/측정 46개와 필수 validator를 통과했다.
선택적 문서 `--with-history`는 기존 main에도 동일한 보존 문서 2개 오류를 남기며 해당
문서/manifest는 변경하지 않았다. 기본 문서 검사와 필수 CI의 판정은 별도로 기록한다.

## 재호에게 넘길 이유와 순서

목표는 일반도로→음영→복귀의 같은 관측·추적을 유지하는 것이다. local 처리와 map
변환을 분리하면 GPS 손실이 검출/ID reset을 강제하지 않는다. 같은 검출을 두 번
실행하지 않으며, 필요한 출력만 직렬화한다. 공개 local은 후륜축 `base_link`, 내부
수평화 센서 좌표는 별개다. 카메라 투영은 당시 수평화 역행렬과 장착 변환을 합성한다.

재호는 먼저 터널 core를 이 단일 경로에 연결하고 객체 보호·잔여 벽 혼입을 개선한다.
그다음 깊이·연결성·돌출부로 혼합 군집을 분리하고 OBB/ID 연속성을 개선한다. 속도는
초기화·중심 변화·연관·source dt·자차 운동을 먼저 분리한 뒤 정확도 근거를 확보한다.
좌표 보정만으로 속도가 정확해졌다고 가정하지 않는다. 실제 지면·군집·tracker·fusion의
`asmc_lidar` 이전은 이러한 동작 보존 단위로 진행한다.

차량 기울기의 scan 단위 보정과 scan 내부 운동 deskew는 다른 과제다. 제거된 벽을
자유 공간으로 바꾸지 않으며, 구조물 충돌 경계는 판제와 별도 계약으로 정한다.
카메라는 픽셀 검출/모델 상태를 제공하고 판제가 구간 활성화·결과 소비·주행 모드를 결정한다.
