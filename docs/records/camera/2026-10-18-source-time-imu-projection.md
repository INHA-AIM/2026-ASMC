# 원본 시각 IMU 차체 보정 구현·정차 대조

## 결정과 범위

**M1 보정을 실제 실행 경로에 연결하고 시험했으며, 정본 camera.launch는 M0 고정 평면을 유지한다.**
사용자의 2026-10-18 적용·단계별 커밋 요청으로 진행했다. 모델·학습·차량 제어·스폰·기존 header·MD5를 변경하지 않았다.
지면 법선 거리 0.36m, 카메라/IMU 장착 위치, TensorRT Large E8, 전방 544×960·측방 480×640을 유지한다.
M1 구현은 자세 보정의 실행 가능성을 제공한다. 이번 정차 시험은 주행 안정성·독립 위치 정확도·음영 ODD 수락을 증명하지 않는다.

- `d241b2e2`: bounded source-time IMU 코어·SLERP·sparse 지면 투영·CPU 검사.
- `eb9753d4`: JPEG→grid wire 시각 계약·실제 IMU 수신·family 투영 및 xyz·진단·내부 시험 launch.
- 선행 `eb6870fc`: bridge의 원본·kernel 수신·발행 시각 및 실제 sensor wire 연결 보존.

현재 구현·제한·직접 명령의 정본은 [IMU 지면 투영 설계](../../architecture/camera-imu-projection.md)다.
공용 additive 메시지 `CameraModelGridTiming`의 MD5는 `3d9d2bf10966b2aac416bb6a1f66ea99`다.
기존 메시지와 `SensorFrameTiming`의 MD5는 보존했다. 공용 메시지·카메라를 runtime overlay에서 재빌드했다.

## 물리·시각 의미

영상 원본 시각을 IMU의 같은 source clock·bridge session에 연결한 뒤 두 quaternion 사이에서만 보간한다.
세계 평지 법선을 차체로 변환하고 장착 변환을 한 번 적용한다. 정차 Euler 값을 임의로 빼는 M1-rest는 사용하지 않는다.
새 높이 추정·yaw/병진 이동 정렬·과거 근거 누적은 하지 않는다. 시각 skew 35ms 이내에도 상대 이동 오차는 남는다.
보간 간격 100ms·수신 freshness 350ms·기울기 8도 제한은 확대하지 않았다.

세 뷰 모두 보정 가능한 경우에만 M1을 사용한다. source가 없거나 모호하거나 너무 오래됐으면 세 뷰 전체 M0로 fallback한다.
source duplicate는 독립 자세로 세지 않고, 충돌·역행·재시작·wire key 모호성을 배제한다.
보정 사용률과 family validity는 서로 다른 지표다. grid timing 자체가 없는 구형 result bag은 M0로 재생한다.

Family xy를 얻은 후 header 기준 뷰의 보정 평면으로 z를 계산한다. 최종 M1 기록의 z는 약 -0.4002~-0.3001m였고,
정본 0.36m 설정 객체를 변경하지 않았다. 일반 RGB BEV warp는 여전히 고정 평면이다.
따라서 갤러리는 각 후보의 자세로 **같은 원본 RGB에 재투영**한다. 고정 BEV 배경에 M1 선을 올려 정답을 판정하지 않는다.

## 시험 자료와 확인 결과

실제 MORAI 정차 위치는 **A2256W000748**이었다. 터널 A126 시험으로 표시하지 않는다.
읽기 전용 gRPC 조회에서 gear P, 속력 약 0.00015m/s, roll 0.4526도·pitch 0.1084도를 확인했다.
Ego pose는 시험 위치·정차 확인에만 사용하며 보정 계산의 운영 입력으로 사용하지 않았다.
날씨·시간·차량 위치·제어·기존 actor를 바꾸지 않았고 rqt를 새로 띄우지 않았다.

원본·그리드·정확한 JPEG SHA·source/IMU 대응·family·진단은 `data/runtime/imu-lane-20261018-*`에 보존한다.
[E388 원본 RGB 대조 갤러리](http://127.0.0.1:8000/camera-imu-projection-20261018-r1/index.html)는
`artifacts/camera-imu-projection-20261018-r1/`에 있으며 실험 목록에도 등록했다. HTTP 200과 새 페이지의 로컬 링크를 확인했다.
목록 전체 링크 검사에는 별도 LiDAR 다운로드 인덱스의 wheel 링크 24개 누락이 남았다. 이번 페이지의 끊어진 링크는 0개다.
`capture_imu_lane.py`는 제한된 journal과 최신 뷰만 유지하며 파일 기록은 수신 callback 밖에서 수행한다.
첫 m0-r1은 bytes 직렬화 오류로 실패했다. 실패 파일을 보존하고 uint8 배열 변환을 고쳐 새 run으로 기록했다.

초기 동일 입력 166묶음에서 M0/M1 모두 family 166개가 유효했다. 관측된 역할의 짝지은 곡선 위치 변화 중앙값은 약 5.8cm였다.
두 후보의 내부 semantic 재투영 거리 중앙값은 모두 model-grid 2.8px로 사실상 같았다.
이는 독립 GT 정확도나 실제 차선 중심 오차가 아니다. 정차 자세 변화가 매우 작아 시간적 흔들림 개선도 결론내릴 수 없다.
동일 입력 sparse remap 추가 비용은 p50 약 5.2ms·p95 약 10.4ms였다. 다른 fit 단계·source/수신·ROS 발행을 포함한 총 지연과 구분한다.

초기 m1-r1의 60초 기록은 CPU 회귀 검사와 일부 겹쳤다. 해당 공급률을 보정의 인과적 속도 차이나 수락 근거로 사용하지 않는다.
최종 코드로 M1 30초와 복원한 M0 30초를 별도로 기록하고 집계한다. 전체 GUI·음영 주행·10분 수락은 이번에 수행하지 않았다.

최종 집계는 [고정 JSON](../../../config/baselines/evidence/camera_imu_projection_v1/summary.json)에 입력·설정·도구 SHA와 함께 보존한다.
별도 30초 기록의 125묶음 중 source skew를 통과한 120묶음을 같은 M0/M1에 넣었고 두 후보 모두 120/120 유효였다.
관측 역할의 짝지은 곡선 위치 변화 중앙값은 5.65cm, 내부 재투영 중앙값은 양쪽 2.8 model-grid px였다.
관측 support가 서로 달라 위치 변화량을 비교할 수 없는 묶음도 있어, 위치 변화 집계는 98묶음이다.

| 최종 정차 기록 | M0 고정 평면 | M1 IMU |
|---|---:|---:|
| 수신 family·유효 | 339/339 | 288/288 |
| 30초 기록당 평균 수신량 | 약 11.3Hz | 약 9.6Hz |
| 완전한 5초 창 유효 공급 최솟값 | 9.8Hz | 8.2Hz |
| 수신 공백 p95 / 최대 | 129.0 / 233.0ms | 149.7 / 221.3ms |
| 중복·역행 stamp | 0 | 0 |
| 기록 journal 탈락 | 0 | 0 |

M1 진단의 첫·끝 구간 사이 266회 계산 중 257회가 보정을 사용하고 source skew 초과 9회는 전체 M0로 fallback했다.
이 진단 구간은 30초 기록 전체와 시작·종료 시점이 같지 않다. fallback을 M1 적용 성공으로 세지 않는다.
두 실행 모두 일부 5초 창이 10Hz에 미달했다. 기본 M0 역시 전체 실행 수락 통과로 표시하지 않는다.
정차 한 위치·한 번씩의 비교여서 1.7Hz 차이를 IMU 수학 연산만의 인과적 비용으로 확정하지 않는다.
같은 입력의 최종 CPU 대조에서는 sparse remap p50 6.0ms·p95 13.0ms가 추가됐다.
NN forward·입력 디코딩·ROS 수신을 포함한 전체 지연과 이 remap 시간을 혼동하지 않는다.

기본값을 M1로 바꾸기에는 독립 품질 향상 근거가 부족하고 공급 조건에도 여유가 없다.
일반 camera.launch로 M0를 복원했으며 객체·차선·신호등 ACTIVE와 단일 생산자를 확인했다.
M1은 `_imu_compare.launch enabled:=true`로 직접 켤 수 있다. 기존 camera.launch를 먼저 종료해야 한다.

## 검사와 남은 조건

- Source timing의 정확한 header/wire 연결, 보간/외삽 차단·중복/충돌·재시작·bounded TTL 검사.
- 세 뷰 중 하나가 무효일 때 전체 M0 fallback, 다른 모델 session·잘못된 output sequence 배제.
- 알려진 자세의 광선/평면 교차·장착 pitch 중복 방지·yaw 불변성·identity 동등성.
- Family xyz가 현재 보정 평면에 있고 정본 지면 객체를 바꾸지 않는 검사.
- Noetic runtime·BEV 회귀와 실제 메시지 직렬화·기존 MD5 보존.
- 계약 registry·topic/bag 정책·문서 링크/실행 검사를 확인한다. Hosted CI 실행이나 병합은 하지 않았다.

Runtime의 planner 연계 launch 검사는 별도 test-support의 XML을 사용했다. production overlay에 가짜 planner 노드를 추가하지 않았다.
초기 실행기의 전역 LAUNCH 경로 변경으로 경로 검사 4개가 실패했으며, fixture 적용 범위를 해당 시험 하나로 고쳐 모두 재확인했다.
Runtime은 112개 검사, BEV는 187개 검사, 공용 메시지 직렬화는 3개 검사를 확인했다.
모드 전환 시험에서 worker thread 예외가 시험 결과에 빠질 수 있어 예외 수집 assertion을 추가했다.
고정 평면의 optional plane 경로를 정리한 뒤 관련 worker 6개·IMU adapter 4개도 예외 없이 재확인했다.

다음 판단은 음영 진입·내부·출구의 실제 이동에서 M0/M1의 위치·역할·유효 family 공백을 같은 입력으로 비교한 뒤 한다.
현재 연결 성공과 작은 정차 기하 차이만으로 기본값을 승격하지 않는다. 공급 ≥10Hz·공백≤250ms·freshness350ms 조건도 유지한다.
Yaw/병진 정렬은 터널에서 유효한 상대 이동 입력이 확보된 경우 별도 M2로 검토한다.

후속 사용자가 허용한 터널 주행·기존 bag 분석은 [별도 운동 대조 기록](2026-10-18-tunnel-imu-motion-and-fit.md)에
고정한다. 이 문서의 정차 수치와 당시 기본값 판단은 덮어쓰지 않는다.
