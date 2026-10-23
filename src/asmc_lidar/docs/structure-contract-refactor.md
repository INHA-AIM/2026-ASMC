# LiDAR 구조·관측 계약 정리

2026-10-23. 안승현이 구조와 공용 계약을 정리하고 손재호가 검토할 작업 단위다.
기준선은 `config/baselines/lidar_structure_contract_v1.json`을 따른다.
PR #103의 계산 결과를 보존하며 기존 Main의 판제 출력을 전환하지 않는다.

## 공동 개발 책임

| 담당 | 후속 작업 |
|---|---|
| 안승현 | HD맵 기반 필터, 카메라 수신·보정·시간·라벨 연결 지원 |
| 손재호 | 추적·속도·운동 상태, 객체 연결과 카메라 fusion 보완 |
| 공동 확인 | 좌표·시간·공용 메시지 변경, 전체 비교와 판제 전환 |
| 판제 | TTC·추종·정지·회피·센서 장애 대응 |

## 이번 작업

1. 책임별 디렉토리·실행 이름과 모든 저장소 호출부를 함께 수정한다.
2. 검출 관측·추적 상태·카메라 분류 근거를 분리하고 계산의 ROS 의존성을 경계로 옮긴다.
3. `asmc_msgs/LidarObservation`으로 생산자·초기화 세대·scan 식별을 함께 전달한다.
4. 고정 입력의 계산·기존 메시지 동등성, 초기화·재시작, 빌드·계약을 검사한다.

새 실행 출력은 `/migration/lidar/*`를 사용한다. 구형 이름 wrapper는 만들지 않는다.
기존 Main·Msg 운영 경로와 외부에 제공하던 라이브러리 이름은 유지한다.
알고리즘·임계값·속도 validity 변경과 HD맵 필터 적용은 후속 작업이다.

상태: 구현·로컬 구조/계약 회귀 완료. 운영 수락·판제 전환은 아직 하지 않았다. Hosted CI는 PR 결과와 별도로 확인한다.

## 실행 진입점 마무리

`lidar.launch` 하나가 `lidar_pipeline_node`를 실행하고 기본 `fusion:=true`를 적용한다.
`fusion:=false`는 카메라 보정 조회와 구독을 끄므로 카메라 패키지가 없어도 같은 launch를 사용한다.
`rviz:=true`는 같은 namespace에 읽기 전용 Marker 도구와 RViz를 추가한다. 기본은 OFF이며
`view.launch`의 웹 모니터와 구분한다. 기존 RViz/처리가 있다면 두 번째 실행을 시작하지 않는다.
내부 `enable_camera_fusion`·YAML 이름과 계산 알고리즘은 유지한다. 이전 두 launch는 wrapper 없이 통합했다.
`lidar_replay`는 오프라인 대조 도구로 분리한다. 재호의 PR #105 웹 시각화를 현재 구조로 통합해 `view.launch`에서
결과만 구독한다. 처리 시작은 `lidar.launch debug:=true`로 분리한다.

이 후속 변경은 launch·문서·구조 검사만 바꾼다. 아래 3,428회 대조는 계산 소스가 동일하므로
재사용하며, 최종 launch의 ROS 해석과 카메라 없는 환경은 별도로 검사한다.
최신 HD맵 적용 상태와 병합 후 담당별 순서는 [통합 인계](../../../docs/handoffs/lidar-migration.md#이번-pr-이후-공동-작업-순서)를 따른다.

후속 실행 검사(2026-10-23):

- 구조 계약8개, 문서 unit17개, runtime 문서26 launch·71개 literal 명령과 history/동결5개 검사를 통과했다.
- 독립 ROS master11541에서 기본 launch가 처리 노드 하나를 시작하고 카메라 관측·공용 상태 두 토픽을
  구독하며 diagnostics의 `camera_fusion=enabled`를 확인했다.
- 별도 master11542와 카메라 패키지가 없는 `ROS_PACKAGE_PATH`에서 `fusion:=false`를 실행해
  `camera_fusion=disabled`, `camera_reason=disabled_by_config`, 카메라 구독0개를 확인했다.
- 두 시험 모두 운영 객체·TF 발행이 없고 종료 시 노드·rosout·시험 master를 정리했다.
  센서가 없는 실행 확인이므로 실제 검출 품질·공급률 검증으로 사용하지 않는다.

## 처리 객체의 RViz 표시

`scripts/lidar_markers.py`와 `rviz/lidar.rviz`를 추가했다. 웹 `view.launch`와 별도의
읽기 전용 표시이며 `/migration/lidar/observations_local`을 `/migration/lidar/markers`로 변환한다.
OBB8코너·source header·추적ID·소비 그룹/카메라 원본분류를 그대로 사용한다. IMU·장착 변환이나
검출·추적·fusion을 다시 계산하지 않고, 속도 validity가 false인 결과에 운동 화살표를 만들지 않는다.
빈 결과·무효·세션/세대 전환에서 이전 표시를 지우고 입력350ms 중단은 wall clock으로 제거한다.
중간 지면 제거/군집 점군은 이 표시 도구의 입력이나 출력이 아니다.

2026-10-23 검증: CPU 표시 검사6개, 독립 ROS11545의8코너/header·빈 결과 삭제·wall 만료·
Marker 전용 발행을 확인했다. 일반 `/root/ws/devel` 재빌드 후 전체 CTest27/27이 통과했다.
실제 실행 중인 local 관측3객체와 Marker3박스의 source stamp·`base_link`·8코너가 정확히 일치했다.
기존 bridge·처리 노드·RViz는 유지하고 표시 도구 하나만 연결했다. 알고리즘 품질·공급률의
새 수락 시험으로 해석하지 않는다. 사용법은 [README](../README.md#처리-결과를-rviz에서-보기)를 따른다.

## 관측과 내부 데이터

- `DetectionObservation`: 한 scan의 군집 점군·centroid·AABB·OBB·지면 근거.
- `TrackState`: KF 상태·이력·이전 좌표 context·ID/age/miss. `CameraClassEvidence`: 분류 확인/미검출 근거.
- `TrackedObject`는 위 세 레코드를 조합한다. 점군은 detection에만 저장하고 기존 이력 복사 횟수는 바꾸지 않는다.
- `obb_length_m`/`obb_width_m`은 기존 `f32_OBB_Width`/`f32_OBB_Height`와 동일 수치인 수평 두 변이다. 수직 크기는 ZMax−ZMin이다.
- `obb_corners_level`은 기존 `OBB_Corners_Body`이며 실제로는 수평화 센서 좌표다. 공개 출력은 관측 context로 역수평화·장착 변환을 한 번 적용한 후륜축 `base_link` 또는 유효 map 좌표다.
- 공개 `center`는 OBB 중심이다. 내부 cluster centroid와 KF 상태 위치는 다른 값이며 같은 이름으로 합치지 않는다.
- yaw는 변환한 장축의 XY 투영이다. roll/pitch가 있을 때 8코너를 yaw 하나로 재구성하지 않는다.
- `source_point_count`는 voxel 이후 군집 점 개수다. 카메라 소비 그룹과 실제 운동 상태는 별개다. 현재 상대/절대속도·동정적 validity는 false로 유지한다.

`asmc_msgs/LidarObservation`은 기존 `LidarObjectArray objects`와 `producer_session_id`, `generation`, `scan_sequence`, `config_id`, `calibration_id`를 함께 전달한다.
`objects.header.stamp/frame_id`가 정본이고, ROS1이 top-level 배열의 header.seq를 다시 넣으므로 header.seq는 동일 scan의 연결 키가 아니다.
같은 local/map 결과는 `(producer_session_id, generation, scan_sequence)`로 연결한다. 누락 sequence는 latest-wins 대체/거부에서 생길 수 있다.

생산자 시작은 새 UUID·generation1, 명시적 `~reset`(`std_srvs/Trigger`)은 같은 UUID에서 generation을 증가시킨다.
초기화는 대기 scan·tracker·pose/GPS sanity·fusion consumer 상태·trace 이력을 비우고, 계산 중인 이전 runtime의 결과는 세대 확인에서 폐기한다.
scan 순번은 같은 생산자에서 계속 증가한다. 입력 침묵·GPS 무효·카메라 세대 변경·역행 scan을 초기화 요청으로 추측하지 않는다.
카메라 수신 버퍼는 별도 생산자의 정상 관측을 보관하므로 유지하고 새 fusion consumer의 첫 관측 gate를 따른다.

`config_id`는 strict 설정 파일 SHA·보정 ID와 실제 launch/replay override(roll, fusion, tunnel, debug)를 묶은 SHA다. 실행 중 설정을 변경하지 않는다.
`calibration_id`는 적용 LiDAR mount SHA와 실제 전방 카메라 보정 ID를 묶은 SHA다. YAML 원본 SHA는 diagnostics에 별도로 남긴다.
기존 `LidarObject/Array`와 운영 `Msg/TrackedObject*`의 MD5는 유지하며 새 관측은 구형 타입의 이름 alias가 아니다.

## 재호의 연결 시작점

- 검출·추적: `tracking/track.hpp`, `tracking/tracker.hpp`. 내부 속도를 공개 validity로 승격하는 작업은 별도 검증한다.
- 카메라: `fusion/front_camera_fusion.hpp`, `fusion/camera_observations.hpp`, `ros/camera_receiver.hpp`. 현재 `camera_front`만 매칭한다.
- 새 소비자: `observations_local/map`을 구독해 session/generation 변경 시 소비 캐시를 비운다. 같은 scan을 반복 확인 근거로 세지 않는다.
- 후속 필터는 현재 기하를 입력으로 받고 제거 전후·사유를 대조한다. TTC/추종/정지/회피 판단은 판제에 둔다.


## 2026-10-23 검증 결과

[기계 판독 결과](../../../config/baselines/lidar_structure_contract_result_v1.json),
D의 `artifacts/lidar-structure-contract-20261023-r3/`에 원본 replay·자원·검증 로그와 소스 근거를 동결했다.
입력 bag 두 개의 전체 SHA를 확인했고, 실제 실행에는 등록한 같은 run ID를 전달했다.
기준선에는 timing trace만 추가했으며 계산 코드는 바꾸지 않았다.
집계 manifest 작성이 run 동결보다 약12ms 늦어 원본을 수정하지 않고 별도
`lidar-structure-freeze-audit-20261023-r1`로325개 파일의 전체 SHA·목록 일치를 재검증하고 동결했다.

| 검사 | 결과 |
|---|---|
| 새 LiDAR 필수 CTest | 26/26, 생략 없음. 초기화·재시작·이전 세대 결과 차단 포함 |
| 기존 nested Main | 클린 별도 빌드, 고유 사례203개 통과. 요약 도구의378건을 고유 사례로 세지 않음 |
| 외부 링크 경계 | Main 링크28개 검사, 새 private pipeline/runtime 연결 없음 |
| 메시지 | 기존 MD5 보존, 신규 관측 포함39개 등록, 직렬화4개 통과 |
| 고정 입력 동등성 | primary649·shadow-static208개의 독립 scan857개, 4조건 총3,428회 모두 동일 |
| 비교 내용 | 단계별 점군 SHA·군집·OBB·KF/track·클래스·local/map·상태/사유. 실행 ID와 timing만 제외 |
| 메모리 구조 | 기존/신규 `TrackedObject` 모두832B, 점군 소유자 추가 없음 |
| 문서·계약 | 동결5개 SHA/history, 링크, 공용 타입·판제 launch/filter 계약 통과 |

정상 roll/pitch primary의 계산 p95는 최종 matrix에서16.09→16.79ms였다.
추가 교차3쌍에서는 기준선16.14/16.54/16.73ms, 후보16.78/16.35/18.09ms였다.
반복 중앙값은16.54→16.78ms이며, 최대 RSS는 기준선62.7–63.3MiB·후보62.8–63.6MiB였다.
새로운 점군 복사·상태 크기 증가·지속 큐 증가는 없었다. 작은 wall 차이와 한 번의18.09ms 값은
그대로 보존하며 속도 개선으로 주장하지 않는다. 실제 동시 부하·공급률 수락을 대신하는 수치가 아니다.

첫 계측의 RSS는 Python 부모가 전체 JSON을 보유한 채 fork한 highwater가 섞여 제외했다.
보존한 `discarded-paired-repeat-rss.json`을 따르며 최종 streamed 반복과 matrix 자원값만 사용한다.
IMU fault ROS 시험은 재개 명령 당시 마지막 scan이 아니라 실제 invalid IMU source 구간을 검사해
전환 경계의 입력 순서와 정상 복귀를 구분했다. 운영 IMU age/대기 정책은 유지했다.

두 bag에는 정본 카메라 관측이 없어 실제 영상 fusion 품질·3뷰 매칭·실시간 공급을 검증한 것으로
표시하지 않는다. 합성 ROS 시험은 기존 전방 분류 연결과 상태 전환 계약을 검사한다.

재현 비교(읽기 전용, JSON을 stdout으로 출력):

```bash
python3 src/asmc_lidar/tools/compare_traces.py \
  --baseline /artifacts/lidar-structure-contract-20261023-r3/baseline-final \
  --candidate /artifacts/lidar-structure-contract-20261023-r3/candidate-final
```

담당별 최신 인수·보완 순서는 [공동 개발 인수표](migration-tasks.md#공동-개발-인수와-우선-작업)를 따른다.
다음 안승현 작업은 기존 인지 필터 소유권·HD맵 최종 관련성의 오프라인 대조다.
점군 ROI 변경과 실제 객체 제거는 이 첫 단위에 섞지 않았다.

## PR #105 통합 및 직접 확인 후속

최신 main `a63727c5`(PR #105, 원본 `2648f117`)의 단계 observer·debug 발행·웹 모니터와
진단용 motion history를 보존했다. 헤더를 `tracking/`·`ros/`로 옮기고 분리된
`detection/tracking/camera` 필드, `scan_sequence`와 처리 epoch에 맞춰 연결했다.
`view.launch`는 구독 전용이며 처리 시작 기능·구형 launch wrapper는 없다.
기존 공용 메시지·속도/운동 validity와 Main·판제 출력은 유지한다.
과거3,428회 동등성은 PR #103 기준 구조 변경의 증거로 보존한다. PR #105의 진단용
이력 추가로 객체 크기/전체 처리 비용이 달라질 수 있으므로 기존832B·p95를 최신 통합본 실측으로 표시하지 않는다.

사용자는 음영구간에서 박스가 많이 튀는 현상을 확인했다. 현재는 시각화 연결 확인만 완료했으며,
지면·구조물·군집/OBB·추적·TunnelFilter OFF/ON을 같은 입력에서 분리 확인할 후속 과제다.
필터 기본 OFF·운영 입력 미전환을 유지한다. 표시 UI의 지도 숨김을 HD맵 필터 구현으로 세지 않는다.

통합 회귀의 IMU 무효 전환 시험에서 정상 결과1건이 나타났다. 기존 PoseHistory는 마지막 무효 stamp
하나만 저장하므로 다음 무효 표본이 대기 scan보다 미래로 이동했을 때, 아직120ms 이내인 이전 정상
표본이 선택될 수 있다. 이번 구조 통합에서는 이 정책을 바꾸지 않았다. 지속 무효 검사는 이전 정상
표본의 최대 age 이후를 대상으로 하며, 초기 전환의 fail-closed 보장은 후속 과제다.

통합 최종 로컬 검증: Noetic 필수 build 성공, CTest **31/31**·CI gate 통과(생략0),
웹 core29개·JS 표시61개, 문서 unit17개·runtime27 launch/94명령·history/동결5개,
카메라 계약5개·공용 메시지39개 검사를 통과했다. 초기 CTest는30/31이었고 위 IMU 전환
검사창 수정 후 전체를 다시 실행했다. 이전 실패 로그와 최종 로그는 컨테이너 `/tmp/`에 구분해 보존한다.
실행 중 bridge·LiDAR·RViz는 재시작하지 않았으며 신규 실시간 공급/음영구간 품질 수락으로 세지 않는다.

RViz 실행 옵션 후속: 기본 OFF/ON과 터널 namespace·대체 관측/점군 토픽의 실제 ROS launch
해석을 확인했다(기본1노드, ON3노드). 구조 계약10개·카메라 계약5개·문서 unit17개와
runtime27 launch/99명령·동결5개 검사를 통과했다. 새 GUI를 실행하거나 운영 노드를 재시작하지 않았다.
