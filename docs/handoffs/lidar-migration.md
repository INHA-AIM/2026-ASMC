# LiDAR 통합 이관 계획과 소비 인계

**2026-10-22: 안승현이 L0–L8 단위로 직접 이관을 진행한다.** 손재호의 별도 개발은
병행하며 해당 브랜치 제공을 착수 조건으로 두지 않는다. 기존 팀 ownership·리뷰 경계는
유지한다. L0–L1은 문서·설계·기준선 정리이며 코드 이관·운영 전환이 아니다.

현재 `asmc_lidar`는 **좌표·시간 기반과 공용 카메라 수신·버퍼·투영 컴포넌트**, 그리고 Main common-geometry
LiDAR 처리(ROI·Voxel·roll·pitch 수평화·RANSAC·군집·OBB·추적·전방 fusion·typed local/map)를 동작 보존 이식한
**shadow 노드와 bag replay**를 제공한다([Shadow 파이프라인](../../src/asmc_lidar/docs/shadow-pipeline.md)).
[카메라 입력 API](../../src/asmc_lidar/docs/camera-input.md)를 손재호의 연결 시작점으로 사용한다.
shadow 노드는 기존 launch에 포함되지 않고 명시 실행할 때만 `/asmc_lidar/shadow/*`에 발행한다.
MotionClassifier·판제 후처리·구조물 분리·3뷰 fusion과 운영 ROS 출력 전환은 손재호가 원본
[`lidar+sensorfusion`](../../src/lidar+sensorfusion/RUN_SENSOR_FUSION.md)에서 이어서 이전한다.
기존 원본 패키지가 현재 주행 경로다. `asmc_lidar`의 probe와 shadow 노드는 운영 출력을 내지 않으며,
`Main`의 opt-in 어댑터가 코어를 연결해 typed local/map 객체를 제공한다.

shadow의 비교 기준선은 Tracker yaw 수정이 반영된 Main `8f93c22e`(`fix/lidar-tracking-frame-yaw`, main 미병합)이고,
그 수정은 shadow에 이미 들어 있다. 기존 Main은 동결하며 shadow 이식은 Main Tracker 수정의 선행 병합을 요구하지 않는다.
저장소 Main과의 OBB yaw·corner 차이는 의도된 것이다([의도적 차이](../../src/asmc_lidar/docs/shadow-pipeline.md#기준선과-저장소-main과의-의도적-차이)).
[좌표·GPS 경계 구현과 수락 보류](../records/lidar/lidar-camera-frame-boundaries.md)를 참고한다.
[external 6개 좌표 구현과 현재 보정 검토](../records/camera/camera-lidar-coordinate-reference-review.md)에는
행렬 수치 대조와 센서 시각·지면·pose 제공자 차이를 기록했다.

작업 상태·선행 조건·완료 근거의 정본은 [단계별 작업표](../../src/asmc_lidar/docs/migration-tasks.md)다.
L1 구조·입출력·시각·pose 정책은 [이관 설계](../../src/asmc_lidar/docs/migration-design.md),
고정 소스/설정·입력 SHA는 [L1 manifest](../../config/baselines/lidar_migration_l1_v1.yaml)를 따른다.
현재 판제는 nested `lidar+sensorfusion`의 Main과 `/last_lidar`·tracked/motion 경로를 계속 쓴다.
새 경로의 검증과 판제 인수 전에는 이 실행 경로·기본 설정·출력을 변경하지 않는다.

| 구분 | 현재 구현 | 이번 이관에서 할 일 |
|---|---|---|
| 센서 처리·검출·추적·전방 fusion | 기존 Main이 운영 생산자 | 동작 보존 이식 후 좌표·시간 수정은 별도 비교 |
| `asmc_lidar` | 좌표·시간 코어, 카메라 수신·버퍼·보정·투영, 진단 probe | LiDAR 전체 실행·발행 노드는 아직 미이전 |
| 공용 local/map 출력 | Main의 opt-in 어댑터가 생산, 운영 수락 보류 | 새 패키지 생산자와 소비 전환은 L6–L7 |
| 카메라 추론 | `asmc_camera` 단일 소유, Main 구형 YOLO 제거 완료 | 모델을 재이식하지 않고 관측·퓨전만 연결 |
| 남은 구형 기능 | Main의 HSV 영상 처리·UDP 객체 송신·제어 발행 등 | 새 경로에 가져오지 않음. 기존 삭제는 소비 전환 이후 |

현재 연결은 [카메라→LiDAR 인계](camera-to-lidar.md), 실제 수신 API는
[카메라 입력 API](../../src/asmc_lidar/docs/camera-input.md), 좌표·출력 의미는
[소비 계약](../contracts/lidar-planning-consumer-v1.md)을 따른다.
[좌표·GPS 경계](../records/lidar/lidar-camera-frame-boundaries.md)와
[external 좌표 검토](../records/camera/camera-lidar-coordinate-reference-review.md)는 기존 근거로 보존한다.

## 현재 경로와 이관 목표

```text
현재: bridge → nested Main → /last_lidar + tracked/motion → planning_inputs → 판제
목표: bridge → asmc_lidar 계산 라이브러리 + ROS 실행부
                 ← asmc_camera 관측 (LiDAR에서 수신·투영·fusion)
             → 공용 local/map 객체와 상태 → 판제 소비 어댑터
```

목표 내부 책임은 `geometry / preprocessing / detection / tracking / fusion / filtering / ros`다.
계산 라이브러리와 ROS 입출력을 분리하고 단계마다 ROS 노드를 추가하지 않는다.
실제 디렉토리는 해당 구현 때 만든다. 구형 UDP `127.0.0.1:7510`, `/ctrl_cmd_0` 발행,
HSV 차선·정지선·신호등 처리는 새 LiDAR 경로에 이식하지 않는다. 카메라 보정·투영은 유지한다.

## 시험 namespace와 전환

| 대상 | 정식 후보 | 이관 동시 비교 |
|---|---|---|
| local 객체 | `/perception/lidar/objects_local` | `/migration/lidar/objects_local` |
| map 객체 | `/perception/lidar/objects_map` | `/migration/lidar/objects_map` |
| 상태·진단 | L1에서 기존 공용 타입·경로 검토 후 확정 | 확정된 새 출력만 시험 namespace 적용 |

위 시험 경로는 **설계이며 아직 launch에 구현하지 않았다.** L2–L5의 시험 실행기는 기존
판제에 연결하지 않는다. 출력 namespace/remap만 전환하며 센서·카메라 입력은 정본 경로를
계속 구독한다. Main opt-in도 같은 정식 출력 이름을 쓰므로 동시 비교 시 충돌을 검사한다.

L7에서 기존 발행자·호환 어댑터를 정지하고 새 경로를 정식 이름으로 실행한 뒤 판제 입력을
명시적으로 전환한다. 문자열을 코드에서 다시 고치지 않는다. frame·stamp·속도/무효 의미와
재시작 ID·소비자 캐시 초기화를 확인하고 같은 토픽의 생산자는 하나만 유지한다.
복구는 새 생산자·어댑터를 정지한 뒤 고정한 기존 Main과 판제 입력 설정을 함께 복원한다.

## 이관 전 단계의 공통 기준

- local 처리·발행은 GPS·미션 구간에 종속시키지 않는다. 실제 후륜축 `base_link`와 내부
  수평화 좌표를 구분하고 중심·8코너·방향·속도 변환을 일관되게 적용한다.
- roll/pitch 보정은 구간별 다른 정책 대신 같은 좌표·시각 규칙으로 검증한다. 지면 근거나
  IMU 무효를 숨기지 않는다. 자세 보정은 추적기의 ego motion 보상이나 deskew를 대신하지 않는다.
- map 객체는 scan 시각의 유효 pose에서만 만든다. GPS 0·무효·stale에서 이전 위치를
  무기한 재사용하지 않는다. 새 실행은 L1에서 고정한 bounded300ms·복구3회를 사용한다. strict100ms 보간 코어는 별도 시험 API로 유지한다.
- 정상 빈 결과·센서 무효·pose 무효·미수신을 구분한다. 미확정 속도·운동 상태를 유효한
  0/static으로 치환하지 않는다. 현재 `LidarObjectArray`의 상태와 속도 validity를 재사용 검토한다.
- 측정 품질·속도 이상·일반 중복·지도 관련성은 인지 책임이다. TTC·추종·정지·재출발·회피와
  센서 장애 대응은 판제에 유지한다. 최신 main의 동적 장애물 행동 정책도 이관 대상이 아니다.
- Main/planning_inputs의 중복 필터를 단일 구현으로 정리하되 기존 임계값 보존과 정책 변경을
  분리한다. HD맵 점군 ROI·새 구조물 억제·3뷰 매칭 확대는 L8에서 우선순위를 다시 정한다.
- 기존 코드 삭제는 새 경로 검증·소비 전환 뒤로 둔다. 이번 L0에서 launch·메시지·운영값은 변경하지 않는다.


## 읽는 순서와 검증 상태

1. 이 README와 [현재 카메라 인계](camera-to-lidar.md)에서 실행/계약·현재 구현과 후속 과제를 확인한다.
2. [작업표](../../src/asmc_lidar/docs/migration-tasks.md)에서 L1 결과를 확인하고 L2부터 한 단위씩 계획한다. B1a 과제 대응표와 과거 증거를 함께 확인한다.
3. [PR #72 연속 회귀](../records/lidar/lidar-continuous-regression.md)에서 시간·지면·추적 검증과 미수락 항목을 확인한다.
4. [좌표 경계](../records/lidar/lidar-camera-frame-boundaries.md)와 [외부 참고 검토](../records/camera/camera-lidar-coordinate-reference-review.md)를 세부 근거로 읽는다.
5. [소스 README](../../src/asmc_lidar/README.md)에서 코어 API·빌드·시험을 확인한다.

`Main` 연결 구현은 opt-in이며 공통 경계와 roll 기본값은 모두 OFF다.
지면 제거·군집·OBB·tracker·전방 fusion은 shadow 노드로 이식했고 roll·pitch 수평화와 local/map 출력을 검증했지만,
운영 경로에는 연결하지 않았다. 구조물 분류는 아직 이전하지 않았다.
24점 삭제/보존과 신규 분류 차이는 [원본 이미지·점군 비교 갤러리](../../config/baselines/evidence/lidar_continuous_regression_v1/user-review/index.html)에서 확인한다.
24점은 장애물 정답이 없는 합성 지면 높이 입력이며, 실제 장애물 소실 사례로 해석하지 않는다.

## 재호: PR #72 반영 후 비교 갤러리 먼저 보기

현재 개발 PC에서는 [전체 실험 갤러리 목차](http://localhost:8768/)의 `lidar-pr72`에서
LiDAR 비교와 카메라 모델 비교를 함께 찾을 수 있다. 이 주소는 로컬 artifact 서버이며,
다른 PC에서의 재현·인계에는 아래 저장소에 포함한 갤러리와 실행 안내를 사용한다.

PR #72가 반영된 checkout에서
`config/baselines/evidence/lidar_continuous_regression_v1/user-review/index.html`을
파일 탐색기에서 브라우저로 연다. HTML·사진·GIF·상태/속도 JSON을 모두 Git에 포함했으므로
**ROS, GPU, MORAI, bag 원본이나 기존 임시 컨테이너가 없어도 볼 수 있다.** GitHub의
HTML 소스 보기 대신 로컬의 `user-review` 폴더 전체를 사용한다.

HTTP로 보고 싶으면 **저장소 루트** 터미널에서 아래 명령을 실행하고 브라우저로
`http://localhost:8767/index.html`을 연다. 종료는 해당 터미널의 `Ctrl+C`다.

```bash
python3 -m http.server 8767 --bind 127.0.0.1 \
  --directory config/baselines/evidence/lidar_continuous_regression_v1/user-review
```

검토 순서는 **연속 구간 1·2·3의 벽 조각/바닥 노이즈 → 분류 10번의 OBB와 우측 영상 →
11관측의 분류 카운터 → 속도 급변 분포**다. 24점은 사진 없는 합성 정책 시험이다.
같은 source stamp의 point→cluster→OBB→track/분류 이력을 이어서 확인한다.
[작업표](../../src/asmc_lidar/docs/migration-tasks.md)의 과거 구조물·속도 증거로 연결한다. 다음 구현 계획은 L2이며 연구 확대는 L8이다.
현재 세션의 localhost 8766 서버나 `/tmp` 경로는 향후 재현에 필요하지 않다.

L8 위치·속도 개선의 과거 참고 순서는 [과제 목록의 관측 조사·비교 순서](../../src/asmc_lidar/docs/migration-tasks.md#위치속도-관측-조사와-재호의-비교-순서--2026-10-08)를 따른다.
군집 평균·OBB 중심·필터 위치를 구분하고 시간/초기화/연결 → 구조물·군집 → 일관된
관측 위치 → 불확실성/연결 순서로 비교한다. Autoware 공개 구현과 연구 논문은
참고 후보이며 우리 센서에서 실행·성능 검증된 대안처럼 취급하지 않는다.
운영진이 답변한 후륜축 속도 GT와 현재 브릿지의 km/h 단위는
[속도 GT 인계](../../src/asmc_lidar/docs/migration-tasks.md#morai-속도-gt-기준--2026-10-06-전달-qa-반영)에서 확인한다.

**PR #72 병합 범위:** 공통 좌표·시간 코어, Main의 opt-in 연결, 회귀 도구·증거와
인계 자료다. 공통 경계/roll 기본값 OFF와 공개 속도/동정적 validity false를 유지한다.
병합은 객체 보존·속도 정확도·정상 map 추적 품질 또는 운영 기본값 승격의 승인이 아니다.

## 역할과 남긴 파일

카메라 패키지는 `asmc_camera`다(2026-10-06 `asmc_perception`에서 이름 변경). 안승현이
YOLO26l E8/사전학습 생산자를 `asmc_camera`의 정본으로 제공한다. 모델/통합 부하 검증도 카메라가 맡는다. 안승현은 이번 `asmc_lidar` 이관을 직접 진행한다. 손재호의 LiDAR·fusion 개발은 병행하며
소비 변경은 판제와 검토한다. HD맵은 지도와 주행영역 정본을 제공한다.
자세한 역할 분담은 [통합 로드맵](../architecture/consolidation-roadmap.md)을 따른다.

| 제공한 구성 | 사용할 때의 조건 |
|---|---|
| `asmc_lidar_frame_time` 라이브러리 | 실제 수평화 행렬·센서 extrinsic·유효 pose를 호출자가 제공 |
| IMU 선택 함수 | 도착 완료 표본만 전달, 유한값/유효 quaternion 검사 후 valid 지정 |
| v3 장착 설정 | 출처 SHA와 단위 확인. 장착 z를 노면 기준 센서 높이로 사용하지 않음 |
| 회귀시험 | 좌표·8코너·속도·관측시각·pose/IMU 선택 의미 검사. 실제 검출 성능 시험은 후속 |
| 공용 `asmc_msgs` 타입 | LidarObject/Array, CameraObjectBox/Array 정의와 직렬화 검사. 카메라 생산자는 정본 실행, Main의 typed LiDAR 출력은 opt-in 구현; asmc_lidar는 수신 확인용 probe와 shadow 노드 제공(운영 typed 토픽 미발행) |
| shadow 노드·replay | `asmc_lidar_shadow_node`·`asmc_lidar_shadow_replay`와 shadow 전용 launch 2개. 명시 실행, `/asmc_lidar/shadow/*`만 발행. 공개 속도·동정적 validity false, TunnelFilter 기본 OFF. 실제 카메라 bag·실시간·TunnelFilter ON 성능은 미수락 |

빌드·시험 명령과 API 위치는 [패키지 README](../../src/asmc_lidar/README.md)에 있다.
과거 연구용 지면·tracker 후보와 이전 `unified.launch`, `ground_compare_live.launch`는 남기지 않았으며
이를 안내하는 명령은 현재 버전에서 사용할 수 없다. 현재 launch는 shadow 전용 2개뿐이다.

## 통합 이관 순서

[L0–L8 작업표](../../src/asmc_lidar/docs/migration-tasks.md)가 유일한 실행 순서 정본이다.
L0 문서 → L1 구조·계약·기준선 → L2 검출 → L3 좌표·시간·추적 → L4 전방 fusion →
L5 인지 필터 → L6 발행·실행·시각화 → L7 대조·판제 전환 → L8 개선 우선순위로 진행한다.
각 단위는 별도 계획·구현·검증·커밋으로 닫는다. L1 결과를 확인한 뒤 다음은 **L2만** 구체화한다.

아래 모드 없는 구조물 분리·참고 구현 제안은 과거 인계의 후속 연구다. 이관 필수조건으로
모두 확장하지 않는다. 기존 객체 보존 회귀는 재사용하고 새 구조물 표현·억제는 L8에서
별도 선택한다. 기존 실패를 해결된 것으로 표시하거나 전체 승격 기준을 생략하지 않는다.

## 입력·출력과 처리 책임

[소비 계약](../contracts/lidar-planning-consumer-v1.md)과
[기계 판독 계약](../../config/contracts/lidar_candidate_topic_contract_v1.yaml)은
후속 구현의 목표다. 현재 publisher 목록이 아니다. shadow 노드의 `/asmc_lidar/shadow/*`는 이 계약의 토픽이 아니다.

- 현재 Main의 local 입력은 관측 시점 유효 IMU와 차량 속도(CAN)다. 각각 age 120ms, 최대 대기 50ms이며 GPS는 기다리지 않는다. CAN 없이 처리하는 LiDAR+IMU 전용 정책은 후속 설계다.
- scan stamp는 bridge 수신 시각이며 광선 측정 시각으로 바꾸어 설명하지 않는다.
  CPU 처리시간은 monotonic clock으로 별도 측정한다.
- 코어의 엄격 `PoseHistory` API는 동일 clock에서 최대 100ms 보간하며 외삽하지 않는다.
  Main의 별도 `BoundedPoseHistory`는 GPS 최대 age 300ms의 제한 보정과 정상 3회 복구를 사용한다.
  공통 경로는 ROS source ns와 steady 수신·처리 시각을 분리한다. 두 정책을 혼용하지 않는다.
  안테나와 후륜축 offset, HD맵 원점·높이, 복구 위치 점프는 ROS 어댑터에서 검증한다.
- 절대속도 벡터는 축만 회전한다. 상대 위치 미분은 자차 병진/회전을 함께 사용한다.
  twist가 없으면 unknown이며, 0속도나 static으로 위장하지 않는다.
- 객체/구조물 경계/미확정을 구분한다. 평평함·긴 OBB·낮은 점 수·카메라 미검출만으로
  삭제하지 않는다. 인접 차로·횡단 객체와 카메라 drivable의 장애물 구멍을 보존한다.
- 주행 관련성·차로 귀속은 속성으로 전달한다. 추종 대상·기동 유지·회피·정지는 판제 책임이다.

## 모드 없는 구조물 분리 — 손재호의 후속 구현 계약

2026-10-05 추가 인계 사항이다. **2026-10-22 분류: L8 후속 개선 후보이며 이관 선행조건이 아니다.**
아래 단계·자료구조·진단은 설계 목표이며 현재 골격에
구현돼 있지 않다. 건물/터널이라는 의미 클래스를 반드시 맞히는 것이 아니라, 독립 객체와
관측된 구조물 경계를 구분해 벽 전체에 차량용 OBB를 씌우는 문제를 줄이는 것이 목표다.

```text
LiDAR + IMU → 좌표·시간 정합 → 공통 ROI/voxel·지면 처리
  → 구조물 근거 추출 → 경계·돌출부를 고려한 군집 → 혼합 군집 검사
    ├─ 객체 후보: OBB·추적 (의미 클래스/속도는 미확정 가능)
    ├─ 구조물 경계: 실제 지지 범위의 표면·경계와 품질
    └─ 미확정: 관측점·점유 근거 보존, 억제하지 않음
```

같은 단계·설정을 일반도로→터널→복귀에 적용한다. GPS/미션 ID는 분기 조건이 아니며
GPS는 map 변환/지도 문맥의 가용성에만 관여한다. 현재 Main은 LiDAR+IMU+차량 속도로
local을 처리한다. CAN 없는 LiDAR+IMU 경로의 추적·속도 정책은 후속 설계다.
IMU나 지면 근거가 무효이면 관련 구조물 판단도 미확정으로 내리고 실패 이유를 남긴다.

### 근거 추출과 군집 경계

| 사용할 관측 근거 | 반드시 함께 확인할 제한 |
|---|---|
| 중력 대비 법선, 평면 잔차, 연결된 지지점 | 차량/트럭 측면도 평평하다. 평면이라는 이유만으로 구조물로 확정하지 않는다. |
| 실제 지지 범위·연결성·높이 | 벽을 관측 밖까지 무한 연장하지 않는다. 가드레일은 얇고 끊긴 관측일 수 있다. |
| 표면 앞 돌출부·깊이 단절 | 벽 근처 차량/보행자/낮은 물체를 별도 후보로 보존한다. 희소 관측은 삭제 근거가 아니다. |
| 지면 대비 천장 높이·차량 통과 공간 | 검증된 차량 외형/자세와 여유 높이가 필요하다. 낮은 천장·보·돌출물은 충돌 위험으로 남긴다. |
| 반복 관측의 일관성 | 자차 운동·시간 대응이 유효한 경우만 사용한다. 정지 차량도 정적이므로 정적=구조물이 아니다. |

먼저 고정한 기존 군집 결과에 구조물 근거를 겹쳐 혼합 원인을 기록하고, 점 연결을 변경하는
단계는 별도 비교한다. 벽과 차량이 섞인 군집의 큰 OBB를 마지막에 삭제하는 방식은 금지한다.
군집 전 경계 근거와 군집 후 혼합 검사를 연결하되, 분리할 근거가 부족하면 미확정으로 보존한다.
OBB 크기 제한이나 지면 임계값을 동시에 조정해 구조물 분리 효과를 숨기지 않는다.

HD맵/카메라는 선택 문맥이다. frame·stamp·유효 범위·출처·품질을 검사한 뒤에만 보강한다.
누락/stale/잘못된 pose, 카메라 미검출 및 drivable의 장애물 구멍은 억제를 강화하는 근거가 아니다.
현재 경로 밖인 인접 차로 차량·진입/횡단 가능 객체도 남긴다.

### 결과와 진단의 설계 요구

- scan 식별자·관측 stamp·내부 수평화 행렬·출력 frame과 원본점 인덱스 계보를 보존한다.
  voxel 하나와 원본 여러 점의 대응, 삭제/보존 이유를 추적할 수 있어야 한다.
- 구조물 근거에는 지지점 인덱스, 법선/평면 또는 경계 형상, 실제 관측 범위, 잔차·지지량,
  품질/유효성·판정 이유를 둔다. 점 수만으로 보정되지 않은 확률 confidence를 만들지 않는다.
- 구조물 근거 마스크와 실제 억제 마스크는 분리한다. 객체 보호 근거와 충돌하면 억제하지
  않고 미확정 사유를 남긴다. **새 구조물 억제 OFF에서는 기존 비지면 점의 추가 삭제가 0건**이어야 한다.
- 객체 OBB와 구조물 표면/경계, 미확정 점유는 역할이 다르다. 구조물은 객체 추적에서
  제외할 수 있어도 충돌 경계에서 지우면 안 된다. 빈 객체 배열은 자유공간 보장이 아니다.
- 현재 `LidarObject*`는 구조물 경계용 메시지가 아니다. 구조물 형상을 객체 corner나
  classification 필드에 억지로 넣지 않는다. 후속 구현에서 별도 additive/versioned 계약을
  판제와 합의하며, 그 전에는 내부 결과와 검토용 점군/MarkerArray/진단으로 확인한다.
  시각화만으로 판제의 충돌 경계 소비가 구현됐다고 판단하지 않는다.

### 작은 단위의 분석·구현·승격 순서

아래는 L8에서 구조물 개선을 선택할 경우의 내부 순서이며 L0–L7의 실행 순서를 대체하지 않는다.

1. 원본 SHA·설정·입력을 고정하고 부적절한 OBB가 생기는 점→군집→객체 경로를 기록한다.
2. 구조물 근거 추출/시각화만 추가한다. 억제 OFF로 기존 검출 결과와 점 보존 parity를 확인한다.
3. 객체 보호·혼합 군집 분리를 한 단위씩 비교한다. 지면/군집/추적 변경은 서로 다른 실험으로 둔다.
4. 객체 보존과 경계 전달을 함께 검증한 뒤에만 억제 ON을 검토한다. 기본값 승격은 구현 담당과
   판제 담당자의 별도 수락 사항이며, 실패 시 억제 OFF와 고정 기준선으로 되돌린다.

| 고정 회귀 입력 | 비교할 실패와 성공 근거 |
|---|---|
| 일반도로 가드레일·건물, 터널 바닥/벽/천장 | 단일 경로 유지, 구조물 지지 범위, 혼합 군집/대형 OBB 변화 |
| 트럭 측면·정차 차량·벽 근처 보행자/장애물 | 객체점/관측 하부 보존, 추가 완전 소실, 차량을 벽으로 잘못 억제하는지 |
| 낮은 장애물·낮은 천장/보·가드레일 옆 객체 | 충돌 공간 안의 관측이 보존되는지, 경계가 무한 확장되지 않는지 |
| 희소/부분 가림·곡면/단차·인접/횡단 객체 | 미확정 보존, 과분할/합침, 현재 경로 밖 객체를 삭제하지 않는지 |
| 동일 scan/IMU에서 GPS·카메라/지도 가용성 변경 | GPS만 바꾸면 local geometry 동일, 문맥 누락이 추가 삭제를 만들지 않음 |

정량 gate는 실험 전에 고정한다. 원본점/객체 보존과 최종 객체 품질을 별도로 보고하고,
필터가 붙인 구조물 분류를 정답으로 사용하지 않는다. 정답 없는 실제 bag에서는 대표
BEV/측면·군집 대응·평면 선택·판정 이유를 검토한다. 시간 안정성은 위치 대응이 확인된
재관측에서만 평가한다. 15Hz 입력의 처리율·누락·단계별 p50/p95/p99와 설정/소스 SHA를 남긴다.
RViz는 객체 OBB/ID, 구조물 지지 범위/법선, 미확정점, 억제 전후, 상태를 각각 토글하고
stale 화면을 지운다. 이는 구현 후 수락 항목이며 현재 실행 가능한 시각화 목록이 아니다.

### 참고 구현 검수와 현실적인 첫 제안 — 2026-10-05

이번 검수는 **공식 문서/논문·선택한 주요 코드의 읽기 검토**다. 새 후보의 빌드·bag 실험·
실시간 성능 검증은 하지 않았다. 아래 우선순위는 우리 계약에 맞춘 설계 제안이지 검증된
성능 순위가 아니다. external은 개인 워크스페이스의 참고 자료이며 팀 런타임 의존성이 아니다.
이번에 clone/vendor/의존성 추가를 하지 않았고 Patchwork++·linefit 연구 코드를 복구하지 않았다.

| 우선순위·참고 | 참고할 기능 | 우리 입력에서의 제한·적용 판단 |
|---|---|---|
| **1. PCL 평면 지지점 + 연결성** | `SACSegmentation`의 평면 계수와 inlier index, `RegionGrowing`의 법선/곡률 기반 표면 분리 | 기존 PCL 처리와 연결하기 쉬운 첫 후보. 국소 지지 범위를 추출할 뿐 벽/차량 의미를 보장하지 않는다. 희소 VLP16 법선이 불안정하면 unknown으로 두고, 전역 최대 평면을 통째로 삭제하지 않는다. |
| **2. PCL Conditional Euclidean Clustering** | 거리 외에 사용자 정의 점 연결 조건을 적용, 작은/큰 군집도 별도로 회수 가능 | 근거 추출 검증 후 군집 단계만 별도 비교. 구조물 경계/깊이 단절을 연결 조건에 반영할 수 있으나 과분할을 유발할 수 있다. 점 수/크기로 빠진 군집도 계보와 미확정 결과에 보존한다. |
| **조건부 대조군: depth_clustering (IROS 2016)** | range-image에서 빔 간 각도와 거리로 연결성을 판단하는 희소 점군 군집화 | 논문에 VLP16 평가가 있으나 구조물 의미 분류기는 아니다. 비스듬한 벽의 과분할 한계가 명시돼 있다. 센서 원점 투영·빈 셀·셀 충돌·원본점 계보 검증을 먼저 통과해야 한다. |

공식 구현 근거:
[PCL 평면 분할](https://pointclouds.org/documentation/tutorials/planar_segmentation.html),
[Region growing](https://pointclouds.org/documentation/tutorials/region_growing_segmentation.html),
[Conditional Euclidean Clustering](https://pointclouds.org/documentation/tutorials/conditional_euclidean_clustering.html).
이들은 구현 부품이지 구조물 자동 삭제를 완성한 알고리즘이 아니다. 튜토리얼의 거리/점수
기본값을 그대로 사용하지 않으며, 실제 구현 때 Noetic 컨테이너의 PCL 버전·설정을 고정한다.

논문 **Fast Range Image-Based Segmentation of Sparse 3D Laser Scans for Online Operation**
(Bogoslavskyi·Stachniss, IROS 2016)의
[원문](https://www.ipb.uni-bonn.de/pdfs/bogoslavskyi16iros.pdf)과
[공식 코드](https://github.com/PRBonn/depth_clustering)를 확인했다. 로컬
`external/baselines/depth_clustering`은 `2a824e728725be635eec4cc5214350faea364712`,
`LICENCE.txt`는 MIT이며 작업트리가 깨끗했다. `angle_diff.cpp`의 연결각 계산과
`spherical_projection.cpp`의 점 인덱스 저장을 확인했다. 특히 후자는 “가까운 점”이라는
주석과 달리 `current_written_depth < dist_to_sensor`일 때 갱신해 더 먼 깊이를 선택한다.
따라서 그대로 복사하지 말고 다중점 셀의 대표 깊이/라벨 복원 회귀를 먼저 추가해야 한다.
현재 CMake는 OpenCV/Eigen/Boost 및 Qt/QGLViewer/OpenGL을 요구하므로 코어만 격리할 때
GUI 의존성을 따로 검토한다. 논문의 FPS를 우리 전체 처리시간으로 환산하지 않는다.

후순위 또는 이번 목적과 다른 자료도 구분한다.

- **TRAVEL**: 기존 `external/baselines/TRAVEL`의 `95dc2fbd66a343efd9060c45a5711b6307a950a4`
  (작업트리 깨끗함)과 [공식 레포](https://github.com/url-kaist/TRAVEL)의 README를 확인했다.
  지면/군집 코어가 분리된 참고 구현이지만 이전 미채택 결론을 뒤집거나 재실험하지 않는다.
  군집 분리 자체가 구조물 의미 판정을 대신하지 않는다.
- **M-detector**: `external/baselines/M-detector`의
  `0645dabe307a99f3b59a5a3d3007acfdcad1a39f` (작업트리 깨끗함), README/설정과
  LICENSE의 GPL v2 표기를 확인했다. [공식 레포](https://github.com/hku-mars/M-detector)는
  움직임 검출과 odometry 입력을 다룬다. 정차 차량/정적 장애물과 벽을 구별하는 문제를
  해결하지 않으므로 첫 단계로 채택하지 않는다. 시간·운동/배포 의존성 검토도 선행해야 한다.
- **RangeNet++ (IROS 2019)**: [논문](https://www.ipb.uni-bonn.de/pdfs/milioto2019iros.pdf)과
  [공식 추론 레포](https://github.com/PRBonn/rangenet_lib)를 온라인으로만 확인했다.
  학습 기반 점별 의미 분류의 참고 대상이지 현재 SOTA라고 선정한 것이 아니다. 우리 VLP16/
  MORAI에서의 라벨 대응·도메인 차이·GPU 통합 부하가 미검증이며, weights/license와
  revision도 미동결이다. 이번에는 clone/학습/추론하지 않고 후순위로 둔다.

**당시 재호에게 제안한 첫 연구 실험은 1번의 관측 전용 구조물 근거 추출이다.**
현재 다음 구현 계획은 L2이며 아래 연구 제안의 재개 여부는 L8에서 결정한다. 기존 지면·
군집·OBB를 고정하고 동일 scan에서 유한한 평면 지지 범위와 돌출부/미확정만 겹쳐 표시한다.
억제 OFF의 점/객체 결과 동일성, 벽과 트럭 측면을 혼동한 사례, 지지 범위의 누락과 계산
비용을 먼저 기록한다. 가드레일/곡면의 근거가 부족하면 미확정으로 남긴다. 그 결과를 보고
2번의 연결 조건 변경을 별도 실험하며 둘을 한꺼번에 적용하지 않는다. 근거 분리 성능이
확인되기 전에는 새로운 자동 억제나 판제 입력 전환을 하지 않는다.

## 수락과 기존 패키지 삭제 조건

| 대상 | 확인할 증거 |
|---|---|
| 원본 기능 | 동일 입력의 지면/군집/OBB·ID/연관·생애주기·분류 유지, 의도적 차이 목록 |
| 좌표·시간 | 비영점 roll/pitch/yaw·장착 offset·8코너·속도, stamp 영점/역행, 미래 pose 제외 |
| GPS | 정상/NaN/중단/stale/복구 점프에서 local 연속성·map 상태, 한 토픽 frame 고정 |
| 구조물 | 벽/가드레일/차량 측면/낮은·인접·횡단 객체, 새 억제 OFF에서 추가 삭제 없음 |
| 소비 | 기존 MD5 보존, camera·LiDAR 타입 직렬화, 세 판제 모듈과 음영 소비의 보류/freshness 처리 |
| 부하 | 입력 Hz·처리 Hz·drop·queue·p50/p95/p99·CPU/GPU/메모리, 카메라 동시 실행과 RViz 영향 |

15Hz 설정과 실제 수신/처리율을 함께 기록한다. 약 12Hz 완성 cloud를 허용한 운영 요구와
15Hz 부하 시험 결과는 각각 표기한다. 실제 검증하지 않은 주행/이동 객체 조건은 미검증으로 남긴다.
현재 골격 검증은 검출·추적 정확도나 주행 성능 승인에 해당하지 않는다.

기존 패키지 삭제는 모든 consumer 이전과 rollback 확인 후 별도 PR로 한다. rollback은
새 생산자/어댑터를 종료하고 고정한 원본 launch로 복원한다. 같은 토픽에 양쪽 생산자를
동시에 붙이지 않는다. 이번 PR은 운영 입력 전환이나 카메라 rename을 수행하지 않는다.

## 이번 골격의 검증 기록 — 2026-10-05

- 기존 build/devel을 사용하지 않은 Noetic 임시 경로에서 패키지·공용 메시지 빌드와
  전체 상위 catkin workspace 빌드가 통과했다.
- CTest 2개가 통과했다. 좌표·시간 실행파일 내부 35개 사례와 IMU 선택시험을 포함한다.
- 생성 메시지 직렬화/MD5 검사, 공용 계약 unit test 25개와 bridge profile test 2개가 통과했다.
- 문서 보존 검사는 과거 Git 이력 대조까지 통과했다. CI에서 발견한 신규 타입 등록 누락과
  기존 음영 어댑터 디렉터리 이동 후 남은 감사 경로를 교정했다. 실제 ROS 패키지명은
  `lane_boundary_adapter`이며 디렉터리만 `jamming_lane_boundary_adapter`다.
- 이 골격에는 runtime publisher가 없으므로 시뮬레이터 검출·주행 수락은 수행 대상이 아니다.
  손재호의 기능 이전 후 위 수락 목록으로 검증한다.

## 연구 이력 조회

사용자 요청으로 개인 연구 코드·설정·비교 그림·보고서를 현재 트리에서 삭제했다.
정리 전 전체 내용은 `020f6c2`, 원래 연구 worktree의 추가 GT 수집 변경은 `f1dfb84`에서
조회할 수 있다. Git 이력을 재작성하지 않았으며 재호 작업 브랜치와 외부 연구 checkout은
그대로다. 실제 bag·Capture 자료는 `$ASMC_DATA`에 유지한다.

```bash
git show 020f6c2:src/asmc_lidar/src/ground_live.cpp
git log --all -- src/asmc_lidar
```

복구가 필요하면 별도 worktree에서 검토하고 현재 골격에 연구 폴더를 일괄 복사하지 않는다.
