# LiDAR 기능 이전과 손재호 인계

현재 `asmc_lidar`는 **좌표·시간 기반과 공용 계약만 제공하는 빌드 가능한 골격**이다.
검출·군집·OBB·추적·fusion·ROS 어댑터의 실행 구현은 손재호가 원본
[`lidar+sensorfusion`](../../../src/lidar+sensorfusion/RUN_SENSOR_FUSION.md)에서 이전한다.
기존 원본 패키지가 현재 주행 경로이며, 골격은 객체 토픽을 발행하지 않는다.

## 역할과 남긴 파일

카메라 목표 패키지는 `asmc_camera`(현재 이름 `asmc_perception`)다. 안승현이 이름 변경과
YOLO 생산자 이전·모델/통합 부하 검증을 맡는다. 손재호는 `asmc_lidar`의 LiDAR 처리와
카메라 box 매칭·클래스 유지 fusion을 맡는다. HD맵은 지도와 주행영역을 제공한다.
자세한 역할 분담은 [통합 로드맵](../../architecture/consolidation-roadmap.md)을 따른다.

| 제공한 구성 | 사용할 때의 조건 |
|---|---|
| `asmc_lidar_frame_time` 라이브러리 | 실제 수평화 행렬·센서 extrinsic·유효 pose를 호출자가 제공 |
| IMU 선택 함수 | 도착 완료 표본만 전달, 유한값/유효 quaternion 검사 후 valid 지정 |
| v3 장착 설정 | 출처 SHA와 단위 확인. 장착 z를 노면 기준 센서 높이로 사용하지 않음 |
| 회귀시험 | 좌표·8코너·속도·관측시각·pose/IMU 선택 의미 검사. 실제 검출 성능 시험은 후속 |
| 공용 `asmc_msgs` 타입 | LidarObject/Array, CameraObjectBox/Array 정의와 직렬화 검사. 생산자 구현은 아직 없음 |

빌드·시험 명령과 API 위치는 [패키지 README](../../../src/asmc_lidar/README.md)에 있다.
소스에는 지면·tracker 후보나 실행 launch를 남기지 않았다. 이전 `unified.launch`,
`ground_compare_live.launch`를 안내하는 명령은 현재 버전에서 사용할 수 없다.

## 손재호의 이전 순서

1. **원본 기준선 고정:** 실제 작업 브랜치 SHA·launch 인자·센서 SHA·입력 bag·기존
   message MD5를 남긴다. `Main`의 처리 단계, tracker 연관/ID 생애주기/OBB 안정화,
   동정적 판단과 camera 분류 유지 정책을 목록으로 만든다.
2. **동작 보존 이식:** 필요한 소스만 `asmc_lidar`로 옮겨 원본과 같은 입력으로 비교한다.
   새 파일 배치는 `src/ground`, `src/structure`, `src/clustering`, `src/tracking`,
   `src/fusion`, `src/ros` 등의 책임별 구조를 권장하되 실제 구현 때 만든다.
   원본 CA tracker와 임계값을 유지하고 물리 이동과 알고리즘 변경을 별도 커밋으로 한다.
3. **좌표·시간 적용:** local은 후륜축 `base_link`, map은 관측시점 유효 pose로만 변환한다.
   중심·8코너·방향축·속도에 같은 변환을 적용하고 원본 stamp를 보존한다.
   수평화 역변환과 장착 변환을 각각 한 번 적용한다. 좌표 변경으로 생긴 의도적 차이는
   원인별로 기록한다. `frame_time`은 수평화·deskew·추적기를 대신하지 않는다.
4. **단일 처리 경로:** GPS/미션으로 일반/터널 알고리즘을 전환하거나 tracker를 reset하지
   않는다. 기존 천장·벽 처리를 관측 근거로 재구성하고 정상→음영→복귀 동일 단계 회귀를
   남긴다. 무효 지면/문맥은 unknown이며 오래된 평면을 무조건 재사용하지 않는다.
5. **지면·구조물 보완:** 터널 천장을 지면으로 선택하는 경우를 평면 높이·법선·실제
   지지 영역으로 검사한다. 지면 보완은 동일 scan의 근거를 사용해 별도 검증한다.
   기존 하부 삭제/군집 분할과 분리해 시험한다. 구조물 자동 억제는 OFF부터 시작하고
   벽·가드레일·차량 측면·벽 근처/낮은 장애물 보존 회귀를 통과한 후 켠다.
6. **공용 메시지·어댑터:** 아래 소비 계약으로 생산자를 구현한다. `Msg`는 선택적 legacy
   경계에만 두고 새 패키지가 nested build/source 없이 빌드되게 한다. 외부 `morai_msgs`는
   공용 메시지로 복사하지 않는다. map projection과 tracker 상태는 독립시킨다.
7. **카메라·판제 연결:** 카메라 공통 box를 받아 시각/투영 매칭·분류를 보강한다.
   camera empty/missing/stale을 구분하고 LiDAR 일반 장애물을 삭제하지 않는다.
   Behavior/Frenet/MPC·음영 어댑터 담당자가 frame·stamp·MD5·무효 상태를 함께 검증한다.
8. **인계 완료:** RViz 관측/예측 ID·속도 유효성·출력 상태, 일반도로/터널/복귀 실제 주행,
   카메라 동시 실행 부하와 누락/지연을 기록한 후 운영 전환을 합의한다.

검증된 수정사항이라도 원본에 자동 적용됐다고 가정하지 않는다. 기존 연구 구현은 필요할 때
정리 전 Git 커밋에서 확인하고, source SHA·설정·같은 입력의 전후 결과를 붙여 선별 적용한다.

## 입력·출력과 처리 책임

[소비 계약](../../contracts/perception/lidar-planning-consumer-v1.md)과
[기계 판독 계약](../../../config/contracts/lidar_candidate_topic_contract_v1.yaml)은
후속 구현의 목표다. 현재 publisher 목록이 아니다.

- local 객체는 LiDAR+IMU로 계속 처리한다. GPS는 map 가용성에만 사용한다.
- scan stamp는 bridge 수신 시각이며 광선 측정 시각으로 바꾸어 설명하지 않는다.
  CPU 처리시간은 monotonic clock으로 별도 측정한다.
- pose는 이미 도착한 동일 clock의 유효 표본으로 최대 100ms 보간하고 외삽하지 않는다.
  안테나와 후륜축 offset, HD맵 원점·높이, 복구 위치 점프는 ROS 어댑터에서 검증한다.
- 절대속도 벡터는 축만 회전한다. 상대 위치 미분은 자차 병진/회전을 함께 사용한다.
  twist가 없으면 unknown이며, 0속도나 static으로 위장하지 않는다.
- 객체/구조물 경계/미확정을 구분한다. 평평함·긴 OBB·낮은 점 수·카메라 미검출만으로
  삭제하지 않는다. 인접 차로·횡단 객체와 카메라 drivable의 장애물 구멍을 보존한다.
- 주행 관련성·차로 귀속은 속성으로 전달한다. 추종 대상·기동 유지·회피·정지는 판제 책임이다.

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
