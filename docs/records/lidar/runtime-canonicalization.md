# LiDAR 실행 정본화 — B4

이 기록은 PR #111 기준 B4 비교를 보존한다. PR #114를 반영한 현재 타입·실행과 추가 검증은
[최신 통합 기록](2026-10-26-pr114-canonical-integration.md)을 따른다. 아래 당시 수치/MD5·동등성은 새 KF 판제 경로의 검증으로 확대하지 않는다.

2026-10-26. 기준선은 PR #111 병합 main `b2e8329f1cb71f11150d5125335e01c5b8d252c5`다.
새 LiDAR를 선택한 현재 실행 결과를 유지하고 판제·수집·시나리오에서 구 Main/Msg 경로를 폐기한다.

원본 파일 SHA와 역할별 보존 결정은 [기준선 manifest](../../../config/baselines/lidar_canonicalization_v1.json)에 있다.
구 실행 재현은 해당 revision의 별도 checkout에서 수행한다. bag·GT·실험 자료는 삭제하지 않는다.

## 구현 순서

1. 정본 계약·삭제 전 소스/시험 근거 고정.
2. 공용 추적 관측 입력 단일화와 책임별 내부 명칭 정리.
3. 수집·시나리오·속도 분석 및 유효한 회귀 사례 전환.
4. 구 패키지·전용 의존성·CI 폐기와 클린 빌드/출력 대조.

## 최종 실행과 보존 범위

판제·수집·시나리오는 `/perception/lidar/tracks_map`의 `asmc_msgs/LidarTrackingObservation`을
직접 소비한다. `lidar_source_policy`, `main_raw`, `validated_lidar`, 구 wire bridge와 motion JSON
결합을 제거했다. 판제의 private `DetectedObjectArray`는 그대로 유지하며 입력 어댑터는
타입·좌표·시각·순서·유효성을 검사하고 변환한다. TTC·추종·회피·정지·장애 대응은 변경하지 않았다.

구 `lidar+sensorfusion`의 추적 파일375개와 과거 실행기/하네스33개를 현행 checkout에서 제거했다.
각 원본 SHA·현재 대응 구현·미이관 이유를 manifest에 기록했다. 구 도로 mask·시설물 목록·사각 구조물
blacklist·OBB clipping은 현재 HD맵/점군 근거/높이 전달 판정으로 대체한다. 관측 확인3/해제5,
지면 근거·근접 보존·운동 상태 계산·실험 터널 필터는 현재 LiDAR 구현에 유지한다.
구 HSV 인지·UDP7510 객체 송신·LiDAR 차량 제어·중복 센서 패키지는 다시 가져오지 않았다.

실행은 `lidar.launch`, 웹은 `view.launch`, 오프라인은 `lidar_replay`다. 내부 `motion_display`를
`motion_classifier`로 바꾸고 외부 공개 라이브러리의 범위는 유지했다. 일반 빌드에서 필수 처리 노드가
조용히 생략되지 않는다. 메시지 필드·MD5·알고리즘·필터 임계값·`lidar_pipeline.yaml` 바이트는 유지했다.
LiDAR 생산 C++ 소스/헤더73개는 명시한 namespace/member/include 이름 치환 후 기준선과 동일하다.

수집·시나리오는 outer workspace만 source한다. 속도 분석기는 공용 관측의 실제 수치·validity를
읽으며 Marker 문자열이나 JSON을 다시 결합하지 않는다. 기존 저장 결과를 읽는 분석과 역사적
bag schema4는 보존하고, 새 신호등 시험 기록은 atomic 관측을 포함하는 schema5를 사용한다.

## 고정 bag 출력 동등성

고유 run은 `lidar-b4-canonicalization-20261026-r1`이다. 실제 출력은 등록한
`D:/morai dataset/shahn03/artifacts/lidar-b4-canonicalization-20261026-r1`에 보관한다.
입력 bag은 복사/수정하지 않았고 전체 SHA를 확인했다.

| 입력 | 크기 | 고유 scan | SHA256 |
|---|---:|---:|---|
| `lidar_manual_20260929_130623/raw_0.bag` | 367,087,616B | 649 | `859cbd84a1e845b76426f96be2778c6f54ea77fe348631a976ef3adf666d0c02` |
| `20260912_003803_kcity2025_shadow_static_avoid_v1_sunny_11am_wheel_0.bag` | 175,048,680B | 208 | `9bcb0e134a08c9b21cb9e8d29ca0e62710173cf9bebcc091d5dfc82454006b34` |

기준선 revision의 LiDAR/메시지를 별도 workspace에 새로 빌드했다. 같은 seed·설정·입력에서
일반649scan, 터널208scan, GPS 무효/중단 각각649scan과 debug OFF649scan×3쌍을 비교했다.
**고유857scan을 재사용한 총4,102scan 쌍에서 객체·ID·순서·기하·8코너·속도·운동 상태·분류·필터·header와
generation/config/calibration ID가 정확히 일치했다.** session UUID는 생산자마다 달라 비교에서 제외한다.
상세 결과와 자원 수치는 [결과 JSON](../../../config/baselines/lidar_canonicalization_result_v1.json)에 있다.

첫 비교기는 HD맵의 실측 `calculation_ms`와 객체별 `comparison_ms`까지 결과로 비교해 실패했다.
원본 trace·로그·실패한 `comparison.json`을 보존하고, 정확한 schema 위치의 계측 시간만 제외해
`comparison-corrected.json`으로 재판정했다. 임계값·source age·겹침 비율 등 다른 수치는 제외하지 않는다.
수정한 `compare_traces.py --same-contract`와4개 회귀 검사도 저장소에 반영했다.

## replay 자원 비교

debug OFF의3쌍을 교차 실행했다. 표는 각 실행의 percentile을 구한 뒤3회 중앙값이며 합산 percentile이 아니다.

| 항목 | 기준선 | B4 |
|---|---:|---:|
| scan 계산 p50 | 50.63ms | 51.15ms |
| scan 계산 p95 | 66.81ms | 67.28ms |
| scan 계산 p99 | 71.00ms | 73.41ms |
| 649scan CPU 시간 중앙값 | 31.160s | 31.136s |
| 3회 중 최대 RSS | 163,248KiB | 163,228KiB |

p95 증가는0.47ms이며 생산 코드73개가 명칭 치환 외에 동일하다. 새 점군 복사·큐는 추가하지 않았다.
터널 debug ON 한 쌍은 p95가89.12→93.30ms였다. 단일 실행 차이를 알고리즘 개선/악화로 단정하지 않는다.
이 수치는 오프라인 replay의 계산 시간과 process peak RSS다. ROS 전체 공급률·입력→발행 지연이나
장기 메모리 안정성의 새 수락 시험으로 사용하지 않는다.

## 빌드·계약·연결 검사

| 검사 | 결과 |
|---|---|
| 구 devel/install 없이 outer workspace 클린 빌드 | LiDAR·메시지·카메라·bridge·HD맵·판제·controller·수집·음영 어댑터 통과 |
| 분리한 install만 source한 환경 | Main/Msg 없이 LiDAR/웹/autonomy launch 해석·생성 메시지·동적 라이브러리 확인 |
| LiDAR CTest와 필수 실행/시험 gate | 34/34, 누락·생략 없음 |
| planner/controller 전체 CTest | 64/64 · 18/18 |
| 실제 공용 LiDAR ROS 연결 | 16검사 통과; 무효 pose/stale 차단·정상 빈 결과·session/세대·단일 생산자 |
| autonomy ROS 경계 | 공용 관측→`/planning/objects`·header/속도/unknown 유지·생산자 중복 없음 |
| 카메라·IMU | ground projection8개, 음영 어댑터5개 ROS fixture, YOLO baseline/all 및 segmentation fixture 통과 |
| 시나리오 | unit208개·integration112개 통과 |
| Python·문서 | CI Python90명령과 추적 추출/trace 비교6검사 통과; 링크/anchor·현행 명령·역사 manifest·동결5문서 보존 |
| 웹 | LiDAR view와 판제 운동 표시 Node20 검사 통과 |

독립 master/시험 소유 프로세스만 사용했다. 운영 ROS master·차량 제어·MORAI·날씨·사용자 노드는 변경하지 않았다.
호스트의 bare Python 전체 discover에는 PyQt/grpc가 없어 선택한 Noetic 컨테이너의 unit/integration 환경으로 검증했다.

첫 설치 명령에 잘못된 `CATKIN_INSTALL_PREFIX`를 사용해 무시된 결과, 기존 workspace의 무추적 `install/`이
갱신됐다. 당시 해당 경로를 사용하는 프로세스는 없었고 운영 노드를 재시작하지 않았다.
이후 올바른 `CMAKE_INSTALL_PREFIX=/tmp/asmc-b4-install`로 분리 설치해 install-only 검사를 통과했다.
기존 `devel/`은 이번 분리 빌드의 출력 대상이 아니며, 실제 배포 시에는 사용자가 실행을 종료한 후
[빌드·재시작 절차](../../../src/asmc_lidar/docs/planning-runtime.md#배포와-분석-도구)를 따른다.

## CI 변경과 해석

기존 check 이름은 유지한다. `validated-lidar-build`는 구 nested 빌드를 없애고 가벼운 정적 의존성 검사로
바꿨다. 실제 공용 LiDAR→판제·카메라 수신·음영 어댑터 회귀는 기본 CI의 필수 LiDAR 검증으로 모았다.
planning job은 판제/메시지만 빌드하고 AVOID·로터리·도로 제약·충돌 검사를 유지한다.
캐시와 이전 PR 실행 취소 설정은 보존했다. Hosted 성공 여부·소요 시간은 PR의 실제 Actions 결과에서
별도로 확인하며, 로컬 결과를 hosted 성공이나 시간 단축 비율로 표시하지 않는다.

첫 [Hosted CI](https://github.com/INHAautonav/2026-ASMC/actions/runs/40275847115)에서는34개 중
웹 모니터 ROS fixture 하나가 실패했다. 직접 실행 기본 토픽을 `/perception/lidar`로 바꾼 뒤,
fixture의 생산자는 `/migration/lidar`에 두고 모니터의 구독 경로는 명시하지 않은 것이 원인이었다.
시험 XML에 debug/diagnostics/objects의 시험 경로를 모두 주입해 격리를 복구했고, 로컬 ROS
7개 검사를10.93초에 통과했다. 운영 토픽 기본값이나 판정 코드는 변경하지 않았다.
이 시험 전용 후속 수정은 고정 bag run 동결 후 별도 커밋으로 남긴다. 동결된 replay 소스와 결과를
덮어쓰지 않으며 최종 hosted 상태는 [PR #112](https://github.com/INHAautonav/2026-ASMC/pull/112)에서 확인한다.

## 저장과 복구·남은 제한

시작 당시 D 여유49.52GiB였고 사용자가 **이번 고정 bag 결과 저장에 한해서**50GiB 미만 실행을 승인했다.
실제 여유·승인 범위를 run 등록에 기록했으며 공용 저장 정책은 변경하지 않았다. 새 수집·학습은 하지 않았다.
원본 입력·실패 비교·최종 비교·dirty patch·소스/설정 SHA·전체 로그를 같은 run에 보존한다.
다른 데이터·worktree·미추적 bag·기존 build/devel 잔류물은 삭제하지 않았다.

구 실행 재현/복구는 아래 고정 revision의 별도 checkout에서 수행한다. 현재 실행에 구 overlay를 source하거나
같은 정식 토픽의 생산자를 동시에 시작하지 않는다.

```bash
git worktree add --detach /tmp/asmc-lidar-pr111 b2e8329f1cb71f11150d5125335e01c5b8d252c5
```

이 구조 정리는 기존 새 경로 주행 확인과 고정 입력 회귀를 근거로 한다. 새 전체 미션 주행·속도 정확도·
3뷰 matching/fusion·GPS 없는 터널 구조물 제거의 수락은 아니다. 재호는 추적·속도·운동 상태와 터널을,
승현은 HD맵·카메라 연결 지원을 이 단일 기반에서 진행한다. 좌표·IMU 공통화와3뷰 확대는 별도 작업이다.
