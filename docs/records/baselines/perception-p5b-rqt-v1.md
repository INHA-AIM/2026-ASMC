# P5-B perception rqt 기준 v1

> **상태:** 구현·ROS Noetic build·MORAI live smoke·사용자 rqt 육안 검수 통과
>
> **범위:** rqt용 compressed image publisher. RViz와 planning adapter는 변경하지 않음

## 기본 화면

`start_visualization:=true`의 기본 rqt 출력은 다음 두 개다.

| Topic | 내용 | 생성 정책 |
|---|---|---|
| `/perception/camera/debug/mosaic/compressed` | 좌·전방·우 × 원본·주행가능·road-marking 9칸 | subscriber-lazy |
| `/perception/bev/debug/comparison/compressed` | 같은 metric 범위의 원본 RGB BEV와 fused semantic BEV 좌우 비교 | subscriber-lazy |

BEV 비교의 RGB는 각 camera 원본을 운영 calibration으로 투영한 뒤 fusion과 같은 정적
geometry 품질 가중치로 합성한다. 결과 timestamp에 가장 가까운 원본 frame을 짧은
history에서 고르며, 허용 timestamp 차이를 넘기면 잘못 맞춘 화면 대신 해당 frame을
거부한다. semantic 쪽은 주행가능 확률을 0.5에서 이진화하고 차선 class를 고정 색으로
표시해 확률·confidence 밝기를 기본 화면에 인코딩하지 않는다.

## 기본에서 제외한 화면

- camera별 개별 panel
- 기존 fused probability와 fused quality/coverage 영상
- lane evidence, fitted, boundary, shared-family, camera reprojection 영상
- camera별 calibration grid overlay
- per-view projected coverage와 fused source count/view metadata

앞의 세 묶음은 한 호환 기간 동안 `visualization_legacy_rqt:=true`로 다시 켤 수 있다.
calibration grid는 `visualization_calibration_grid:=true`로 별도 활성화한다. 이전 영상이
토픽 목록에는 있지만 frame이 안 나오던 현상 대부분은 subscriber가 없을 때 비용을 쓰지
않는 의도된 lazy 동작이었다.

coverage와 source count/view는 `publish_bev_debug_metadata:=true`로 독립 활성화할 수
있고 legacy rqt를 켜면 호환 기본값으로 함께 켜진다. 일반 comparison은 이 metadata를
구독하지 않는다.

`road_marking/model_grid/*`는 이름에 grid가 있어도 화면용 토픽이 아니다. lane vectorizer의
필수 내부 입력이므로 이 rqt 정리에서 끄거나 이름을 바꾸지 않았다. 마찬가지로
`/perception/bev/debug/{view}/*` 중 semantic raster는 camera fusion 입력이라 유지한다.

## live 검증

- 기존 perception runtime은 유지하고 `/p5b_rqt_smoke` namespace에 새 visualizer만 격리 실행
- 기본 semantic visualizer publication: camera mosaic와 `/rosout`만 확인
- 기본 BEV visualizer publication: BEV comparison과 `/rosout`만 확인
- comparison 구독 시 관측: 약 `5.0 Hz`
- JPEG 해상도: `1344x1050` (`672x1050` 두 패널)
- 원본 RGB와 semantic 결과의 방향·metric crop 일치 확인
- 구독자가 없을 때 semantic mosaic render count가 0인 lazy 동작 확인

검증 명령과 opt-in 옵션의 정본은 [실행 명령어 모음](../../runtime-commands.md)에 있다.

## P5-B runtime closeout

per-view coverage와 fused source count/view의 production default-off까지 적용했다.
P5-A에서 지정한 frame-level `/perception/lane/pipeline_audit`도
`lane_publish_pipeline_audit:=true`에서만 광고한다. 기본 실행에서는 별도 projection
selection audit, 누적 집계와 큰 frame DiagnosticArray 직렬화를 건너뛴다. 정상 lane
boundary·diagnostics 출력과 핵심 알고리즘은 유지한다.

`run_shadow_lane_bag_batch.py`는 상세 rejection 분석이 필요한 기존 result/evaluation
profile을 위해 audit와 BEV debug metadata를 명시적으로 다시 켠다. 따라서 production
기본값 축소와 평가 bag 계약을 분리했다.

Closeout 검증은 ROS Noetic 전체 catkin build와 관련 회귀 테스트 48개를 통과했다. 운영
graph와 분리한 임시 ROS master에서 같은 lane vectorizer를 두 번 실행해 기본값에서는
`/perception/lane/pipeline_audit`가 광고되지 않고, opt-in에서는 광고되는 것도 확인했다.
