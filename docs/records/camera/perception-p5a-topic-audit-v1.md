# P5-A perception topic·시각화 감사 v1

> **상태:** Frozen input audit for P5-B
>
> **기준 commit:** `64cd315f200a9902c9e61327f2eb97ccbded0e36`
>
> **기계 정본:** [`perception_topic_policy_v1.yaml`](../../../config/contracts/perception_topic_policy_v1.yaml)

## 1. 목적과 변경 경계

P4-C6 통합 뒤 perception runtime이 발행하는 semantic, BEV, lane, traffic-light 토픽과
선택 시각화 토픽을 publisher, subscriber, bag profile 기준으로 다시 분류했다. 이름만 보고
`debug` 또는 `grid` 토픽을 끄면 실제 fusion·lane 입력을 끊을 수 있으므로 P5-B 구현 전에
현재 연결을 동결한다.

이 감사에서는 runtime code, topic 이름, launch 기본값과 bag profile을 바꾸지 않았다.
`asmc_bridge`는 계속 별도 foreground process이고, planning 소유 `lane_boundary_adapter`와
미완성 LiDAR/fusion도 변경 범위 밖이다.

## 2. 결과

| 항목 | 값 |
|---|---:|
| 분류한 topic family | 24 |
| `{view}`를 front/left/right로 펼친 topic | 63 |
| 대조한 data collection bag profile | 11 |
| 이름과 달리 반드시 유지할 내부 transport family | 2 |
| P5-B 즉시 default-off 후보 | 2 |

### 끄면 안 되는 `debug`·`grid`

- `/perception/bev/debug/{view}/drivable_probability`
- `/perception/bev/debug/{view}/road_marking/class_id`
- `/perception/bev/debug/{view}/road_marking/confidence`

위 세 계층은 `camera_bev_fusion`의 실제 입력이다. namespace가 잘못 분류돼 있을 뿐 단순
시각화가 아니다. in-process fusion 또는 새 internal topic으로 payload/timestamp parity를
확인하기 전에는 끄거나 이름을 바꾸지 않는다.

- `/perception/camera/{view}/road_marking/model_grid/class_id`
- `/perception/camera/{view}/road_marking/model_grid/confidence`

이 둘도 화면 표시용 grid가 아니라 lane vectorizer가 원 모델 해상도에서 직접 소비하는 입력이다.
full-resolution semantic BEV와 목적이 달라 즉시 중복 제거 대상이 아니다.

### P5-B default-off 후보

| family | 근거 | 후속 처리 |
|---|---|---|
| per-view projected `coverage` | fusion은 구독하지 않고 선택 per-view mosaic만 사용 | 명시적 debug flag로 전환 |
| fused `source_view` | committed runtime consumer와 bag profile이 모두 없음 | 명시적 debug flag로 전환 |

`/perception/lane/pipeline_audit`는 평가 bag 다섯 종류가 사용하지만 frame마다 큰
`DiagnosticArray`를 발행한다. 삭제하지 않고 production 기본 off, result/evaluation 실행에서
명시적으로 on 하는 것이 다음 gate다.

### 이미 비용이 제한된 출력

- `start_visualization` 기본값은 `false`다.
- semantic panel/mosaic, BEV debug image, lane debug image는 visualization launch를 켜야 node가
  생기며 실제 viewer subscriber가 없으면 렌더링·JPEG 직렬화를 건너뛴다.
- lane evidence PointCloud, candidate Marker와 legacy boundary Marker도 subscriber가 있을 때만
  직렬화한다.
- `lane_boundary_family`와 shared-family 계산은 명시적 launch argument가 없으면 실행하지 않는다.

따라서 P5-B는 publisher 수만 보고 제거하지 않고 항상 **계산 여부와 wire serialization 여부**를
따로 확인한다.

## 3. P5-B 확정 순서

1. per-view coverage, source-view와 frame-level pipeline audit에 publish flag를 추가한다.
2. production launch에서는 기본 off, evaluation/bag 절차에서는 명시적으로 on 한다.
3. 원본/투영 semantic BEV와 최종 fused perception BEV를 좌우로 비교하는 subscriber-lazy 단일
   image topic을 추가한다.
4. 일반 visualization preset에서 camera grid overlay를 빼고 calibration-review 경로에만 남긴다.
5. 기존 fused/fused-quality/lane debug image는 한 호환 기간 동안 유지하고 source stamp와 pixel
   parity를 확인한 뒤에만 retire한다.

`source_count`는 현재 planning subscriber는 없지만 overlap 품질과 handoff bag에서 사용한다.
판제 소비 여부를 합의하기 전에는 default-off 대상으로 확정하지 않았다. traffic-light legacy
String도 모든 consumer가 typed topic으로 전환되기 전에는 유지한다.

## 4. 재검증

```bash
python3 tools/audit/validate_perception_topic_policy.py
python3 -m unittest tools/audit/test_perception_topic_policy.py
```

validator는 family 누락·중복, producer/consumer source anchor, bag profile 사용, 필수 내부
transport 오분류, planning 소유 경계와 P5-B 순서를 함께 검사한다. runtime publisher나 bag
profile을 변경하는 PR은 이 policy도 같은 PR에서 검토해야 한다.
