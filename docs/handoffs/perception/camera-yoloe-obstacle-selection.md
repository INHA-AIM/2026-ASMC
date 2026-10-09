# YOLOE m/l・전방640/960/1280 비교

**측정 완료・운영 선정 보류.** 후속 학습 비교 후보는 YOLOE-26l-seg・전방1280・
3뷰15Hz・2+1・bollard17・confidence0.40이다. FOGGY 통합 반복의 공백/판제 deadline,
주 bag 매칭 감소와 신규 분류 차이4건은 미수락으로 남긴다. 현재 YOLO26l detect
기본값을 유지한다. [최종 비교 갤러리](http://localhost:8768/camera-yoloe-selection-v1/final-analysis-v2/index.html)와
`config/baselines/perception_camera_yoloe_selection_v1.yaml`에 원본/실패/판정을 함께 기록했다.

이 작업은 YOLOE의 추가학습 후보를 고르는 실험이며 운영 기본 모델을 바꾸지 않는다.
현재 기준선은 YOLO26l detect・전방1280・좌우640×480・3뷰15Hz・2+1 batch・confidence0.40이다.
작업 브랜치는 `feat/camera-yoloe-obstacle-selection`, worktree와 공유 컨테이너는 유지한다.
PR #76은 사용자 승인에 따라 2026-10-09 CLI로 merge commit `92553d02`에 병합했다.
병합 대상 HEAD는 `503a44ac`이며, hosted CI가 billing 제한으로 시작되지 않았다는
기록은 유지한다. CI 통과로 표시하지 않는다. 후속 PR #77은 main으로 기준을 옮기고
해당 merge commit을 작업 브랜치에 반영했다. YOLOE 운영 승격은 여전히 보류다.

## 고정 비교

`config/experiments/camera_yoloe_selection_v1.yaml`의 obstacle17을 m/l 각각의
checkpoint에서 embedding으로 만들고, 너비별 engine을 따로 만든다. 기존89/42종
embedding을 출력에서 필터링한 결과로 대신하지 않는다. head는 one-to-many,
class-aware NMS・IoU0.7・max_det300을 유지한다. 세 뷰 Ego mask는 ON이다.

| 조합 | 전방 tensor | 좌우 tensor | 기본 batch |
|---|---|---|---|
| m/l・640 |384×640|480×640|2+1|
| m/l・960 |544×960|480×640|2+1|
| m/l・1280 |736×1280|480×640|2+1|

640 batch3 대조는 모두480×640으로 padding한다. 종횡비・FOV와 원본 좌표를 유지한다.
TensorRT profile의 최소 높이는384로 확장한다. checkpoint・embedding・engine SHA와
실제 I/O dtype을 기록하며, internal FP16을 모든 I/O가 FP16이라는 뜻으로 쓰지 않는다.

## 평가 경계와 진행 상태

- GPU engine 준비는 network-none 전용 컨테이너에서 직렬 수행한다. 실제 live trial은
  `asmc-ros-noetic`에서 bridge를 유지한 채 실행하며, 공개 차량 제어는 발행하지 않는다.
- 기존 Capture의 검증된 COCO7 세부 라벨만 정량 평가한다. 새 프롬프트 라벨을 기존
  7종 GT의 배경 FP로 세지 않는다. 미확인 Vehicle/Obstacle 영역도 자동 정답으로 바꾸지 않는다.
- 신규10에셋을 SUNNY/FOGGY・전방12/30/50m・좌우7m에서 수집했다. 최초100장면・1500 RGB의 RPC 오류는0이었다. 그러나 Scooter1의10장면은 영상에서 공중 배치가 확인되어 성능 평가에서 제외하고 원본을 보존했다. 같은10장면・150 RGB를 native ground snap으로 재수집해 바퀴의 지면 접촉을 확인했다. 수치 BBox 하단 높이만으로 올바른 배치를 보장하지 않는다. 시험 actor 정리・날씨 복원과 Ego 유지도 확인했다. actor 상태는 pixel GT가 아니다.
- BabyStroller1에는 밀고 있는 사람이, Scooter1/ElectricScooter1에는 탑승자가 포함된다.
  사람과 장비 검출의 공존은 동일 물체의 유사 이름 중복과 구분한다.
- 동일 RGB 품질 비교 → 6조합 live 선별 → 최대2후보 SUNNY/FOGGY 각3회 확인 →
  필요 이름만 실제 재추론 → 고정 confidence 확인 → 전방 bbox fusion 순서다.
- 목표15Hz 미달・8–9Hz・공급 공백・OOM 등 모든 실패를 남긴다. p95≤150ms,
  p99≤200ms・최대 공백250ms・전체VRAM90% 조건과 다른 모델 공급을 함께 본다.
- engine 준비・실측 결과가 모두 끝나기 전 후보 선정이나 운영 수락을 표시하지 않는다.

원본은 `artifacts/camera-yoloe-selection-v1`과
`artifacts/camera-coco7-live-v1/yoloe-obstacles-{range,sides}-v1`에 보존한다.
[갤러리 목차](http://localhost:8768/)에서 날짜・E번호로 접근한다.
[원본 스폰 장면](http://localhost:8768/camera-yoloe-selection-v1/spawn-source-review/index.html)은
검출 품질 측정 이전의 입력 검토 자료다.

## 직접 실행

준비된 engine과 SHA 고정 설정이 있는 경우에 사용한다. 기본은 YOLO만 ACTIVE이며
TwinLite/ResNet은 아래 옵션으로 명시적으로 함께 켤 수 있다. bridge・GUI는 별도다.

```bash
roslaunch asmc_camera camera_yoloe_experiment.launch model:=m front_width:=960 prompt_profile:=obstacle17
```

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

```bash
roslaunch asmc_camera camera_yoloe_experiment.launch model:=l front_width:=1280 prompt_profile:=bollard17 \
  start_camera_lane:=true start_traffic_light:=true
```

모든 결과는 원본 프롬프트 라벨로 발행한다. 네 그룹 매핑은 평가에만 사용하며 기존
LiDAR 어댑터의 COCO7 제한은 유지한다. fusion은 전방 bbox・unique・50ms/200ms다.
3뷰 mask fusion을 구현된 기능으로 표시하지 않는다.

Roboflow SDK/CLI・22개 데이터 다운로드・라벨 감사・추가학습은 이번 범위에서 제외한다.
후속에서는 정리한 bbox로 YOLO26l detect와 선별 YOLOE detect만 짧게 학습하고,
유리한 하나를 본학습한다. 사전학습 seg 비교만으로 학습 후 성능을 단정하지 않는다.

## 동일 영상 선별 결과와 프롬프트 결정

정량 선정 자료는 Capture 2,252프레임이다. 이전 사용자 검토에서 승인된 정확히2장의
제외 결과(2,250프레임)를 아래에 함께 사용하며 원본 점수도 증거에 보존한다. GT는
person/car/bus/truck만 검증돼 있다. dog/bicycle/motorcycle과 신규 장애물에는 이 표의
recall을 적용하지 않는다. 특히 작은 원거리 GT가 많고 SUNNY216장/FOGGY2,036장으로
불균형하므로 F1을 전체 대회 정확도로 읽지 않는다.

| 모델・전방 | F1 @0.40 | TP / FP | 선정 자료 best cutoff / F1 | 최초 SUNNY C6 p95 최대 | 전체 peak VRAM |
|---|---:|---:|---:|---:|---:|
| YOLOE m・640 |0.2426|292 / 61|0.20 / 0.2533|83.1ms|5,259MiB|
| YOLOE m・960 |0.2535|308 / 92|0.20 / 0.2745|94.4ms|5,343MiB|
| YOLOE m・1280 |0.2600|319 / 96|0.15 / 0.2782|105.1ms|5,518MiB|
| YOLOE l・640 |0.2548|302 / 48|0.25 / 0.2615|92.0ms|5,284MiB|
| YOLOE l・960 |0.2659|322 / 61|0.25 / 0.2736|102.1ms|5,352MiB|
| YOLOE l・1280 |**0.2726**|331 / 49|0.20 / 0.2804|120.3ms|5,533MiB|
| 현재 YOLO26l detect・1280 |**0.2793**|352 / 52|운영 cutoff 0.40|89.8ms|5,429MiB|

시간은 mask 복원까지 포함한 source 입력 수신→결과 수신의 wall 지연이다. 표의 peak는
로드/PREPARE/ACTIVE를 포함하며 8,188MiB 중 비율로 계산한다. 최초 선별의 m960과
기준선은 공급 공백 gate가 실패했다. m960에서는 모든 카메라 입력이 약338–345ms
동시에 끊겼다. 이를 모델 병목으로 확정하지도, 실패를 자동 면제하지도 않는다.
나머지 YOLOE 구성은 첫 선별의 전체 gate를 통과했다. 최종 반복은 별도 증거다.

최대2후보 중 **l・1280 하나**를 다음 검증과 추가학습 후보로 선택했다. 같은0.40에서
검수된 객체 품질이 가장 좋고 최초 C6에서도 지연/메모리 여유가 있었다. l960은
해상도 하향 대조로 남긴다. 640 후보를 최종 후보로 선택하지 않았으므로 조건부
batch3 추가 대조는 생략했다. 20Hz 달성이나 운영 기본값 승격을 뜻하지 않는다.

프롬프트는 원래 obstacle17을 먼저6조합 모두 실제 추론한 후 변경했다. 상자를
wooden crate로 바꿔도 WoodBox 미검출은 해결되지 않았고, stroller로 바꿀 이유도
확인되지 않았다. **bollard만 traffic bollard로 바꾼 bollard17**은 SUNNY12m의
볼라드를 새로 검출했다. FOGGY・30/50m까지 개선됐다는 뜻은 아니다.
PyTorch 원본/변경 각각2,252장 실제0.40 재추론에서 기존7종 TP331/FP48과
검출→누락 전환 수가 정확히 같았다. Capture의 실제 도로 볼라드는 원래1관측에서 변경 후2관측으로 늘었고 공통 관측의 confidence가 높아졌다. 구조물 배경의 추가 검출로 확인된 사례는 없었지만, 같은 볼라드에 traffic cone도 붙는 중복은 남았다.
최종 l1280 engine은 이 목록으로 다시 export했다. 목록 변경 시 반드시 re-export한다.

Confidence는 **0.40 유지**다. 동일 predictor0.10 결과에서0.20으로 내리면 검토된
선정 자료의 TP331→402, FP49→219, macro F1 0.2726→0.2804였다. +71정답 대비
+170오검출이며, 상점을 bus로 잡거나 차량 창문을 person으로 잡는 사례를 확인했다.
F1 최대값만으로 낮은 cutoff를 채택하지 않는다. 선정0.40은 실제 predictor cutoff로
다시 추론하고 분리 heldout도 고정0.40으로 평가했다.

| 고정0.40 실제 추론 | 선정2,252장 F1 / TP / FP | heldout2,474장 F1 / TP / FP |
|---|---:|---:|
| YOLOE l1280・bollard17 |0.2725 / 331 / 49|0.2681 / 918 / 50|
| 현재 YOLO26l detect1280 |원본/검토 점수는 baseline 참조|0.2809 / 977 / 68|

YOLOE는 이 자료에서 기존 객체 TP가 적고 FP도 적었다. 현재 YOLO26을 대체할
근거는 부족하다. 새 상자・사슴 등의 zero-shot 미검출과 dog/person 라벨 혼동은
후속 라벨 정리/추가학습 대상으로 남긴다. 근거리 콘・통・바리케이드・측면 유모차
추가 검출은 육안 검토 결과이며 신규 클래스의 GT recall로 계산하지 않았다.

## 실행 실패와 변환 검증

첫 live 시도2건은 이전 노드의 private batched_runtime=true가 ROS parameter server에
남아 COCO backend를 잘못 선택했다. 설정에서 false를 명시하고 재검증했다.
이 실패를 성능 측정으로 섞지 않으며 live-screen-v1과 수정 근거를 그대로 남긴다.

원래6engine의 PT/FP16 대응36장씩에서 m 세조합은 박스/라벨 대응이 모두 일치했다.
l640은 FP16의 임계값 근처 원거리 실제 car1개가 추가됐고, l960은 같은 화물상자에
붙은 traffic barrel 중복1개가 사라졌다. 불일치 원본・mask・좌표차를 보존했으며
FP16을 bit-exact로 수락하지 않는다. 최종 bollard17 l1280은36장에서22/22박스가
동일 라벨로 일대일 대응했다. engine I/O는 FLOAT32이며 내부 FP16 계산을 사용한다.

[6조합과 반복 전체](http://localhost:8768/camera-yoloe-selection-v1/final-analysis-v2/index.html),
[동일 RGB 스폰 비교](http://localhost:8768/camera-yoloe-selection-v1/spawn-inspection-v1/comparison.html),
[정상 Scooter1 재수집](http://localhost:8768/camera-yoloe-selection-v1/spawn-scooter-native-v1/comparison.html),
[실제 프롬프트 대조](http://localhost:8768/camera-yoloe-selection-v1/prompt-bollard-v1/comparison.html),
[confidence 오검출/회복 목록](http://localhost:8768/camera-yoloe-selection-v1/confidence-review-v1/index.html),
[PT/FP16 차이](http://localhost:8768/camera-yoloe-selection-v1/parity-review-v1/index.html)을
갤러리 목차의 날짜・E번호와 함께 확인한다. 원본 실패는 삭제하지 않는다.

## 최종 C6 반복과 수락 상태

실행코드 `e09e201ca33186deb52a62862f2a0ba045778a42`에서 최종 prompt/engine/confidence를
고정한 후 SUNNY・FOGGY 각각3회를 기준선과 순서를 번갈아 측정했다.

| 구성 | SUNNY 전체 gate | FOGGY 전체 gate | 뷰별 평균 Hz 범위 | p95 / p99 최대 | 최대 공백 | 전체 peak VRAM |
|---|---:|---:|---:|---:|---:|---:|
| YOLOE l1280・bollard17・0.40 |3/3|1/3|14.55–14.90|121.1 / 141.7ms|406.6ms|5,507MiB (67.3%)|
| YOLO26l detect1280・0.40 |2/3|3/3|14.60–14.95|94.1 / 109.2ms|412.8ms|5,396MiB (65.9%)|

중복/역행 stamp는 모두0이다. YOLOE의 완전5초 창은14.0–15.2Hz로, 목표15Hz를
항상 달성한 것으로 쓰지 않는다. OOM・silent fallback・지속 큐 증가는 관측되지 않았다.
정상측정은 GUI/debug/trace OFF이며 다른 모델도 ACTIVE, 판제/제어는 실제 입력을
처리하되 시험 토픽으로만 발행했다. 공개 `/ctrl_cmd`로 주행하지 않았다.

실패는 다음과 같이 보존한다.

- 기준선 SUNNY r1과 YOLOE FOGGY r1: 여러 카메라・TwinLite・ResNet・판제 입력이
  같은 실행에서 공백 gate를 넘었다. 입력 수신 간격/원본 stamp 차이를 결과 JSON에
  연결한다. GPU 모델 원인으로 확정하지 않으며 입력 문제라는 이유로 수락을 면제하지 않는다.
- YOLOE FOGGY r2: YOLO 지연/공급은 통과했으나 판제 trajectory deadline이 실패했다.
  이를 YOLO 단독 실패로 표시하지 않으며 전체 통합 gate는 실패로 유지한다.
- 첫 선별・startup 실패・debug 프로토콜 실패도 별도 원본으로 남긴다. 유리한 반복만
  골라 통과 처리하지 않는다.

따라서 **추가학습 비교 후보는 YOLOE l1280・bollard17・0.40이지만, 통합 운영 수락과
기본값 승격은 보류**다. 현재 YOLO26l detect 기본값을 유지한다. 사전학습 YOLOE가
대회 객체 목록을 안정적으로 전부 덮는다는 근거는 없다.

후속 bbox 파일럿에서는 YOLO26l detect와 YOLOE 후보의 학습/배포 구조를 먼저
확인한다. 이번 seg 모델에서 mask를 소비하지 않는 것만으로 detection 전용 구조의
속도나 정확도가 검증됐다고 주장하지 않는다. Roboflow 감사와 본학습은 별도 작업이다.

## Debug 비용과 회귀

세 모델・LiDAR・판제 preview를 동일 SUNNY actor fixture에서 실행하고, GUI는 열지
않은 채 bbox3뷰・mosaic・seg instance3뷰・TwinLite/BEV・ResNet debug 영상을 실제
구독했다.3회 모두 latency gate가 실패했고 YOLO 뷰별 평균은8.95–9.10Hz,
p95 최대181.3–190.4ms였다. 이것은 **전체 debug 동시 발행 프로필**의 비용이며
YOLO bbox mosaic 하나만 보는 비용으로 일반화하지 않는다. 운영 측정은 debug OFF다.

첫 debug 호출은 축약된 revision을 전달해 provenance preflight에서 거부됐다.
다음 직접 runner 호출은 정상 측정과 달리9개 actor fixture가 빠져 preview 준비가
실패했다. 둘 다 비용 비교에서 제외하고 실패 원본/사유를 보존한 후, 같은
weather_matrix 프로토콜에서 위3회를 다시 수행했다.

전체 outer catkin → source → nested Main/Msg/morai_msgs 빌드가 통과했다.
최종 YOLOE15・COCO7 도구43・detection75・계약33 CPU 시험과 LiDAR 코어3개가
통과했다. 카메라 전체306개는305통과・기존 facade1실패다.
`autonomy_no_bridges_v2.launch`의 traffic_light=true 대 expected=false이며 해당
판제 파일과 facade 시험은 부모 브랜치와 동일하다. 이번에 판제 기본값을 수정하지 않았다.
GPU 없는 생성 메시지 시험은 batch3뷰・세대 전환・STANDBY 무효화・깨진 JPEG・
원본 크기/header・legacy adapter10항목을 통과했다. m640 원본 프롬프트・선정
l1280 프롬프트・다른 두 모델 동시 실행 launch도 실제 ROS 해석으로 확인했다.
ROS가 숫자 launch 인자를 int로 바꾸므로 설정 파일명에서는 명시적으로 str로 변환한다.

GitHub hosted CI는 #76 HEAD503a44ac의3개 annotation 모두 결제/사용 한도 때문에
job이 시작되지 않았다고 보고한다. 코드 실패로 위장하거나 검사를 우회하지 않는다.
PR #76은 위 제한을 안내한 뒤 사용자 승인으로 일반 CLI 병합했다. #77은 main 기준
Draft로 유지하며 최종 HEAD에서 hosted CI 확인이 필요하다. 로컬 검증을 hosted CI
통과로 대신 표시하지 않는다.

## 전방 bbox fusion 대조와 해석

같은 코드/설정 경계에서 현재 YOLO26l detect0.40도 새로 재생했다. 과거의
YOLO26m-seg0.65 fusion 자료를 현재 기준선으로 잘못 사용하지 않는다.

| bag・repeat | YOLO26l matched / dropped | YOLOE matched / dropped | YOLO26l / YOLOE offset p95 최대 |
|---|---:|---:|---:|
| 주 r1 |484 / 11|474 / 19|48.6 / 48.8ms|
| 주 r2 |486 / 10|476 / 15|48.6 / 48.3ms|
| 음영 r1 |173 / 0|174 / 0|41.9 / 39.2ms|
| 음영 r2 |178 / 0|171 / 0|29.4 / 28.3ms|

양쪽 모두4회에서 offset・단일 `/YOLO/BBox` 발행자・집계 검사는 통과했다.
기준선 음영r1과 후보 음영r2는3뷰 공급 공백 검사에서 실패했다. 후보 음영r2는
평균14.7Hz 정도였지만 최대 약430ms 공백이었다. 평균 Hz만으로 수락하지 않는다.
종료 pending LiDAR는 주r1/음영각1, 주r2는2였고 YOLO는각1이다. 결과 보존/집계가
완전하다는 뜻이며 pending0이나 모든 입력 처리 완료를 주장하지 않는다.

공통 LiDAR stamp에서 중심이1mm 이내인 상호 유일 객체로 비교했다. 주 bag은
4실행 공통328stamp・기준선에서 안정된2,747객체 관측 중 분류 차이4건,
미대응 공통 관측1,101건이다. 음영은135stamp・879안정 관측에서 차이0,
미대응876건이다. 숫자 track ID를 그대로 동일 객체로 간주하지 않았다.
이 안정 기준선도 독립 GT가 아니므로4건을 YOLOE 정확도 오류로 단정하지 않는다.
기존 승인된 분류2건의 예외를 새4건에 적용하지 않으며 원본/중심/라벨을 report.json과
baseline에 보존한다. 주 bag 매칭10쌍 감소 역시 연속성 개선으로 해석하지 않는다.
이 결과도 운영 승격 보류 이유다. 3뷰 mask fusion・구조물 개선・재사용 정책 변경은
이번에 적용하지 않았다.

원거리 GT 중10px 미만은2,242개로 매우 많으며 l1280의 해당 recall은 약0.0054,
20px 이상345개는 약0.7913이다. 이를 거리별 대회 성능으로 대체하지 않는다.
연속5입력 중3검출의 첫 안정 검출은 프레임 기준으로 보고하며, 선정 자료에는
안정 검출 미달62이력・연속 GT 부족44이력이 있다. 벽시계 초나 충분한 안정성
입증값을 만들어 채우지 않았다. 자세한 객체별 이력은 최종 report.json에 있다.

실험 전체 baseline은 `config/baselines/perception_camera_yoloe_selection_v1.yaml`이다.
성공・실패・원본 SHA와 선별 근거를 함께 검토한 뒤, 데이터 감사/짧은 bbox 학습
비교로 넘어간다. 최종 운영 선정과 hosted CI는 여전히 보류다.
