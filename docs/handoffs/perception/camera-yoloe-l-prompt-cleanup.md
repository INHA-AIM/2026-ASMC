# YOLOE-26l 프롬프트 정리: 42종 검토 → obstacle17 후보

## 상태와 근거

2026-10-09, `feat/camera-yoloe-l-first-run`에서 저장된 rich42 TensorRT 결과와 원본 RGB를
검토했다. confidence 0.3, 기존 선정 GT 2252프레임을 사용했다. 이번 정리는 **CPU 검토와
새 vocabulary 후보 작성**까지이며, 새 embedding·engine·재추론·성능 검증은 아직 하지 않았다.
MORAI와 카메라 추론 종료 완료를 확인한 뒤 수행한다. 기존 실험 launch 기본값과
rich42/base27 설정·원본 증거는 유지한다.

[후속 실험 설정](../../../config/experiments/camera_yoloe_l_prompt_cleanup_v1.yaml)에
42종별 결정·관측 수·기준 결과 SHA를 기록했다. 로컬 원본 검토 갤러리는 상위 workspace의
`artifacts/camera-yoloe-l-first-v1/prompt-cleanup-v1/audit-rich42-v2/index.html`이다.
각 라벨의 최고 confidence·최대 면적·중간·마지막 사례를 선택했으며 검출이 없었던 이름은
영상 검증을 완료했다고 표시하지 않는다. 42종 전수 설정 검토와 모든 박스 전수 육안 검토는 다르다.

Capture actor GT는 고정 배경의 모든 객체를 라벨링하지 않는다. 실제로 영상상 올바른
도로변 traffic barrel이 GT에 대응하지 않는 사례를 확인했다. **미대응 수를 모두 오검출 수로
해석하지 않는다.** coarse Obstacle GT에 대응해도 정확한 재질·세부 이름의 정답은 아니다.

## 확인한 문제와 선택

- `concrete barrier`: 저장 영상에서 가드레일을 검출했다. 사용자가 보고한 WoodBox·컨테이너
  건물 오검출은 별도 실시간 관측이며, 해당 RGB/결과 쌍은 확보하지 못했다. 이 표현은 제외한다.
- `plastic road barrier`: 검토 표본은 드럼통 또는 가드레일이었다. 현재 표현은 제외하고
  실제 이동식 플라스틱 바리케이드 지원은 학습·검증 과제로 남긴다.
- `traffic light`: 가드레일 반사판을 높은 confidence로 검출했다. 이 객체 실험에서는 제외하며
  기존 ResNet 신호등 상태 판정 기능은 유지한다.
- `lane separator`·`fence`·`speed bump`: 가드레일·터널 벽·노면/연석 혼동을 확인했다.
- `utility pole`·`guardrail`·일부 표지판은 이름이 맞더라도 이번 일반 장애물 출력 후보에서 제외한다.
  구조물의 충돌 경계를 자유 공간으로 바꾸는 조치는 아니다.
- `cargo box`·`wooden box`·`traffic cone`·`traffic barrel`은 실제 객체 표본이 있어 유지한다.
  박스를 WoodBox로 강제 재분류하지 않고 모델의 원래 라벨을 보존한다.

`wooden box`와 `concrete barrier`가 모두 obstacle 그룹이어도 잘못된 위치·범위·instance mask를
안전한 검출로 간주할 수 없다. 프롬프트 삭제 후 같은 건물이 다른 남은 이름으로 검출될 수 있으므로
**42종 결과의 라벨만 필터링해서 개선됐다고 보고하지 않는다.** 별도 obstacle17 추론이 필요하다.

## 후보 목록과 미지원 범위

```text
person, dog, deer,
bicycle, car, truck, bus, motorcycle, scooter, electric scooter,
bollard, cargo box, wooden box, steel barricade, traffic cone, traffic barrel, baby stroller
```

원본 라벨과 `person / animal / vehicle / obstacle` 그룹은 별도로 유지한다.
검출되지 않은 dog·deer·bicycle·이륜차·stroller·steel barricade는 대회 후보를 보존한 것이며
검출 성능을 수락한 것이 아니다. tree·실제 concrete/plastic barrier·범용 drum 등 제외된
물리적 에셋도 대회 목록에서 삭제하지 않는다. 이번 zero-shot 후보가 커버하지 못하는 범위로 기록한다.
알 수 없는 Ped_Pet1 종류는 dog로 확정하지 않는다. 판제가 소비할 회피 대상과 배경/노면 정책은
최종 객체 지원 감사에서 별도로 정해야 한다.

## 42종별 결정

관측 수는 confidence 0.3 이상의 저장된 박스 수이며 객체 수·TP 수가 아니다.
0은 데이터에서 검출이 없다는 뜻이며 출제 제외나 모델 지원을 증명하지 않는다.

| 원본 prompt | 결정 | 관측 박스 | 근거/남은 확인 |
|---|---|---:|---|
| `person` | 유지 | 143 | 사람 검출 표본 확인; 원래 라벨 유지 |
| `dog` | 유지·미검증 | 0 | 대회 동물 후보; 선정 자료에는 검출 없음 |
| `deer` | 유지·미검증 | 0 | 대회 동물 후보; 선정 자료에는 검출 없음 |
| `animal` | 유사 표현 제외 | 0 | dog/deer와 넓은 범주 중복; 미확인 pet의 종류는 별도 확인 |
| `bicycle` | 유지·미검증 | 0 | 대회 자전거 후보; 선정 자료에는 검출 없음 |
| `car` | 유지 | 236 | 차량 표본 확인; 일부 세부 종류 GT 있음 |
| `truck` | 유지 | 43 | 트럭 표본 확인; 일부 세부 종류 GT 있음 |
| `bus` | 유지 | 132 | 버스 표본 확인; 일부 세부 종류 GT 있음 |
| `motorcycle` | 유지·미검증 | 0 | 대회 이륜차 후보; 선정 자료에는 검출 없음 |
| `scooter` | 유지·미검증 | 0 | 대회 후보; Scooter 에셋의 실제 외형 확인 필요 |
| `electric scooter` | 유지·미검증 | 0 | 대회 킥보드 후보; 선정 자료에는 검출 없음 |
| `bollard` | 유지·미검증 | 2 | 대회 후보; 작은 원거리 표본만 있어 검증 부족 |
| `box` | 유사 표현 제외 | 0 | cargo box/wooden box 유지; 범용 box 검출 없음 |
| `cargo box` | 유지 | 75 | 도로 위 상자 표본 확인; 재질 정답은 미확인 |
| `wooden box` | 유지 | 9 | 나무 상자 외형 표본 확인; 세부 GT는 미확인 |
| `wooden crate` | 유사 표현 제외 | 0 | wooden box와 중복 표현; 검출 없음 |
| `cardboard box` | 유사 표현 제외 | 2 | 나무 상자 외형을 cardboard로 반환한 표본 |
| `pallet` | 유사 표현 제외 | 0 | 현재 미션·표본 근거 부족; 팔레트 지원을 입증하지 않음 |
| `concrete barrier` | 오검출 표현 제외 | 68 | 가드레일 오검출 확인; 사용자는 WoodBox/건물 혼동도 보고 |
| `road barrier` | 유사 표현 제외 | 0 | 범용 바리케이드 표현 미검증; 실제 에셋 지원은 남은 과제 |
| `plastic road barrier` | 오검출 표현 제외 | 5 | 검토 표본은 드럼통 또는 가드레일 |
| `steel barricade` | 유지·미검증 | 0 | 이동식 바리케이드 후보 보존; 실제 검출 검증 없음 |
| `barricade` | 유사 표현 제외 | 0 | 범용 중복 표현; steel barricade 후보 유지 |
| `lane separator` | 배경/노면 제외 | 417 | 검토 표본은 가드레일; 일반 장애물 출력에서 제외 |
| `traffic cone` | 유지 | 860 | 실제 콘 표본 확인; GT의 세부 종류는 미확인 |
| `traffic barrel` | 유지 | 137 | 실제 도로변 드럼통 표본 확인; 일부 고정 배경은 GT에 없음 |
| `barrel` | 유사 표현 제외 | 1 | traffic barrel 유지; 단일 범용 barrel 표본은 미검증 |
| `drum` | 유사 표현 제외 | 0 | traffic barrel 표현 유지; 기타 드럼 에셋 지원은 미검증 |
| `road sign` | 배경/노면 제외 | 15 | 실제 상부 표지판 표본; 별도 인프라 소비 정책 필요 |
| `speed limit sign` | 배경/노면 제외 | 7 | 인프라 인식은 일반 장애물 실험에서 제외 |
| `stop sign` | 배경/노면 제외 | 2 | 인프라 인식은 일반 장애물 실험에서 제외 |
| `traffic light` | 오검출 표현 제외 | 994 | 가드레일 반사판을 높은 confidence로 오검출 |
| `speed bump` | 오검출 표현 제외 | 42 | 검토 박스는 노면·연석 영역 |
| `manhole cover` | 배경/노면 제외 | 0 | 노면 요소; 검출 없음, 별도 통행 가능성 판단 필요 |
| `baby stroller` | 유지·미검증 | 0 | 대회 유모차 후보; 검출 없음 |
| `stroller` | 유사 표현 제외 | 0 | baby stroller와 중복 표현; 검출 없음 |
| `tree` | 배경/노면 제외 | 0 | 배경 나무와 미션 에셋 구분 필요; 제외된 물리적 에셋 지원 과제로 기록 |
| `streetlight` | 배경/노면 제외 | 1 | 가로등 머리 표본; 일반 장애물 출력에서 제외 |
| `utility pole` | 배경/노면 제외 | 13 | 실제 전신주 외형은 맞지만 현재 일반 장애물 출력에서 제외 |
| `guardrail` | 배경/노면 제외 | 182 | 실제 가드레일 외형은 맞지만 충돌 경계는 별도 처리 |
| `fence` | 오검출 표현 제외 | 2 | 검토 박스는 터널 벽 영역 |
| `fire` | 배경/노면 제외 | 0 | 물리적 장애물이 아닌 효과·이벤트; 검출 없음 |

## 다음 검증과 직접 실행

1. l checkpoint와 obstacle17 전체 순서로 새 embedding을 만들고 one-to-many head를 유지해
   TensorRT FP16 engine을 생성한다. 기존 42/27종 cache·engine은 덮어쓰지 않는다.
2. 같은 선정 RGB·보넷 mask에서 PyTorch와 TensorRT를 실제 재추론한다. 공통 confidence
   0.3/0.7과 기존 탐색 범위를 비교하고 사람·차량·상자 누락 및 배경의 다른 이름 재검출을 보존한다.
3. 실제 raw label·box·mask 비교 갤러리를 제공한다. 세부 종류 GT 없는 항목은 검토 사례로 표시한다.
4. 3뷰 단독/통합 성능을 확인한 뒤 실험 launch에 새 프로필을 연결한다. 운영 YOLO11m 기본값,
   `/YOLO/BBox` 네 클래스·0.7, 메시지/MD5, 50ms·200ms와 jamming crop은 유지한다.

현재 새 프로필의 모델 설정과 engine이 없으므로 `prompt_profile:=obstacle17` 직접 실행은
아직 제공하지 않는다. GPU 재검증 후 실행 명령과 브라우저 갤러리 주소를 갱신한다.
