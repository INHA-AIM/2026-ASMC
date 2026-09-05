# TwinLiteNet+ v4 2-head 학습 평가

> **문서 역할:** Experiment Record — 특정 dataset/checkpoint의 재현 가능한 결과
> **현재 의사결정:** [perception-map-fusion-roadmap.md](./perception-map-fusion-roadmap.md)
> **문서 허브:** [README.md](./README.md)
> 평가일: 2026-09-04 · 18 epoch 결과 반영: 2026-09-05<br>
> 데이터: `twinlite_morai_v4_comp8_train_val_sample_b`<br>
> 모델: TwinLiteNet+ Medium, lane + drivable, 현재 후보 epoch 18 EMA<br>
> 체크포인트: `artifacts/perception_eval/runs/twinlite_medium_morai_v4_comp8_init_ema_b12_e18_20260904/best_mean.pt`

## 1. epoch 10 중간 기준의 고정 validation 개선

재학습 전후에 같은 v4 val 1,404 sample과 같은 head-valid mask를 사용했다.

| 범위 | 이전 평균 IoU | v4 epoch 10 | 변화 |
|---|---:|---:|---:|
| 전체 | 0.8210 | 0.9323 | +0.1113 |
| sunny | 0.8956 | 0.9374 | +0.0418 |
| foggy | 0.7582 | 0.9277 | +0.1695 |
| front | 0.7503 | 0.8821 | +0.1318 |
| foggy 3pm run | 0.7167 | 0.9238 | +0.2071 |

최종 전체 head별 IoU는 drivable `0.9822`, lane `0.8824`다. 조건별 상세 수치는
`artifacts/perception_eval/analysis/twinlite_medium_morai_v4_e10_on_v4_val_20260904/metrics.json`을
기준으로 한다.

## 2. 동일 frame overlay 비교

각 이미지는 왼쪽부터 RGB, GT, prediction이다. 아래 표의 이전/이후는 동일 frame과
동일 전처리로 렌더링했다.

| 장면 | 관찰 | 이전 | 이후 |
|---|---|---|---|
| foggy 3pm, NPC 근접 | 끊긴 drivable이 복구되고 원거리 lane이 연속화됨 | [before](../artifacts/perception_eval/analysis/twinlite_medium_morai_v4_e10_on_v4_val_20260904/overlays_selected/before/20260904_170026_kcity_comp_sample_b_seg06_foggy_sim3pm_wheel_front_000023.jpg) | [after](../artifacts/perception_eval/analysis/twinlite_medium_morai_v4_e10_on_v4_val_20260904/overlays_selected/after/20260904_170026_kcity_comp_sample_b_seg06_foggy_sim3pm_wheel_front_000023.jpg) |
| foggy 11am, 가드레일 곡선 | 도로 내부의 큰 false negative가 거의 제거됨 | [before](../artifacts/perception_eval/analysis/twinlite_medium_morai_v4_e10_on_v4_val_20260904/overlays_selected/before/20260904_165640_kcity_comp_sample_b_seg04_foggy_sim11am_wheel_front_000016.jpg) | [after](../artifacts/perception_eval/analysis/twinlite_medium_morai_v4_e10_on_v4_val_20260904/overlays_selected/after/20260904_165640_kcity_comp_sample_b_seg04_foggy_sim11am_wheel_front_000016.jpg) |
| sunny 3pm, 교차로 | drivable은 복구됐으나 그림자·횡단보도 부근 lane 누락이 남음 | [before](../artifacts/perception_eval/analysis/twinlite_medium_morai_v4_e10_on_v4_val_20260904/overlays_selected/before/20260904_165508_kcity_comp_sample_b_seg03_sunny_sim3pm_wheel_front_000019.jpg) | [after](../artifacts/perception_eval/analysis/twinlite_medium_morai_v4_e10_on_v4_val_20260904/overlays_selected/after/20260904_165508_kcity_comp_sample_b_seg03_sunny_sim3pm_wheel_front_000019.jpg) |
| foggy 3pm, 구조물·장애물 | 크게 개선됐지만 근거리 경계 파편화와 lane 단절이 남음 | [before](../artifacts/perception_eval/analysis/twinlite_medium_morai_v4_e10_on_v4_val_20260904/overlays_selected/before/20260904_170026_kcity_comp_sample_b_seg06_foggy_sim3pm_wheel_front_000071.jpg) | [after](../artifacts/perception_eval/analysis/twinlite_medium_morai_v4_e10_on_v4_val_20260904/overlays_selected/after/20260904_170026_kcity_comp_sample_b_seg06_foggy_sim3pm_wheel_front_000071.jpg) |

조건별 개선 대표 6장과 잔여 취약 대표 6장의 전체 결과는 같은 분석 디렉터리의
`overlays_selected/before/`, `overlays_selected/after/`에 있다.

## 3. epoch 10 시점의 장기학습 판단 근거

| Epoch | Train loss | Val loss | 평균 IoU | Front lane IoU |
|---:|---:|---:|---:|---:|
| 1 | 0.02145 | 0.01758 | 0.91744 | 0.75061 |
| 4 | 0.01249 | 0.01248 | 0.92599 | 0.77404 |
| 6 | 0.01092 | 0.01134 | 0.92991 | 0.78365 |
| 8 | 0.01002 | 0.01092 | 0.93163 | 0.78649 |
| 9 | 0.00975 | 0.01083 | 0.93203 | 0.78758 |
| 10 | 0.00962 | 0.01076 | 0.93232 | 0.78821 |

Train/val loss가 함께 내려가고 front lane도 epoch 10까지 상승하므로 과적합 징후는 없다.
따라서 더 긴 학습으로 소폭 개선될 여지는 있다. 다만 평균 IoU 증가폭은 epoch 8→9
`+0.00040`, 9→10 `+0.00029`로 줄어 이미 수렴 구간이다. 같은 설정을 단순히 오래 돌려
얻을 가능성이 큰 개선은 수 %p가 아니라 대체로 수천분의 일~수백분의 일 수준이다.

다음 장기 실험은 현재 종료 직전의 거의 0인 learning rate를 그대로 이어 쓰지 않는다.
동일 초기 EMA·seed·split에서 polynomial schedule을 15 epoch로 다시 설정한 독립 run을
권장한다. 10 epoch 실측 1시간 54분 기준 약 2시간 50분이 예상된다. 현재 val을 계속 보고
epoch를 선택하면 val 적응이 누적되므로, 장기 학습 또는 모델 변경의 최종 판단은 별도 test
run에서 한다.

교차로 얇은 lane과 구조물 주변 경계가 그대로 남으면 epoch 증가보다 해당 장면 표본 보강,
lane loss/해상도 조정, 동일 split에서의 다른 backbone 비교를 먼저 검토한다. image-plane
모델 선택을 마친 뒤 BEV 투영으로 넘어간다.

## 4. 18 epoch 독립 장기학습 설정

`20260903_211826_kcity_comp_sample_sunny_sim_unknown_wheel`의 환경 시각은 수집자 확인에
따라 sunny 1pm으로 추정한다. 원본 lineage 보존을 위해 run 이름은 변경하지 않는다.

10 epoch의 거의 종료된 LR schedule을 resume하지 않고, 같은 v1 epoch 10 EMA 초기값과
동일 v4 train/val manifest·seed에서 18 epoch polynomial schedule을 새로 시작한다.
Batch 12, AMP, EMA, LR `2e-4`, augmentation 없음은 동일하게 유지해 학습 길이만 공정하게
비교한다. 출력 run은
`twinlite_medium_morai_v4_comp8_init_ema_b12_e18_20260904`로 구분한다.

## 5. 18 epoch 종료 결과

18 epoch 학습은 정상 종료됐고 `best_mean`, `best_lane`, `best_drivable`이 모두 epoch 18에서
갱신됐다. 동일 v4 validation 1,404 sample의 frozen EMA 평가는 다음과 같다.

| 범위 | epoch 10 | epoch 18 | 변화 |
|---|---:|---:|---:|
| 전체 mean head IoU | 0.9323 | **0.9386** | +0.0063 |
| 전체 drivable IoU | 0.9822 | **0.9842** | +0.0020 |
| 전체 lane IoU | 0.8824 | **0.8930** | +0.0106 |
| front mean head IoU | 0.8821 | **0.8900** | +0.0079 |
| front lane IoU | 0.7882 | **0.8015** | +0.0133 |
| foggy mean head IoU | 0.9277 | **0.9348** | +0.0070 |
| sunny mean head IoU | 0.9374 | **0.9429** | +0.0055 |

front sample 468개 중 463개가 epoch 10보다 개선됐고, 나머지 5개의 퇴행 폭도 작았다.
주행가능영역은 안정적이며 남은 주요 취약점은 전방 교차로·횡단보도·그림자와 겹치는
가는 차선이다. 종료 LR은 약 `2.6e-6`으로 단순 resume의 기대효과가 낮다. 추가 학습보다
현재 checkpoint를 동결해 live/독립 test를 먼저 수행하고, 그 결과가 부족할 때 데이터,
loss/해상도 또는 다른 backbone을 같은 조건에서 비교한다.

상세 수치는
`artifacts/perception_eval/analysis/twinlite_medium_morai_v4_e18_on_v4_val_20260905/metrics.json`,
동일 frame 시각화는 같은 디렉터리의 `overlays_selected/`를 기준으로 한다. 이 validation은
매 epoch model selection에 사용했으므로 최종 unbiased test로 해석하지 않는다.
