# 인지 에이전틱 평가 루프 (Loc-Aid · MGeo bake · BEV)

> **문서 역할:** Research Reference — 2026-07 초기 반복 평가 아이디어
> **현재 로드맵:** 우선순위와 인터페이스는 [perception-map-fusion-roadmap.md](../../packages/asmc_perception/roadmap.md)를 따른다.
> **문서 허브:** [README.md](../../README.md)
> **최종 수정:** 2026-07-25
> **담당:** 안승현
> **후보 모듈:** Loc-Aid, MGeo bake, 3캠 BEV, 차선 이식, sync 수집 — **동등 선택지** (팀 합의·최종 채택 전)
> 코드 골격: [`tools/perception_eval/`](../../../tools/perception_eval) · `src/perception/drivable_bev/`

조향 E2E가 아니다. **bake/crop/BEV 품질을 메트릭으로 올리고**, Cursor 에이전트가 eval→분석→패치→재실행을 돕는다.

---

## 1. 목표

| 목표 | 방법 |
|------|------|
| GPS noise·음영에서도 crop이 쓸 만한지 | Loc-Aid vs GPS-only 등 **비교** IoU |
| 빠른 반복 | MORAI 풀스택 없이 **오프라인 eval** 우선 |
| 에이전트 자동화 | `metrics.json`만 근거로 축 1개씩 개선 |

---

## 2. 디렉터리

```
tools/perception_eval/
  README.md
  run_all.sh
  run_bake.py
  run_eval_drivable.py
  run_eval_bev.py
  inject_pose_noise.py
  report.py
artifacts/perception_eval/     # gitignore (대용량)
  runs/<timestamp>/
    metrics.json
    config.yaml
    failure_gallery/
src/perception/drivable_bev/   # 구현 예정 (bake · crop · IPM)
```

---

## 3. metrics.json 스키마

```json
{
  "run_id": "20260725_120000",
  "modules": ["loc_aid", "mgeo_bake", "cam_bev"],
  "scenarios": [
    {"name": "open_sky_clean"},
    {"name": "gps_noise_sigma_X"},
    {"name": "gps_blackout_tunnel"}
  ],
  "scores": {
    "drivable_iou_mean": 0.0,
    "drivable_iou_p10": 0.0,
    "oob_false_positive_rate": 0.0,
    "crop_latency_ms_p95": 0.0,
    "bev_iou_mean": null
  },
  "gate": {
    "pass": false,
    "thresholds": {
      "drivable_iou_mean": 0.85,
      "drivable_iou_p10": 0.70
    }
  }
}
```

초안 임계값은 튜닝 가능. 변경 시 본 문서와 `config.yaml`을 같이 고친다.

---

## 4. 실험 매트릭스

| 실험 | 입력 | 성공 |
|------|------|------|
| Bake 정합 | MGeo / road_mesh | overlay·면적 sanity |
| Crop @ clean | bake + GT pose | IoU ≥ 게이트 |
| Crop @ GPS noise | bake + noisy pose | IoU_p10 유지 |
| Crop @ blackout | bake + Loc-Aid pose | GPS 없이 IoU |
| Cam BEV vs GT | 3캠 + GT mask | bev_iou (보조) |

---

## 5. 에이전트 1사이클

1. Human: 게이트 또는 “crop 반경만 개선”
2. Agent: `./tools/perception_eval/run_all.sh`
3. Agent: `metrics.json` + `failure_gallery/` 분석
4. Agent: **변경 축 1개**만 패치
5. 재실행 → pass면 중단, fail이면 요약 후 Human에게

**자동화**

| 수단 | 용도 |
|------|------|
| `run_all.sh` | bake→noise→eval→report |
| Cursor `/loop` 15–30m | 게이트 미달 시 축 1개 수정 후 재eval |
| (이후) Automation | 스케줄 리포트만 |

**가드레일**

- 사이클당 축 1개
- 빨간 `main` 푸시 금지 (Draft/로컬)
- Privileged GT는 eval 전용
- artifact 대용량 gitignore

---

## 6. 구현 상태

| 항목 | 상태 |
|------|------|
| 본 문서 · CLI 스텁 | ✅ |
| Loc-Aid / bake 알고리즘 | ⬜ 이후 |
| 실제 IoU 계산 | ⬜ 이후 |
| `/loop` 실운전 | ⬜ 이후 |

---

## 7. 관련

- [architecture 로컬](../../../../docs/competition/architecture.md) · [team-status](../../../../docs/competition/team-status.md)
- [운영 가이드](../../operations/README.md) · [협업 규칙](../../governance/README.md)
