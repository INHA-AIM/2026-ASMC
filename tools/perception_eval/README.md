# perception_eval — 초기 CLI 골격

> **역할:** Research/Legacy Stub — 초기 Loc-Aid·bake 평가 인터페이스
> **현재 평가 경로:** repo 루트 `scripts/evaluate_twinlite.py`, `scripts/render_twinlite_predictions.py`
> **현재 로드맵:** [인지 융합 로드맵](../../docs/packages/asmc_perception/roadmap.md)

상세 초기 아이디어: [perception-agentic-loop.md](../../docs/archive/perception/agentic-loop.md)

```bash
./tools/perception_eval/run_all.sh --help
python3 tools/perception_eval/run_bake.py --help
```

현재는 **인터페이스·exit code만** 구현한 스텁이다. TwinLite 학습 결과 평가에 이 경로를
사용하지 않는다. 추후 공통 BEV offline evaluator로 재사용할지는 camera/map pilot 뒤 결정한다.
