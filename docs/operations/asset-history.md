# Git 이력에 보존한 모델과 실험 자료

현재 운영 패키지는 객체 E3·E8, TwinLite Large E8 checkpoint/engine,
신호등 ResNet을 포함한다. 사용자 승인에 따라 다음 65개 파일 약 916 MB를 현재 Git tree에서 제거했다.

- 과거 실험 압축자료 51개: 718,253,777 bytes.
- YOLO11m, COCO pretrained 비교 PT/engine, BDD 비교 PT, Medium v6 PT/engine,
  TwinLite ONNX 및 Medium manifest/프로필: 나머지 14개.

[파일별 보존 목록](../../config/artifacts/git-history-assets-v1.json)은 원래 경로·크기·SHA256·Git blob과
복구할 main 커밋을 기록한다. 요약 JSON·설정·평가 수치·작은 회귀 fixture는 유지한다.
D의 원본과 reference 사본, full checkpoint는 삭제하지 않았다.

## 과거 상태 재현

목록의 `history_commit`은 main의 고정 커밋이다. 과거 실험은 해당 코드·모델·설정을 함께 사용한다.
현재 주행 작업트리와 분리해 실행한다.

```bash
git worktree add --detach ../2026-ASMC-assets-history af6df9e66ecb439bc44be94ef4370e2d97e010e0
```

현재 배포에서는 `detector:=pretrained`를 지원하지 않는다. BDD/YOLO11m 과거 실험 launch와
일부 비교 설정은 재현 도구이므로 역사 가중치가 필요하다. 현재 tree에서 파일 누락을 우회해
다른 모델로 자동 대체하거나 실험 결과를 현재 모델 성능으로 해석하지 않는다.
TwinLite의 현재 복구는 `lane_runtime_pytorch.yaml`의 같은 Large E8 가중치를 사용한다.

## CI와 파일 크기

`check_git_history_assets.py`는 폐기 자산의 재유입과 현행 E3·E8·Large E8 파일의 SHA/존재를 검사한다.
전체 이력을 가진 로컬 clone에서는 `--verify-history`로 원래 바이트까지 대조한다.
기본 CI는 과거 대형 blob을 다시 다운로드하지 않는다.

```bash
python3 tools/audit/check_git_history_assets.py
python3 tools/audit/check_git_history_assets.py --verify-history
```

Git 이력을 다시 쓰지는 않았다. 기존 clone의 `.git` 용량은 바로 줄지 않으며,
새 CI의 shallow checkout과 현재 작업트리에서 가져오는 파일 크기가 줄어든다.

정리 후 추적 파일은 약 1.559 GB → 642.4 MB(58.8% 감소)다. 이는 Git history/object DB 전체 용량과 다르다.
