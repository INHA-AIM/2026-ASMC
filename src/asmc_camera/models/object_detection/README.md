# 객체 검출 배포 모델

**팀 배포 시 기존 YOLO를 신버전 YOLO26l collect v5 E3(15종)로 교체하세요.**
현재 기본은 `yolo26l-mission15-e3.pt` / `yolo26l-mission15-e3.engine`입니다.
기존 `detector:=e8`·`detector:=pretrained` 지정은 `detector:=e3`로 바꾸고 재시작하세요.
E8·사전학습·YOLO11m은 비교·복구용으로 보관합니다.
다른 GPU의 엔진 생성과 PR 포함 파일은 [배포·교체 안내](../../../../docs/operations/camera-deployment.md#모델-선택파일-배포)를 따릅니다.

기본 YOLO26l collect E3(15종)와 기존 E8/사전학습 `.pt`·`.engine` 실제 파일을 배포합니다.
E3 engine은 RTX4060·TensorRT10.13.0.35 CUDA11용이며, 기존 E8/사전학습 engine은 RTX4060Ti용입니다.
모델 출처와 환경·경로는
[모델 안내](../README.md)를 따릅니다.

아래 YOLO11m은 구형 호환용입니다.

`yolo11m.pt` is the identical Git blob copied from
`src/lidar+sensorfusion/src/Main/YOLOWeight/yolo11m.pt`.
SHA-256: `d5ffc1a674953a08e11a8d21e022781b1b23a19b730afc309290bd9fb5305b95`.

The old Main path records the copy's provenance; that package is retired. The
package-local file preserves the YOLO11m COCO reference path and introduces no new classes.
Upstream model: https://docs.ultralytics.com/models/yolo11/
