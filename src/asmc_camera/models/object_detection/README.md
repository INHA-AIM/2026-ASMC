# 객체 검출 배포 모델

YOLO26l E8/사전학습 `.pt`·`.engine` 실제 파일을 일반 Git으로 배포합니다.
모델 출처와 환경·경로는
[모델 안내](../README.md)를 따릅니다.

아래 YOLO11m은 구형 호환용입니다.

`yolo11m.pt` is the identical Git blob copied from
`src/lidar+sensorfusion/src/Main/YOLOWeight/yolo11m.pt`.
SHA-256: `d5ffc1a674953a08e11a8d21e022781b1b23a19b730afc309290bd9fb5305b95`.

The original two Main weights remain in place. This copy preserves the existing
YOLO11m COCO inference path; it does not introduce a trained model or new classes.
Upstream model: https://docs.ultralytics.com/models/yolo11/
