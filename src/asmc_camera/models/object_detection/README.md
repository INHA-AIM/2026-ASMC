# 객체 검출 배포 모델

현재 기본은 YOLO26l collect v5 E3(15종)의 `yolo26l-mission15-e3.pt` / `.engine`이다.
`detector:=e8`은 이전 E8(16종) checkpoint/engine을 선택한다. 가중치 변경 시 기존 detector를
종료한 뒤 재시작한다. [배포·교체 안내](../../../../docs/operations/camera-deployment.md#모델-선택파일-배포)를 따른다.

E3 배포 engine은 RTX4060·TensorRT10.13.0.35 CUDA11용이며 E8은 RTX4060Ti용이다.
다른 GPU에서는 같은 checkpoint로 생성·검증한 로컬 engine을 선택한다. 개인 GPU의 생성본으로
Git 배포 파일을 덮어쓰지 않는다. [모델 안내](../README.md)에 출처·선택 조건이 있다.

YOLO11m과 COCO pretrained 비교 파일은 현재 배포에서 제외했다.
원래 경로·SHA·Git blob과 복구할 main 커밋은
[Git 이력 자산 목록](../../../../config/artifacts/git-history-assets-v1.json)에 보존한다.
현재 공개 `detector` 선택은 E3·E8이다.
