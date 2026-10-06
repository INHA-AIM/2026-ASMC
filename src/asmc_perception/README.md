# asmc_perception → asmc_camera 이전 안내

카메라 런타임 패키지는 2026-10-06 **`asmc_camera`**로 이름을 바꾸고 전부 옮겼다.
코드·설정·모델·문서는 [`src/asmc_camera/`](../asmc_camera/README.md)에 있다.

이 디렉터리는 ROS 패키지가 아니다. 옛 링크가 끊기지 않도록 이 안내 문서만 남겼다.
`roslaunch asmc_perception ...`와 Python `import asmc_perception`은 더 이상 동작하지 않는다.

| 이전 | 현재 |
|---|---|
| `roslaunch asmc_perception perception.launch` | `roslaunch asmc_camera perception.launch` |
| `pkg="asmc_perception"`, `$(find asmc_perception)` | `pkg="asmc_camera"`, `$(find asmc_camera)` |
| `src/asmc_perception/...` | `src/asmc_camera/...` |
| `import asmc_perception.camera`, `asmc_perception.bev` | `import asmc_camera.camera`, `asmc_camera.bev` |

node 이름, `/perception/*` 토픽, launch 인자와 기본값은 바뀌지 않았다.
