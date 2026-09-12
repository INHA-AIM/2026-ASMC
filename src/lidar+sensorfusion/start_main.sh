#!/usr/bin/env bash
set -eo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
start_velodyne_driver="${START_VELODYNE_DRIVER:-false}"

# Keep the lock descriptor open for the lifetime of roslaunch. This prevents a
# second start_main.sh from replacing ROS nodes and racing for the camera/LiDAR
# UDP ports.
exec 9>"$workspace/.start_main.lock"
if ! flock -n 9; then
    echo "ERROR: DAECHANGMO is already running (start_main.sh lock is held)." >&2
    echo "Stop the existing launch with Ctrl+C before starting it again." >&2
    exit 1
fi

# Also catch a launch started manually, or a partial launch left alive after
# /Main exited. roslaunch must be the only owner of these three receive ports.
if [[ "$start_velodyne_driver" == "true" ]] &&
   ss -H -lun "sport = :9299" 2>/dev/null | grep -q .; then
    echo "ERROR: UDP 9299 is already in use." >&2
    echo "Use morai_sensor_bridge with START_VELODYNE_DRIVER=false," >&2
    echo "or stop the other UDP receiver before enabling the local driver." >&2
    exit 1
fi

source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"

# YOLO GPU 사전 점검. 읽기 전용이며 설치나 빌드를 하지 않는다. 여기서 실패해도
# 실행은 계속한다. 실제 차단은 YOLO 노드의 preflight가 담당하고, 이 단계는
# 실패 원인이 드라이버인지, Docker GPU 전달인지, PyTorch 아키텍처인지를 먼저
# 구분해서 보여 주기 위한 것이다. ASMC_SKIP_GPU_CHECK=1 로 건너뛸 수 있다.
yolo_python="${ASMC_YOLO_PYTHON:-/usr/bin/python3}"
if [[ "${ASMC_SKIP_GPU_CHECK:-0}" == "1" ]]; then
    echo "[GPU] check skipped (ASMC_SKIP_GPU_CHECK=1)"
elif [[ ! -x "$yolo_python" ]]; then
    echo "[GPU][FAIL] YOLO Python 실행 파일이 없습니다: $yolo_python" >&2
    echo "[GPU]       원인: 선택한 GPU 프로필 이미지가 최신이 아닙니다." >&2
    echo "[GPU]       해결: RTX 40은 rebuild, RTX 50은 rebuild-rtx50" >&2
elif ! command -v nvidia-smi >/dev/null 2>&1; then
    echo "[GPU][FAIL] 컨테이너에서 nvidia-smi를 찾을 수 없습니다." >&2
    echo "[GPU]       원인: Docker GPU 전달(NVIDIA Container Toolkit / gpus: all)" >&2
elif ! nvidia-smi -L >/dev/null 2>&1; then
    echo "[GPU][FAIL] nvidia-smi가 GPU를 나열하지 못했습니다." >&2
    echo "[GPU]       원인: host NVIDIA 드라이버 또는 Docker GPU 전달" >&2
    echo "[GPU]       참고: 컨테이너 안에 host 드라이버를 설치하지 마십시오." >&2
else
    "$yolo_python" - <<'PYCHECK' || true
import sys

try:
    import torch
except Exception as exc:  # pragma: no cover - 환경 진단용
    print("[GPU][FAIL] import torch 실패: %s" % exc, file=sys.stderr)
    sys.exit(0)

arch_list = []
try:
    arch_list = torch.cuda.get_arch_list()
except Exception:
    pass

if not torch.cuda.is_available():
    print("[GPU][FAIL] torch.cuda.is_available() == False", file=sys.stderr)
    print("[GPU]       원인: host 드라이버 또는 Docker GPU 전달", file=sys.stderr)
    print("[GPU]       torch=%s torch_cuda=%s" % (torch.__version__,
                                                  torch.version.cuda),
          file=sys.stderr)
    sys.exit(0)

name = torch.cuda.get_device_name(0)
major, minor = torch.cuda.get_device_capability(0)
capability = "sm_%d%d" % (major, minor)
try:
    probe = torch.randn(64, 64, device="cuda:0")
    torch.cuda.synchronize()
    float((probe @ probe).sum().item())
except Exception as exc:
    print("[GPU][FAIL] CUDA 커널 실행 실패: %s" % exc, file=sys.stderr)
    print("[GPU]       원인: PyTorch 바이너리가 %s를 지원하지 않습니다."
          % capability, file=sys.stderr)
    print("[GPU]       torch=%s torch_cuda=%s arch_list=%s"
          % (torch.__version__, torch.version.cuda, ",".join(arch_list)),
          file=sys.stderr)
    print("[GPU]       해결: GPU 프로필에 맞는 이미지를 재빌드하십시오.", file=sys.stderr)
    sys.exit(0)

print("[GPU][OK] %s (%s) torch=%s torch_cuda=%s"
      % (name, capability, torch.__version__, torch.version.cuda))
PYCHECK
fi

cd "$workspace"
echo "Starting lidar+sensorfusion (local Velodyne driver: $start_velodyne_driver)..."
exec roslaunch Main main.launch \
    start_velodyne_driver:="$start_velodyne_driver" \
    lidar_port:=9299 lidar_cut_angle:=0.0 "$@"
