#!/usr/bin/env bash
#
# Build the swarm simulation image and run it on an NVIDIA GPU host.
#
# Usage:
#   docker/sim/run_docker.sh                         # Gazebo + RViz2 in the browser: http://<server-ip>:6080
#   docker/sim/run_docker.sh headless:=true          # no viewer: Gazebo server + swarm stack only
#   docker/sim/run_docker.sh headless:=true rviz:=true   # browser shows RViz2 only (no Gazebo GUI)
#   docker/sim/run_docker.sh --shell                 # bash inside the container
#
# Options:
#   --no-build      run the existing image without rebuilding it
#   --detach, -d    run in the background (follow with: docker logs -f $CONTAINER_NAME)
#   --shell         start bash instead of the simulation
# Every other argument (name:=value) goes to run_gazebo_swarm.sh -> gazebo_swarm.launch.py.
# Without launch arguments the image default is rviz:=true (Gazebo window + RViz2), and
# sensor_profile:=full is added unless you pass a sensor_profile.
#
# Environment:
#   IMAGE_NAME (swarm_sim:jazzy), CONTAINER_NAME (swarm_sim_cloud)
#   NOVNC_PORT (6080)   host port for the browser desktop
#   BIND_ADDR (0.0.0.0) host address to publish it on; 127.0.0.1 for SSH-tunnel-only access
#   VNC_PASSWORD        desktop password (first 8 characters); a random one is printed if unset
#   VNC_GEOMETRY        desktop size (1920x1080)
#   SENSOR_PROFILE      default sensor_profile (full); VGLRUN=0 disables VirtualGL (troubleshooting)

set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-swarm_sim:jazzy}"
CONTAINER_NAME="${CONTAINER_NAME:-swarm_sim_cloud}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
BIND_ADDR="${BIND_ADDR:-0.0.0.0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

build=1
detach=0
shell=0
launch_args=()
for arg in "$@"; do
    case "$arg" in
        --no-build) build=0 ;;
        --detach|-d) detach=1 ;;
        --shell) shell=1 ;;
        -h|--help) sed -n '2,25p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) launch_args+=("$arg") ;;
    esac
done

# Same rule as the container entrypoint: headless:=true without rviz:=true needs no viewer
headless=false
rviz=false
for arg in ${launch_args[@]+"${launch_args[@]}"}; do
    case "${arg,,}" in
        headless:=true) headless=true ;;
        rviz:=true) rviz=true ;;
    esac
done
viewer=1
if $headless && ! $rviz; then viewer=0; fi

if (( build )); then
    echo "Building ${IMAGE_NAME} from ${REPO_ROOT} ..."
    docker build -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile" "$REPO_ROOT"
fi

run_opts=(
    --rm --init
    --name "$CONTAINER_NAME"
    --gpus all
    -e NVIDIA_VISIBLE_DEVICES=all
    -e NVIDIA_DRIVER_CAPABILITIES=all
    --shm-size=2g
)
if (( detach )); then
    run_opts+=(-d)
elif [ -t 0 ] && [ -t 1 ]; then
    run_opts+=(-it)
else
    run_opts+=(-i)
fi

for var in SENSOR_PROFILE VGLRUN; do
    [ -n "${!var:-}" ] && run_opts+=(-e "$var")
done

if (( shell )); then
    exec docker run "${run_opts[@]}" "$IMAGE_NAME" bash
fi

if (( viewer )); then
    run_opts+=(-e VIEWER=1 -p "${BIND_ADDR}:${NOVNC_PORT}:6080")
    [ -n "${VNC_PASSWORD:-}" ] && run_opts+=(-e VNC_PASSWORD)
    [ -n "${VNC_GEOMETRY:-}" ] && run_opts+=(-e VNC_GEOMETRY)
    echo "Browser desktop: http://<server-ip>:${NOVNC_PORT}"
else
    run_opts+=(-e VIEWER=0)
    echo "Headless run: no viewer"
fi

docker run "${run_opts[@]}" "$IMAGE_NAME" ${launch_args[@]+"${launch_args[@]}"}
