#!/bin/bash
#
# Container entrypoint for the swarm simulation image (docker/sim/Dockerfile)
#
# Arguments of the form name:=value are passed to run_gazebo_swarm.sh -> gazebo_swarm.launch.py.
# Anything else is executed as a command instead (e.g. `bash`).
#
# The Gazebo server always renders its sensors off-screen with EGL (--headless-rendering).
# Viewer mode (default): TurboVNC desktop on :1 (localhost only), noVNC on port 6080, and the launch
#   runs under vglrun so the Gazebo GUI client and RViz2 render on the NVIDIA GPU through VirtualGL's
#   EGL back end. VirtualGL only redirects GLX/X11 rendering; the server's device EGL passes through.
# Headless mode: chosen when headless:=true is given without rviz:=true (or VIEWER=0). No X server
#   and DISPLAY stays unset.
#
# Environment:
#   VIEWER=0|1           force headless / viewer mode
#   SENSOR_PROFILE       sensor_profile launch argument added when none is given (default full)
#   VGLRUN=0             viewer mode without VirtualGL (GUI renders with llvmpipe; for troubleshooting)
#   VNC_PASSWORD         VNC password (only the first 8 characters count); random if unset
#   VNC_GEOMETRY         desktop size, default 1920x1080
#   VGL_DISPLAY          VirtualGL back end, default egl (egl1, ... selects another GPU)
#   KEEP_DESKTOP=0|1     keep the desktop up after the simulation exits (default 1)

set -e

if [ $# -gt 0 ] && [[ "$1" != *:=* ]]; then
    exec "$@"
fi

source /opt/ros/jazzy/setup.bash
WS=/root/swarm_ws
mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"

headless=false
rviz=false
profile_given=false
for arg in "$@"; do
    case "${arg,,}" in
        headless:=true) headless=true ;;
        rviz:=true) rviz=true ;;
        sensor_profile:=*) profile_given=true ;;
    esac
done
# Cloud GPU default: full-rate depth cameras (320x240 @ 30 Hz)
$profile_given || set -- "$@" "sensor_profile:=${SENSOR_PROFILE:-full}"
if [ -z "${VIEWER:-}" ]; then
    if $headless && ! $rviz; then VIEWER=0; else VIEWER=1; fi
fi

start_desktop() {
    export DISPLAY=:1
    mkdir -p ~/.vnc
    if [ -z "${VNC_PASSWORD:-}" ]; then
        VNC_PASSWORD="$(tr -dc 'A-Za-z0-9' < /dev/urandom | head -c 8)"
        echo "VNC password (generated): ${VNC_PASSWORD}"
    fi
    echo "$VNC_PASSWORD" | vncpasswd -f > ~/.vnc/passwd
    chmod 600 ~/.vnc/passwd

    # VNC listens on localhost only; noVNC/websockify is the single exposed port
    vncserver :1 -geometry "${VNC_GEOMETRY:-1920x1080}" -depth 24 -localhost \
        -securitytypes VNC -rfbauth ~/.vnc/passwd \
        -xstartup /usr/local/share/swarm/xstartup.sh > /tmp/vncserver.log 2>&1 \
        || { cat /tmp/vncserver.log; exit 1; }
    for _ in $(seq 1 50); do
        [ -S /tmp/.X11-unix/X1 ] && break
        sleep 0.2
    done

    websockify --web /usr/share/novnc 6080 localhost:5901 > /tmp/novnc.log 2>&1 &
    websockify_pid=$!

    echo "Desktop ready: open http://<server-ip>:6080 in a browser"
}

cd "$WS"
if [ "$VIEWER" = "1" ]; then
    start_desktop
    if [ "${VGLRUN:-1}" = "1" ]; then
        launcher=(vglrun -d "${VGL_DISPLAY:-egl}" ./run_gazebo_swarm.sh)
    else
        launcher=(./run_gazebo_swarm.sh)
    fi
else
    unset DISPLAY
    $headless || set -- "$@" headless:=true
    $rviz && echo "Warning: rviz:=true has no display in headless mode (set VIEWER=1 to view it)"
    launcher=(./run_gazebo_swarm.sh)
fi

stopping=false
"${launcher[@]}" "$@" &
sim_pid=$!
trap 'stopping=true; kill -INT "$sim_pid" 2>/dev/null || true' INT TERM

status=0
while kill -0 "$sim_pid" 2>/dev/null; do
    wait "$sim_pid" && status=0 || status=$?
done

if [ "$VIEWER" = "1" ] && ! $stopping && [ "${KEEP_DESKTOP:-1}" = "1" ]; then
    echo "Simulation exited (status ${status}). The desktop stays up; press Ctrl+C or 'docker stop' to quit."
    trap 'kill "$websockify_pid" 2>/dev/null || true' INT TERM
    wait "$websockify_pid" || true
fi
vncserver -kill :1 > /dev/null 2>&1 || true
exit "$status"
