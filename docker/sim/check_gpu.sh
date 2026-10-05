#!/bin/bash
# Report whether the simulation is rendering on the NVIDIA GPU.
# Run inside the container:  docker exec -it swarm_sim_cloud check-gpu

echo "== nvidia-smi =="
if command -v nvidia-smi > /dev/null; then
    nvidia-smi --query-gpu=index,name,driver_version,utilization.gpu,memory.used,memory.total --format=csv
    echo
    nvidia-smi | sed -n '/Processes:/,$p'
    echo "(Process names are often hidden inside a container; run nvidia-smi on the host to see gz/ruby and rviz2.)"
else
    echo "nvidia-smi not found: the container was started without --gpus all / the NVIDIA container toolkit."
fi

echo
echo "== VirtualGL (viewer mode) =="
if [ -S /tmp/.X11-unix/X1 ]; then
    DISPLAY=:1 vglrun -d "${VGL_DISPLAY:-egl}" glxinfo -B 2>&1 | grep -E "OpenGL (vendor|renderer)|direct rendering" \
        || echo "glxinfo under vglrun failed"
else
    echo "No VNC desktop running (headless mode)."
fi

echo
echo "== Gazebo Ogre2 renderer =="
log="$HOME/.gz/rendering/ogre2.log"
if [ -f "$log" ]; then
    grep -m 5 -E "GL_RENDERER|GL_VENDOR|GL_VERSION|EGL" "$log" || echo "No renderer lines in $log yet"
else
    echo "$log not found (Gazebo has not rendered yet)."
fi

echo
echo "== Gazebo processes =="
pgrep -af "gz sim|rviz2" || echo "none running"

echo
echo "Expected: NVIDIA renderer above. 'llvmpipe' means software rendering (GPU not used)."
