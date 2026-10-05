# syntax=docker/dockerfile:1
#
# GPS-Denied Drone Swarm - Gazebo simulation image for NVIDIA cloud GPUs
# Ubuntu 24.04 + ROS 2 Jazzy desktop + Gazebo Harmonic (ros_gz) + RViz2
#
# The Gazebo server always renders its depth cameras off-screen through EGL on the GPU.
# Viewer mode: TurboVNC desktop served to the browser by noVNC on port 6080; the Gazebo GUI client
#              and RViz2 run under VirtualGL (EGL back end) so their OpenGL renders on the GPU too.
# Headless:    no X server, no GUI client.
#
# Build from the repository root (docker/sim/run_docker.sh does this):
#   docker build -t swarm_sim:jazzy -f docker/sim/Dockerfile .

FROM osrf/ros:jazzy-desktop

ARG VIRTUALGL_VERSION=3.1.5
ARG TURBOVNC_VERSION=3.3.1

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    ROS_DISTRO=jazzy \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    VGL_DISPLAY=egl \
    QT_X11_NO_MITSHM=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root \
    PATH=/opt/TurboVNC/bin:/opt/VirtualGL/bin:${PATH}

# Gazebo Harmonic bridge, RViz2, Python deps of coordination/ and validation/, desktop + noVNC
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-rviz2 \
        python3-colcon-common-extensions \
        python3-numpy python3-pandas python3-matplotlib \
        ca-certificates curl procps psmisc \
        libgl1 libegl1 libgles2 libglvnd0 mesa-utils \
        fluxbox xterm x11-xserver-utils xfonts-base \
        novnc websockify \
    && rm -rf /var/lib/apt/lists/*

# VirtualGL + TurboVNC (not packaged by Ubuntu)
RUN arch="$(dpkg --print-architecture)" \
    && curl -fsSL -o /tmp/virtualgl.deb \
        "https://github.com/VirtualGL/virtualgl/releases/download/${VIRTUALGL_VERSION}/virtualgl_${VIRTUALGL_VERSION}_${arch}.deb" \
    && curl -fsSL -o /tmp/turbovnc.deb \
        "https://github.com/TurboVNC/turbovnc/releases/download/${TURBOVNC_VERSION}/turbovnc_${TURBOVNC_VERSION}_${arch}.deb" \
    && apt-get update && apt-get install -y --no-install-recommends /tmp/virtualgl.deb /tmp/turbovnc.deb \
    && rm -rf /tmp/*.deb /var/lib/apt/lists/*

# The NVIDIA container toolkit mounts libEGL_nvidia but not always its glvnd vendor file; without it
# EGL (VirtualGL's EGL back end and Gazebo --headless-rendering) falls back to Mesa llvmpipe.
RUN mkdir -p /usr/share/glvnd/egl_vendor.d \
    && echo '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' \
        > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# coordination/gz_overlay.py imports the gz-transport13 / gz-msgs10 Python bindings. Use the ones
# from the ROS vendor packages; fall back to the OSRF Harmonic packages if they are missing.
RUN source /opt/ros/jazzy/setup.bash \
    && if ! python3 -c "import gz.transport13, gz.msgs10" 2>/dev/null; then \
        curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
            -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
        && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
            > /etc/apt/sources.list.d/gazebo-stable.list \
        && apt-get update && apt-get install -y --no-install-recommends python3-gz-transport13 python3-gz-msgs10 \
        && rm -rf /var/lib/apt/lists/*; \
    fi \
    && python3 -c "import gz.transport13, gz.msgs10"

# Container entrypoint, desktop session and GPU check
COPY docker/sim/entrypoint.sh /usr/local/bin/swarm-entrypoint
COPY docker/sim/xstartup.sh /usr/local/share/swarm/xstartup.sh
COPY docker/sim/check_gpu.sh /usr/local/bin/check-gpu
RUN sed -i 's/\r$//' /usr/local/bin/swarm-entrypoint /usr/local/share/swarm/xstartup.sh /usr/local/bin/check-gpu \
    && chmod +x /usr/local/bin/swarm-entrypoint /usr/local/share/swarm/xstartup.sh /usr/local/bin/check-gpu \
    && echo '<meta http-equiv="refresh" content="0; url=vnc.html?autoconnect=1&resize=remote&reconnect=1">' \
        > /usr/share/novnc/index.html

# Workspace: the repository root is the colcon workspace. Only coordination + simulation are built;
# src/control and src/localization need px4_msgs / gazebo_ros, which this stack does not use.
# Scripts are tracked without the executable bit and may carry CRLF from a Windows checkout.
WORKDIR /root/swarm_ws
COPY . /root/swarm_ws
RUN find . -name '*.sh' -exec sed -i 's/\r$//' {} + -exec chmod +x {} + \
    && source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-select coordination simulation \
    && ln -s /root/swarm_ws/docker/sim/smoke_test.sh /usr/local/bin/smoke-test \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo 'source /root/swarm_ws/install/setup.bash' >> /root/.bashrc

EXPOSE 6080

ENTRYPOINT ["/usr/local/bin/swarm-entrypoint"]
# The entrypoint also adds sensor_profile:=full (depth 320x240 @ 30 Hz) unless one is given
CMD ["rviz:=true"]
