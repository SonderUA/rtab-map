FROM ros:noetic-ros-base-focal@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# ROS, Gazebo and the build dependencies used by the original v1.0 packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg build-essential libeigen3-dev \
    ros-noetic-gazebo-ros-pkgs ros-noetic-cmake-modules ros-noetic-serial \
    ros-noetic-tf ros-noetic-dynamic-reconfigure ros-noetic-joy \
    ros-noetic-rtabmap-ros \
    xvfb xauth xterm x11-utils mesa-utils dbus-x11 \
    && rm -rf /var/lib/apt/lists/*

# Xpra exposes the Linux application windows through a local browser.
ARG XPRA_VERSION=5.0.12-r0-1
RUN mkdir -p /etc/apt/keyrings \
    && curl -fsSL --retry 3 https://xpra.org/gpg.asc | gpg --dearmor -o /etc/apt/keyrings/xpra.gpg \
    && echo 'deb [signed-by=/etc/apt/keyrings/xpra.gpg] https://xpra.org/lts focal main' > /etc/apt/sources.list.d/xpra.list \
    && apt-get update && apt-get install -y --no-install-recommends \
       "xpra-common=${XPRA_VERSION}" "xpra-server=${XPRA_VERSION}" "xpra-x11=${XPRA_VERSION}" \
       xpra-html5=5.5-r13-1 \
    && rm -rf /var/lib/apt/lists/*

RUN useradd --create-home --uid 1000 --shell /bin/bash omni \
    && mkdir -p /home/omni/ws/src /home/omni/.ros /home/omni/.gazebo \
    && chown -R omni:omni /home/omni

COPY --chown=omni:omni src/ /home/omni/ws/src/
COPY scripts/ /usr/local/bin/
RUN chmod +x /usr/local/bin/omni-*

ENV DISPLAY=:100 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
    GAZEBO_MODEL_DATABASE_URI="" \
    GAZEBO_MODEL_PATH=/home/omni/ws/src/AeRoSim/uav_simulator/models \
    XDG_RUNTIME_DIR=/tmp/omni-runtime \
    ROS_MASTER_URI=http://localhost:11311

ARG BUILD_JOBS=2
USER omni
WORKDIR /home/omni/ws
RUN BUILD_JOBS="${BUILD_JOBS}" omni-build \
    && printf '\nsource /opt/ros/noetic/setup.bash\nsource /home/omni/ws/devel/setup.bash\n' >> /home/omni/.bashrc

EXPOSE 8080
HEALTHCHECK --interval=10s --timeout=3s --start-period=60s --retries=6 \
    CMD curl -fsS http://localhost:8080/ >/dev/null || exit 1
ENTRYPOINT ["/usr/local/bin/omni-desktop"]
