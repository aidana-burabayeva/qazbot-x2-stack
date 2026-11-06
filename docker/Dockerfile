# QazBot control_backend LAB environment: build, unit/launch tests, offline bag replay.
# ROS 2 Humble (Ubuntu 22.04), arm64 on the X2 Jetson or amd64/arm64 on a lab PC - no architecture-specific steps.
# System dependencies come from the package.xml files (rosdep). aimdk_msgs (vendor, not in rosdep) and fast_lio
# (built from ws/third_party) are skipped here. Nav2 1.1.18 runtime parity with the robot: fetch the pinned debs
# (ws/third_party/fetch_runtime_debs.sh) and point QAZBOT_RUNTIME at them (docker/lab/build.sh does this).
# The workspace itself is built by docker/tools/build_workspace.sh (via lab/build.sh), never by hand.
# Production does NOT run in this image (see docker/README.md).
ARG ROS_IMAGE=ros:humble-ros-base-jammy
FROM ${ROS_IMAGE}
ENV DEBIAN_FRONTEND=noninteractive RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=1
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential ca-certificates curl git python3-colcon-common-extensions python3-pytest \
      python3-rosdep python3-vcstool python3-yaml \
    && rm -rf /var/lib/apt/lists/*
# rosdep from package.xml only (copied alone so the layer is reused while sources change)
COPY ws/src /tmp/rosdep_src
RUN apt-get update && (rosdep init 2>/dev/null || true) && rosdep update --rosdistro humble \
    && rosdep install --from-paths /tmp/rosdep_src --ignore-src -r -y --rosdistro humble \
         --skip-keys "aimdk_msgs fast_lio" \
    && rosdep install --from-paths /tmp/rosdep_src --ignore-src -r -y --rosdistro humble --dependency-types test \
         --skip-keys "aimdk_msgs fast_lio" \
    && apt-get install -y --no-install-recommends libpcl-dev ros-humble-pcl-conversions libeigen3-dev \
    && rm -rf /var/lib/apt/lists/* /tmp/rosdep_src
# Unprivileged user; the source tree is mounted read-only at /workspace/control_backend, outputs go to /lab.
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} lab 2>/dev/null || true && useradd -m -u ${UID} -g ${GID} -s /bin/bash lab 2>/dev/null || true \
    && mkdir -p /lab && chown ${UID}:${GID} /lab
USER ${UID}:${GID}
WORKDIR /workspace/control_backend
CMD ["bash"]
