# syntax=docker/dockerfile:1.7
# X2 digital twin — SIM image: runtime for the OFFICIAL AgiBot motion controller (mc) and MuJoCo simulator.
# The AgiBot binaries are NOT in the image: they are mounted at run time from twin/vendor (see twin/scripts/fetch_vendor.sh),
# so the image contains only open-source dependencies and our supervisor. Build context: digital_twin/; named context `vendor`.
ARG BASE_IMAGE=ros:humble-ros-base@sha256:566b87e6dfc48392189a9c32e594e84060fa9948de93b04e536f39475a1f6963

# ---- AimDK (official SDK: interface messages + the py_examples we drive MC with), built once
FROM ${BASE_IMAGE} AS aimdk-build
ARG DEBIAN_FRONTEND=noninteractive
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && apt-get update \
 && apt-get install -y --no-install-recommends build-essential cmake python3-colcon-common-extensions libeigen3-dev \
      libyaml-cpp-dev ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-joy ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
 && rm -rf /var/lib/apt/lists/*
COPY --from=vendor aimdk_sdk/src /ws/src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && cd /ws \
 && colcon build --merge-install --install-base /opt/aimdk --packages-up-to py_examples --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# ---- runtime: libraries the official mc and sim need (AgiBot "X2 MuJoCo Motion-Control Simulation", dependency list)
FROM ${BASE_IMAGE} AS sim
ARG DEBIAN_FRONTEND=noninteractive
ARG UID=1000
ARG GID=1000
LABEL org.opencontainers.image.title="x2-twin-sim" \
      org.opencontainers.image.description="Runtime for the official AgiBot mc + MuJoCo simulator (binaries mounted, not baked)"
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      libcurl4 libeigen3-dev libgflags2.2 libgl1 libgl1-mesa-dri libglx-mesa0 libncurses6 liboctomap1.9 libopencv-core4.5d \
      libscrypt0 libyaml-cpp0.7 mesa-utils xvfb x11-utils procps qemu-user-static libassimp5 libboost-filesystem1.74.0 libpcl-common1.12 python3-yaml python3-pil \
      ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-grid-map-msgs ros-${ROS_DISTRO}-grid-map-ros ros-${ROS_DISTRO}-joy \
      ros-${ROS_DISTRO}-rmw-fastrtps-cpp ros-${ROS_DISTRO}-nav-msgs python3-numpy \
 && rm -rf /var/lib/apt/lists/*
RUN groupadd -g ${GID} twin && useradd -m -u ${UID} -g ${GID} -s /bin/bash twin \
 && install -d -o twin -g twin /opt/mc /opt/sim /agibot/data /var/log/twin
COPY --from=aimdk-build /opt/aimdk /opt/aimdk
COPY docker/sim/ /opt/twin-sim/
# map -> physical walls for MuJoCo: one source of truth, tested in the x2_twin_sim package
COPY ws/src/x2_twin_sim/x2_twin_sim/world.py /opt/twin-sim/world.py
RUN chmod -R 0755 /opt/twin-sim && ln -s /opt/twin-sim/twin-sim-health /usr/local/bin/twin-sim-health \
 && ln -s /opt/twin-sim/sim_supervisor.sh /usr/local/bin/twin-sim-up
ENV TWIN_SANDBOX=1 ROS_DOMAIN_ID=87 RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONDONTWRITEBYTECODE=1
USER twin
WORKDIR /home/twin
ENTRYPOINT ["/usr/local/bin/twin-sim-up"]
