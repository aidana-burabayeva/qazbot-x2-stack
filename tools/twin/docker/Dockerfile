# QazBot X2 digital twin: official AimDK sim image (docs: "X2 MuJoCo Motion-Control Simulation", Docker option)
# plus the Nav2 stack QazBot uses. Built from the official Dockerfile; additions are marked.
FROM osrf/ros:humble-desktop-full
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential ca-certificates cmake curl libcurl4-openssl-dev libeigen3-dev libgflags-dev \
    libgl1 libgl1-mesa-dri libglx-mesa0 libncurses-dev liboctomap1.9 libopencv-dev libscrypt0 \
    libyaml-cpp-dev mesa-utils pkg-config python3-colcon-common-extensions python3-pip \
    ros-humble-cv-bridge ros-humble-grid-map-msgs ros-humble-grid-map-ros ros-humble-joy \
    ros-humble-rmw-fastrtps-cpp \
    # --- QazBot additions (not in the official image) ---
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-smac-planner \
    ros-humble-nav2-regulated-pure-pursuit-controller ros-humble-nav2-velocity-smoother \
    python3-numpy python3-yaml python3-pytest \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
CMD ["bash"]
