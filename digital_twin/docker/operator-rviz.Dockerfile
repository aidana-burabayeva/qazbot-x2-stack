# syntax=docker/dockerfile:1.7
# X2 digital twin — OPERATOR RViz image. Only open-source ROS packages; no AgiBot binaries. Runs on the internal twin network.
ARG BASE_IMAGE=ros:humble-ros-base@sha256:566b87e6dfc48392189a9c32e594e84060fa9948de93b04e536f39475a1f6963
FROM ${BASE_IMAGE}
ARG DEBIAN_FRONTEND=noninteractive
LABEL org.opencontainers.image.title="x2-twin-operator-rviz"
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-nav2-rviz-plugins ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
      libgl1-mesa-dri libglx-mesa0 mesa-utils x11-utils \
 && rm -rf /var/lib/apt/lists/* \
 && groupadd -g 1000 twin && useradd -m -u 1000 -g 1000 -s /bin/bash twin
COPY docker/dds/udp_only.xml /etc/qazbot/dds.xml
ENV ROS_DOMAIN_ID=88 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=/etc/qazbot/dds.xml \
    LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 XDG_RUNTIME_DIR=/tmp/runtime-twin
USER twin
WORKDIR /home/twin
