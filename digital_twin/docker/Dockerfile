# syntax=docker/dockerfile:1.7
# X2 digital twin — STACK image: the unmodified navigation stack + the twin's colcon packages.
# Build context: digital_twin/. Named contexts: `vendor` = vendor/ (AgiBot SDK source, not redistributable) and `backend` = ../control_backend
# (the navigation stack source; read, never modified).
#
#   rosdep-manifests -> deps (runtime apt, via rosdep) -> build (colcon: aimdk_msgs, patched Smac, twin packages) -> stack (runtime)
#
# Pinned: base image by digest; navigation2 source by commit. Runs as a non-root user.
ARG BASE_IMAGE=ros:humble-ros-base@sha256:566b87e6dfc48392189a9c32e594e84060fa9948de93b04e536f39475a1f6963
ARG NAV2_TAG=1.1.20
ARG NAV2_COMMIT=a097086719c88f781aa59788eca29ac6ca5e56db

# ---- 1. package manifests only (keeps the rosdep layer cached while code changes)
FROM ${BASE_IMAGE} AS rosdep-manifests
COPY ws/src /src
RUN mkdir /manifests && cd /src && find . -name package.xml -exec cp --parents {} /manifests \;

# ---- 2. runtime dependencies resolved by rosdep from package.xml (exec deps only)
FROM ${BASE_IMAGE} AS deps
ARG DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
COPY --from=rosdep-manifests /manifests /tmp/manifests
# rosdep only RESOLVES the apt packages; we install them with --no-install-recommends (rosdep's own installer would pull recommends:
# compilers, -dev packages, VTK...). python3-opencv drags in VTK/LLVM (~400 MB); the stack only needs cv2.imread, so the
# pinned headless wheel is used instead (4.7.x: 4.8+ needs numpy>=1.22, apt has 1.21) (the manifest still declares python3-opencv; see --skip-keys).
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
 && apt-get update \
 && rosdep update --rosdistro "$ROS_DISTRO" \
 && pkgs="$(rosdep install --from-paths /tmp/manifests --ignore-src -y -s --dependency-types exec --skip-keys 'aimdk_msgs python3-opencv' | sed -n 's/^ *apt-get install -y //p' | tr '\n' ' ')" \
 && [ -n "$pkgs" ] \
 && apt-get install -y --no-install-recommends $pkgs ros-${ROS_DISTRO}-nav2-smac-planner python3-pip \
 && pip3 install --no-cache-dir 'opencv-python-headless==4.7.0.72' \
 && apt-get purge -y --auto-remove python3-pip \
 && rm -rf /tmp/manifests /var/lib/apt/lists/* /root/.cache

# ---- 3. build: vendor aimdk_msgs, the patched Smac planner, the twin packages
FROM deps AS build
ARG DEBIAN_FRONTEND=noninteractive
ARG NAV2_TAG
ARG NAV2_COMMIT
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends build-essential git patch python3-colcon-common-extensions python3-rosdep \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
# AgiBot SDK interface definitions (vendor/, mounted as a named build context)
COPY --from=vendor aimdk_sdk/src/aimdk_msgs /ws/src/aimdk_msgs
# Smac bounds fix on the pinned upstream tag (the stack refuses to start without it; see candidates/nav2_smac_bounds)
COPY --from=backend candidates/nav2_smac_bounds/bounds.patch /tmp/bounds.patch
RUN git clone -q --depth 1 --branch "${NAV2_TAG}" https://github.com/ros-navigation/navigation2 /tmp/nav2 \
 && [ "$(git -C /tmp/nav2 rev-parse HEAD)" = "${NAV2_COMMIT}" ] \
 && mkdir -p /ws/src/smac_only && cp -r /tmp/nav2/nav2_smac_planner /ws/src/smac_only/ \
 && patch -p1 -d /ws/src/smac_only < /tmp/bounds.patch \
 && rm -rf /tmp/nav2
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && rosdep update --rosdistro "$ROS_DISTRO" \
 && rosdep install --from-paths /ws/src --ignore-src -y --dependency-types build --dependency-types buildtool --skip-keys "aimdk_msgs" \
 && rm -rf /var/lib/apt/lists/*
# heavy vendor-level packages first: they only rebuild when the SDK or the patch changes, not when twin code changes
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && colcon build --merge-install --install-base /opt/x2twin/overlay --packages-select aimdk_msgs nav2_smac_planner \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 && python3 - <<'PY'
import hashlib, json, os
lib = '/opt/x2twin/overlay/lib/libnav2_smac_planner_2d.so'
sha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()
json.dump({'library_sha256': sha(lib), 'patch_sha256': sha('/tmp/bounds.patch'), 'base': 'navigation2 ' + os.environ['NAV2_TAG'] + ' (image build)'},
          open('/opt/x2twin/overlay/qazbot_bounds_fix.json', 'w'), indent=1)
PY
# twin packages (change often): copy, resolve their build deps, build
COPY ws/src /ws/src/twin
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && rosdep install --from-paths /ws/src/twin --ignore-src -y --dependency-types build --dependency-types buildtool --skip-keys "aimdk_msgs" \
 && rm -rf /var/lib/apt/lists/*
# SIM-only candidate: MC turn regulator over stock RPP (copy of research/tools/twin/mc_turn_controller, guard adapted to the twin).
# Built always, loaded only when the stack runs with QAZBOT_SIM_MC_TURN=1 (make scenario-turn).
COPY candidates/qazbot_mc_turn /ws/src/mc_turn
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && rosdep install --from-paths /ws/src/mc_turn --ignore-src -y --dependency-types build --dependency-types buildtool \
 && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && . /opt/x2twin/overlay/setup.sh \
 && colcon build --merge-install --install-base /opt/x2twin/overlay --packages-select qazbot_mc_turn \
      --base-paths /ws/src/mc_turn --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && . /opt/x2twin/overlay/setup.sh \
 && colcon build --merge-install --install-base /opt/x2twin/overlay --packages-select x2_twin_sim x2_twin_bringup x2_twin_scenarios

# ---- 3b. tests: unit tests of the twin packages run inside the build (a failing test fails `make test`)
FROM build AS test
ARG DEBIAN_FRONTEND=noninteractive
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends python3-pytest python3-flake8 && rm -rf /var/lib/apt/lists/*
RUN --mount=type=bind,from=backend,source=ws/src,target=/mnt/src \
    cd /mnt/src && find navigation maps -type f ! -name '*.bak*' ! -path '*/__pycache__/*' -exec cp --parents {} /opt/x2twin/stack/ \;
ENV TWIN_STACK_DIR=/opt/x2twin/stack PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && . /opt/x2twin/overlay/setup.sh \
 && python3 -m flake8 --max-line-length 140 --exclude build,install /ws/src/twin \
 && python3 -m pytest -q -p no:cacheprovider /ws/src/twin/x2_twin_sim/test /ws/src/twin/x2_twin_bringup/test \
      /ws/src/twin/x2_twin_scenarios/test

# ---- 4. runtime: the stack
FROM deps AS stack
ARG UID=1000
ARG GID=1000
LABEL org.opencontainers.image.title="x2-twin-stack" \
      org.opencontainers.image.description="X2 navigation stack running against the digital twin" \
      org.opencontainers.image.licenses="Proprietary"
RUN groupadd -g ${GID} twin && useradd -m -u ${UID} -g ${GID} -s /bin/bash twin \
 && install -d -o twin -g twin /results /var/log/twin
COPY --from=build /opt/x2twin/overlay /opt/x2twin/overlay
# The navigation stack source, unmodified (no *.bak*, no caches). Bind-mount over /opt/x2twin/stack for development.
RUN --mount=type=bind,from=backend,source=ws/src,target=/mnt/src \
    install -d /opt/x2twin/stack && cd /mnt/src && find navigation lio maps -type f ! -name '*.bak*' ! -path '*/__pycache__/*' \
      -exec cp --parents {} /opt/x2twin/stack/ \;
# Twin-only: the stack's SIM-candidate guard asks for ROS_LOCALHOST_ONLY=1 (single-container research setup). Here the isolation is the
# internal network + TWIN_SANDBOX=1, so that one check (and only in this image copy) is made to require TWIN_SANDBOX instead.
RUN python3 - <<'PY'
p = '/opt/x2twin/stack/navigation/nav_control.launch.py'
s = open(p).read()
old = ("    if experimental_turn:\n        if any(os.environ.get(k) != v for k,v in "
       "{'QAZBOT_SIM':'1','ROS_DOMAIN_ID':'87','ROS_LOCALHOST_ONLY':'1'}.items()):")
assert s.count(old) == 1, 'turn guard not found'
open(p, 'w').write(s.replace(old, old.replace("'ROS_LOCALHOST_ONLY':'1'", "'TWIN_SANDBOX':'1'")))
PY
# Twin-only I1 switch (QAZBOT_I1_INF_VALID=1): the local costmap's obstacle_layer treats +inf as observed free (NaN stays ignored).
# Applied to the image copy of nav_control.launch.py; the stack source in control_backend is not touched. Off by default.
RUN python3 - <<'PY'
p = '/opt/x2twin/stack/navigation/nav_control.launch.py'
s = open(p).read()
old = "'obstacle_layer.scan.obstacle_max_range': 3.0, 'obstacle_layer.scan.raytrace_max_range': 3.5})"
assert s.count(old) == 1, 'obstacle layer params not found'
new = old[:-2] + ",\n                      'obstacle_layer.scan.inf_is_valid': os.environ.get('QAZBOT_I1_INF_VALID') == '1'})"
open(p, 'w').write(s.replace(old, new))
PY
COPY docker/dds/udp_only.xml /etc/qazbot/dds.xml
COPY docker/entrypoints/ros_entrypoint.sh /usr/local/bin/ros_entrypoint.sh
COPY docker/entrypoints/stack_health.sh /usr/local/bin/twin-stack-health
RUN chmod 0755 /usr/local/bin/ros_entrypoint.sh /usr/local/bin/twin-stack-health
ENV TWIN_SANDBOX=1 ROS_DOMAIN_ID=87 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    TWIN_STACK_DIR=/opt/x2twin/stack QAZBOT_SMAC_PREFIX=/opt/x2twin/overlay \
    FASTRTPS_DEFAULT_PROFILES_FILE=/etc/qazbot/dds.xml TWIN_RESULTS_DIR=/results \
    PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
USER twin
WORKDIR /home/twin
ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]
CMD ["ros2", "launch", "x2_twin_bringup", "twin.launch.py", "plant:=official"]
