# Local documentation references:
# /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.md
# /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.md
# /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_python.md
# /home/gtk/ai_docs/docs.ros.org/en/rolling/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.md
# Strict reproducibility baseline from the Isaac Sim 5.1.0 local documentation:
# /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.md
ARG BASE_IMAGE=nvcr.io/nvidia/isaac-sim:5.1.0@sha256:93b0f99635ab126fb5b33298d513c11520f119f0ee60ff8414ccef67ea977829
FROM ${BASE_IMAGE}

USER root

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO_TARGET=jazzy

RUN mkdir -p /var/lib/apt/lists/partial \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gnupg2 \
        lsb-release \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu %s main\n' \
        "$(dpkg --print-architecture)" "$(. /etc/os-release && printf '%s' "$UBUNTU_CODENAME")" \
        > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        bash-completion \
        ca-certificates \
        cmake \
        curl \
        git \
        python3-colcon-common-extensions \
        python3-pip \
        python3-pygame \
        python3-rosdep \
        python3-venv \
        python3-vcstool \
        rsync \
        tmux \
        vim \
        wget \
    && rm -rf /var/lib/apt/lists/*

# Isaac Sim 5.1's official container is Ubuntu 24.04. The local 5.1
# install_ros.md support matrix recommends ROS 2 Jazzy on that platform.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libbrotli-dev \
        libfreetype6-dev \
        libfontconfig1-dev \
        libxft-dev \
        libvtk9-dev \
        libpcl-dev \
        libgmp-dev \
        libmpfr-dev \
        libgl1 \
        libglx-mesa0 \
        libglu1-mesa \
        mesa-utils \
        qtbase5-dev \
        x11-apps \
        "ros-${ROS_DISTRO_TARGET}-ament-lint-common" \
        "ros-${ROS_DISTRO_TARGET}-grid-map-cmake-helpers" \
        "ros-${ROS_DISTRO_TARGET}-grid-map-core" \
        "ros-${ROS_DISTRO_TARGET}-grid-map-msgs" \
        "ros-${ROS_DISTRO_TARGET}-grid-map-cv" \
        "ros-${ROS_DISTRO_TARGET}-grid-map-ros" \
        "ros-${ROS_DISTRO_TARGET}-grid-map-filters" \
        "ros-${ROS_DISTRO_TARGET}-grid-map-demos" \
        "ros-${ROS_DISTRO_TARGET}-moveit-servo" \
        "ros-${ROS_DISTRO_TARGET}-moveit-planners" \
        "ros-${ROS_DISTRO_TARGET}-moveit-simple-controller-manager" \
        "ros-${ROS_DISTRO_TARGET}-moveit-setup-assistant" \
        "ros-${ROS_DISTRO_TARGET}-picknik-ament-copyright" \
        "ros-${ROS_DISTRO_TARGET}-hpp-fcl" \
        "ros-${ROS_DISTRO_TARGET}-pinocchio" \
        "ros-${ROS_DISTRO_TARGET}-ament-lint-auto" \
        "ros-${ROS_DISTRO_TARGET}-ros2-control" \
        "ros-${ROS_DISTRO_TARGET}-ros2-control-cmake" \
        "ros-${ROS_DISTRO_TARGET}-ros2-control-test-assets" \
        "ros-${ROS_DISTRO_TARGET}-ros-gz-sim" \
        "ros-${ROS_DISTRO_TARGET}-ros-gz-bridge" \
        "ros-${ROS_DISTRO_TARGET}-ros-testing" \
        "ros-${ROS_DISTRO_TARGET}-ros2cli" \
        "ros-${ROS_DISTRO_TARGET}-ros2controlcli" \
        "ros-${ROS_DISTRO_TARGET}-ros2launch" \
        "ros-${ROS_DISTRO_TARGET}-ros2node" \
        "ros-${ROS_DISTRO_TARGET}-ros2param" \
        "ros-${ROS_DISTRO_TARGET}-ros2pkg" \
        "ros-${ROS_DISTRO_TARGET}-ros2run" \
        "ros-${ROS_DISTRO_TARGET}-ros2service" \
        "ros-${ROS_DISTRO_TARGET}-ros2topic" \
        "ros-${ROS_DISTRO_TARGET}-controller-manager" \
        "ros-${ROS_DISTRO_TARGET}-diff-drive-controller" \
        "ros-${ROS_DISTRO_TARGET}-forward-command-controller" \
        "ros-${ROS_DISTRO_TARGET}-rmw-fastrtps-cpp" \
        "ros-${ROS_DISTRO_TARGET}-rviz2" \
        "ros-${ROS_DISTRO_TARGET}-rviz-common" \
        "ros-${ROS_DISTRO_TARGET}-rviz-default-plugins" \
        "ros-${ROS_DISTRO_TARGET}-interactive-markers" \
        "ros-${ROS_DISTRO_TARGET}-tf2-ros" \
        "ros-${ROS_DISTRO_TARGET}-tf2-tools" \
        "ros-${ROS_DISTRO_TARGET}-robot-state-publisher" \
        "ros-${ROS_DISTRO_TARGET}-joint-state-publisher" \
        "ros-${ROS_DISTRO_TARGET}-joint-state-publisher-gui" \
        "ros-${ROS_DISTRO_TARGET}-simulation-interfaces" \
    && rm -rf /var/lib/apt/lists/*

# OCS2 MPC-Net requires the C++ SDK specified by the repository's own
# advanced-examples README. Keep the release payload in the build context and
# verify it before installation so the Docker build does not depend on a live
# GitHub download.
ARG ONNXRUNTIME_SHA256=8627cae49b51544ed15bb70c8ecf25b187f22a301c99c1ced5f4f3af5c36b44c
COPY dependencies/docker_source_cache/onnxruntime-linux-x64-1.19.0.tgz /tmp/onnxruntime-linux-x64-1.19.0.tgz
RUN printf '%s  %s\n' \
        "$ONNXRUNTIME_SHA256" \
        /tmp/onnxruntime-linux-x64-1.19.0.tgz \
        | sha256sum -c - \
    && mkdir -p /opt/onnxruntime \
    && tar -xzf /tmp/onnxruntime-linux-x64-1.19.0.tgz \
        --strip-components=1 \
        -C /opt/onnxruntime \
    && ln -s lib /opt/onnxruntime/lib64 \
    && ln -s . /opt/onnxruntime/include/onnxruntime \
    && mkdir -p /usr/local/include \
    && ln -s /opt/onnxruntime/include /usr/local/include/onnxruntime \
    && ln -s /opt/onnxruntime/lib/libonnxruntime.so \
        /usr/local/lib/libonnxruntime.so \
    && printf '%s\n' /opt/onnxruntime/lib \
        > /etc/ld.so.conf.d/onnxruntime.conf \
    && ldconfig \
    && rm /tmp/onnxruntime-linux-x64-1.19.0.tgz

COPY dependencies/docker-python-requirements.lock /tmp/docker-python-requirements.lock
COPY dependencies/docker-wheelhouse /tmp/docker-wheelhouse

# Upgrading the bundled `packaging` distribution removes the original 23.0
# module that Torch's vendored compatibility link targets. Restore that one
# link target from the installed distribution and fail the image build
# immediately if the Isaac Sim Torch runtime is no longer importable.
RUN /isaac-sim/python.sh -m pip install --no-index --find-links /tmp/docker-wheelhouse \
        -r /tmp/docker-python-requirements.lock \
    && install -D -m 0644 \
        /isaac-sim/kit/python/lib/python3.11/site-packages/packaging/_structures.py \
        /isaac-sim/exts/omni.isaac.core_archive/pip_prebundle/packaging/_structures.py \
    && /isaac-sim/python.sh -c \
        'import onnxruntime, packaging, pytest, torch, vuer; print("Isaac Python dependency smoke test: PASS")'

ENV WORKSPACE=/workspace \
    CMAKE_PREFIX_PATH=/opt/onnxruntime \
    ROS2_WS=/workspace/projects/ros2_ws \
    ROS2_INSTALL_DIR=/workspace/projects/ros2_ws/install_docker \
    TRAJECTORY_DIR=/workspace/projects/Trajectory \
    SIM1_DIR=/workspace/projects/Trajectory/SIM1 \
    ROS2_LOG_DIR=/workspace/data/ros2_log \
    ISAAC_USD_PATH=/workspace/assets/scenes/scene.usd \
    OUTPUT_DIR=/workspace/outputs \
    ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_DISTRO_TARGET=jazzy \
    GZ_VERSION=harmonic \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display

COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/shell_aliases.sh /etc/profile.d/isaac_ocs_aliases.sh
COPY scripts/docker_env_check.sh /usr/local/bin/docker_env_check.sh

RUN chmod 0755 \
        /entrypoint.sh \
        /etc/profile.d/isaac_ocs_aliases.sh \
        /usr/local/bin/docker_env_check.sh \
    && printf '\nsource /etc/profile.d/isaac_ocs_aliases.sh\n' >> /root/.bashrc \
    && mkdir -p /workspace \
    && chown isaac-sim:isaac-sim /workspace

WORKDIR /workspace
USER isaac-sim
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
