> **Historical record / 历史记录**：本页描述其记录日期的状态；当前运行以[中文指南](../docs/getting-started.zh-CN.md) / [English guide](../docs/getting-started.md)为准。Historical failures and paths are not current release claims.

# Environment Compatibility Report (2026-07-23)

Date: 2026-07-23

## Verified state

- Local Isaac Sim: `5.1.0-rc.19` under `/home/gtk/isaac-sim-5.1`.
- Host operating system family: Ubuntu 22.04.
- Installed host ROS 2 distribution: Humble (`/opt/ros/humble`).
- Isaac Lab Docker pin:
  `nvcr.io/nvidia/isaac-sim:5.1.0`.
- Requested project documentation set: Isaac Sim 5.1.0 and ROS 2 Rolling.

## Compatibility decision

The local Isaac Sim 5.1.0 ROS installation guide explicitly supports Humble
and Jazzy. For Ubuntu 22.04 it recommends Humble; Rolling is not listed.
Therefore the normalized Docker environment defaults to Humble for operational
compatibility. Rolling documentation remains mounted for source/API review,
and `ROS_DISTRO_TARGET` can be overridden, but a Rolling bridge combination is
treated as unsupported until separately validated.

References:

- `https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.md`
- `https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.md`

## MCP documentation mount

The persistent MCP configuration now mounts:

- `https://docs.isaacsim.omniverse.nvidia.com/5.1.0`
- `https://docs.ros.org/en/rolling`

The MCP process already attached to the current conversation still retains its
old 6.0.1 filesystem allowlist. A new Codex session is required for the updated
5.1.0 allowlist to take effect. The 5.1.0 files used in this pass were read
directly from the same local documentation tree.

## Host runtime preflight

- Sandbox-external `nvidia-smi` succeeds: GPU 0 is an NVIDIA GeForce RTX 5090
  D v2 with driver `580.126.09` and 24,455 MiB total memory.
- Sandbox-external Docker access succeeds. Docker Server `29.1.3` uses
  `overlay2`, and the registered runtimes include `nvidia`.
- The existing `issac_ocs_docker:latest` image (23.8 GB, created
  2026-06-08) successfully ran `nvidia-smi` with `--gpus all` and detected the
  same RTX 5090 D v2. This verifies the host NVIDIA container runtime.
- The existing image predates this normalization pass. The normalized source
  image has not been rebuilt, so this is not yet an end-to-end validation of
  the current Dockerfile and project copy.
- Host physical memory is approximately 125 GiB. The requested Docker
  `-m 500G` setting is retained as specified, but it is only an upper limit and
  cannot make more physical memory available.
- The normalized Dockerfile's default base image,
  `isaac-lab-ros2:latest`, is not currently present in the local image list.
  Building the normalized image therefore first requires preparing or pulling
  that base image.
