# Operation and reproduction

[中文](getting-started.zh-CN.md) · [Validation](../reports/VALIDATION_20260916.md)

Run commands from the repository root. Supported local stack: Ubuntu 24.04,
Isaac Sim 5.1, ROS 2 Jazzy, Conda Python 3.12. Isaac uses its own Python 3.11.
Set `ISAAC_SIM_ROOT` to your external installation; project files resolve from
this repository and do not depend on the current user's home directory.

## Prepare

```bash
git lfs install
git lfs pull
python3 scripts/prepare_portable_urdf.py
export ISAAC_SIM_ROOT=/path/to/isaac-sim-standalone-5.1.0-linux-x86_64
source scripts/activate_sim1_conda.sh
./run.sh build
./run.sh preflight
```

A ZIP archive has no usable Git/LFS checkout. Hydrate assets from a Git clone.
Install ROS system dependencies from `dependencies/docker-apt-packages.lock`.
Use `dependencies/sim1-requirements.txt` for the extra Conda wheels (`pip install
--no-deps -r ...`). RViz2 is a ROS system application, not a pip package.
This machine has an ignored `.runtime/ros-jazzy` deb overlay; the activation
script detects it automatically. It is a local convenience, not a distributable
replacement for installing the ROS dependencies. Rebuild after moving the workspace.

Generate the scene wrapper when needed:

```bash
"$ISAAC_SIM_ROOT/python.sh" scripts/prepare_portable_scene.py
```

The wrapper preserves **Z-up, metres and 60 time codes/second** and replaces
missing camera-housing MDL assets with OmniPBR. Do not omit root-layer metadata.

## Complete interactive session

```bash
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source scripts/activate_sim1_conda.sh
./run.sh sim1-session --name my_episode
```

This opens Isaac GUI, starts Play, waits for ROS control, starts pygame with six
camera recording, and opens the 2×3 RGB/depth viewer. Focus the **pygame** window:
`1` left arm, `2` right arm, `3` base; arrows drive the selected base;
`W/S`, `A/D`, `R/F` translate an arm; `V/B` left close/open and `N/M` right close/open.
`Q` finishes the recording and shuts down this session's processes. Data and
logs are under `outputs/sessions/my_episode/`; the trace is `trace.csv`, cameras
are in `trace/camera/`. Existing session names are refused to prevent overwriting.

The session disables RViz's target manager: it must not publish competing arm
goals while pygame is publishing. Joint constraints remain enabled. Current
OCS2 tracking weights were validated with small motions, not arbitrary grasp tasks.

Individual terminals remain available:

```bash
./run.sh start-isaac
./run.sh launch-ocs2 enable_rviz:=false enable_target_manager:=false
./run.sh sim1-teach
./run.sh sim1-camera-grid
```

`start-isaac` alone starts the scene and Play; **recording belongs to the session
or teach command**. `--headless`, `--duration SECONDS`, `--no-play` are available
on `start-isaac`. For RViz marker control, enable its target manager and stop pygame.

## Conversion and verification

```bash
./run.sh sim1-process outputs/sessions/my_episode/trace.csv --with-v21
python scripts/validate_lerobot_export.py \
  projects/teleoperation/sim1/trace_data/lerobot_v21/my_episode_cleaned \
  projects/teleoperation/sim1/trace_data/lerobot_v30/my_episode_cleaned
```

Cleaning removes long idle intervals. Export resamples observations and RGB to
20 FPS using wall-time alignment; `ros_time_sec` preserves simulation time in
raw recordings. Standard LeRobot video features contain three RGB cameras.
Three float32 depth streams remain lossless auxiliary `depth/*.npy` with
`depth/index.parquet`; they are not RGB video features. Raw CSV retains base
commands/odometry. Current LeRobot state/action vectors remain the original
16-element dual-arm/gripper interface; base values are not silently added to it.
`action` is the next measured state, not a calibrated hardware command.

The supplied validator checks metadata, finite state/effort values, exact FPS
timestamps, video frame counts and decoding, and depth paths/dtype/shape.
It does not replace testing a specific official LeRobot training release.

Optional integration exercise, only when a new recording is wanted:
`./run.sh sim1-session --name unique_test --validate-10s`.

## Interfaces and troubleshooting

| Interface | Type / meaning |
| --- | --- |
| `/clock` | simulation clock |
| `/isaac_joint_states` | measured joints from Isaac |
| `/arm_joint_cmd` | OCS2 arm joint position commands |
| `/left_target/stamped`, `/right_target/stamped` | one active end-effector goal publisher per arm |
| `/odom` | chassis odometry, `odom` → `base_link` |
| `/sim1/base_diff_cmd` | Twist base command |
| `/{head,left,right}_cam/color/image_raw` | RGB, reliable QoS |
| `/{head,left,right}_cam/depth/image_rect_raw` | float32 metric depth, reliable QoS |

An inverted scene indicates missing USD root metadata. An earlier native bridge
crash was resolved by updating Kit after enabling ROS extensions and before
opening the scene. Slow control can accompany low rendering FPS; inspect the
runtime log's `playing`, `simulation_time` and `frames` rather than GUI visibility.
Use Fast DDS consistently in Isaac and the external ROS environment. Repeated
subscriptions with mixed middleware previously caused severe image loss/slowdown.
