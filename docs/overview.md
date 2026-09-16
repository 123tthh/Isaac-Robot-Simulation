# Project overview

[中文](overview.zh-CN.md) · [Getting started](getting-started.md) · [Documentation](README.md)

This project provides simulation, keyboard teaching, RGB-D recording and LeRobot
export for the R1 mobile dual-arm robot. The runtime baseline is Isaac Sim 5.1,
ROS 2 Jazzy and OCS2. Conda manages the external ROS Python environment; Isaac
uses its own Python runtime.

Isaac publishes joint, odometry and three RGB-D camera pairs through ROS Bridge.
Pygame controls the base and sends arm targets to OCS2. Recorded trajectories and
six camera streams are cleaned, resampled and converted to LeRobot v2.1/v3.0.

## Entry point

Run **`./run.sh`** from the repository root for help. Prepare the environment using
the [operation guide](getting-started.md) before launching a session.

| Task | Command |
| --- | --- |
| Environment checks | `./run.sh preflight` |
| Build core ROS packages | `./run.sh build` |
| Simulation only | `./run.sh start-isaac` |
| Teaching and six-stream recording | `./run.sh sim1-session --name my_episode` |
| Export both dataset versions | `./run.sh sim1-process outputs/sessions/my_episode/trace.csv --with-v21` |

No arguments show help. Recording starts only through the session/teaching commands.

## Organization and portability

`assets/` contains LFS assets; `projects/ros2_ws/` contains ROS source;
`projects/teleoperation/sim1/` contains the existing teaching and dataset modules;
`projects/physics_parameters/` holds optional parameter experiments.
`scripts/` implements commands, `dependencies/` and `docker/` define dependencies,
`docs/` explains operation, and `reports/` and `evaluations/` retain measurements.
`releases/` holds snapshots; `reproducibility/` holds recovery materials.
`outputs/`, `data/ros2_log/` and `.runtime/` are local runtime content.

Current user documentation uses stable English filenames with `.zh-CN.md` for
Chinese translations. Dates identify historical evidence and releases. Existing
ROS package names remain compatible. Project-owned module directories use descriptive names. The earlier
`scripts/project.sh` still forwards to the current implementation.

Project paths resolve from script locations or the project marker. Set
`ISAAC_SIM_ROOT` for the external simulator installation. Container `/workspace`,
system `/opt/ros` and USD `/World` are intentional namespaces, not author-specific
filesystem paths. Rebuild the ROS workspace after relocation; build caches are
not portable source artifacts.

## Evidence

Six-stream capture and both local LeRobot exports were checked. The final
automatic-base correction was not re-recorded, the official LeRobot training
loader was not executed, and OmniSim object-contact validation remains unresolved.
See [local validation](../reports/VALIDATION_20260916.md) and
[cross-simulator evaluation](../evaluations/omnisim/README.md).
