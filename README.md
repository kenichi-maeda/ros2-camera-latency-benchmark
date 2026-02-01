# ros2-camera-latency-benchmark

[![Pre-commit](https://github.com/kenichi-maeda/ros2-camera-latency-benchmark/actions/workflows/pre-commit.yaml/badge.svg)](https://github.com/kenichi-maeda/ros2-camera-latency-benchmark/actions/workflows/pre-commit.yaml) [![Test](https://github.com/kenichi-maeda/ros2-camera-latency-benchmark/actions/workflows/test.yaml/badge.svg)](https://github.com/kenichi-maeda/ros2-camera-latency-benchmark/actions/workflows/test.yaml)


## Quick Start

### Environment Entry and Configuration


Then, from within either the project parent folder or the Docker image home directory, run the following
to activate the environment(s).

```bash
# You may need to increase ulimits before running GPU-intensive tasks:
# ulimit -n 64000  # Increase file descriptor limit
# ulimit -u 8192   # Increase process limit (helps for multi-GPU training)

pixi s  # Activate environment, add -e for specific env,
# Envs: gpu|ros2-gpu|ros2-cpu|genesis-gpu|genesis-ros2-gpu|isaaclab-gpu|isaaclab-ros2-gpu|isaaclab-newton-gpu


# On some systems (e.g., RHEL 9.4), you may need the following
# export CONDA_OVERRIDE_GLIBC=2.35
# For Isaac Lab (PhysX)
pixi r -e isaaclab-gpu install-isaaclab
# To use: pixi s -e isaaclab-gpu
# For Isaac Lab Newton (Warp-based)
pixi r install-isaaclab-newton
# To use: pixi s -e isaaclab-newton-gpu

# For ROS, build the ros2_ws (colcon build is auto-configured by ros2_ws/colcon-defaults.yaml)
pixi r build-ros
# To build your package, clone it into ros2_ws/src, and add its name to the colcon-defaults.yaml
# To use: pixi s -e ros2-gpu or ros2-cpu
# Genesis + Isaac Lab environments are still a bit flaky despite my best efforts ;(
```

### Run Commands

All run commands must occur from within the project parent folder or Docker image home directory to function correctly.

Here is where to put the entrypoints your user may care about.

## Community Contributions

If something doesn't work, please create a GitHub issue!
PRs are always welcome!

## Testing and Linting

```bash
pixi run test  # Run with test environment
pixi run lint
```

## Changing License

1. Update `LICENSE.txt` file with your new license
3. Run: `pixi run lint`.

## Acknowledgement

This repository was created from the [pixidock_template created by Gary Lvov](https://github.com/garylvov/pixidock_template), under fair use of the BSD 1-Clause License
