# ROS2 Tutorials Workspace — Agent Guide

ROS2 Humble colcon workspace with 13 packages under `src/`.

## Build & Run

```bash
# Build everything (or --packages-up-to <name> for a package + its deps)
colcon build
# Fast iteration on Python-only packages (no reinstall needed)
colcon build --symlink-install
# Rebuild a single package (run from workspace root)
colcon build --packages-select <package_name>
```

```bash
# Source before running any node
source install/setup.bash
```

## Test

```bash
colcon test
colcon test --packages-select <package_name>
colcon test --event-handlers console_direct+   # verbose output
colcon test-result --verbose                    # detailed results
```

Tests are `pytest` in `src/<package>/test/`. Standard ROS2 ament linters: `flake8`, `pep257`, `copyright` (copyright always skipped).

## Packages & Dependencies

- **`interfaces_tutorial`** (CMake) — custom `msg/Num`, `msg/Sphere`, `srv/AddTwoInts`, `action/Counter`. Must be built before packages that use it (`pubsub_package`, `srvcli_package`, `action_package`). `colcon build --packages-up-to` handles this automatically.
- **12 Python packages** (`ament_python`), all at version `0.0.0` except `gps_sim_gazebo` (`0.0.1`, Apache 2.0).
- Simulation packages: `diff_drive_sim` (Webots), `diff_drive_sim_gazebo` (Gazebo), `mecanum_robot_sim` (Webots), `rosmasterx3_sim` (Webots), `gps_sim_gazebo` (Gazebo).

## Notable Quirks

- **SLAM Toolbox** configs (`mecanum_robot_sim`, `diff_drive_sim`, `rosmasterx3_sim`) set `use_scan_matching: false` — workaround for a [Webots lidar bug](https://github.com/cyberbotics/webots/issues/5540).
- **`mecanum_robot_sim`**`s robot driver is a Webots controller plugin registered via URDF (not a `console_scripts` entry point). Its `ekf_params.yaml` is unused (EKF params hardcoded in launch files).
- **`generate_ros_map/`** at root is a standalone Python/OpenCV script, not a ROS2 package — run directly with Python.
- No CI, no pre-commit, no formatter config.
- `webots_ros2_driver` provides `WebotsLauncher`, `WebotsController`, `WaitForControllerConnection` used in simulation launch files.
