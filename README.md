# ROS2 Tutorials Workspace

This repository includes examples of ROS2 (Robot Operating System) packages demonstrating basic concepts and robot simulations using ROS2.

## Prerequisites

- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/)
- [colcon](https://colcon.readthedocs.io/)
- [Webots](https://cyberbotics.com/) (for simulation packages)
- [Gazebo Harmonic](https://gazebosim.org/) (for Gazebo simulation packages)

## Build & Run

```bash
# Build all packages
colcon build
# Build a package and its dependencies
colcon build --packages-up-to <package_name>
# Fast iteration on Python packages (no reinstall needed)
colcon build --symlink-install
# Rebuild a single package
colcon build --packages-select <package_name>
```

```bash
# Source the workspace before running any node
source install/setup.bash
# Run a node
ros2 run <package_name> <node_name>
```

## Test

```bash
# Run all tests
colcon test
# Run tests for a single package
colcon test --packages-select <package_name>
# Verbose output
colcon test --event-handlers console_direct+
# Detailed results
colcon test-result --verbose
```

Tests use `pytest` with standard ROS2 ament linters (`flake8`, `pep257`). Copyright checks are skipped.

## Packages

### Core Concepts

- [**my_first_package**](src/my_first_package) and [**my_second_package**](src/my_second_package): simple packages with a message.
- [**pubsub_package**](src/pubsub_package): publisher and subscriber nodes communicating over a topic.
- [**parameters_tutorial**](src/parameters_tutorial): node with a custom parameter modifiable via console or launch file.
- [**interfaces_tutorial**](src/interfaces_tutorial): custom interfaces (`msg/Num`, `msg/Sphere`, `srv/AddTwoInts`, `action/Counter`) used by other packages.
- [**srvcli_package**](src/srvcli_package): service server and client example.
- [**action_package**](src/action_package): action server and client, including advanced goal cancellation and modification.
- [**lifecycle_nodes**](src/lifecycle_nodes): lifecycle node example for enabling/disabling nodes via service.

### Simulation

- [**diff_drive_sim**](src/diff_drive_sim): differential drive robot simulation in Webots with SLAM and navigation.
- [**diff_drive_sim_gazebo**](src/diff_drive_sim_gazebo): differential drive robot simulation in Gazebo Harmonic.
- [**gps_sim_gazebo**](src/gps_sim_gazebo): GPS sensor simulation in Gazebo with position control.
- [**mecanum_robot_sim**](src/mecanum_robot_sim): omnidirectional mecanum wheel robot in Webots with SLAM and navigation.
- [**rosmasterx3_sim**](src/rosmasterx3_sim): Yahboom ROSMASTER X3 robot in Webots with SLAM, navigation, and multi-robot systems.

## Generate ROS Map Scripts

The scripts in [`generate_ros_map`](generate_ros_map) convert floor plan images into black-and-white binary files and generate `.pgm` / `.yaml` map files required by packages like `slam_toolbox`. Run them directly with Python (standalone, not a ROS2 package).

## WSL Configuration

If you are using Windows with WSL, you may need to modify the network configuration. This applies if you see the following log when running simulations:

```bash
[webots_controller_robot] Cannot connect to Webots instance, retrying for another 50 seconds...
...
[webots_controller_robot] Cannot connect to Webots instance, retrying for another 5 seconds...
[webots_controller_robot] Giving up...
[webots_controller_robot] [ros2run]: Process exited with failure 1
[ERROR] [webots_controller_robot-2]: process has died [pid 2287, exit code 1, cmd '/opt/ros/jazzy/share/webots_ros2_driver/scripts/webots-controller --robot-name=robot --protocol=tcp --ip-address= --port=1234 ros2 --ros-args -p robot_description:=path/robot.urdf'] 
```

### Configuration Steps

1. **In Windows**, open Command Prompt and run `ipconfig`.
2. Find the `Ethernet adapter vEthernet (WSL (Hyper-V firewall))` section and **copy the IPv4 Address** — this is the connection address for WSL.
3. **In WSL**, modify `/etc/wsl.conf` to include:

```bash
[boot]
systemd=true

[network]
generateResolvConf=false
```

> **Note:** The last line prevents WSL from automatically configuring the IP to connect with Windows.

4. Save and **restart WSL** with `wsl --shutdown` in Windows Command Prompt.
5. **In WSL**, modify `/etc/resolv.conf` (create if it does not exist) and add:

```bash
nameserver "Insert the IP address copied in step 2"
```

6. Restart WSL again with `wsl --shutdown`.

**IMPORTANT:** Verify in **Windows Firewall settings** that Webots has permission through both private and public networks.

## Docker

A pre-built Docker image is available with the full workspace and all dependencies (ROS2 Jazzy, Gazebo Harmonic, Webots R2025a) pre-installed. No ROS2 setup required on the host.

> **Requirements:** [Docker](https://docs.docker.com/engine/install/) and [Docker Compose](https://docs.docker.com/compose/install/).

### Pull from Docker Hub

```bash
docker pull alejotoroo/ros2-tutorials:latest
```

### Build from source

```bash
git clone https://github.com/alejotoro-o/ros2-tutorials-ws.git
cd ros2-tutorials-ws
docker compose build          # initial build (~15 min)
docker compose build --no-cache  # force full rebuild if needed
```

### Available profiles

| Profile | Service | Description |
|---------|---------|-------------|
| (default) | `ros2-tutorials` | Core nodes, Gazebo headless |
| `gui` | `ros2-tutorials-gui` | Gazebo GUI + Rviz2 |
| `webots` | `ros2-tutorials-webots` | Webots simulation |

### X11 setup (GUI apps)

If you plan to use Gazebo GUI, Rviz2, or Webots, allow Docker to access your X11 server:

```bash
xhost +local:docker
```

### Launch examples

**Core packages (default profile):**

```bash
# Interactive shell with workspace sourced
docker compose run --rm ros2-tutorials bash

# Run a node
docker compose run --rm ros2-tutorials ros2 run pubsub_package publisher

# Run tests
docker compose run --rm ros2-tutorials colcon test --packages-select pubsub_package
```

**Gazebo headless (default profile):**

```bash
docker compose run --rm ros2-tutorials \
  ros2 launch gps_sim_gazebo spawn_ugv_explorer.launch.py gui:=false
```

**Gazebo GUI + Rviz2 (`gui` profile):**

```bash
docker compose --profile gui run --rm ros2-tutorials-gui \
  ros2 launch diff_drive_sim_gazebo diff_drive_launch.py
```

**Webots simulation (`webots` profile):**

```bash
docker compose --profile webots run --rm ros2-tutorials-webots \
  ros2 launch diff_drive_sim robot_launch.py

docker compose --profile webots run --rm ros2-tutorials-webots \
  ros2 launch mecanum_robot_sim robot_launch.py

docker compose --profile webots run --rm ros2-tutorials-webots \
  ros2 launch rosmasterx3_sim rosmaster_launch.py
```

### Opening a second terminal

To run commands like `teleop_twist_keyboard` alongside a simulation, you need to join the same running container from another terminal.

**Option A — `docker compose up` (recommended for multi-terminal use):**

```bash
# Terminal 1: start the service interactively
docker compose --profile webots up ros2-tutorials-webots

# From the shell that opens, launch your simulation:
ros2 launch diff_drive_sim robot_launch.py

# Terminal 2: join the same container
docker compose --profile webots exec ros2-tutorials-webots bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Option B — find and join a running container:**

```bash
# Terminal 1: launch simulation
docker compose --profile webots run --rm ros2-tutorials-webots bash
ros2 launch diff_drive_sim robot_launch.py

# Terminal 2: find and join
docker ps
docker exec -it <container_id_or_name> bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Webots mesh paths

The `rosmasterx3_sim` package uses STL mesh files referenced by absolute paths in `.wbt` world files. During the Docker build, these paths are automatically rewritten to resolve correctly inside the container via a `sed` step in the `Dockerfile`.

If you add new packages that reference external assets (meshes, textures) via absolute paths in `.wbt` files, you must add a corresponding `sed` rewrite to the Dockerfile's build step to ensure those assets are found at runtime inside the container.

## Reference

See [AGENTS.md](AGENTS.md) for a compact command reference and workspace quirks for agent-assisted workflows.

Complementary material can be found on my [YouTube Channel](https://youtube.com/playlist?list=PLT81OVhq-1oGK_vuh3fxGKS4t42RWlPXJ&si=C_owJ659ElRTWOvu) (videos are in Spanish).
