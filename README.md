# ROS2 Tutorials Workspace

This repository includes multiple examples of ROS2 (Robot Operating System) packages. The objective of the examples in this repository is to show the basic concepts of using and simulating applications that use ROS2.

## Packages:

- [**my_first_package**](src/my_first_package) and [**my_second_package**](src/my_first_package)**:** simple packages with a message.
- [**pubsub_package:**](src/pubsub_package) example on how to create two nodes (publisher and subscriber) and perform comunication between them using a topic.
- [**parameters_tutorial:**](src/parameters_tutorial) this package includes a node with a custom parameter that can be modified via console or launch file.
- [**interfaces_tutorial:**](src/interfaces_tutorial) this package includes custom interfaces (messages, services and actions) used in some of the other packages of this workspace.
- [**srvcli_package:**](src/srvcli_package) example on how to create a service server and client.
- [**action_package:**](src/action_package) example on how to create an action server an a client, it also includes an advanced example on how to cancel and modify actions.
- [**diff_drive_sim:**](src/diff_drive_sim) this package includes the simulation of a differential drive robot using the robotics simulator WEBOTS. The package includes multiple applications like SLAM and navigation.
- [**diff_drive_sim_gazebo:**](src/diff_drive_sim) this package includes a basic simulation of a differential drive robot using the robotics simulator GAZEBO FORTRESS.
- [**mecanum_robot_sim:**](src/mecanum_robot_sim) this package includes the simulation of a omnidirectional robot with mecanum wheels using the robotics simulator WEBOTS. The package includes multiple applications like SLAM and navigation.
- [**lifecycle_nodes:**](src/mecanum_robot_sim) this package includes an example on how to use the lifecycle nodes which allow to enable or disable a node using a service.
- [**rosmasterx3_sim:**](src/rosmasterx3_sim) this package includes the simulation of a Yahboom ROSMASTER X3 omnidirectional robot with mecanum wheels using the robotics simulator WEBOTS. The package includes multiple applications like SLAM, navigation and multirobot systems.

## Generate ROS Map Scripts

The scrips in [`generate_ros_map`](generate_ros_map) folder allow you to convert any map into a black-and-white binary file. They also enable you to generate the `.pgm` and `.yaml` files required by packages such as `slam_toolbox`.

## WSL Configuration:

If you are using Windows with WSL, you may need to make some modifications to the network configuration. This applies if you see the following log when running simulations:

```bash
[webots_controller_robot] Cannot connect to Webots instance, retrying for another 50 seconds...
...
[webots_controller_robot] Cannot connect to Webots instance, retrying for another 5 seconds...
[webots_controller_robot] Giving up...
[webots_controller_robot] [ros2run]: Process exited with failure 1
[ERROR] [webots_controller_robot-2]: process has died [pid 2287, exit code 1, cmd '/opt/ros/humble/share/webots_ros2_driver/scripts/webots-controller --robot-name=robot --protocol=tcp --ip-address= --port=1234 ros2 --ros-args -p robot_description:=path/robot.urdf'] 
```

### Configuration Steps

To correctly configure the connection between Windows and WSL, follow these steps:

1.  **In Windows**, open the Command Prompt (CMD) and enter the command `ipconfig`.
2.  Look for the section labeled `Ethernet adapter vEthernet (WSL (Hyper-V firewall))`. Find the line that says `IPv4 Address` and **copy that IP address**. This is the connection address for WSL.
3.  **In WSL (Linux)**, go to the `/etc` folder and modify the `wsl.conf` file to include the following:

```bash
[boot]
systemd=true

[network]
generateResolvConf=false
```

> **Note:** The last line prevents WSL from automatically configuring the IP to connect with Windows, which is the root cause of the conflict.

4.  Once the file is modified and saved, **restart WSL**. You can do this by running `wsl --shutdown` in the Windows Command Prompt (CMD).
5.  **In WSL (Linux)**, return to the `/etc` folder and modify the `resolv.conf` file (create it if it does not exist). Add the following content to this file:

```bash
nameserver "Insert the IP address copied in step 2"
```

6.  Restart WSL again using `wsl --shutdown`.

**IMPORTANT:** Verify in your **Windows Firewall settings** that Webots has permission to connect through both private and public networks.

Complementary material to this repository can be found in my [YouTube Channel](https://youtube.com/playlist?list=PLT81OVhq-1oGK_vuh3fxGKS4t42RWlPXJ&si=C_owJ659ElRTWOvu) (**NOTE**: The videos are in spanish).