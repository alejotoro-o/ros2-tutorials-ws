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
- [**mecanum_robot_sim:**](src/diff_drive_sim) this package includes the simulation of a omnidirectional robot with mecanum wheels using the robotics simulator WEBOTS. The package includes multiple applications like SLAM and navigation. 

Complementary material to this repository can be found in my [YouTube Channel](https://youtube.com/playlist?list=PLT81OVhq-1oGK_vuh3fxGKS4t42RWlPXJ&si=C_owJ659ElRTWOvu) (**NOTE**: The videos are in spanish).