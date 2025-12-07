---
sidebar_position: 3
title: Robot Modeling (URDF)
---

# Robot Modeling (URDF)

## Learning Objectives

Upon completing this chapter, you will be able to:

-   Understand the purpose and structure of the Unified Robot Description Format (URDF).
-   Define a basic robot structure using `link` and `joint` elements in URDF/Xacro.
-   Create a simple Xacro file to modularize URDF definitions.
-   Visualize your robot model in ROS 2's `RViz2`.
-   Grasp the concept of the Transform Tree (TF2) and its importance for coordinate frame management.

## Introduction to URDF

The **Unified Robot Description Format (URDF)** is an XML-based file format used in ROS to describe all aspects of a robot model. It specifies the robot's kinematic and dynamic properties, visual appearance, and collision geometry. URDF is essential for simulating robots, path planning, and performing motion control.

A URDF file primarily consists of two main elements:

-   **`<link>`**: Represents a rigid body segment of the robot (e.g., a wheel, a torso, an arm segment). It defines mass, inertia, visual, and collision properties.
-   **`<joint>`**: Represents a connection between two links, defining their kinematic relationship and allowed motion. Joints typically have a type (e.g., `revolute`, `continuous`, `prismatic`, `fixed`).

For complex robots, URDF files can become very large and repetitive. This is where **Xacro** (XML Macros) comes in handy. Xacro allows you to use macros, properties, and mathematical expressions within your robot descriptions, making them more modular, readable, and maintainable.

## 1. Creating a Simple Humanoid Segment (Xacro)

Let's create a simplified URDF model for a humanoid's `torso` connected to a `shoulder_link` by a `shoulder_yaw_joint`. This will demonstrate the core concepts of links and joints.

First, create a new ROS 2 package to hold our robot descriptions. We'll call it `humanoid_description`.

```bash
cd ~/colcon_ws/src
ros2 pkg create --build-type ament_python humanoid_description
```

Inside `humanoid_description`, create a `urdf` directory:

```bash
mkdir -p ~/colcon_ws/src/humanoid_description/urdf
cd ~/colcon_ws/src/humanoid_description/urdf
```

Now, create a Xacro file named `humanoid_segment.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot name="humanoid_segment" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Properties (constants) for easier modification -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_length" value="0.3" />
  <xacro:property name="torso_radius" value="0.08" />
  <xacro:property name="shoulder_link_length" value="0.2" />
  <xacro:property name="shoulder_link_radius" value="0.03" />

  <!-- BASE LINK: Torso -->
  <link name="torso_link">
    <visual>
      <geometry>
        <cylinder length="${torso_length}" radius="${torso_radius}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${torso_length}" radius="${torso_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- JOINT: Connects torso_link to shoulder_link -->
  <joint name="shoulder_yaw_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 ${torso_length/2}" rpy="0 0 0"/> <!-- Top center of torso_link -->
    <axis xyz="0 0 1"/> <!-- Rotates around Z-axis (yaw) -->
    <limit lower="-${M_PI/2}" upper="${M_PI/2}" effort="100" velocity="10"/>
  </joint>

  <!-- CHILD LINK: Shoulder -->
  <link name="shoulder_link">
    <visual>
      <geometry>
        <cylinder length="${shoulder_link_length}" radius="${shoulder_link_radius}"/>
      </geometry>
      <origin xyz="0 0 ${shoulder_link_length/2}" rpy="0 0 0"/> <!-- Offset for visualization -->
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${shoulder_link_length}" radius="${shoulder_link_radius}"/>
      </geometry>
      <origin xyz="0 0 ${shoulder_link_length/2}" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

</robot>
```
*File: `humanoid_segment.urdf.xacro`*

This Xacro file defines two links (`torso_link`, `shoulder_link`) and one `revolute` joint (`shoulder_yaw_joint`) connecting them. The `origin` element in the joint defines the position and orientation of the `child` link relative to the `parent` link's origin.

### Convert Xacro to URDF

Before visualizing, Xacro files must be processed into a plain URDF file:

```bash
ros2 run xacro xacro humanoid_segment.urdf.xacro > humanoid_segment.urdf
```
This command generates `humanoid_segment.urdf` in the same directory.

## 2. Visualizing the Model in RViz2

**RViz2** is a 3D visualization tool for ROS 2 that allows you to view sensor data, robot models, and planning results.

1.  **Install `joint_state_publisher_gui`:**
    This tool allows you to manually control the joint angles of your robot model for visualization purposes.

    ```bash
    sudo apt install ros-humble-joint-state-publisher-gui ros-humble-rviz2 -y
    ```

2.  **Launch `RViz2` and `robot_state_publisher`:**
    You'll need `robot_state_publisher` to publish the robot's state (including its URDF description and joint states) to the ROS 2 system.

    Open a terminal, navigate to your workspace, source it, and run:

    ```bash
    cd ~/colcon_ws
    source install/setup.bash

    ros2 launch urdf_tutorial display.launch.py model:=src/humanoid_description/urdf/humanoid_segment.urdf
    ```
    (Note: `urdf_tutorial` is a standard package that provides `display.launch.py` for easily viewing URDFs).

    If you don't have `urdf_tutorial` installed, you can launch `RViz2` and `robot_state_publisher` manually:

    ```bash
    # Terminal 1: Launch RViz2
    rviz2

    # Terminal 2: Launch robot_state_publisher and joint_state_publisher_gui
    cd ~/colcon_ws
    source install/setup.bash
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="<command_to_get_urdf_content>"
    # A simpler way if you have the file locally:
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat src/humanoid_description/urdf/humanoid_segment.urdf)"

    ros2 run joint_state_publisher_gui joint_state_publisher_gui
    ```

    In `RViz2`:
    -   Add a `RobotModel` display, and ensure its `Description Source` is set to `Robot Description` (default).
    -   Add a `TF` display to see the coordinate frames.
    -   In the `joint_state_publisher_gui` window, you should see a slider for `shoulder_yaw_joint`. Moving this slider will rotate the `shoulder_link` relative to the `torso_link` in `RViz2`.

## 3. The Transform Tree (TF2)

The **Transform Tree (TF2)** is a crucial ROS 2 concept that keeps track of the relationships between all coordinate frames in your robot system. For any given time, TF2 can answer questions like: "What is the pose of the camera relative to the robot's base?" or "Where is the robot's end-effector in the world frame?".

In our `humanoid_segment.urdf.xacro`, each `link` defines its own coordinate frame. The `joint` elements define the transformations between these frames. When `robot_state_publisher` processes the URDF, it publishes these transformations to the ROS 2 system.

### Key concepts of TF2:

-   **Frames**: Every `link` has an associated coordinate frame.
-   **Transforms**: The relationships (translation and rotation) between two frames. These are represented by `tf2_msgs/msg/TFMessage`.
-   **`tf2_ros`**: The client library for TF2, allowing you to query transforms between any two frames at any point in time.
-   **Tree structure**: All frames form a tree structure, where each frame has one parent, except for the root frame.

You can visualize the TF tree using `rqt_tf_tree`:

```bash
ros2 run rqt_tf_tree rqt_tf_tree
```
This tool will graphically display the parent-child relationships between your robot's links as defined by the URDF and published by `robot_state_publisher`. You should see `torso_link` as the parent of `shoulder_link`.

## Conclusion

You've learned how to describe your robot's physical structure using URDF/Xacro, enabling you to define links and joints. You then visualized this model in `RViz2` and explored the critical concept of the Transform Tree (TF2), which manages coordinate frames essential for all robot navigation and manipulation tasks.

---
**Next Steps**: With your core ROS 2 knowledge established, we will move on to simulating your robot in a virtual environment using Gazebo.
