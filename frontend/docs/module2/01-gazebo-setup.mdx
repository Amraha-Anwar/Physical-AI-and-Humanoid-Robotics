---
sidebar_position: 1
title: Gazebo Simulation Setup
---

# Gazebo Simulation Setup

## Learning Objectives

Upon completing this chapter, you will be able to:

-   Understand the role of Gazebo (Ignition) as a robot simulator in the ROS 2 ecosystem.
-   Install and configure Gazebo (Ignition) on your system.
-   Create a basic Gazebo world file.
-   Integrate your URDF robot model into a Gazebo simulation.
-   Differentiate between URDF and SDF and understand their complementary roles in robot description.

## Introduction to Gazebo (Ignition)

**Gazebo** is a powerful 3D robot simulator that is widely used in the robotics community. It allows you to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. Gazebo provides robust physics, graphics, and a convenient programming interface.

The current generation of Gazebo, often referred to as **Gazebo (Ignition)**, has a modular architecture and improved integration with ROS 2. It's an essential tool for developing and testing robotic algorithms without requiring physical hardware, offering a safe and repeatable environment for experimentation.

## 1. Installing Gazebo (Ignition)

Gazebo (Ignition) is developed as a set of independent libraries (Ignition Math, Ignition Physics, Ignition Rendering, etc.) and integrated into ROS 2. For Humble Hawksbill on Ubuntu 22.04, the default Gazebo version is usually **Garden** or **Harmonic**. We will install the full desktop version.

```bash
# Add the OSRF repository
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install gazebo-garden gazebo-garden-plugins -y
```

After installation, verify that Gazebo can launch:

```bash
gazebo
```

You should see an empty Gazebo GUI window appear.

## 2. Creating a Basic Gazebo World

A Gazebo **world** describes the environment in which your robot will operate. It defines static objects (like walls, floors), dynamic objects, lights, and sensors. Gazebo worlds are defined using **Simulation Description Format (SDF)** files.

Let's create a simple world with a flat ground plane and a light source.

1.  **Create a `worlds` directory in your `humanoid_description` package:**

    ```bash
    cd ~/colcon_ws/src/humanoid_description
    mkdir -p worlds
    cd worlds
    ```

2.  **Create `empty_humanoid_world.world`:**

    ```xml
    <?xml version="1.0" ?>
    <sdf version="1.8">
      <world name="empty_humanoid_world">
        <physics name="1ms" type="ode">
          <max_step_size>0.001</max_step_size>
          <real_time_factor>1.0</real_time_factor>
        </physics>
        <light name="sun" type="directional">
          <cast_shadows>1</cast_shadows>
          <pose>0 0 10 0 -30 0</pose>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
          <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
          </attenuation>
          <direction>-0.5 0.1 -0.9</direction>
          <spot>
            <inner_angle>0</inner_angle>
            <outer_angle>0</outer_angle>
            <falloff>0</falloff>
          </spot>
        </light>
        <model name="ground_plane">
          <static>true</static>
          <link name="link">
            <collision name="collision">
              <geometry>
                <plane>
                  <normal>0 0 1</normal>
                  <size>100 100</size>
                </plane>
              </geometry>
              <surface>
                <friction>
                  <ode>
                    <mu>100</mu>
                    <mu2>50</mu2>
                  </ode>
                </friction>
              </surface>
            </collision>
            <visual name="visual">
              <geometry>
                <plane>
                  <normal>0 0 1</normal>
                  <size>100 100</size>
                </plane>
              </geometry>
              <material>
                <ambient>0.8 0.8 0.8 1</ambient>
                <diffuse>0.8 0.8 0.8 1</diffuse>
                <specular>0.8 0.8 0.8 1</specular>
              </material>
            </visual>
          </link>
        </model>
      </world>
    </sdf>
    ```
    *File: `empty_humanoid_world.world`*

3.  **Launch Gazebo with the custom world:**

    ```bash
    # Ensure you are in your colcon_ws and have sourced setup.bash
    cd ~/colcon_ws
    source install/setup.bash

    # Set GAZEBO_MODEL_PATH so Gazebo can find models if you add them later
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix humanoid_description)/share/humanoid_description/models

    gazebo --verbose -r src/humanoid_description/worlds/empty_humanoid_world.world
    ```

    You should see Gazebo launch with a grey ground plane.

## 3. Integrating URDF Models into Gazebo

While URDF is excellent for describing a robot's kinematic and dynamic properties, **SDF (Simulation Description Format)** is Gazebo's native format for describing everything in a simulation, including robots, environments, and sensors. SDF is more expressive than URDF, supporting features like nested models, plugins, and more complex sensor definitions.

Fortunately, Gazebo can directly parse URDF files and convert them internally to SDF. For full simulation fidelity, however, you might want to augment your URDF with Gazebo-specific extensions or even write a full SDF model.

To spawn our `humanoid_segment.urdf` from the previous chapter into this Gazebo world, we'll use a ROS 2 package called `ros_ign_gazebo`.

1.  **Install `ros_ign_gazebo`:**

    ```bash
    sudo apt install ros-humble-ros-ign-gazebo -y
    ```

2.  **Create a launch file to spawn the robot:**
    Create a `launch` directory in `humanoid_description` and a file `spawn_humanoid_segment.launch.py`:

    ```bash
    cd ~/colcon_ws/src/humanoid_description
    mkdir -p launch
    cd launch
    ```

    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node

    def generate_launch_description():
        pkg_humanoid_description = get_package_share_directory('humanoid_description')
        urdf_file = os.path.join(pkg_humanoid_description, 'urdf', 'humanoid_segment.urdf')
        world_file = os.path.join(pkg_humanoid_description, 'worlds', 'empty_humanoid_world.world')

        # Launch Gazebo
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')
            ),
            launch_arguments={'ign_args': '-r ' + world_file}.items()
        )

        # Spawn robot
        spawn_entity = Node(
            package='ros_ign_gazebo',
            executable='create',
            output='screen',
            arguments=['-string', open(urdf_file).read(),
                       '-name', 'humanoid_segment',
                       '-x', '0',
                       '-y', '0',
                       '-z', '0.5']
        )

        # Publish robot states
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        )

        # Joint State Publisher GUI for manual joint control in RViz2
        joint_state_publisher_gui = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        )

        return LaunchDescription([
            gazebo,
            spawn_entity,
            robot_state_publisher,
            joint_state_publisher_gui
        ])
    ```
    *File: `spawn_humanoid_segment.launch.py`*

3.  **Update `setup.py` to install launch and world files:**
    Open `~/colcon_ws/src/humanoid_description/setup.py` and add the following inside the `data_files` list:

    ```python
    import os
    from glob import glob
    # ...
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][y]'))), # Add this line for launch files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*urdf*'))), # Add this line for URDF/Xacro files
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*world*'))), # Add this line for world files
    ],
    # ...
    ```

4.  **Rebuild the package:**

    ```bash
    cd ~/colcon_ws
    colcon build --packages-select humanoid_description
    source install/setup.bash
    ```

5.  **Launch the simulation:**

    ```bash
    ros2 launch humanoid_description spawn_humanoid_segment.launch.py
    ```

    You should now see Gazebo launch with your `humanoid_segment` model in the world. You can manipulate its joints using the `joint_state_publisher_gui`.

## URDF vs. SDF: Complementary Roles

-   **URDF (Unified Robot Description Format)**:
    -   **Purpose**: Primarily for kinematic and dynamic description of a single robot. It defines the robot's structure, joints, and some physical properties.
    -   **Strengths**: Simplicity, widely adopted in ROS, good for basic robot visualization and control.
    -   **Limitations**: Cannot describe environments, complex sensors, or nested models effectively.

-   **SDF (Simulation Description Format)**:
    -   **Purpose**: A comprehensive XML format for describing environments, robots, and other objects in a simulator (like Gazebo). It's designed to be more expressive than URDF.
    -   **Strengths**: Supports nested models, advanced physics properties, detailed sensor definitions, environmental elements.
    -   **Limitations**: Can be more complex to write manually, primarily used within Gazebo.

In practice, many workflows involve starting with a URDF for the robot and then either:
1.  Using a `ros_ign_gazebo` to directly spawn the URDF into Gazebo (as we did above).
2.  Creating an SDF file that imports the URDF and adds Gazebo-specific plugins, sensors, and environmental details. This hybrid approach leverages the simplicity of URDF for robot structure and the power of SDF for full simulation fidelity.

## Conclusion

You have successfully set up the Gazebo (Ignition) simulation environment, created a custom world, and integrated your URDF robot model. You also understand the distinct but complementary roles of URDF and SDF in describing robotic systems for simulation. This foundation is crucial for developing and testing complex robotic behaviors in a virtual environment.

---
**Next Steps**: Building on this simulation setup, the next chapter will explore advanced physics modeling and collision detection to ensure realistic robot interactions within Gazebo.
