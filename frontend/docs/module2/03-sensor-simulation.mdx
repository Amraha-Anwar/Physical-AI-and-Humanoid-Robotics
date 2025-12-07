---
sidebar_position: 3
title: Sensor Simulation
---

# Sensor Simulation

## Learning Objectives

Upon completing this chapter, you will be able to:

-   Integrate common robotic sensors (LiDAR, Depth Camera, IMU) into a Gazebo simulated environment.
-   Configure Gazebo plugins for each sensor to simulate realistic data.
-   Understand how to enable these simulated sensors to publish data to ROS 2 topics.
-   Visualize simulated sensor data in RViz2.

## Introduction to Sensor Simulation

Sensors are the robot's eyes and ears, providing crucial information about its internal state and the external environment. Simulating sensors accurately is vital for developing and testing perception, mapping, and navigation algorithms without relying on expensive and delicate hardware. Gazebo, through its extensive plugin system, allows us to model a wide range of sensors and their characteristics.

We will focus on three key sensor types essential for humanoid robotics:
1.  **LiDAR (Light Detection and Ranging)**: Provides 2D or 3D point cloud data for environmental mapping and obstacle avoidance.
2.  **Depth Camera (e.g., RealSense model)**: Offers RGB-D (color and depth) images, critical for object recognition, pose estimation, and 3D reconstruction.
3.  **IMU (Inertial Measurement Unit)**: Measures linear acceleration and angular velocity, essential for odometry, state estimation, and balance control.

## 1. Integrating a Simulated LiDAR

A LiDAR sensor typically scans its environment and provides distance measurements. In Gazebo, this is achieved using a ray sensor plugin.

### 1.1 Add LiDAR to URDF/Xacro

First, let's extend our `humanoid_segment.urdf.xacro` (or a full humanoid URDF) to include a LiDAR sensor. We'll attach it to the `torso_link`.

1.  **Open `humanoid_segment.urdf.xacro`** (or your main robot Xacro file).
2.  **Add a new link for the LiDAR sensor:** It's good practice to create a separate link for each sensor, typically connected by a `fixed` joint.

    ```xml
      <!-- LiDAR Link -->
      <link name="lidar_link">
        <visual>
          <geometry>
            <cylinder radius="0.02" length="0.03"/>
          </geometry>
          <material name="black">
            <color rgba="0 0 0 1"/>
          </material>
        </visual>
        <collision>
          <geometry>
            <cylinder radius="0.02" length="0.03"/>
          </geometry>
        </collision>
        <inertial>
          <mass value="0.1"/>
          <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.00005"/>
        </inertial>
      </link>

      <!-- Fixed joint to attach LiDAR to torso -->
      <joint name="lidar_joint" type="fixed">
        <parent link="torso_link"/>
        <child link="lidar_link"/>
        <origin xyz="0 0.1 0.15" rpy="0 0 0"/> <!-- Offset from torso -->
      </joint>
    ```

### 1.2 Add Gazebo LiDAR Plugin

Now, add the Gazebo plugin to the `lidar_link` to define its behavior and publish to ROS 2. This usually goes inside a `<gazebo>` tag that references the link.

```xml
  <!-- Gazebo plugin for LiDAR -->
  <gazebo reference="lidar_link">
    <material>Gazebo/DarkGrey</material>
    <sensor name="lidar_sensor" type="ray">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-${M_PI}</min_angle>
            <max_angle>${M_PI}</max_angle>
          </horizontal>
          <vertical>
            <samples>1</samples>
            <resolution>1</resolution>
            <min_angle>0</min_angle>
            <max_angle>0</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin filename="libgazebo_ros_ray_sensor.so" name="gazebo_ros_lidar_controller">
        <ros>
          <namespace>/</namespace>
          <argument>~/out:=scan</argument>
          <remap>scan:=/scan</remap> <!-- Remap to /scan topic -->
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
```
This plugin creates a `LaserScan` message on the `/scan` topic, which is standard for 2D LiDAR data.

## 2. Integrating a Simulated Depth Camera (RealSense Model)

Simulating a depth camera like the Intel RealSense D435i involves generating both RGB and depth images, often with an associated point cloud. Gazebo provides specific plugins for this.

### 2.1 Add Depth Camera to URDF/Xacro

Similar to the LiDAR, we'll add a new link for the camera.

```xml
      <!-- Camera Link -->
      <link name="camera_link">
        <visual>
          <geometry>
            <box size="0.03 0.09 0.03"/>
          </geometry>
          <material name="grey">
            <color rgba="0.5 0.5 0.5 1"/>
          </material>
        </visual>
        <collision>
          <geometry>
            <box size="0.03 0.09 0.03"/>
          </geometry>
        </collision>
        <inertial>
          <mass value="0.05"/>
          <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001"/>
        </inertial>
      </link>

      <!-- Fixed joint to attach camera to torso -->
      <joint name="camera_joint" type="fixed">
        <parent link="torso_link"/>
        <child link="camera_link"/>
        <origin xyz="0 0.1 0.2" rpy="0 0 0"/> <!-- Slightly above LiDAR -->
      </joint>
```

### 2.2 Add Gazebo Depth Camera Plugin

For a RealSense-like camera, we use the `libgazebo_ros_depth_camera.so` plugin.

```xml
  <!-- Gazebo plugin for Depth Camera (RealSense) -->
  <gazebo reference="camera_link">
    <material>Gazebo/Grey</material>
    <sensor name="depth_camera_sensor" type="depth">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>30.0</update_rate>
      <camera>
        <horizontal_fov>1.089</horizontal_fov> <!-- ~60 deg -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin filename="libgazebo_ros_depth_camera.so" name="camera_controller">
        <ros>
          <namespace>/</namespace>
          <argument>depth:=depth/image_raw</argument>
          <argument>image:=color/image_raw</argument>
          <argument>points:=depth/color/points</argument>
          <argument>camera_info:=camera_info</argument>
        </ros>
        <camera_name>camera</camera_name>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
```
This plugin publishes `Image` messages (RGB and Depth), `PointCloud2` messages, and `CameraInfo` messages to topics like `/camera/color/image_raw`, `/camera/depth/image_raw`, and `/camera/depth/color/points`.

## 3. Integrating a Simulated IMU

An IMU measures angular velocity and linear acceleration. These are crucial for robot localization and control algorithms.

### 3.1 Add IMU to URDF/Xacro

The IMU is typically placed near the robot's center of mass, often within the main body link (e.g., `torso_link`).

```xml
      <!-- IMU Link -->
      <link name="imu_link">
        <visual>
          <geometry>
            <box size="0.01 0.01 0.01"/>
          </geometry>
          <material name="red">
            <color rgba="1 0 0 1"/>
          </material>
        </visual>
        <collision>
          <geometry>
            <box size="0.01 0.01 0.01"/>
          </geometry>
        </collision>
        <inertial>
          <mass value="0.01"/>
          <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
        </inertial>
      </link>

      <!-- Fixed joint to attach IMU to torso -->
      <joint name="imu_joint" type="fixed">
        <parent link="torso_link"/>
        <child link="imu_link"/>
        <origin xyz="0 0 0" rpy="0 0 0"/> <!-- At center of torso -->
      </joint>
```

### 3.2 Add Gazebo IMU Plugin

Use the `libgazebo_ros_imu_sensor.so` plugin for IMU simulation.

```xml
  <!-- Gazebo plugin for IMU -->
  <gazebo reference="imu_link">
    <material>Gazebo/Red</material>
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>100</update_rate>
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_controller">
        <ros>
          <namespace>/</namespace>
          <argument>~/out:=imu</argument>
          <remap>imu:=/imu</remap> <!-- Remap to /imu topic -->
        </ros>
        <frame_name>imu_link</frame_name>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
        <!-- Noise parameters can be added for more realism -->
      </plugin>
    </sensor>
  </gazebo>
```
This plugin publishes `sensor_msgs/msg/Imu` messages to the `/imu` topic.

## 4. Building, Launching, and Visualizing

1.  **Update `humanoid_description` with new Xacro (if you copied to a new file, otherwise just update the existing one) and rebuild:**

    ```bash
    cd ~/colcon_ws
    colcon build --packages-select humanoid_description
    source install/setup.bash
    ```
    Ensure your `spawn_humanoid_segment.launch.py` now points to the updated Xacro file. You might need to re-generate the URDF file if you made changes to the Xacro.

2.  **Launch the simulation:**

    ```bash
    ros2 launch humanoid_description spawn_humanoid_segment.launch.py
    ```

3.  **Verify ROS 2 topics:**
    Open a new terminal, source your workspace, and list active topics:

    ```bash
    source ~/colcon_ws/install/setup.bash
    ros2 topic list
    ```
    You should see topics like `/scan`, `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/depth/color/points`, `/imu`.

4.  **Visualize in `RViz2`:**
    Open `RViz2` (either launched from your launch file or separately) and add the following displays:
    -   **`LaserScan`**: Subscribe to `/scan` to see LiDAR data.
    -   **`Image`**: Subscribe to `/camera/color/image_raw` and `/camera/depth/image_raw` to see camera feeds.
    -   **`PointCloud2`**: Subscribe to `/camera/depth/color/points` to see the 3D point cloud.
    -   **`IMU`**: Subscribe to `/imu` (requires `sensor_msgs/Imu` message type, display type depends on RViz2 version).

    You may need to set the `Fixed Frame` in `RViz2` to `odom` or `world` for some displays to function correctly.

## Conclusion

You have successfully integrated and configured simulated LiDAR, depth camera, and IMU sensors into your Gazebo environment. These sensors are now publishing data to ROS 2 topics, providing the necessary perceptual input for your humanoid robot. This detailed simulation setup is crucial for developing and testing advanced perception and navigation algorithms.

---
**Next Steps**: With a fully simulated robot capable of physical interaction and sensing its environment, the next modules will focus on building the robot's "brain" using NVIDIA Isaac Sim for visual SLAM and navigation, and ultimately integrating VLA for high-level cognitive planning.
