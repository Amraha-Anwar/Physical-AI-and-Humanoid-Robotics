---
sidebar_position: 2
title: Physics and Collision Simulation
---

# Physics and Collision Simulation

## Learning Objectives

Upon completing this chapter, you will be able to:

-   Understand the importance of realistic physics simulation for humanoid robotics.
-   Configure rigid body dynamics (mass, inertia) in your robot's URDF/SDF.
-   Define gravity and contact properties within a Gazebo world.
-   Implement collision geometries for accurate physical interactions.
-   Troubleshoot common physics and collision issues in Gazebo.

## Introduction to Realistic Physics Simulation

For humanoid robots, realistic physical simulation is paramount. Accurate physics allow us to:
-   **Test balance and gait**: Crucial for bipedal locomotion.
-   **Simulate interactions**: With objects in the environment.
-   **Evaluate robustness**: Of control algorithms under various conditions.
-   **Generate synthetic data**: For machine learning models.

Gazebo's physics engine provides these capabilities, but it requires careful configuration of your robot model and the simulation environment.

## 1. Configuring Rigid Body Dynamics

The core of realistic physics lies in correctly defining the properties of each `link` in your robot's URDF/SDF. These properties are defined within the `<inertial>` tag of each link.

-   **Mass**: The total mass of the link (in kg).
-   **Inertia Matrix**: A 3x3 matrix (`ixx`, `ixy`, `ixz`, `iyy`, `iyz`, `izz`) that describes how difficult it is to rotate the link about its center of mass. This is often the most challenging part to get right without CAD tools.
-   **Origin**: The center of mass of the link relative to the link's own frame.

Let's enhance our `torso_link` from `humanoid_segment.urdf.xacro` with more realistic inertial properties. For a cylinder, these can be calculated or approximated.

### Example: Inertial Properties for a Cylinder

For a cylinder with mass `m`, radius `r`, and length `l`, aligned with the Z-axis, the inertia matrix is:
- `ixx = iyy = m * (3*r^2 + l^2) / 12`
- `izz = m * r^2 / 2`
- `ixy = ixz = iyz = 0`

We'll use estimated values for simplicity:

```xml
  <!-- BASE LINK: Torso -->
  <link name="torso_link">
    <!-- ... visual and collision tags remain ... -->
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/> <!-- Center of mass at link origin -->
      <mass value="5.0"/> <!-- 5 kg -->
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>
```

**Key Takeaway**: Incorrect `mass` and `inertia` values will lead to unrealistic robot behavior (e.g., floating, excessive wobbling, or inability to balance). Start with reasonable estimates and refine if necessary.

## 2. Gravity and Contact Properties

### Gravity

Gravity is a fundamental force in any physical simulation. In Gazebo, it's typically defined at the world level within the SDF file. Our `empty_humanoid_world.world` already includes gravity by default. You can explicitly set it:

```xml
        <gravity>0 0 -9.8</gravity> <!-- Standard gravity along Z-axis -->
```
This line is often implicit if using a standard `physics` block, but it's good to be aware of.

### Contact Properties and Friction

For bipedal locomotion, the interaction between the robot's feet and the ground is critical. This is governed by contact properties, mainly friction. These are defined within the `<collision>` element's `<surface>` tag.

In our `empty_humanoid_world.world`, the `ground_plane` has friction properties:

```xml
            <surface>
                <friction>
                  <ode>
                    <mu>100</mu>    <!-- Coefficient of static friction -->
                    <mu2>50</mu2>   <!-- Coefficient of dynamic friction -->
                  </ode>
                </friction>
            </surface>
```
-   `mu`: Coefficient of static friction.
-   `mu2`: Coefficient of dynamic friction.

**For your robot's links**, you also need to define friction, especially for parts that will make contact with the environment (e.g., feet, end-effectors). You can add a similar `<surface>` tag inside the `<collision>` of your links if they need to interact with friction.

### Physics Engine Parameters

Gazebo allows you to choose and configure different physics engines (ODE, Bullet, DART, Simbody). The `<physics>` block in the world file controls global simulation parameters:

```xml
        <physics name="1ms" type="ode"> <!-- Using ODE physics engine -->
          <max_step_size>0.001</max_step_size> <!-- Simulation step size (1ms) -->
          <real_time_factor>1.0</real_time_factor> <!-- Run simulation at real-time speed -->
        </physics>
```
-   `max_step_size`: Controls the precision of the physics simulation. Smaller values are more accurate but computationally more expensive.
-   `real_time_factor`: The ratio of simulated time to real time. A value of 1.0 means the simulation attempts to run at real-time speed.

## 3. Implementing Collision Geometries

The `<collision>` element within a URDF `link` defines the robot's physical shape for collision detection. This geometry is often simpler than the `<visual>` geometry to reduce computational load during collision checks.

Gazebo primarily uses the collision geometry to calculate contacts and apply forces.

```xml
  <!-- BASE LINK: Torso -->
  <link name="torso_link">
    <!-- ... visual tag ... -->
    <collision name="torso_collision"> <!-- Give collision a unique name -->
      <geometry>
        <cylinder length="${torso_length}" radius="${torso_radius}"/>
      </geometry>
      <!-- Optional: Add origin if collision geometry is offset from link origin -->
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <!-- Optional: Add surface properties if this link will make contact -->
      <surface>
        <friction>
          <ode>
            <mu>0.9</mu>
            <mu2>0.8</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
    <!-- ... inertial tag ... -->
  </link>
```

**Important**:
-   **Collision shapes should fully enclose the visual shapes.** If not, parts of your robot might pass through objects in the simulation.
-   **Prefer simple collision shapes** (boxes, spheres, cylinders) for performance. Use `<mesh>` for complex shapes only if necessary.
-   **Always define collision geometry**, even for visually insignificant parts, if they might physically interact with the environment or other robot parts.

## Troubleshooting Physics and Collision

-   **Robot floating or sinking**: Incorrect mass or inertia values.
-   **Robot shaking or unstable**: Physics engine parameters (`max_step_size`, `friction`), or unstable joints. Try reducing `max_step_size`.
-   **Robot falling through ground**: Missing collision geometry for the ground or robot parts, or incorrect `origin` for collision.
-   **Unrealistic "bounciness"**: Adjust restitution properties (`bounce`) in the `<surface>` tag.
-   **Excessive sliding**: Adjust friction coefficients (`mu`, `mu2`).

## Conclusion

By carefully defining rigid body dynamics, gravity, and accurate collision geometries, you can create a physically realistic simulation of your humanoid robot in Gazebo. This detailed attention to physics enables more meaningful development and testing of locomotion and interaction algorithms.

---
**Next Steps**: With robust physics in place, the next crucial step is to integrate various simulated sensors into our Gazebo environment, allowing our robot to perceive its surroundings and publish that data to ROS 2.
