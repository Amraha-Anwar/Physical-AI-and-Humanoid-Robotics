---
sidebar_position: 2
title: Packages and rclpy
---

# Packages and `rclpy`

## Learning Objectives

Upon completing this chapter, you will be able to:

-   Create a new ROS 2 Python package using `colcon` and `ros2 pkg create`.
-   Understand the purpose and structure of `package.xml` for dependency management.
-   Implement a basic ROS 2 Publisher node in Python using `rclpy`.
-   Implement a basic ROS 2 Subscriber node in Python using `rclpy`.
-   Compile and run your new ROS 2 Python package.

## Introduction to ROS 2 Packages

In ROS 2, code is organized into **packages**. A package is the fundamental unit of software organization in ROS 2, containing nodes, libraries, configuration files, and other resources. They facilitate code reuse and modular development.

Python is a popular choice for rapid prototyping and developing ROS 2 applications due to its readability and extensive libraries. The `rclpy` client library is the Python interface for ROS 2, providing the necessary APIs to create nodes, publishers, subscribers, services, and actions.

## 1. Creating a ROS 2 Python Package

We will use `colcon` for building ROS 2 packages and `ros2 pkg create` for package creation. `colcon` is a command-line tool that orchestrates the build, test, and install processes of a set of packages.

First, let's create a new `colcon` workspace to house our packages. A workspace is a directory where you store your ROS 2 packages and build them.

```bash
# Create a colcon workspace
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
```

Now, create a new Python package named `my_py_pkg`:

```bash
ros2 pkg create --build-type ament_python my_py_pkg
```

This command creates a directory structure like this:

```
colcon_ws/src/
└── my_py_pkg/
    ├── my_py_pkg/
    │   └── __init__.py
    ├── package.xml
    ├── setup.py
    └── resource/
        └── my_py_pkg
```

### `package.xml`

The `package.xml` file is crucial for defining metadata about your package, including its name, version, description, maintainers, license, and most importantly, its **dependencies**.

Open `my_py_pkg/package.xml` and ensure it has (or add) the following dependencies:

```xml
<dependency>rclpy</dependency>
<dependency>std_msgs</dependency>
```

-   `rclpy`: The Python client library for ROS 2.
-   `std_msgs`: Contains standard message types (like `String`, `Int32`, etc.) that are commonly used.

### `setup.py`

For Python packages, `setup.py` is used by Python's `setuptools` to define how the package is built and installed. When creating a Python package with `ros2 pkg create`, a basic `setup.py` is generated. We'll modify this file to declare our executable scripts (nodes).

## 2. Implementing a Publisher Node (`talker`)

A publisher node will periodically send messages to a topic. We'll create a simple "talker" node that publishes "Hello World" messages to a topic named `topic`.

1.  **Create the script:**
    Create a file named `talker.py` inside `my_py_pkg/my_py_pkg/` directory:

    ```bash
    # Ensure you are in the my_py_pkg/my_py_pkg directory
    touch ~/colcon_ws/src/my_py_pkg/my_py_pkg/talker.py
    ```

2.  **Add Python code to `talker.py`:**

    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class MinimalPublisher(Node):

        def __init__(self):
            super().__init__('minimal_publisher')
            self.publisher_ = self.create_publisher(String, 'topic', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            msg.data = 'Hello World: %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1

    def main(args=None):
        rclpy.init(args=args)

        minimal_publisher = MinimalPublisher()

        rclpy.spin(minimal_publisher)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

3.  **Declare the executable in `setup.py`:**
    Open `~/colcon_ws/src/my_py_pkg/setup.py` and add the `entry_points` section inside `setup()` function's `kwargs` (usually after `zip_safe=True`):

    ```python
    # ... other imports and setup config ...

    entry_points={
        'console_scripts': [
            'talker = my_py_pkg.talker:main',
        ],
    },
    ```

    This tells `colcon` to create an executable named `talker` that runs the `main` function from `my_py_pkg.talker` module.

## 3. Implementing a Subscriber Node (`listener`)

A subscriber node will listen for messages on a topic. We'll create a simple "listener" node that subscribes to the `topic` and prints the received messages.

1.  **Create the script:**
    Create a file named `listener.py` inside `my_py_pkg/my_py_pkg/` directory:

    ```bash
    # Ensure you are in the my_py_pkg/my_py_pkg directory
    touch ~/colcon_ws/src/my_py_pkg/my_py_pkg/listener.py
    ```

2.  **Add Python code to `listener.py`:**

    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class MinimalSubscriber(Node):

        def __init__(self):
            super().__init__('minimal_subscriber')
            self.subscription = self.create_subscription(
                String,
                'topic',
                self.listener_callback,
                10)
            self.subscription  # prevent unused variable warning

        def listener_callback(self, msg):
            self.get_logger().info('I heard: "%s"' % msg.data)

    def main(args=None):
        rclpy.init(args=args)

        minimal_subscriber = MinimalSubscriber()

        rclpy.spin(minimal_subscriber)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

3.  **Declare the executable in `setup.py`:**
    Open `~/colcon_ws/src/my_py_pkg/setup.py` again and add the `listener` executable to the `console_scripts` entry points:

    ```python
    # ... other imports and setup config ...

    entry_points={
        'console_scripts': [
            'talker = my_py_pkg.talker:main',
            'listener = my_py_pkg.listener:main', # Add this line
        ],
    },
    ```

## 4. Building and Running the Package

Now that we have our `talker` and `listener` nodes defined, let's build the package and run them.

1.  **Navigate to the workspace root and build:**

    ```bash
    cd ~/colcon_ws
    colcon build --packages-select my_py_pkg
    ```

    If the build is successful, you will see output indicating that `my_py_pkg` was built.

2.  **Source the workspace:**
    After building, you need to "source" the workspace to make the new executables available in your current shell.

    ```bash
    source install/setup.bash
    ```
    *(Remember to source this every time you open a new terminal or add it to your `~/.bashrc`)*

3.  **Run the `talker` node:**
    Open a new terminal, source your workspace, and run the talker:

    ```bash
    source ~/colcon_ws/install/setup.bash
    ros2 run my_py_pkg talker
    ```

    You should see the talker publishing "Hello World" messages.

4.  **Run the `listener` node:**
    Open *another* new terminal, source your workspace, and run the listener:

    ```bash
    source ~/colcon_ws/install/setup.bash
    ros2 run my_py_pkg listener
    ```

    You should see the listener node printing the messages received from the talker. This demonstrates successful inter-node communication via ROS 2 topics.

## Conclusion

You have successfully created a ROS 2 Python package, defined its dependencies, implemented basic publisher and subscriber nodes using `rclpy`, and verified their communication. This forms the foundation for more complex robotic behaviors and data processing within the ROS 2 ecosystem.

---
**Next Steps**: Building upon this understanding, the next chapter will introduce how to define and simulate robot kinematics and dynamics using the Unified Robot Description Format (URDF) and visualize them in `rviz2`.
