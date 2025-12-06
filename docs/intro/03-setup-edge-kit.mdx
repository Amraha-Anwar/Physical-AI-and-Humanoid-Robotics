---
sidebar_position: 3
title: Setup Guide - Physical AI Edge Kit
---

# Setup Guide: Physical AI Edge Kit

## Learning Objectives

Upon completing this guide, you will be able to:

- Understand the role of the NVIDIA Jetson Orin Nano in edge AI for Physical AI.
- Configure and set up your Jetson Orin Nano development environment.
- Integrate the Intel RealSense D435i depth camera with your Jetson Orin Nano.
- Install essential ROS 2 Humble packages and dependencies for Physical AI applications.
- Verify the correct operation of your Edge Kit components.

## Prerequisites

Before you begin, ensure you have:

- Completed the "Setup Guide: Digital Twin Workstation" (docs/intro/setup-workstation.mdx) to establish your development environment.
- A stable internet connection.
- Basic familiarity with Linux command-line operations.

## Hardware Components

This Edge Kit focuses on a cost-effective yet powerful setup for Physical AI development.

| Component                  | Description                                                | Quantity |
| :------------------------- | :--------------------------------------------------------- | :------- |
| NVIDIA Jetson Orin Nano    | Edge AI compute platform, 8GB version recommended          | 1        |
| Intel RealSense D435i      | Depth Camera with IMU, crucial for 3D perception           | 1        |
| Micro SD Card (UHS-I A2)   | 64GB or larger, for Jetson OS                              | 1        |
| USB-C Power Supply         | 15W+ (5V, 3A+) for Jetson Orin Nano                        | 1        |
| USB 3.0 Cable (Type-A to C)| For RealSense data transfer to Jetson                      | 1        |
| HDMI Cable & Display       | For initial Jetson setup (can be headless later)           | 1        |
| USB Keyboard & Mouse       | For initial Jetson setup                                   | 1        |

## 1. NVIDIA Jetson Orin Nano Setup

The Jetson Orin Nano is the brain of your edge AI system, providing the necessary compute for onboard perception, navigation, and control.

### 1.1 Flash JetPack OS

JetPack SDK includes the OS, CUDA, cuDNN, TensorRT, and other developer tools crucial for AI on the Jetson.

1.  **Download NVIDIA SDK Manager:**
    On your **workstation**, download and install [NVIDIA SDK Manager](https://developer.nvidia.com/nvidia-sdk-manager).
2.  **Prepare Jetson for Recovery Mode:**
    -   Ensure the Jetson Orin Nano Developer Kit is powered off.
    -   Connect a jumper wire across `FC REC` and `GND` pins on the `J14` header (or refer to your specific Jetson model's documentation for recovery mode pins).
    -   Connect the Jetson to your workstation via the USB-C cable (usually the `micro-USB` port on older Jetsons, or `USB-C` on Orin Nano).
    -   Power on the Jetson.
3.  **Flash using SDK Manager:**
    -   Launch SDK Manager on your workstation.
    -   Log in with your NVIDIA Developer account.
    -   Select "Jetson Orin Nano Developer Kit" as your target hardware.
    -   Choose the latest **JetPack** version.
    -   Follow the on-screen instructions to flash the OS and install all SDK components. This process can take a significant amount of time.
4.  **Initial Boot & Setup:**
    -   Once flashing is complete, remove the jumper and reboot the Jetson.
    -   Connect an HDMI display, keyboard, and mouse to the Jetson.
    -   Complete the initial Ubuntu setup (language, timezone, user account).

### 1.2 Basic System Configuration

Update and upgrade your system packages.

```bash
sudo apt update
sudo apt full-upgrade -y
sudo reboot # Reboot to apply kernel updates
```

## 2. Intel RealSense D435i Integration

The RealSense D435i provides crucial depth and RGB-D data, along with an Inertial Measurement Unit (IMU), for environmental perception.

### 2.1 Install RealSense SDK (librealsense)

You can install `librealsense` from source or via `apt` packages. For a Jetson, building from source often provides better optimization and access to the latest features.

1.  **Add RealSense repository key:**

    ```bash
    sudo mkdir -p /etc/apt/keyrings
    sudo curl -fsSL https://librealsense.intel.com/Debian/apt-repo/pool/Release.gpg | sudo gpg --dearmor -o /etc/apt/keyrings/librealsense.gpg
    echo "deb [arch=arm64 signed-by=/etc/apt/keyrings/librealsense.gpg] https://librealsense.intel.com/Debian/apt-repo/pool/ jammy main" | sudo tee /etc/apt/sources.list.d/librealsense.list
    ```

2.  **Install SDK:**

    ```bash
    sudo apt update
    sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg -y
    ```

3.  **Connect RealSense Camera:**
    Connect your Intel RealSense D435i camera to a USB 3.0 port on the Jetson Orin Nano.

### 2.2 Verify RealSense Installation

Run the RealSense viewer to ensure the camera is detected and streaming data.

```bash
realsense-viewer
```

You should see live streams from the RGB, Depth, and Infrared sensors. Verify the IMU data is also present.

## 3. ROS 2 Humble for Physical AI

ROS 2 (Robot Operating System 2) will be your primary framework for robot software development. We'll install the Humble Hawksbill distribution, which is compatible with Ubuntu 22.04.

### 3.1 Configure ROS 2 Repositories

1.  **Set locale:**

    ```bash
    sudo apt update && sudo apt install locales -y
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```

2.  **Add ROS 2 GPG key:**

    ```bash
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```

3.  **Add ROS 2 repository:**

    ```bash
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

### 3.2 Install ROS 2 Humble

For Physical AI, we recommend the `ros-humble-desktop` installation which includes ROS 2, `rviz2`, and other useful tools.

```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```

### 3.3 Install RealSense ROS 2 Package

This package provides a ROS 2 interface for the Intel RealSense cameras.

```bash
sudo apt install ros-humble-realsense2-camera -y
```

### 3.4 Environment Setup

Source your ROS 2 environment every time you open a new terminal. For convenience, add it to your `~/.bashrc`.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc # Assuming a colcon workspace later
source ~/.bashrc
```

## Verification Checklist

Use this checklist to ensure all components of your Physical AI Edge Kit are correctly set up.

- [ ] **Jetson Orin Nano OS:**
    - [ ] `jetson_stats` is installed and running (`sudo apt install jetson-stats -y` then `jtop`).
    - [ ] CUDA samples compile and run (`/usr/local/cuda/samples/bin/x86_64/linux/release/deviceQuery`).
- [ ] **Intel RealSense D435i:**
    - [ ] `realsense-viewer` launches and displays live camera streams (RGB, Depth, IR).
    - [ ] Camera is detected: `lsusb | grep "Intel(R) RealSense(TM)"`.
- [ ] **ROS 2 Humble:**
    - [ ] `ros2 doctor` reports no major issues.
    - [ ] You can `ros2 run demo_nodes_cpp talker` and `ros2 run demo_nodes_py listener` in separate terminals and see communication.
    - [ ] RealSense ROS 2 node launches successfully: `ros2 launch realsense2_camera rs_launch.py`.
        - Verify topics are published: `ros2 topic list`. Look for `/camera/depth/image_rect_raw`, `/camera/color/image_raw`, `/camera/imu`.

---
**Next Steps**: With your Edge Kit configured, you're ready to explore foundational Physical AI concepts and develop your first robot behaviors.
