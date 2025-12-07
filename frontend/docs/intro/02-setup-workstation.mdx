---
sidebar_position: 2
title: "Setup Guide: Digital Twin Workstation"
---

# Setup Guide: Digital Twin Workstation

To train complex humanoid behaviors and run photorealistic simulations (NVIDIA Isaac Sim), a powerful workstation is required. This machine serves as the "Training Ground" where your Digital Twin lives.

## 2.1 Hardware Requirements

*   **CPU**: Intel Core i9-13900K or AMD Ryzen 9 7950X (High single-core clock preferred for ROS 2 compilation).
*   **GPU**: **NVIDIA RTX 4070 Ti** (12GB VRAM) or higher.
    *   *Why?* Isaac Sim requires heavy Ray Tracing capabilities and substantial VRAM for loading high-fidelity environments.
*   **RAM**: 64GB DDR5 (32GB minimum).
*   **Storage**: 1TB NVMe SSD (Gen 4).
*   **OS**: **Ubuntu 22.04 LTS (Jammy Jellyfish)**.

:::warning Hardware Constraint
Isaac Sim features rely specifically on **NVIDIA RTX** architecture. AMD GPUs are not supported for the physics-accurate rendering pipeline used in this book.
:::

## 2.2 Installing Ubuntu 22.04 & Drivers

1.  **Install Ubuntu 22.04**: Flash a USB drive with the desktop image and install.
2.  **Install NVIDIA Drivers**:
    ```bash
    sudo apt update
    sudo apt install ubuntu-drivers-common
    sudo ubuntu-drivers autoinstall
    sudo reboot
    ```
    *Verify installation*: Run `nvidia-smi`. You should see your RTX 4070 Ti and Driver Version 535+.

## 2.3 Installing ROS 2 Humble

We follow the official Open Robotics installation guide for Debian packages.

1.  **Set Locale**:
    ```bash
    locale  # check for UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```

2.  **Add Sources**:
    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

3.  **Install ROS 2 Packages**:
    ```bash
    sudo apt update
    sudo apt upgrade
    sudo apt install ros-humble-desktop
    sudo apt install ros-dev-tools
    ```

4.  **Environment Setup**:
    Add the sourcing command to your `.bashrc` so ROS 2 is available in every terminal.
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

5.  **Test Installation**:
    Open two terminals.
    *   Terminal 1: `ros2 run demo_nodes_cpp talker`
    *   Terminal 2: `ros2 run demo_nodes_py listener`
    *   *Success Criteria*: You should see "Publishing" messages in T1 and "I heard" messages in T2.

## 2.4 Installing NVIDIA Omniverse & Isaac Sim

1.  Download the **NVIDIA Omniverse Launcher**.
2.  Install **Cache**, **Nucleus**, and **Workstation** from the Exchange tab.
3.  Install **Isaac Sim** (latest stable release).
4.  Launch Isaac Sim and run the "Hello World" sample to verify RTX rendering is active.

:::info Capstone Linkage
This workstation will be used to train the reinforcement learning policies for walking stability in **Module 3** before transferring them to the physical robot.
:::
