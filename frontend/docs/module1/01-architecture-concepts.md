---
sidebar_position: 1
title: ROS 2 Architecture & Core Concepts
---

# ROS 2 Architecture & Core Concepts

## Learning Objectives

Upon completing this chapter, you will be able to:

-   Understand the fundamental building blocks of ROS 2.
-   Differentiate between the four core communication patterns in ROS 2 (Nodes, Topics, Services, Actions).
-   Explain the role of Data Distribution Service (DDS) in ROS 2's distributed architecture.
-   Compare and contrast the architectural philosophies of ROS 1 and ROS 2.

## Introduction to ROS 2

The Robot Operating System (ROS) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behaviors across a wide variety of robotic platforms. ROS 2 is the latest iteration, re-engineered to address the limitations of ROS 1, particularly concerning real-time performance, multi-robot systems, and embedded hardware integration.

Unlike its predecessor, ROS 2 is built on a **distributed, decentralized architecture** leveraging the Data Distribution Service (DDS) standard. This change is fundamental, moving away from ROS 1's centralized master node to a peer-to-peer communication model, which inherently offers better scalability, reliability, and real-time capabilities.

## Core Communication Patterns

ROS 2 organizes robot applications into a graph of **nodes**, which communicate with each other using well-defined communication patterns.

### 1. Nodes

A **Node** is an executable process that performs computation. Nodes are typically designed to do one thing well, following the Unix philosophy. For example, one node might control a motor, another might process camera data, and yet another might implement a navigation algorithm.

Nodes can be written in various programming languages (Python, C++) and execute independently. Communication between nodes is handled by the underlying ROS 2 middleware (DDS).

### 2. Topics

**Topics** are the primary mechanism for asynchronous, many-to-many, publish-subscribe communication in ROS 2.

-   **Publisher**: A node that sends messages to a topic.
-   **Subscriber**: A node that receives messages from a topic.

When a node publishes data to a topic, any node subscribed to that topic will receive the data. This pattern is ideal for streaming data like sensor readings (e.g., camera images, LiDAR scans), odometry, or joint states.

**Key characteristics:**
-   **Asynchronous**: Publishers don't wait for subscribers to receive messages.
-   **Many-to-many**: Multiple publishers can send to the same topic, and multiple subscribers can listen to the same topic.
-   **Message Types**: Each topic has a defined message type (e.g., `sensor_msgs/msg/Image`, `geometry_msgs/msg/Twist`).

### 3. Services

**Services** provide a synchronous, request-reply communication model. They are used for operations that involve a request for computation and a response with the result.

-   **Service Client**: A node that sends a request and waits for a response.
-   **Service Server**: A node that receives a request, performs a computation, and sends back a response.

This pattern is suitable for discrete tasks that require immediate feedback, such as triggering a robot to perform a specific action (e.g., "turn on a light," "get the current map," "solve an inverse kinematics problem").

**Key characteristics:**
-   **Synchronous**: The client blocks until it receives a response or times out.
-   **One-to-one**: A single request from a client receives a single response from a server.
-   **Service Types**: Each service has a defined request and response structure.

### 4. Actions

**Actions** are a high-level communication pattern designed for long-running, goal-oriented tasks that provide periodic feedback and can be preempted. They combine aspects of both Topics and Services.

-   **Action Client**: Sends a goal, can receive continuous feedback, and eventually a result. Can also send a cancellation request.
-   **Action Server**: Receives a goal, executes the task, sends periodic feedback, and finally a result. Can handle cancellation requests.

Actions are perfect for tasks like "navigate to a specific location," "pick up an object," or "perform a complex manipulation sequence," where progress updates are valuable, and the task might need to be interrupted.

**Key characteristics:**
-   **Asynchronous (Client perspective)**: Client doesn't block entirely, receives feedback.
-   **Goal-oriented**: Defined by a goal, feedback, and result.
-   **Preemptable**: Clients can cancel a running action.

## ROS 2 Architecture Sketch (Diagram Tag)

```mermaid
graph TD
    subgraph "ROS 2 System (DDS Middleware)"
        A[Node A] -- Publish/Subscribe (Topic) --> B[Node B]
        A -- Call/Reply (Service) --> C[Node C]
        A -- Send Goal/Get Feedback/Get Result (Action) --> D[Node D]
        B -- Publish/Subscribe (Topic) --> E[Node E]
        D -- Action Server (Feedback/Result) --> D
    end

    subgraph "Hardware Abstraction"
        E -- Control Commands --> F[Robot Hardware]
        F -- Sensor Data --> E
    end

    subgraph "Tooling"
        G[Rviz2] -- Subscribe to Topics --> B, E
        H[rqt_graph] -- Introspect Graph --> B, C, D, E
    end

    style A fill:#f9f,stroke:#333,stroke-width:2px
    style B fill:#bbf,stroke:#333,stroke-width:2px
    style C fill:#bfb,stroke:#333,stroke-width:2px
    style D fill:#fdd,stroke:#333,stroke-width:2px
    style E fill:#dbf,stroke:#333,stroke-width:2px
    style F fill:#ccc,stroke:#333,stroke-width:2px
    style G fill:#fff,stroke:#333,stroke-width:2px
    style H fill:#fff,stroke:#333,stroke-width:2px

    linkStyle 0 stroke:#00a,stroke-width:1.5px,fill:none;
    linkStyle 1 stroke:#0a0,stroke-width:1.5px,fill:none;
    linkStyle 2 stroke:#a00,stroke-width:1.5px,fill:none;
    linkStyle 3 stroke:#00a,stroke-width:1.5px,fill:none;
    linkStyle 4 stroke:#a0a,stroke-width:1.5px,fill:none;
    linkStyle 5 stroke:#000,stroke-width:1.5px,fill:none;
    linkStyle 6 stroke:#000,stroke-width:1.5px,fill:none;
    linkStyle 7 stroke:#555,stroke-width:1.5px,fill:none;
    linkStyle 8 stroke:#555,stroke-width:1.5px,fill:none;
```
*Figure 1: Simplified ROS 2 System Architecture illustrating core communication patterns.*

## The Role of Data Distribution Service (DDS)

A cornerstone of ROS 2's design is its reliance on the **Data Distribution Service (DDS)** standard for all inter-process communication. DDS is an open international standard for publish-subscribe communication for real-time and embedded systems.

### Key benefits of DDS in ROS 2:

-   **Decentralization**: Unlike ROS 1's `roscore` (a central master node), DDS enables direct peer-to-peer communication between nodes. This eliminates single points of failure, improves fault tolerance, and simplifies multi-robot deployments.
-   **Quality of Service (QoS)**: DDS provides a rich set of QoS policies that allow developers to fine-tune communication behavior. This includes:
    -   **Reliability**: Guaranteed delivery vs. best-effort.
    -   **Durability**: Whether late-joining subscribers receive historical data.
    -   **Liveliness**: Detection of whether publishers are still active.
    -   **History**: How much data to keep for late joiners or transient local storage.
    These policies are critical for real-time and safety-critical applications.
-   **Pluggable Middleware**: ROS 2 abstracts the DDS implementation, allowing different DDS vendors (e.g., Fast RTPS, Cyclone DDS, RTI Connext) to be "plugged in" without changing ROS 2 application code. This provides flexibility and performance optimization options.
-   **Multi-robot & Network Support**: DDS is inherently designed for distributed systems, making ROS 2 suitable for complex multi-robot scenarios and communication across various network topologies.

## ROS 1 vs. ROS 2: A Comparison

Understanding the architectural shift from ROS 1 to ROS 2 is crucial for developers transitioning between the two versions.

| Feature               | ROS 1                                   | ROS 2                                              |
| :-------------------- | :-------------------------------------- | :------------------------------------------------- |
| **Architecture**      | Centralized (`roscore` master node)     | Decentralized (DDS, peer-to-peer)                  |
| **Communication**     | TCP/IP (Topics, Services), XML-RPC      | DDS (Topics, Services, Actions)                    |
| **Real-time**         | Limited (best-effort)                   | Improved (DDS QoS, real-time executors)            |
| **Multi-robot**       | Challenging, often custom solutions     | Built-in (DDS discovery, network isolation)        |
| **Embedded Systems**  | Resource-intensive                      | More suitable (smaller footprint, efficient DDS)   |
| **Quality of Service**| Basic, mostly implicit                  | Explicit and configurable via DDS QoS policies     |
| **Security**          | Lacks built-in security                 | DDS-Security (Authentication, Encryption)          |
| **Language Support**  | Python 2/3, C++                         | Python 3, C++, Java, C#, etc. (more robust APIs)   |
| **Windows/macOS**     | Primarily Linux                         | First-class support for Windows, macOS, Linux      |
| **Long-running Tasks**| Services with external state/feedback   | Dedicated Action interface                         |

The shift to DDS and a decentralized architecture fundamentally improves ROS 2's capabilities in areas critical for modern robotics, such as real-time performance, scalability, and security.

## Conclusion

ROS 2 provides a robust and flexible framework for developing advanced robotic applications. Its decentralized architecture, built upon the DDS standard, and its comprehensive set of communication patterns (Nodes, Topics, Services, Actions) empower developers to create highly capable and distributed robot software. The improvements over ROS 1 make it a powerful choice for contemporary Physical AI and humanoid robotics projects.

---
**Next Steps**: In the next chapter, we will delve into practical aspects of writing ROS 2 nodes, focusing on Python (`rclpy`), and explore how to create your first ROS 2 packages.
