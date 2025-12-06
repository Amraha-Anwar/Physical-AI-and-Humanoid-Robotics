# Research & Key Decisions: Humanoid Robotics Book

**Status**: Phase 0 Complete
**Date**: 2025-12-05

## 1. ROS 2 Distribution
- **Decision**: **ROS 2 Humble Hawksbill** (LTS).
- **Rationale**: Standard for Ubuntu 22.04 (the specified OS). Provides long-term stability for educational content. Iron/Rolling are too volatile for a printed/static book.
- **Alternatives**: ROS 2 Iron (shorter support), ROS 1 Noetic (Explicitly forbidden by constitution).

## 2. VLA / LLM Integration
- **Decision**: **OpenAI API** (Python).
- **Rationale**: Industry standard, excellent documentation, matches "GPT model" spec. Simplifies the code examples so students focus on the *robotics* integration rather than obscure API details.
- **Alternatives**: Local LLM (Llama.cpp) - too complex for setup; Google Gemini - valid, but OpenAI specified in clarification to align with "GPT" terminology.

## 3. Visualization vs. Simulation
- **Decision**: **Unity for Visualization Only**.
- **Rationale**: Keeps the focus on ROS 2/Python. Writing C# scripts introduces a new language barrier. Existing ROS-TCP-Connector packages handle the visualization bridge adequately.
- **Alternatives**: Full Unity Sim (PhysX) - duplicates Gazebo/Isaac functionality and requires C#.

## 4. Navigation Stack (Nav2)
- **Decision**: **Conceptual Balance Control**.
- **Rationale**: Bipedal locomotion math (ZMP/LIPM) is graduate-level control theory. The book focuses on *system integration*. We will use Nav2's configuration to tune for "stability" (e.g., slower acceleration, rotation limits) rather than writing a custom controller from scratch.

## 5. Sim-to-Real Workflow
- **Decision**: **Scripted Model Transfer**.
- **Rationale**: Students struggle most with the "gap" between dev PC and robot. Providing explicit `rsync` or `scp` scripts and ROS 2 launch file overrides for the Jetson is critical for the "Capstone" success.

## 6. Documentation Structure
- **Decision**: **Explicit `sidebars.js`**.
- **Rationale**: Provides rigid control over the reading order, ensuring the "Step-by-Step Pedagogy" constitution principle is enforced programmatically.
