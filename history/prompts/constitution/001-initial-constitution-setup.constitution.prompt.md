---
id: 001
title: Initial Constitution Setup
stage: constitution
date: 2025-12-05
surface: agent
model: gemini-2.5-flash
feature: none
branch: main
user: User
command: /sp.constitution
labels: [governance, setup, docusaurus]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

/sp.constitution

Project: AI/Spec-Driven Book Creation: Physical AI & Humanoid Robotics (Docusaurus)

Core principles:
- Technical Accuracy: All content, code snippets, and hardware specifications must be factually correct and aligned with the official documentation for ROS 2, Gazebo, NVIDIA Isaac, and Docusaurus.
- Step-by-Step Pedagogy: The content must be structured to facilitate student learning, moving from core concepts to practical implementation, as detailed in the four modules.
- Consistency: Maintain a consistent professional/academic tone suitable for a university-level capstone course. Use standardized terminology (e.g., always use "ROS 2" not "ROS 2.0").

Key standards:
- Output Format: All content must be in clean, valid **Markdown (MDX)** syntax, ready for direct inclusion into a Docusaurus documentation structure.
- Structure Mandate: The book must adhere strictly to the user-provided structure: [Introduction/Setup] -> [Module 1] -> [Module 2] -> [Module 3] -> [Module 4].
- Chapter Granularity: Each Module must be broken down into logically sequenced chapters with high-level, detailed, and accurate content (as demonstrated in the Chapter 1 reference).
- Code Integrity: All code examples (e.g., Python `rclpy` nodes, URDF snippets) must be complete, syntactically correct, and use appropriate Markdown code blocks with language highlighting.
- Visualization Tagging: For complex concepts (e.g., ROS 2 computation graph, URDF structure, Sim-to-Real workflow), embed **Diagram Tags** for instructional value.

Constraints:
- Technology Stack: Content generation must focus exclusively on: **ROS 2 Humble/Iron**, **Python 3.10+ (`rclpy`)**, **Gazebo**, **Unity (for visualization/HRI)**, **NVIDIA Isaac Sim/ROS**, and **LLM/VLA integration (OpenAI Whisper/GPT)**.
- Exclusion: Do NOT include content on the legacy **ROS 1** platform, except for brief explanatory context on why ROS 2 is superior, if necessary.
- Deployment: All final content must be compatible with Docusaurus and deployment to GitHub Pages (e.g., no external file dependencies outside of the defined project structure).

Success criteria:
- The generated Docusaurus Markdown files align perfectly with the required Module/Chapter hierarchy.
- Content is technically accurate and complete for all core concepts in the **Weekly Breakdown**.
- Visualization tags are strategically placed to enhance understanding of complex systems.

## Response snapshot

Updated constitution to v1.0.0 with 5 core principles focusing on technical accuracy, pedagogy, and Docusaurus formatting.

## Outcome

- ✅ Impact: Established project governance and quality standards.
- 🧪 Tests: N/A
- 📁 Files: Updated constitution.md
- 🔁 Next prompts: Start creating content specs.
- 🧠 Reflection: Constitution successfully adapted for a content creation project rather than a pure code project.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none
