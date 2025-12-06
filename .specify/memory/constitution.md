<!--
SYNC IMPACT REPORT
==================
Version: [Initial Template] -> 1.0.0
Principles Modified:
- [PRINCIPLE_1_NAME] -> I. Technical Accuracy & Integrity
- [PRINCIPLE_2_NAME] -> II. Step-by-Step Pedagogy
- [PRINCIPLE_3_NAME] -> III. Consistency & Tone
- [PRINCIPLE_4_NAME] -> IV. Format & Standardization
- [PRINCIPLE_5_NAME] -> V. Code Integrity
- [PRINCIPLE_6_NAME] -> Removed
Added Sections:
- Constraints & Technology Stack
- Development Workflow
Templates Checked:
- .specify/templates/plan-template.md (✅ No changes needed)
- .specify/templates/spec-template.md (✅ No changes needed)
- .specify/templates/tasks-template.md (✅ No changes needed)
-->

# AI/Spec-Driven Book Creation: Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Technical Accuracy & Integrity
All content, code snippets, and hardware specifications must be factually correct and aligned with the official documentation for ROS 2, Gazebo, NVIDIA Isaac, and Docusaurus. This is non-negotiable. Misinformation or deprecated API usage (e.g., ROS 1 patterns in ROS 2 context) is a critical failure.

### II. Step-by-Step Pedagogy
The content must be structured to facilitate student learning, strictly moving from core concepts to practical implementation. The flow must adhere to the defined structure: [Introduction/Setup] -> [Module 1] -> [Module 2] -> [Module 3] -> [Module 4]. Each module must be broken down into logically sequenced chapters.

### III. Consistency & Tone
Maintain a consistent professional/academic tone suitable for a university-level capstone course. Use standardized terminology (e.g., always use "ROS 2" not "ROS 2.0"). The voice should be authoritative yet accessible, guiding the reader through complex physical AI concepts.

### IV. Format & Standardization
All content must be in clean, valid **Markdown (MDX)** syntax, ready for direct inclusion into a Docusaurus documentation structure. No external file dependencies outside the defined project structure are permitted. Visualization tags must be strategically placed for complex concepts (e.g., ROS 2 computation graph, URDF structure).

### V. Code Integrity
All code examples (e.g., Python `rclpy` nodes, URDF snippets) must be complete, syntactically correct, and use appropriate Markdown code blocks with language highlighting. Placeholder code is strictly forbidden unless explicitly pedagogically necessary (and clearly marked).

## Constraints & Technology Stack

**Mandatory Tech Stack**:
- **ROS 2 Humble/Iron** (No ROS 1)
- **Python 3.10+** (`rclpy`)
- **Gazebo** (Simulation)
- **Unity** (Visualization/HRI)
- **NVIDIA Isaac Sim/ROS**
- **LLM/VLA Integration** (OpenAI Whisper/GPT)

**Deployment**:
- Content must be compatible with Docusaurus static site generation.
- Deployment target is GitHub Pages.

## Development Workflow

**Content Generation**:
1.  **Plan**: Define the module and chapter scope based on the Weekly Breakdown.
2.  **Draft**: Generate MDX content adhering to principles.
3.  **Verify**: Check code validity and technical accuracy against official docs.
4.  **Format**: Ensure Docusaurus compatibility (frontmatter, relative links, admonitions).

**Governance Rules**:
- All PRs/changes must verify compliance with the strictly defined module structure.
- No content related to legacy ROS 1 is permitted except for brief context.
- Amendments to this constitution require a version bump and explicit rationale.

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05