# Data Model: Content Hierarchy

**Description**: The schema for the educational content structure.

## Entities

### 1. Book
- **Title**: Physical AI & Humanoid Robotics
- **Platform**: Docusaurus
- **Target**: Advanced Undergrad/Grad

### 2. Module
- **Definition**: A high-level grouping of related technical concepts.
- **Cardinality**: Strictly 5 (Intro + 4 Technical Modules).
- **Attributes**:
  - `id`: string (e.g., "module1")
  - `title`: string
  - `order`: integer

### 3. Chapter (MDX File)
- **Definition**: A single page of content covering a specific topic.
- **Cardinality**: 14 total.
- **Attributes**:
  - `slug`: string (URL path)
  - `title`: string (Frontmatter)
  - `sidebar_position`: integer (Frontmatter)
  - `content`: MDX string
  - `hardware_ref`: string[] (e.g., ["RTX 4070 Ti", "Jetson Orin"])
  - `capstone_link`: section_id

### 4. Code Example
- **Definition**: A block of executable code.
- **Attributes**:
  - `language`: "python" | "xml" | "bash" | "yaml"
  - `source`: string (The code itself)
  - `file_path`: string (Virtual path, e.g., `src/my_node.py`)

### 5. Diagram
- **Definition**: A visualization of a system.
- **Attributes**:
  - `type`: "Graph" | "Stack" | "Flow"
  - `tool`: "Mermaid" (preferred) or "Image"
