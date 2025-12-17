---
id: 003
title: implement-agent-rag-orchestration
stage: tasks
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: agent-rag-orchestration
branch: 004-agent-rag-orchestration
user: User
command: /sp.tasks
labels: ["rag", "agents", "openai-sdk", "tasks"]
links:
  spec: specs/004-agent-rag-orchestration/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/004-agent-rag-orchestration/tasks.md
 - specs/004-agent-rag-orchestration/checklists/tasks.md
tests:
 - none
---

## Prompt

sp.tasks:
  plan_file: specs/004-agent-rag-orchestration/plan.md

  task_generation_objective:
    Decompose the approved architectural plan into minimal,
    sequential, and spec-compliant implementation tasks
    WITHOUT expanding scope or introducing new behavior.

  hard_constraints:
    - No code generation
    - No refactoring tasks
    - No task may alter UI, layout, or frontend behavior
    - No task may change existing RAG logic
    - No task may add new SDKs or libraries
    - No task may expand file impact beyond:
        - agents.py (new)
        - query_service.py (minimal modification only)

  task_rules:
    - Each task must:
        - Have a single responsibility
        - Be independently reviewable
        - Map directly to the architectural plan
    - Tasks must be ordered and dependency-aware
    - Tasks must explicitly state:
        - What is touched
        - What is NOT touched

  mandatory_task_categories:
    - Environment configuration
    - Agent definitions
    - Agent orchestration wiring
    - FastAPI integration touchpoint
    - Validation and regression safety

  forbidden_task_patterns:
    - “Refactor existing code”
    - “Improve performance”
    - “Clean up architecture”
    - “Redesign prompt”
    - “Optimize retrieval”
    - “Update UI or Chatkit components”

  validation_expectations:
    - Tasks collectively implement the plan fully
    - Tasks do not exceed plan scope
    - Tasks preserve existing system behavior
    - Tasks enable safe `sp.implement`

  output_format:
    - Ordered task list
    - Each task includes:
        - Task ID
        - Description
        - Files affected
        - Explicit non-effects
        - Acceptance criteria

## Response snapshot

Created task list for implementing Agent-based RAG using OpenAI Agents SDK.

## Outcome

- ✅ Impact: Task list created for implementation.
- 🧪 Tests: None run.
- 📁 Files: Created tasks and checklist.
- 🔁 Next prompts: /sp.implement
- 🧠 Reflection: Tasks are granular and follow the strict "no refactor" rule.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
