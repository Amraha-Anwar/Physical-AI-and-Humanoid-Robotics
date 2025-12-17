---
id: 003
title: specify-chat-retrieval-fix
stage: tasks
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: chat-retrieval-fix
branch: 005-chat-retrieval-fix
user: User
command: /sp.tasks
labels: ["chatkit", "retrieval", "tasks", "frontend"]
links:
  spec: specs/005-chat-retrieval-fix/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/005-chat-retrieval-fix/tasks.md
 - specs/005-chat-retrieval-fix/checklists/tasks.md
tests:
 - none
---

## Prompt

sp.tasks:
  plan_file: specs/005-chat-retrieval-fix/plan.md

  task_generation_objective:
    Decompose the approved architectural plan into minimal,
    sequential, and spec-compliant implementation tasks
    WITHOUT expanding scope or introducing new behavior.

  hard_constraints:
    - No code generation outside tasks
    - No frontend layout or style changes
    - No modifications to agent orchestration
    - Only touch files identified in plan:
        - query_service.py (backend)
        - Root.js / frontend component (frontend)
        - Remove chatsSDK.js
    - No new SDKs besides Chatkit for frontend

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
    - Backend retrieval tuning
    - Frontend component replacement
    - Removal of old SDK file
    - Validation of context retrieval
    - Chatkit integration check

  forbidden_task_patterns:
    - Refactor existing code
    - Update unrelated endpoints
    - Modify UI layout or styles
    - Add new features or SDKs

  validation_expectations:
    - Tasks collectively implement the plan fully
    - Tasks do not exceed plan scope
    - Tasks preserve existing system behavior

  output_format:
    - Ordered task list
    - Each task includes:
        - Task ID
        - Description
        - Files affected
        - Explicit non-effects
        - Acceptance criteria

## Response snapshot

Created implementation tasks:
1. Tune backend retrieval limit.
2. Install Chatkit.
3. Create `ChatkitWidget`.
4. Integrate into `Root.js`.
5. Remove legacy SDK.
6. Final validation.

## Outcome

- ✅ Impact: Detailed task list created.
- 🧪 Tests: None run.
- 📁 Files: Created tasks and checklist.
- 🔁 Next prompts: /sp.implement
- 🧠 Reflection: Tasks cover both backend tuning and frontend replacement cleanly.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
