---
id: 002
title: specify-chat-retrieval-fix
stage: plan
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: chat-retrieval-fix
branch: 005-chat-retrieval-fix
user: User
command: /sp.plan
labels: ["chatkit", "retrieval", "plan", "frontend"]
links:
  spec: specs/005-chat-retrieval-fix/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/005-chat-retrieval-fix/plan.md
 - specs/005-chat-retrieval-fix/checklists/plan.md
tests:
 - none
---

## Prompt

sp.plan:
  spec_file: specs/005-chat-retrieval-fix/spec.md
  
  planning_objective:
    Produce a minimal, spec-compliant architectural plan for:
      - Adjusting backend retrieval wrapper to ensure full context fetch
      - Replacing chatsSDK.js with Chatkit frontend integration
    WITHOUT modifying any existing UI layout or agent orchestration

  hard_constraints:
    - No code generation
    - No frontend layout changes
    - No refactoring of existing agent orchestration
    - No new SDKs besides Chatkit and existing OpenAI Agents SDK
    - No modification of database schemas

  allowed_outputs:
    - File-level impact list (additive only)
    - Retrieval tuning plan (backend)
    - Chatkit integration flow (frontend)
    - Agent interaction adjustments (textual)

  forbidden_outputs:
    - Code snippets
    - Refactoring suggestions
    - Prompt rewrites outside agents
    - UI changes

  planning_requirements:
    backend:
      - _execute_retrieval wrapper adjustment
      - Ensure Qdrant + Neon fetch all relevant context
    frontend:
      - Remove chatsSDK.js
      - Integrate standard Chatkit component with agent-based backend
    sdk_usage:
      - OpenAI Agents SDK (already implemented)
      - Chatkit (frontend only)

  validation_expectations:
    - Queries return context-grounded answers
    - Chatkit communicates correctly with backend
    - Existing RAG & agent orchestration remains intact
    - UI and layout unchanged

  success_criteria:
    - Architectural plan can be implemented with minimal diff
    - Plan fully satisfies spec.md
    - No violation of non-goals

  output_format:
    - High-level architecture overview
    - Step-by-step integration flow
    - Minimal file addition / modification list
    - Risk assessment (blast radius = LOW)

## Response snapshot

Created architectural plan:
- Tune backend retrieval limit to 10.
- Replace frontend ChatSDK with `@chatscope/chat-ui-kit-react`.
- Integrate via `Root.js` component swap.

## Outcome

- ✅ Impact: Plan defined for frontend swap and backend tuning.
- 🧪 Tests: None run.
- 📁 Files: Created plan and checklist.
- 🔁 Next prompts: /sp.tasks
- 🧠 Reflection: Simple component swap strategy minimizes risk.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
