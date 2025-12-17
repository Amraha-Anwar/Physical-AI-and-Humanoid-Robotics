---
id: 002
title: specify-chatkit-ui-backend-fix
stage: plan
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: chatkit-ui-backend-fix
branch: 006-chatkit-ui-backend-fix
user: User
command: /sp.plan
labels: ["chatkit", "ui", "plan", "frontend"]
links:
  spec: specs/006-chatkit-ui-backend-fix/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/006-chatkit-ui-backend-fix/plan.md
 - specs/006-chatkit-ui-backend-fix/checklists/plan.md
tests:
 - none
---

## Prompt

sp.plan:
  spec_file: specs/006-chatkit-ui-backend-fix/spec.md
  

  planning_objective:
    Produce a minimal, spec-compliant architectural plan for:
      - Applying "frontend/styles/chatbot.css" to ChatkitWidget
      - Updating the message send handler to communicate with FastAPI backend
    WITHOUT modifying any other UI or backend logic

  hard_constraints:
    - No code generation (architecture-only)
    - No layout changes outside the chat widget
    - Backend logic (agents, retrieval, RAG) must remain intact
    - Only touch files identified: ChatkitWidget / Root.js / chatbot.css

  allowed_outputs:
    - High-level frontend component integration steps
    - File-level impact list
    - Data flow for message sending (frontend → backend)
    - CSS integration plan (class mapping / overrides)
    - Validation checkpoints

  forbidden_outputs:
    - Any backend refactor
    - UI changes outside chat widget
    - New SDKs besides Chatkit

  validation_expectations:
    - ChatkitWidget fully uses chatbot.css styles
    - Messages typed in widget reach FastAPI backend
    - Existing RAG agent orchestration remains functional

  output_format:
    - High-level architecture overview
    - Step-by-step integration plan
    - Minimal file additions / modifications
    - Risk assessment (blast radius = LOW)

## Response snapshot

Created architectural plan:
- Adapt `chatbot.css` to `ChatkitWidget.module.css`.
- Add CSS overrides for Chatkit inner components.
- Verify `ChatkitWidget.js` integration.

## Outcome

- ✅ Impact: Plan defined for detailed UI styling and integration.
- 🧪 Tests: None run.
- 📁 Files: Created plan and checklist.
- 🔁 Next prompts: /sp.tasks
- 🧠 Reflection: Focus on CSS mapping and overriding library styles.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
