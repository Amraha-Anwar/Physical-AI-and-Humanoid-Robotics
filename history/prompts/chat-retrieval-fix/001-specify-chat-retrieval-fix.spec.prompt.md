---
id: 001
title: specify-chat-retrieval-fix
stage: spec
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: chat-retrieval-fix
branch: 005-chat-retrieval-fix
user: User
command: /sp.specify
labels: ["chatkit", "retrieval", "spec", "frontend"]
links:
  spec: specs/005-chat-retrieval-fix/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/005-chat-retrieval-fix/spec.md
 - specs/005-chat-retrieval-fix/checklists/requirements.md
tests:
 - none
---

## Prompt

sp.specify:
  project: Physical AI & Humanoid Robotics – Chat & Retrieval Fix

  objective:
    - Ensure agents retrieve all relevant context from Qdrant & Neon
    - Replace chatsSDK.js with proper Chatkit integration
    - Maintain all existing UI, layout, backend logic, and RAG flow
  non_goals:
    - Do NOT alter other backend endpoints
    - Do NOT modify existing agent orchestration logic
    - Do NOT change frontend layout or styling

  mandatory_corrections:
    - Adjust retrieval wrapper to ensure proper context fetching
    - Remove chatsSDK.js
    - Integrate Chatkit frontend with agent-driven backend

  required_tech_stack:
    backend: FastAPI + OpenAI Agents SDK + Qdrant + Neon
    frontend: Docursaurus + React + Chatkit
  validation_criteria:
    - Queries return context-grounded answers
    - No UI changes
    - Chatkit communicates correctly with backend
    - Existing RAG & Agents logic remains intact

## Response snapshot

Created feature branch 005-chat-retrieval-fix, wrote specification focusing on Chatkit integration and retrieval tuning, and validated it against the checklist.

## Outcome

- ✅ Impact: Created spec for frontend chat fix and retrieval tuning.
- 🧪 Tests: None run.
- 📁 Files: Created spec and checklist.
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: Defined clear boundaries between backend tuning and frontend replacement.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
