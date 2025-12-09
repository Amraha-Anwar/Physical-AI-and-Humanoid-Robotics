---
id: 020
title: include-query-router-in-main
stage: tasks
date: 2025-12-07
surface: agent
model: gemini-2.0-flash-exp
feature: 001-content-chunking
branch: 001-content-chunking
user: user
command: /sp.implement
labels: ["task-implementation", "api-router"]
links:
  spec: specs/001-content-chunking/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/main.py
tests:
  - null
---

## Prompt

/sp.implement P3.1 Query Engine Setup: Execute Task T038

Goal: Make the new query endpoint accessible by integrating its router into the main FastAPI application.

Execute:
- T038 Include the query router in backend/main.py

Constraint:
- The code must be modified in `backend/main.py`.
- The `APIRouter` from `backend/api/query.py` must be imported and included in the `FastAPI` app instance using `app.include_router()`.
- The router should be prefixed with `/api`.

## Response snapshot

Included query router in `backend/main.py`.

## Outcome

- ✅ Impact: Query endpoint is now accessible via the main FastAPI application.
- 🧪 Tests: None run.
- 📁 Files: Modified backend/main.py.
- 🔁 Next prompts: Restart Uvicorn server and test the /api/query endpoint.
- 🧠 Reflection: Standard FastAPI router inclusion.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
