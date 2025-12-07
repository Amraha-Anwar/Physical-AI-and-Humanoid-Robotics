---
id: 001
title: fix-missing-chat-history-table
stage: implement
date: 2025-12-07
surface: agent
model: gemini-2.0-flash-exp
feature: none
branch: (current branch for this fix)
user: user
command: /sp.implement
labels: ["fix", "backend", "database"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - backend/neon_client.py
tests:
 - none
---

## Prompt

T064 Fix Missing Database Table: Create Chat History Table

Goal: Fix the runtime error caused by the missing 'chat_history' table by adding table creation logic to the database initialization.

Execute:
- T064 Modify `backend/neon_client.py` to ensure the 'chat_history' table is created during application startup if it does not already exist.

Constraint:
- The creation logic must be idempotent (safe to run multiple times).
- The table must include the following columns: `id` (SERIAL PRIMARY KEY), `session_id` (TEXT, NOT NULL), `user_query` (TEXT, NOT NULL), `llm_response` (TEXT, NOT NULL), and `timestamp` (TIMESTAMP DEFAULT NOW()).
- Add an index on `session_id` for efficient history retrieval.

## Response snapshot

Implemented T064: Added 'chat_history' table creation logic and index to `backend/neon_client.py` within the `initialize_neon_db` function.

## Outcome

- ✅ Impact: The missing `chat_history` table runtime error should be resolved.
- 🧪 Tests: Manual verification by starting the backend.
- 📁 Files: Modified `backend/neon_client.py`.
- 🔁 Next prompts: Verify the fix, potentially continue with chat history logging.
- 🧠 Reflection: Important fix for a core chat feature dependency.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): N/A
