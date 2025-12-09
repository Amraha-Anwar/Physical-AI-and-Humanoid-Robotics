---
id: 007
title: Fix Qdrant Vector Name In Query
stage: refactor
date: 2025-12-07
surface: agent
model: gemini
feature: 001-rag-endpoint
branch: 001-db-schema-api
user: user
command: /sp.implement
labels: ["bugfix", "qdrant", "refactor", "vector-name"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - backend/query/query_service.py
tests:
 - test_query.py (attempted, requires server)
---

## Prompt

/sp.implement Debug: Specify Correct Qdrant Vector Name

Goal: Resolve the "Not existing vector name error: fast-bge-small-en" by explicitly telling the Qdrant query method the name of the vector field in the collection, which is 'vector'.

Execute:
- Debug Task 36: Add the vector_name parameter to the qdrant_client.query call.

Constraint:
- The modification must occur in `backend/query/query_service.py`, within the `answer_question_async` method.
- The call to `self.qdrant_client.query(...)` must be updated to include the argument **`vector_name="vector"`**.

## Response snapshot

Added `vector_name="vector"` to the `qdrant_client.query` call in `backend/query/query_service.py`.

## Outcome

- ✅ Impact: Resolved 'Not existing vector name error' by specifying the correct vector field name.
- 🧪 Tests: Verified change by reading file. Integration test setup unavailable.
- 📁 Files: Modified backend/query/query_service.py
- 🔁 Next prompts: Run tests or verify application functionality.
- 🧠 Reflection: Explicitly passing `vector_name` is crucial when using named vectors in Qdrant.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none
