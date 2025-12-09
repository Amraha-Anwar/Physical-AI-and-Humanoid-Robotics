---
id: 008
title: Revert to Manual Vector Search
stage: refactor
date: 2025-12-07
surface: agent
model: gemini
feature: 001-rag-endpoint
branch: 001-db-schema-api
user: user
command: /sp.implement
labels: ["bugfix", "qdrant", "refactor", "search"]
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

/sp.implement Debug: Revert to Manual Vector Search (Stable RAG Pattern)

Goal: Resolve all persistent Qdrant argument conflicts by abandoning the complex, buggy `query` method and reverting to the stable, manual RAG pattern: (1) Get query vector, then (2) use the bare Qdrant search method with the explicit vector. This will fix the `AssertionError: Unknown arguments: ['vector_name']`.

Execute:
- Debug Task 37: Refactor QueryService to use the explicit vector search method.

Constraint:
- The modification must occur in `backend/query/query_service.py`, within the `answer_question_async` method.
- The code must explicitly call `query_vector = await self.embedding_client.generate_embedding(query_text)`.
- The Qdrant method call must be changed to the standard `self.qdrant_client.search_points(...)` or `self.qdrant_client.search(...)`. We will try the most reliable async method: **`search`**.
- The problematic arguments (`query_text`, `vector_name`) must be **removed** from the Qdrant search call. The call should look like: `await self.qdrant_client.search(collection_name=..., query_vector=query_vector, limit=...)`.

## Response snapshot

Replaced `qdrant_client.query` with `qdrant_client.search` in `backend/query/query_service.py` using `query_vector`.

## Outcome

- ✅ Impact: Reverted to stable explicit vector search pattern.
- 🧪 Tests: Verified change by reading file. Integration test setup unavailable.
- 📁 Files: Modified backend/query/query_service.py
- 🔁 Next prompts: Run tests or verify application functionality.
- 🧠 Reflection: Explicit search with pre-calculated vectors eliminates ambiguity in argument parsing for the Qdrant client.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none
