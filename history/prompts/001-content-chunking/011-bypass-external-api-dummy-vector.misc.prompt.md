---
id: 011
title: bypass-external-api-dummy-vector
stage: misc
date: 2025-12-07
surface: agent
model: gemini-2.0-flash-exp
feature: 001-content-chunking
branch: 001-content-chunking
user: user
command: /sp.implement
labels: ["debug", "testing", "dummy-data"]
links:
  spec: specs/001-content-chunking/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/ingestion/embeddings_client.py
tests:
  - null
---

## Prompt

/sp.implement Debug: Bypass External API for Final Verification

Goal: Force the ingestion pipeline to succeed by temporarily replacing the external API call with a dummy vector to prove the entire workflow (chunking, Qdrant formatting, and Neon saving) is correct.

Execute:
- Debug Task 24: Modify the EmbeddingClient to return a static dummy vector.

Constraint:
- The modification must occur in `backend/ingestion/embeddings_client.py`, specifically within the `generate_embedding` method.
- The method must be changed to **NOT** call the external API.
- It must return a static list of 768 floats (e.g., [0.0] * 768), which is the standard dimensionality for the Gemini embedding model your Qdrant collection is expecting.

## Response snapshot

Modified EmbeddingClient to return a static dummy vector, bypassing external API calls.

## Outcome

- ✅ Impact: Enabled full ingestion pipeline verification without external API dependency.
- 🧪 Tests: None run.
- 📁 Files: Modified embeddings_client.py.
- 🔁 Next prompts: Run ingestion pipeline to verify end-to-end functionality.
- 🧠 Reflection: Implemented temporary bypass for debugging.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
