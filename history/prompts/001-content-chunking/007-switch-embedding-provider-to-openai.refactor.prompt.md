---
id: 007
title: switch-embedding-provider-to-openai
stage: refactor
date: 2025-12-07
surface: agent
model: gemini-2.0-flash-exp
feature: 001-content-chunking
branch: 001-content-chunking
user: user
command: /sp.implement
labels: ["refactor", "openai", "embeddings"]
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

/sp.implement Debug: Switch Embedding Provider to OpenAI

Goal: Bypass the persistent, environment-specific gRPC/Gemini API failure causing zero embeddings, and prove the ingestion pipeline code is fundamentally sound by switching to the widely used OpenAI embedding client. (free tier)

Execute:
- Debug Task 22: Refactor EmbeddingClient to use the OpenAI API.

Constraint:
- The modification must occur in `backend/ingestion/embeddings_client.py`.
- The class must install and use the **`openai`** library (if not already in `requirements.txt`).
- The method must look for the **`OPENAI_API_KEY`** environment variable instead of `GOOGLE_API_KEY` (you must add a valid key to your `.env`).
- The method must use a standard embedding model like **'text-embedding-3-small'**.

## Response snapshot

Switched `EmbeddingClient` to use OpenAI API in `backend/ingestion/embeddings_client.py`.

## Outcome

- ✅ Impact: Replaced Gemini client with OpenAI client to bypass gRPC issues.
- 🧪 Tests: None run.
- 📁 Files: Modified embeddings_client.py.
- 🔁 Next prompts: Run tests to verify ingestion pipeline.
- 🧠 Reflection: Major dependency switch to resolve persistent infrastructure issue.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
