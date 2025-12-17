id: 010
title: Implement Groq Integration
stage: implement
date: 2025-12-17
surface: agent
model: gemini-2.0-flash-exp
feature: fix-rag-runtime
branch: 007-fix-rag-runtime
user: user
command: /sp.implement
labels: [implement, feature, groq, embeddings]
links:
  spec: specs/007-fix-rag-runtime/spec.md
  plan: specs/007-fix-rag-runtime/plan.md
  tasks: specs/007-fix-rag-runtime/tasks.md
  phr: history/prompts/fix-rag-runtime/010-implement-groq-integration.implement.prompt.md
files:
  - backend/ingestion/embeddings_client.py
tests: []
---
/sp.implement T095 Revised Integrate Groq API for Embeddings and Retain Gemini for Agent

RESPONSE_TEXT
Implemented T095 to switch embeddings to Groq.
- **Client**: Updated `EmbeddingClient` to use `AsyncOpenAI` pointing to `https://api.groq.com/openai/v1`.
- **Model**: Defaulted to `text-embedding-ada-002` (configurable via `GROQ_EMBEDDING_MODEL`).
- **Warning**: Added critical log warning users to re-ingest data.
Ready for verification (requires GROQ_API_KEY).
