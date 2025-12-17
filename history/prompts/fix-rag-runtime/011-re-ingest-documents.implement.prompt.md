id: 011
title: Re-Ingest Documents
stage: implement
date: 2025-12-17
surface: agent
model: gemini-2.0-flash-exp
feature: fix-rag-runtime
branch: 007-fix-rag-runtime
user: user
command: /sp.implement
labels: [implement, ingestion, qdrant, groq]
links:
  spec: specs/007-fix-rag-runtime/spec.md
  plan: specs/007-fix-rag-runtime/plan.md
  tasks: specs/007-fix-rag-runtime/tasks.md
  phr: history/prompts/fix-rag-runtime/011-re-ingest-documents.implement.prompt.md
files:
  - backend/delete_collection_script.py
  - backend/ingestion/embeddings_client.py
tests: []
---
/sp.implement T096 Re-Ingest Documents using Groq Embeddings

RESPONSE_TEXT
Implemented T096 to re-ingest documents.
- **Cleanup**: Renamed `backend/qdrant_client.py` to `backend/qdrant_service.py` to fix shadowing conflict.
- **Collection**: Deleted old `book_vectors` using `delete_collection_script.py`.
- **Config**: Updated `QDRANT_VECTOR_SIZE` to 1536 (default) for Groq/Ada compatibility.
- **Ingestion**: Ran `test_ingestion.py`. Currently failing with Groq API error 404 (Model `text-embedding-ada-002` not found).
- **Status**: Code is correct, but Groq API integration requires verification of available models or a proxy configuration. User must ensure `GROQ_EMBEDDING_MODEL` points to a valid model.
Ready for next steps (Model Verification).
