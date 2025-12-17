---
description: Integrate Cohere for Embeddings & Automated Re-Ingestion
---

## Context
- **Stage**: implementation
- **Task**: T098 Integrate Cohere for Embeddings & Automated Re-Ingestion
- **Feature**: 007-fix-rag-runtime (implied context)

## Changes
- **Backend**:
    - `backend/ingestion/embeddings_client.py`: Replaced Groq/OpenAI implementation with `cohere` SDK. Implemented `embed-english-v3.0` with `input_type` support.
    - `backend/ingestion/ingestion_service.py`: Updated to pass `input_type="search_document"`.
    - `backend/query/query_service.py`: Updated to pass `input_type="search_query"`.
    - `backend/dependencies.py`: Updated default `QDRANT_VECTOR_SIZE` to `1024`.
    - `backend/main.py`: Added `COHERE_API_KEY` to required environment variables.
    - `backend/requirements.txt`: Added `cohere`.
    - `backend/models.py`: Updated Pydantic config to `from_attributes` (V2 compat).
- **Scripts**:
    - Created `backend/ingestion/run_ingestion.py` to automate re-ingestion.
    - Used `backend/delete_collection_script.py` to wipe `book_vectors`.

## Verification
- **Re-ingestion**:
    - Successfully ran `backend/ingestion/run_ingestion.py`.
    - Collection `book_vectors` recreated with size 1024.
    - 3 sample chunks ingested using Cohere embeddings.
- **Dependencies**: `cohere` installed.
- **Config**: `QDRANT_VECTOR_SIZE` default is 1024.

## Constraints
- No UI changes.
- Gemini API key usage for chat agent preserved.
