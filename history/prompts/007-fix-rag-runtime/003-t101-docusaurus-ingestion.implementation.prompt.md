---
description: Full Docusaurus Markdown Ingestion to Qdrant (T101)
---

## Context
- **Stage**: implementation
- **Task**: T101 Full Docusaurus Markdown Ingestion to Qdrant
- **Feature**: 007-fix-rag-runtime (implied context)

## Changes
- **Backend**:
    - Created `backend/ingestion/docusaurus_loader.py`: Recursively loads `.md`/`.mdx` from `frontend/docs`, strips frontmatter, and splits by H1/H2/H3 headers.
    - Created `backend/ingestion/run_docusaurus_ingestion.py`: Orchestrates the full ingestion process (Clear -> Load -> Embed -> Save).
    - Modified `backend/ingestion/ingestion_service.py`: 
        - Added batch processing (size 10) to prevent DB connection timeouts.
        - Added rate limiting (`asyncio.sleep(0.7)`) for Cohere Trial key (100 req/min).
        - Added check for empty embeddings.
    - Modified `backend/ingestion/embeddings_client.py`: Removed try/except to allow `tenacity` to handle retries properly.

## Verification
- **Ingestion Execution**:
    - Ran `backend/ingestion/run_docusaurus_ingestion.py`.
    - 15 chapters (files) loaded.
    - 172 chunks generated and processed.
    - 18 batches processed successfully.
- **Qdrant**:
    - `book_vectors` collection cleared and recreated (1024 dim).
    - Final point count: 172.
- **Neon**:
    - Metadata for 172 chunks saved to `rag_metadata`.

## Constraints Met
- **No PDF loaders**: Used custom Markdown loader.
- **No UI changes**: Frontend untouched.
- **Cohere Usage**: Used `embed-english-v3.0` with `COHERE_API_KEY`.
- **Context-Aware**: Split by headers.
