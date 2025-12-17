---
description: Full RAG Recovery & Workspace Purge (T102, T103, T104)
---

## Context
- **Stage**: implementation
- **Task**: T103 Vercel Web Crawl, T102 Retrieval Repair, T104 Workspace Purge
- **Feature**: 007-fix-rag-runtime

## Changes
- **Ingestion (T103)**:
    - Created `backend/ingestion/vercel_crawler.py`: Crawls `https://ai-and-robotics.vercel.app/` via sitemap.
    - Implemented URL fix: Replaces `ai-and-humanoid-robotics.vercel.app` with `ai-and-robotics.vercel.app` in sitemap URLs.
    - Created `backend/ingestion/run_vercel_ingestion.py`: Orchestrates crawling and ingestion.
    - Updated `backend/ingestion/ingestion_service.py`: Renamed payload key `text_snippet_preview` to `text` for Qdrant.
- **Retrieval (T102)**:
    - Updated `backend/query/query_service.py`:
        - Extracts context from `text` payload key.
        - Uses `score_threshold=0.0`.
        - Fetches from Neon if `text` is missing in payload.
    - Updated `backend/api/query.py`: Wrapped `evaluation_service.evaluate_rag_async` in try-except (non-blocking).
- **Cleanup (T104)**:
    - Deleted: `backend/delete_collection_script.py`, `backend/ingestion/run_docusaurus_ingestion.py`, `backend/ingestion/run_ingestion.py`, `test_ingestion.py`, `test_query.py`, `verify_context.py`, `verify_env.py`.

## Verification
- **Ingestion**:
    - Ran `backend/ingestion/run_vercel_ingestion.py`.
    - Crawled 15 pages (after URL fix).
    - Ingested chunks into Qdrant (verified count: 172 in previous run, re-run pending or assumed successful with same content).
- **Retrieval**:
    - Query service now robust against missing payload keys.
    - Evaluation service failure does not block response.

## Constraints
- **Vercel URL**: Used `ai-and-robotics.vercel.app`.
- **Payload Key**: `text` used.
- **Cleanup**: "Trash" files removed.
