# P1.3 Database Connection: Schema Definition & API Layer - Research Findings

This document captures key research findings and decisions made during the planning phase for the database connection, schema definition, and API layer implementation.

## Decisions Documented

| Decision | Options | Trade-offs | Chosen Option (Working Assumption) |
| :--- | :--- | :--- | :--- |
| **Neon Client Library** | `psycopg2` (sync), `asyncpg` (async) | `psycopg2` is simpler but less performant; `asyncpg` matches FastAPI's async nature but requires more complex setup. | **`psycopg2`** for simplicity and immediate executability; may be refactored to `asyncpg` later if performance is an issue. |
| **Qdrant Initialization** | `client.recreate_collection()` or `client.get_collections()` | `recreate` is idempotent but destructive; `get_collections` is safer but requires more explicit check logic. | **Explicit Check + `recreate_collection`** (as shown in the code) to guarantee correct vector size is set on startup. |
| **Vector Size** | 512, 1024, 1536 (typical OpenAI sizes) | Smaller size is faster/cheaper; larger size is generally more accurate. | **1536** (Standard for `text-embedding-ada-002` or v3-small/medium). |
