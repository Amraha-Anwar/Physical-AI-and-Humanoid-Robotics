# P1.3 Database Connection: Schema Definition & API Layer - Implementation Plan

## Overview

This plan outlines the implementation strategy for establishing database connections to Neon Postgres and Qdrant Cloud, defining their respective schemas, and integrating them via FastAPI. It ensures the foundational data persistence and vector storage layers are correctly set up, aligning with the project's RAG architecture.

---

### Specification: P1.3 Database Connection: Schema Definition & API Layer

#### 🏗️ Architecture Sketch (Logical View)

*   **FastAPI Core**: Serves as the application entry point and manages dependencies.
*   **Dependencies**: Functions (`get_qdrant_client`, `get_neon_db`) manage client instantiation and connection pools, ensuring availability across API requests.
*   **Neon Postgres**: Stores relational **RAG Metadata** (`rag_metadata` table).
*   **Qdrant Cloud**: Stores **Vector Embeddings** (`book_vectors` collection).
*   **Linkage**: The `chunk_id` in Neon must match the Qdrant Point ID for successful retrieval correlation.

#### 🧱 Section Structure (Task Breakdown)
1.  **P1.3.1 Model Definition**: Define Pydantic models for `QdrantConfig`, `NeonConfig`, and `RagChunk` (containing `chunk_id`, `text`, `chapter_title`, etc.).
2.  **P1.3.2 Qdrant Client Implementation**: Write `backend/qdrant_client.py`. Implement connection, collection check, and creation logic (ensuring `vector_size=1536`).
3.  **P1.3.3 Neon Client Implementation**: Write `backend/neon_client.py`. Implement connection using `psycopg2` or equivalent, and DDL logic to create the `rag_metadata` table.
4.  **P1.3.4 Dependency Injection**: Write `backend/dependencies.py`. Implement `setup_db_clients` and dependency functions (`get_qdrant_client`, `get_neon_db`) using environment variables for configuration.
5.  **P1.3.5 Verification Endpoints**: Update `backend/main.py`. Add the `/db/status/qdrant` and `/db/status/neon` endpoints to use the new dependencies and perform live connection checks.

#### 🔬 Research Approach (N/A for this foundational step)
This is an implementation step. Research is only needed for:
*   Confirming the most efficient Python library for Neon serverless connectivity (e.g., `asyncpg` vs. `psycopg2`).
*   Confirming the required vector size (e.g., 1536) for the chosen OpenAI embedding model.

#### ✅ Quality Validation (Acceptance Criteria)
*   **Neon Check**: Hitting `/db/status/neon` returns status code 200 and confirms the connection is active and the `rag_metadata` table exists.
*   **Qdrant Check**: Hitting `/db/status/qdrant` returns status code 200 and confirms the connection is active and the `book_vectors` collection is accessible.
*   **Structure Check**: All Python code resides exclusively within the `backend/` directory as specified in the constraint.
*   **Linkage Check**: Schema definitions ensure a unique `chunk_id` is defined to link the Neon metadata row to the Qdrant vector point.

---

### ✍️ Decisions Needing Documentation
| Decision | Options | Trade-offs | Chosen Option (Working Assumption) |
| :--- | :--- | :--- | :--- |
| **Neon Client Library** | `psycopg2` (sync), `asyncpg` (async) | `psycopg2` is simpler but less performant; `asyncpg` matches FastAPI's async nature but requires more complex setup. | **`psycopg2`** for simplicity and immediate executability; may be refactored to `asyncpg` later if performance is an issue. |
| **Qdrant Initialization** | `client.recreate_collection()` or `client.get_collections()` | `recreate` is idempotent but destructive; `get_collections` is safer but requires more explicit check logic. | **Explicit Check + `recreate_collection`** (as shown in the code) to guarantee correct vector size is set on startup. |
| **Vector Size** | 512, 1024, 1536 (typical OpenAI sizes) | Smaller size is faster/cheaper; larger size is generally more accurate. | **1536** (Standard for `text-embedding-ada-002` or v3-small/medium). |

### 🧪 Testing Strategy
1.  **Unit Tests**: Mock database clients to ensure Pydantic models serialize/deserialize correctly and client functions are called with correct parameters.
2.  **Integration Tests**: Run the FastAPI server pointing to **live, provisioned** Neon and Qdrant instances using the set environment variables. Use `httpx` to check for status code 200 on both `/db/status/neon` and `/db/status/qdrant` endpoints.
3.  **Schema Test**: Manually verify in the Neon console that the `rag_metadata` table structure matches the defined schema (e.g., `chunk_id` is primary key, `text` is `TEXT`).

---

## Constitution Check

The plan aligns well with the project's constitution (v2.0.0).

*   **RAG Accuracy & Contextual Relevance**: This phase lays the groundwork by establishing the database connections and schemas required for accurate RAG.
*   **Scalability & Component Reliability**: The plan addresses this by configuring FastAPI dependencies and ensuring Qdrant/Neon connections. The decision to start with `psycopg2` for Neon is a pragmatic choice for immediate executability, with a documented potential for future refactoring if performance becomes a bottleneck, thus maintaining alignment with scalability goals.
*   **Code Integrity & Tool Adherence**: The plan adheres to using Python/FastAPI, the Qdrant Python client, and Pydantic models, aligning with the constitution's mandate for best practices and tool adherence.
*   **Key Standards (Embedding Model, Database Schema)**: The plan explicitly mentions `vector_size=1536` for Qdrant, aligning with common OpenAI models, and defines the schemas for Neon and Qdrant.
*   **Constraints & Technology Stack**: The plan directly implements components using the mandated technologies: FastAPI, Neon Serverless Postgres, and Qdrant Cloud Free Tier. All Python code is constrained to reside within the `backend/` directory.

No explicit violations of the constitution were identified. The technical specifics within the plan align with the accepted deviation from strict agnosticism, which was clarified in the previous `/sp.clarify` step.

---
