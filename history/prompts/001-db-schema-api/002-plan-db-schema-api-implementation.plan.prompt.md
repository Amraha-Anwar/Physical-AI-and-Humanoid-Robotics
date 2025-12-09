---
id: 002
title: Plan DB Schema API Implementation
stage: plan
date: 2025-12-07
surface: agent
model: gemini-1.5-pro
feature: 001-db-schema-api
branch: 001-db-schema-api
user: user
command: /sp.plan

labels: ["plan", "db-schema-api"]
links:
  spec: specs/001-db-schema-api/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-db-schema-api/plan.md
  - specs/001-db-schema-api/research.md
  - specs/001-db-schema-api/data-model.md
  - specs/001-db-schema-api/quickstart.md
  - specs/001-db-schema-api/contracts/
  - GEMINI.md
tests:
  - null
---

## Prompt

Create: architecture sketch, section structure, research approach, quality validation.
Decisions needing documentation: list important choices with options and tradeoffs.
Testing strategy: validation checks based on acceptance criteria.

Technical details:
- Use research-concurrent approach (research while writing, not all upfront)
- Follow APA citation style from Constitution
- Organize by phases: Research → Foundation → Analysis → Synthesis

---

### Specification: P1.3 Database Connection: Schema Definition & API Layer

#### 🏗️ Architecture Sketch (Logical View)

* **FastAPI Core**: Serves as the application entry point and manages dependencies.
* **Dependencies**: Functions (`get_qdrant_client`, `get_neon_db`) manage client instantiation and connection pools, ensuring availability across API requests.
* **Neon Postgres**: Stores relational **RAG Metadata** (`rag_metadata` table).
* **Qdrant Cloud**: Stores **Vector Embeddings** (`book_vectors` collection).
* **Linkage**: The `chunk_id` in Neon must match the Qdrant Point ID for successful retrieval correlation.

#### 🧱 Section Structure (Task Breakdown)
1.  **P1.3.1 Model Definition**: Define Pydantic models for `QdrantConfig`, `NeonConfig`, and `RagChunk` (containing `chunk_id`, `text`, `chapter_title`, etc.).
2.  **P1.3.2 Qdrant Client Implementation**: Write `backend/qdrant_client.py`. Implement connection, collection check, and creation logic (ensuring `vector_size=1536`).
3.  **P1.3.3 Neon Client Implementation**: Write `backend/neon_client.py`. Implement connection using `psycopg2` or equivalent, and DDL logic to create the `rag_metadata` table.
4.  **P1.3.4 Dependency Injection**: Write `backend/dependencies.py`. Implement `setup_db_clients` and dependency functions (`get_qdrant_client`, `get_neon_db`) using environment variables for configuration.
5.  **P1.3.5 Verification Endpoints**: Update `backend/main.py`. Add the `/db/status/qdrant` and `/db/status/neon` endpoints to use the new dependencies and perform live connection checks.

#### 🔬 Research Approach (N/A for this foundational step)
This is an implementation step. Research is only needed for:
* Confirming the most efficient Python library for Neon serverless connectivity (e.g., `asyncpg` vs. `psycopg2`).
* Confirming the required vector size (e.g., 1536) for the chosen OpenAI embedding model.

#### ✅ Quality Validation (Acceptance Criteria)
* **Neon Check**: Hitting `/db/status/neon` returns status code 200 and confirms the connection is active and the `rag_metadata` table exists.
* **Qdrant Check**: Hitting `/db/status/qdrant` returns status code 200 and confirms the connection is active and the `book_vectors` collection is accessible.
* **Structure Check**: All Python code resides exclusively within the `backend/` directory as specified in the constraint.
* **Linkage Check**: Schema definitions ensure a unique `chunk_id` is defined to link the Neon metadata row to the Qdrant vector point.

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

The next specified module in the overall project plan is **P2.1 Content Pre-processing**.

Would you like me to now generate the detailed specification for **P2.1 Content Pre-processing**?

## Response snapshot

The implementation planning phase for "P1.3 Database Connection: Schema Definition & API Layer" is complete. The `plan.md` has been created, and related artifacts (`research.md`, `data-model.md`, `quickstart.md`, `contracts/`) have been generated. The agent context in `GEMINI.md` has also been updated.

## Outcome

- ✅ Impact: Detailed plan for database connection and API layer implementation has been created and documented.
- 🧪 Tests: No automated tests were executed during the planning phase.
- 📁 Files: `specs/001-db-schema-api/plan.md`, `specs/001-db-schema-api/research.md`, `specs/001-db-schema-api/data-model.md`, `specs/001-db-schema-api/quickstart.md`, `specs/001-db-schema-api/contracts/`, and `GEMINI.md` were created/modified.
- 🔁 Next prompts: The next step is to generate tasks based on this plan using `/sp.tasks`.
- 🧠 Reflection: The planning process successfully integrated user input, constitution checks, and generated necessary documentation, preparing for the task generation phase.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
