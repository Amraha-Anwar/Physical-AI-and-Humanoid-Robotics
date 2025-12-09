---
id: 002
title: Plan Vectorization Index Implementation
stage: plan
date: 2025-12-07
surface: agent
model: gemini-1.5-pro
feature: 001-vectorize-index
branch: 001-vectorize-index
user: user
command: /sp.plan

labels: ["plan", "vectorization", "indexing"]
links:
  spec: specs/001-vectorize-index/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-vectorize-index/plan.md
  - specs/001-vectorize-index/research.md
  - specs/001-vectorize-index/data-model.md
  - specs/001-vectorize-index/quickstart.md
  - specs/001-vectorize-index/contracts/
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

### Specification: P2.2 Vectorization & Indexing: OpenAI Embeddings and Dual-DB Ingestion

#### 🏗️ Architecture Sketch (Indexing Flow)
* **Input Processor**: Reads the `rag_data.json` file (output of P2.1).
* **Chunk Batcher**: Groups `RagChunk` objects into batches (e.g., 50-100 items).
* **OpenAI Embedder**: Calls the OpenAI Embeddings API using **Exponential Backoff and Retry Logic** to mitigate rate limiting (429 errors).
* **Dual Ingester**: Orchestrates the simultaneous insertion of data:
    * **Qdrant**: Batched `PointStruct` creation and `client.upsert` call.
    * **Neon Postgres**: Batched SQL `INSERT` statements using the Neon client.
* **Output**: Console logs showing progress, total records indexed, and any failed batches.

#### 🧱 Section Structure (Task Breakdown)
1.  **P2.2.1 Environment & Setup**: Create the indexing script structure and ensure the OpenAI Python SDK and necessary helper libraries (e.g., `tenacity` for backoff) are installed.
2.  **P2.2.2 OpenAI Batch Embedding Function**: Implement a robust, batched function to call the OpenAI Embeddings API. This function *must* wrap the API call with exponential backoff and retry logic.
3.  **P2.2.3 Qdrant Batch Ingestion Logic**: Implement the transformation of `RagChunk` objects and their vectors into Qdrant `PointStruct` objects, and utilize the Qdrant client's batched `upsert` method.
4.  **P2.2.4 Neon Batch Ingestion Logic**: Implement the function to generate batched SQL `INSERT` queries for the `rag_metadata` table, ensuring the `chunk_id` is included and the insertion is atomic per batch.
5.  **P2.2.5 Orchestration and Verification**: Write the main script loop that processes the input JSON in batches, calls the embedding function, performs the dual ingestion (Qdrant & Neon), and logs final counts for integrity validation.

#### 🔬 Research Approach
* **Batch Sizing**: Confirm the optimal batch size for concurrent API calls (OpenAI) and efficient database writes (Qdrant/Neon), aiming for **100-250 chunks** per batch as a starting point.
* **Rate Limit Handling**: Verify the latest best practices for implementing **exponential backoff** within the OpenAI Python SDK to handle 429 errors gracefully.

#### ✅ Quality Validation (Acceptance Criteria)
* **Count Match**: `Qdrant Point Count` == `Neon Row Count` == `Total Chunks in JSON`.
* **Error Tolerance**: The script successfully completes the full index run, automatically recovering from simulated or actual rate limit errors using exponential backoff.
* **Vector Shape**: Every point in Qdrant must have a vector of dimension 1536 (or the final chosen embedding size).
* **Metadata Check**: Manually query Neon Postgres to confirm that the `text` field and other metadata are correctly stored.

---

### ✍️ Decisions Needing Documentation
| Decision | Options | Trade-offs | Chosen Option (Working Assumption) |
| :--- | :--- | :--- | :--- |
| **Rate Limit Library** | Manual `time.sleep` loop, `tenacity` library | Manual is simple but error-prone; `tenacity` provides robust, production-ready backoff/retry decorators. | **`Tenacity` library** for robust, standard implementation of exponential backoff. |
| **OpenAI Batch Size** | 1, 50, 2048 (Max texts per request) | Smaller is slow; Larger risks hitting token limits faster; 2048 is the absolute maximum *text* count. | **100 chunks per batch**. This balances API limits (TPM) and minimizes request overhead (RPM). |
| **Neon Batch Method** | One SQL `INSERT` per row, `psycopg2.extras.execute_batch` | Single inserts are slow; `execute_batch` is highly efficient for bulk insertion of multiple rows. | **`psycopg2.extras.execute_batch`** for high-performance metadata ingestion. |

### 🧪 Testing Strategy
1.  **Unit Tests (TDD)**: Test the batching function (P2.2.3) with mock vectors to ensure correct `PointStruct` generation for Qdrant and correct SQL tuple generation for Neon.
2.  **Mock Integration Test**: Use a mock server or tool to simulate a **429 Rate Limit Error** from the OpenAI API and verify that the backoff logic (P2.2.2) correctly waits and retries the request until success.
3.  **Final Integration Test**: Execute the full script against the **live** Neon and Qdrant free-tier instances and perform the final count check validation (Acceptance Criteria).

---

## Constitution Check

The plan for "P2.2 Vectorization & Indexing: OpenAI Embeddings and Dual-DB Ingestion" aligns exceptionally well with the project's constitution (v2.0.0).

*   **I. RAG Accuracy & Contextual Relevance**: This phase is paramount for RAG accuracy, as it directly handles the generation and indexing of high-quality vector embeddings and their corresponding metadata, which are the backbone of retrieval.
*   **III. Scalability & Component Reliability**: The plan explicitly incorporates critical design patterns for scalability and reliability, including **batch processing** for both OpenAI API calls and database ingestion, and **exponential backoff and retry logic** (`Tenacity` library) to gracefully handle API rate limits and transient errors. This ensures the ingestion process is robust and efficient, crucial for large datasets.
*   **IV. Code Integrity & Tool Adherence**: The plan strictly adheres to the mandated tools, specifying the use of the official **OpenAI Python SDK** and leveraging the pre-defined `qdrant_client.py` and `neon_client.py` modules from P1.3. This promotes code consistency and best practices.
*   **Key Standards (Embedding Model, API Security, Database Schema)**:
    *   **Embedding Model**: Explicitly specifies the use of OpenAI Embeddings API with 1536-dimensional vectors, directly meeting the standard.
    *   **API Security**: Implicitly addressed through the use of OpenAI API keys (which would be managed via environment variables), ensuring secure access to external services.
    *   **Deployment**: The script is standalone and prepares data for the live system.
    *   **Database Schema**: Ensures data integrity by stipulating that Qdrant and Neon Postgres counts must match, and that `chunk_id` is used for linkage, maintaining consistency with the `RagChunk` schema defined in P1.3.

No explicit violations of the constitution were identified. The technical specifics within the plan are in full alignment with the project's requirements and the previously clarified and accepted deviation from strict technology-agnosticism.

---
