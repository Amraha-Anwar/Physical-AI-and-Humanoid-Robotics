# P2.2 Vectorization & Indexing: OpenAI Embeddings and Dual-DB Ingestion - Research Findings

This document captures key research findings and decisions made during the planning phase for the vectorization and indexing script.

## Decisions Documented

| Decision | Options | Trade-offs | Chosen Option (Working Assumption) |
| :--- | :--- | :--- | :--- |
| **Rate Limit Library** | Manual `time.sleep` loop, `tenacity` library | Manual is simple but error-prone; `tenacity` provides robust, production-ready backoff/retry decorators. | **`Tenacity` library** for robust, standard implementation of exponential backoff. |
| **OpenAI Batch Size** | 1, 50, 2048 (Max texts per request) | Smaller is slow; Larger risks hitting token limits faster; 2048 is the absolute maximum *text* count. | **100 chunks per batch**. This balances API limits (TPM) and minimizes request overhead (RPM). |
| **Neon Batch Method** | One SQL `INSERT` per row, `psycopg2.extras.execute_batch` | Single inserts are slow; `execute_batch` is highly efficient for bulk insertion of multiple rows. | **`psycopg2.extras.execute_batch`** for high-performance metadata ingestion. |
