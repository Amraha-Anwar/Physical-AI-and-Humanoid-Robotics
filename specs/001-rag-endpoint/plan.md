# P2.3 Retrieval & Agent Logic: Core RAG FastAPI Endpoint - Implementation Plan

## Overview

This plan outlines the implementation of the core FastAPI endpoint responsible for orchestrating the Retrieval-Augmented Generation (RAG) process. It encompasses vector search in Qdrant, contextual data retrieval from Neon Postgres, dynamic prompt construction for the LLM, and final response generation using the OpenAI Chat API, including support for a "highlight-to-query" feature.

---

### Specification: P2.3 Retrieval & Agent Logic: Core RAG FastAPI Endpoint

#### 🏗️ Architecture Sketch (RAG Query Flow)

*   **Query Input**: Accepts `query` (user question) and optional `context_snippet` (highlighted text).
*   **Retrieval Path**:
    *   If `context_snippet` is provided, skip vector search.
    *   If only `query` is provided, embed the query and perform a vector search in **Qdrant**.
*   **Context Aggregation**: Use retrieved Qdrant IDs to fetch full context from **Neon Postgres**.
*   **Generation**: Use the aggregated context and the **OpenAI Chat API** (simulating Agents/ChatKit) to generate the final response.

#### 🧱 Section Structure (Task Breakdown)
1.  **P2.3.1 Endpoint and Data Model**: Create the `/api/rag/query` endpoint in FastAPI and define the Pydantic input model to handle the optional `context_snippet` parameter.
2.  **P2.3.2 Query Embedding & Qdrant Search**: Implement the logic to embed the user query and execute a vector search in Qdrant (using the existing client), returning the top N chunk IDs.
3.  **P2.3.3 Neon Context Retrieval**: Implement logic to take the retrieved Qdrant IDs and perform a batched `SELECT` query against the Neon Postgres `rag_metadata` table to fetch the full text and metadata for context.
4.  **P2.3.4 Prompt Construction & Routing**: Implement the logic to construct the final **System Prompt** by injecting the retrieved context (or the `context_snippet` if provided). This logic acts as the RAG router.
5.  **P2.3.5 OpenAI Generation**: Implement the final call to the OpenAI Chat API, using a temperature of $0.0-0.3$ and a system message that strictly enforces grounding the answer in the provided context.

#### 🔬 Research Approach
*   **Prompt Engineering**: Research optimal system prompt phrasing to enforce the "answer *only* from the context" constraint, which is crucial for RAG integrity.
*   **Retrieval Tuning (N)**: Determine the optimal number of chunks ($N$) to retrieve from Qdrant to maximize relevance while staying within the LLM's context window (e.g., $N=5$).

#### ✅ Quality Validation (Acceptance Criteria)
*   **General RAG Test**: A question about a specific topic returns an answer derived from the book's content (P2.3.2 -> P2.3.3 -> P2.3.5 successful).
*   **Highlight Test (Isolation)**: A question about a specific paragraph (using `context_snippet`) forces the LLM to answer only using that paragraph, even if the general RAG search would yield different results.
*   **Context Window Integrity**: The combined length of the System Prompt, Context, and User Query never exceeds the context window limit of the chosen OpenAI model.
*   **Attribution**: Responses should inherently reflect the content's technical accuracy as per Principle I.

---

### ✍️ Decisions Needing Documentation
| Decision | Options | Trade-offs | Chosen Option (Working Assumption) |
| :--- | :--- | :--- | :--- |
| **LLM Model** | `gpt-3.5-turbo`, `gpt-4-turbo` | `gpt-3.5-turbo` is faster/cheaper; `gpt-4-turbo` is better at complex reasoning and instruction following (better for RAG). | **`gpt-4-turbo`** for higher quality, context-aware RAG responses, adhering to the quality focus. |
| **Qdrant Retrieval Strategy** | Simple Top-k Search, Contextual Re-ranking | Simple Top-k is fast; Re-ranking is more accurate but adds latency. | **Simple Top-N (N=5) Search** for initial low-latency implementation, as quality relies more on chunking (P2.1) and prompt (P2.3.4). |
| **Highlight Implementation** | Bypass RAG, Include in RAG | Bypassing is simpler/faster but requires dedicated prompt; Including maintains one prompt structure. | **Router/Bypass Logic**: If `context_snippet` is present, it directly becomes the *only* context, bypassing the Qdrant search entirely. |

### 🧪 Testing Strategy
1.  **Unit Tests (TDD)**: Test the Prompt Construction function (P2.3.4) to ensure the system message correctly changes based on the presence of the `context_snippet`.
2.  **Integration Test (General)**: Hit the `/api/rag/query` endpoint with a known question from the book. Verify that the Qdrant search is executed, the Neon lookup occurs, and the response is coherent.
3.  **Integration Test (Highlight)**: Hit the endpoint with a question and a specific, non-relevant `context_snippet`. Verify that the answer is *forced* to be about the snippet, proving the bypass logic works.
4.  **Performance Test**: Measure the average latency of the full RAG cycle (Embedding -> Qdrant -> Neon -> LLM) to ensure a responsive user experience.

---

## Constitution Check

The plan for "P2.3 Retrieval & Agent Logic: Core RAG FastAPI Endpoint" aligns exceptionally well with the project's constitution (v2.0.0).

*   **I. RAG Accuracy & Contextual Relevance**: This phase is the core implementation of RAG accuracy, ensuring contextual answers are strictly derived from the book's content. The explicit prompt construction ("answer *only* based on provided context") and the "Highlight-to-Query" feature directly enforce this principle.
*   **II. Interactive UX & Selection Integrity**: The implementation of the `/api/rag/query` endpoint provides the interactive user experience, and the "Highlight-to-Query" feature directly addresses the "selection integrity" mandate by prioritizing user-selected text as context.
*   **III. Scalability & Component Reliability**: The plan leverages existing P1.3 database clients, which are designed for low-latency and high-availability. The decision to retrieve a "controlled number of chunks" (N=5) is crucial for managing the LLM's context window size, thus contributing to the scalability and reliability of the overall system.
*   **IV. Code Integrity & Tool Adherence**: The plan strictly adheres to the use of FastAPI, the OpenAI Python SDK (mimicking Agents/ChatKit functionality), and the pre-defined Qdrant and Neon clients. This ensures code consistency, maintainability, and alignment with the mandated technology stack and best practices.

**Key Standards**:
*   **Embedding Model**: Utilizes the OpenAI Embeddings API for query embedding.
*   **Chunking Strategy**: Implicitly relies on the output of P2.1, which produces context-aware chunks.
*   **API Security**: The endpoint is part of the existing FastAPI application, meaning any existing API security measures (e.g., from P1.2) will apply.
*   **Deployment**: The endpoint is integrated into the FastAPI application, adhering to the serverless-compatible deployment target.
*   **Database Schema**: Directly leverages the `rag_metadata` table from Neon and `book_vectors` from Qdrant, ensuring consistency with the established schemas.

No explicit violations of the constitution were identified. The technical specifics within the plan are in full alignment with the project's requirements and the previously clarified and accepted deviation from strict technology-agnosticism.

---
