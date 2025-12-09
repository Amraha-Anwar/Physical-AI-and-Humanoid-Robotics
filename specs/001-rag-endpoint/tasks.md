# P2.3 Retrieval & Agent Logic: Core RAG FastAPI Endpoint - Tasks

**Feature Branch**: `001-rag-endpoint`  
**Goal**: Implement the main FastAPI endpoint that orchestrates the entire RAG cycle, including Qdrant search, Neon metadata lookup, prompt construction, and LLM generation using the OpenAI SDKs.

---

## Phase 1: Foundational Endpoint Setup

*   - [ ] T001 Define Pydantic models `RAGQueryRequest`, `RAGQueryResponse`, and `SourceMetadata` in `backend/models.py`.
*   - [ ] T002 Create `backend/rag_service.py` file for core RAG logic components.
*   - [ ] T003 Implement `generate_query_embedding(query: str)` function in `backend/rag_service.py` using OpenAI Embeddings API, including robust error handling.

## Phase 2: User Story 5 - Prioritize Highlight-to-Query Snippet [US5]

**Goal**: Handle an optional `context_snippet` parameter, which is prioritized over standard RAG search.
**Independent Test**: Submit a query with a `context_snippet` and observe that the LLM's response is limited to information within that snippet.

*   - [ ] T004 [US5] Implement the `POST /api/rag/query` endpoint in `backend/main.py`, injecting existing database dependencies (`get_qdrant_client`, `get_neon_db`).
*   - [ ] T005 [US5] Implement initial context routing logic within the `/api/rag/query` endpoint in `backend/main.py`: if `context_snippet` is present in the `RAGQueryRequest`, use it directly as context for prompt construction, bypassing Qdrant search.

## Phase 3: User Story 2 - Retrieve Relevant Chunks from Qdrant [US2]

**Goal**: Efficiently retrieve the most relevant text chunks from the Qdrant vector database.
**Independent Test**: Submitting a query and inspecting logs or intermediate output to confirm that Qdrant returns the top N chunk IDs.

*   - [ ] T006 [US2] Implement `search_qdrant(query_embedding: List[float], qdrant_client: QdrantClient, top_n: int = 5)` function in `backend/rag_service.py` to perform vector search against the `book_vectors` collection.
*   - [ ] T007 [US2] Integrate `generate_query_embedding` and `search_qdrant` into a `retrieve_chunk_ids(query: str, qdrant_client: QdrantClient)` function in `backend/rag_service.py` for cases where `context_snippet` is not present.

## Phase 4: User Story 3 - Fetch Full Context from Neon Postgres [US3]

**Goal**: Fetch the full text and metadata for the retrieved chunk IDs from Neon Postgres.
**Independent Test**: After retrieving chunk IDs from Qdrant, verify that the system successfully fetches corresponding full `RagChunk` data from Neon Postgres.

*   - [ ] T008 [US3] Implement `fetch_context_from_neon(chunk_ids: List[str], neon_db_conn: Connection)` function in `backend/rag_service.py` to retrieve `RagChunk` data (full text and metadata) from the `rag_metadata` table.

## Phase 5: User Story 4 - Construct Prompt and Generate Response [US4]

**Goal**: Combine the user's query with the retrieved context into a clear prompt template and send it to OpenAI.
**Independent Test**: Submit queries with specific contexts and verify that the LLM's response strictly adheres to the provided information.

*   - [ ] T009 [US4] Implement `construct_prompt(query: str, context: str)` function in `backend/rag_service.py` to create the system message for the OpenAI Chat API, strictly enforcing grounding in the provided context.
*   - [ ] T010 [US4] Implement `generate_llm_response(prompt_messages: List[Dict])` function in `backend/rag_service.py` to call OpenAI Chat API (`gpt-4-turbo`) with a low temperature (0.0-0.3) for factual consistency.

## Phase 6: User Story 1 - Query the RAG Endpoint and Receive Contextual Answer [US1]

**Goal**: Orchestrate the entire RAG process within the API endpoint and return a contextual answer.
**Independent Test**: Submitting a query to `/api/rag/query` and verifying the response contains an accurate answer derived from the book.

*   - [ ] T011 [US1] Integrate `retrieve_chunk_ids` (or use `context_snippet`), `fetch_context_from_neon`, `construct_prompt`, and `generate_llm_response` into the `/api/rag/query` endpoint in `backend/main.py`, orchestrating the full RAG flow based on context routing.
*   - [ ] T012 [US1] Implement error handling and logging for the entire RAG flow in `backend/main.py` and `backend/rag_service.py`, ensuring graceful degradation and informative messages (FR-010).
*   - [ ] T013 [US1] Ensure the `/api/rag/query` endpoint returns a `RAGQueryResponse` object (FR-009).

## Phase 7: Polish & Cross-Cutting Concerns

*   - [ ] T014 Add comprehensive docstrings and type hints to all new functions and classes in `backend/rag_service.py` and `backend/main.py`.
*   - [ ] T015 Create `backend/tests/test_rag_endpoint.py` for unit/integration tests for the `/api/rag/query` endpoint, covering general RAG, highlight-to-query, and error scenarios.
*   - [X] T073 Fix Logging: Add missing `import logging` in `backend/api/query.py`.
*   - [X] T074 Fix Final Backend NameError: Import `RAGQueryRequest` and `RAGQueryResponse` in `backend/api/query.py`.
*   - [X] T075 Fix QueryService Import: Import `QueryService` from `backend.query.query_service` in `backend/api/query.py`.

---

## Dependencies

This section outlines the completion order of user stories.

1.  **Foundational Endpoint Setup** -> **User Story 5 (Prioritize Highlight-to-Query Snippet)** -> **User Story 2 (Retrieve Relevant Chunks from Qdrant)** -> **User Story 3 (Fetch Full Context from Neon Postgres)** -> **User Story 4 (Construct Prompt and Generate Response)** -> **User Story 1 (Query the RAG Endpoint and Receive Contextual Answer)**

## Parallel Execution Opportunities

*   **Phase 2 (User Story 5)** and **Phase 3 (User Story 2)** can be developed somewhat in parallel, as US5 handles the bypass logic and US2 handles the primary RAG retrieval path, which are distinct conditional branches.
*   **Within Phase 3 (Retrieval)** and **Phase 4 (Context Fetch)**: While US3 depends on US2's output, the implementation details of the Qdrant search and Neon lookup can be developed by different individuals simultaneously.
*   **Phase 6 (User Story 1)** is primarily integration, so its tasks can progress as soon as dependent phases are complete.

## Implementation Strategy

The implementation will follow a modular and progressive approach:
1.  Establish the core endpoint and define its input/output models.
2.  Implement the specialized "Highlight-to-Query" logic first, as it bypasses the full RAG cycle.
3.  Develop the RAG retrieval components (query embedding, Qdrant search, Neon lookup) independently.
4.  Focus on robust prompt construction and LLM integration.
5.  Finally, integrate all components into the main endpoint, with comprehensive testing and error handling.