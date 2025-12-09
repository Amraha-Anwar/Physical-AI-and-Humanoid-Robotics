# Feature Specification: P2.3 Retrieval & Agent Logic: Core RAG FastAPI Endpoint

**Feature Branch**: `001-rag-endpoint`  
**Created**: 2025-12-07  
**Status**: Draft  
**Input**: User description: "P2.3 Retrieval & Agent Logic: Core RAG FastAPI Endpoint Target module: P2.3 Retrieval & Agent Logic Focus: Implement the main FastAPI endpoint that orchestrates the Retrieval-Augmented Generation (RAG) process, including vector search in Qdrant, context lookup in Neon, prompt construction, and final response generation using the OpenAI Agents/ChatKit SDKs. Success criteria: - **Core RAG Endpoint**: A new FastAPI endpoint (`/api/rag/query`) accepts a user query and returns a contextual answer. - **Retrieval**: The system successfully uses the input query to generate an embedding and performs a fast **vector similarity search** against the Qdrant `book_vectors` collection, retrieving the top N most relevant chunk IDs. - **Context Lookup**: The retrieved chunk IDs are used to fetch the full text and metadata from the **Neon Postgres** `rag_metadata` table. - **Agent/Prompt Construction**: The full retrieved text context is combined with the user query into a single, comprehensive prompt template, instructing the OpenAI model to answer *only* based on the provided context. - **Highlight-to-Query Feature**: The endpoint can handle an optional `context_snippet` parameter, which is prioritized over the standard RAG search if provided, forcing the LLM to answer based on that snippet alone. - **Tool Adherence**: The final generation step uses the mandated **OpenAI Agents/ChatKit SDKs**. Constraints: - **Endpoint Structure**: The endpoint must be part of the existing FastAPI application in the `backend/` directory. - **Dependency Use**: Must rely on the existing database client dependencies (`get_qdrant_client`, `get_neon_db`) defined in P1.3. - **OpenAI Tooling**: Must use the **OpenAI Python SDK** for both query embedding and final response generation (mimicking Agent/ChatKit functionality). - **Retrieval Limit**: The search must retrieve a controlled number of chunks (e.g., N=5) to manage LLM context window size. Not building: - The Docusaurus frontend or user interface (Phase 3). - User authentication beyond basic API security (P1.2). - Any data ingestion logic (completed in P2.2)."

## User Scenarios & Testing

### User Story 1 - Query the RAG Endpoint and Receive Contextual Answer (Priority: P1)
As a user of the RAG chatbot, I want to submit a natural language query and receive a concise, contextual answer based on the book's content, so that I can quickly get information without browsing the entire book.
**Why this priority**: This is the primary user interaction and the core functionality of the Retrieval-Augmented Generation (RAG) system.
**Independent Test**: Submitting a query to `/api/rag/query` and verifying the response contains an accurate answer derived from the book.
**Acceptance Scenarios**:
1.  **Given** the FastAPI application is running and the databases are populated with book content, **When** I send a `POST` request to `/api/rag/query` with a user query, **Then** I receive a JSON response containing a contextual answer that is factually consistent with the book's content.

### User Story 2 - Retrieve Relevant Chunks from Qdrant (Priority: P1)
As the RAG system, I need to efficiently retrieve the most relevant text chunks from the Qdrant vector database based on a user's query, so that I can provide pertinent context to the Large Language Model (LLM).
**Why this priority**: Fast and accurate retrieval is fundamental to RAG performance and answer quality.
**Independent Test**: Submitting a query and inspecting logs or intermediate output to confirm that Qdrant returns the top N chunk IDs.
**Acceptance Scenarios**:
1.  **Given** a user query, **When** the system generates an embedding for the query and performs a vector similarity search against Qdrant's `book_vectors` collection, **Then** the system successfully retrieves the `N` most relevant `chunk_id`s, where `N` is configurable (e.g., N=5).

### User Story 3 - Fetch Full Context from Neon Postgres (Priority: P1)
As the RAG system, I need to fetch the full text and metadata for the retrieved chunk IDs from Neon Postgres, so that I can reconstruct the complete context for the LLM.
**Why this priority**: Essential for providing the LLM with sufficient information to generate accurate answers.
**Independent Test**: After retrieving chunk IDs from Qdrant, verify that the system successfully fetches corresponding full `RagChunk` data (e.g., `text_snippet_preview` and other metadata) from Neon Postgres.
**Acceptance Scenarios**:
1.  **Given** a list of `chunk_id`s retrieved from Qdrant, **When** the system queries the Neon Postgres `rag_metadata` table using these IDs, **Then** the system successfully retrieves the full text (`text_snippet_preview`) and other associated metadata for each `chunk_id`.

### User Story 4 - Construct Prompt and Generate Response (Priority: P1)
As the RAG system, I need to combine the user's query with the retrieved context into a clear prompt template and send it to the OpenAI model for generation, ensuring the LLM answers *only* based on the provided context.
**Why this priority**: This orchestrates the final generation step and enforces the core RAG constraint of grounded answers, preventing hallucination.
**Independent Test**: Submit queries with specific contexts and verify that the LLM's response strictly adheres to the provided information, not introducing external knowledge.
**Acceptance Scenarios**:
1.  **Given** a user query and relevant text context (either retrieved or provided), **When** the system constructs a prompt and sends it to the OpenAI model, **Then** the OpenAI model generates an answer that is directly derived from the provided context and addresses the user's query.

### User Story 5 - Prioritize Highlight-to-Query Snippet (Priority: P2)
As a user of the RAG chatbot, I want to provide a specific text snippet along with my query, and have the LLM answer based *only* on that snippet, ignoring the general RAG search, so that I can get highly targeted answers from a specific piece of text.
**Why this priority**: Enhances user control and enables precise, context-specific questioning, which is a key requirement of the overall chatbot.
**Independent Test**: Submit a query with a `context_snippet` and observe that the LLM's response is limited to information within that snippet, even if a broader RAG search would yield different results.
**Acceptance Scenarios**:
1.  **Given** a `POST` request to `/api/rag/query` with both a `query` and an optional `context_snippet` parameter, **When** the `context_snippet` is provided, **Then** the system bypasses the Qdrant/Neon RAG search, constructs a prompt using *only* the `context_snippet`, and the LLM generates an answer strictly based on that snippet.

---

### Edge Cases

-   What happens if no relevant chunks are found in Qdrant? (The system should indicate that it cannot answer based on the provided context, or potentially fall back to a more general LLM response if explicitly allowed by prompt design.)
-   What if retrieved `chunk_id`s from Qdrant do not have corresponding entries in Neon Postgres? (This indicates a data integrity issue; the system should handle gracefully, e.g., by logging the discrepancy and omitting missing chunks from the context.)
-   What if the combined context from retrieved chunks (or `context_snippet`) exceeds the LLM's context window? (The system should prioritize chunks, potentially summarize them, or indicate context overflow, ensuring the LLM still receives a valid prompt.)
-   How does the system handle an empty or malformed user query? (The API should return a validation error.)
-   What if the OpenAI API is unreachable or returns an error during embedding generation or response generation? (Implement retries, error logging, and graceful degradation, returning an informative error to the user.)
-   What if the `context_snippet` is provided but is too short to be meaningful or too long to fit the LLM's context window? (The system should handle this by either supplementing with RAG search, truncating, or returning a specific error/warning.)

## Requirements

### Functional Requirements

-   **FR-001**: The FastAPI application MUST expose a `POST /api/rag/query` endpoint that accepts a user `query` (string) and an optional `context_snippet` (string).
-   **FR-002**: If `context_snippet` is provided in the request, the system MUST prioritize it and use it as the sole context for the LLM, bypassing the vector similarity search.
-   **FR-003**: If `context_snippet` is NOT provided, the system MUST generate an embedding for the user's `query` using the OpenAI Embeddings API.
-   **FR-004**: The system MUST perform a vector similarity search against the Qdrant `book_vectors` collection using the query embedding (or an embedding of the `context_snippet` if it is being used as primary context) to retrieve the top N (configurable, default 5) most relevant `chunk_id`s.
-   **FR-005**: The system MUST use the retrieved `chunk_id`s to query the Neon Postgres `rag_metadata` table and fetch the corresponding full text and metadata (specifically `text_snippet_preview` of relevant `RagChunk` objects).
-   **FR-006**: The system MUST construct a prompt template that combines the user's `query` and the full retrieved text context (or the provided `context_snippet`) into a single input for the LLM.
-   **FR-007**: The prompt template MUST explicitly instruct the OpenAI model to answer *only* based on the provided context and to state if it cannot answer from the given information.
-   **FR-008**: The system MUST send the constructed prompt to the OpenAI Chat Completion API (mimicking Agents/ChatKit functionality) and return the generated response.
-   **FR-009**: The `POST /api/rag/query` endpoint MUST return a JSON response containing the generated answer, and potentially the sources/chunk IDs used for the answer.
-   **FR-010**: The endpoint MUST handle OpenAI API errors (e.g., rate limits, invalid API key, network issues) and database connection errors (Qdrant, Neon) gracefully, providing informative error messages to the user.
-   **FR-011**: The system MUST utilize existing database client dependencies (`get_qdrant_client`, `get_neon_db`) defined in P1.3 for database interactions.
-   **FR-012**: The system MUST use the official **OpenAI Python SDK** for both query embedding generation and LLM response generation.

### Key Entities

-   **User Query**: The natural language question submitted by the user.
-   **Context Snippet**: An optional, user-provided text block that, if present, overrides RAG search and constrains the LLM's answer.
-   **Query Embedding**: A vector representation of the User Query (or Context Snippet).
-   **Qdrant Client**: Used for performing vector similarity searches against `book_vectors`.
-   **Neon Postgres Client**: Used for retrieving full `text_snippet_preview` and metadata from the `rag_metadata` table.
-   **Retrieved Chunks**: A collection of `RagChunk` objects (or their `text_snippet_preview` and metadata) identified as relevant to the query.
-   **Prompt Template**: The structured input that combines the query and context, sent to the OpenAI LLM.
-   **OpenAI LLM**: The language model (via OpenAI Chat Completion API) used for generating the final answer.
-   **Contextual Answer**: The generated response from the LLM, strictly grounded in the provided context.

## Success Criteria

### Measurable Outcomes

-   **SC-001**: The `POST /api/rag/query` endpoint returns a contextual answer within 2 seconds (p95 latency) for queries requiring RAG search.
-   **SC-002**: For 90% of valid queries (not using `context_snippet`), the vector similarity search in Qdrant retrieves at least 3 relevant chunks that contain information pertinent to the query, as judged by a human evaluator.
-   **SC-003**: The system correctly prioritizes `context_snippet` when provided, resulting in answers strictly derived from the snippet, 100% of the time, and avoiding external knowledge.
-   **SC-004**: For a set of 50 test queries with known answers in the book, the endpoint provides a correct contextual answer (judged for relevance and grounding) in 80% of cases, based on human evaluation.
-   **SC-005**: The endpoint can handle 5 concurrent requests without significant performance degradation (average latency increase by no more than 50% compared to single-user load).
-   **SC-006**: In case of transient OpenAI API or database connection errors, the endpoint returns an informative error message to the user within 5 seconds, rather than crashing.
