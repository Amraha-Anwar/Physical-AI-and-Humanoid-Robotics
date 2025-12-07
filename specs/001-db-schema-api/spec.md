# Feature Specification: P1.3 Database Connection: Schema Definition & API Layer

**Feature Branch**: `001-db-schema-api`  
**Created**: 2025-12-07  
**Status**: Draft  
**Input**: User description: "P1.3 Database Connection: Schema Definition & API Layer Target module: P1.3 Database Connection Focus: Define the minimal necessary schemas for Neon Postgres and Qdrant, and implement the corresponding Python client connections and initialization logic within FastAPI. Success criteria: - **Neon Postgres**: A FastAPI dependency (`get_neon_db`) can successfully connect and initialize the `rag_metadata` table. - **Qdrant**: A FastAPI dependency (`get_qdrant_client`) can connect and initialize the `book_vectors` collection with the correct vector size (matching the chosen OpenAI embedding model). - **Schema Alignment**: The Neon and Qdrant schemas are defined such that the Neon `chunk_id` acts as the foreign key/identifier for the Qdrant vector point ID. - **Connection Test**: Dedicated utility endpoints (`/db/status/neon`, `/db/status/qdrant`) return success upon connection validation. Constraints: - **Directory Structure**: **All related Python files (models, clients, dependencies, main app) must reside within a top-level directory named `backend/`.** - **FastAPI**: Must use Python classes (e.g., Pydantic) to model the data schemas. - **Neon Postgres**: Schema must be minimal (Metadata Standard). - **Qdrant**: Must use the Qdrant Python client, not the raw REST API. - **Vector Size**: The collection must be sized to match the chosen OpenAI embedding model (e.g., 1536 for `text-embedding-ada-002` or smaller/larger for v3 models). Not building: - The content pre-processing script (P2.1). - The full RAG retrieval and generation logic (P2.3). - Frontend integration (Phase 3). - User authentication/authorization beyond basic API key management (P1.2 scope)."

## User Scenarios & Testing

### User Story 1 - Verify Neon Postgres Connection (Priority: P1)
As a developer, I want to confirm that the FastAPI application can successfully establish a connection to the Neon Postgres database and initialize the `rag_metadata` table, so that the metadata storage is ready for use.
**Why this priority**: Essential for the core functionality of the RAG system, as it's the relational data store for document metadata.
**Independent Test**: Running the dedicated utility endpoint `/db/status/neon` and verifying a success response.
**Acceptance Scenarios**:
1.  **Given** the FastAPI application is running and Neon Postgres credentials are configured, **When** I access the `/db/status/neon` endpoint, **Then** the endpoint returns a success status (e.g., HTTP 200 OK) indicating a successful connection and `rag_metadata` table initialization.

### User Story 2 - Verify Qdrant Cloud Connection (Priority: P1)
As a developer, I want to confirm that the FastAPI application can successfully establish a connection to the Qdrant Cloud instance and initialize the `book_vectors` collection with the correct vector size, so that the vector database is ready for embedding storage.
**Why this priority**: Essential for the core functionality of the RAG system, as it's the vector data store for document embeddings.
**Independent Test**: Running the dedicated utility endpoint `/db/status/qdrant` and verifying a success response.
**Acceptance Scenarios**:
1.  **Given** the FastAPI application is running and Qdrant Cloud credentials are configured, **When** I access the `/db/status/qdrant` endpoint, **Then** the endpoint returns a success status (e.g., HTTP 200 OK) indicating a successful connection and `book_vectors` collection initialization with the correct vector size.

### User Story 3 - Schema Alignment Verification (Priority: P2)
As a developer, I want to ensure that the Neon Postgres and Qdrant schemas are aligned, where the Neon `chunk_id` serves as the foreign key/identifier for the Qdrant vector point ID, so that data integrity and cross-database referencing are maintained.
**Why this priority**: Important for data consistency and the correct functioning of the RAG retrieval process, allowing for efficient lookup between metadata and vector data.
**Independent Test**: This is primarily a design-time verification and can be confirmed by reviewing the database schema definitions and Pydantic models. Integration tests might also confirm correct data linking.
**Acceptance Scenarios**:
1.  **Given** the Neon Postgres schema for `rag_metadata` and the Qdrant `book_vectors` collection are defined, **When** examining their structures and related Pydantic models, **Then** the `chunk_id` in Neon is clearly identifiable as the linking mechanism to Qdrant's vector point ID, ensuring a one-to-one or one-to-many relationship as appropriate.

### Edge Cases

-   What happens when Neon Postgres is unreachable during application startup? (Application should gracefully handle the error and potentially retry or log a critical failure.)
-   How does the system handle Qdrant Cloud being unreachable during application startup? (Application should gracefully handle the error and potentially retry or log a critical failure.)
-   What if environment variables for database connections (e.g., connection strings, API keys) are missing or malformed? (Application should fail fast with clear error messages.)
-   What if the `rag_metadata` table or `book_vectors` collection already exist but have an incompatible schema/configuration? (Application should detect and report the inconsistency, or attempt a safe migration if within scope.)

## Requirements

### Functional Requirements

-   **FR-001**: The FastAPI application MUST define and use Python classes (e.g., Pydantic models) to represent the minimal schema for the Neon Postgres `rag_metadata` table.
-   **FR-002**: The FastAPI application MUST provide a dependency (`get_neon_db`) that establishes and manages a connection pool to Neon Postgres.
-   **FR-003**: The `get_neon_db` dependency MUST ensure the `rag_metadata` table is created or verified to exist with the correct schema upon its first invocation.
-   **FR-004**: The FastAPI application MUST provide a dependency (`get_qdrant_client`) that instantiates and manages a client connection to Qdrant Cloud.
-   **FR-005**: The `get_qdrant_client` dependency MUST ensure the `book_vectors` collection is created or verified to exist with the specified vector size (matching the chosen OpenAI embedding model, e.g., 1536) upon its first invocation.
-   **FR-006**: The FastAPI application MUST expose a `GET /db/status/neon` utility endpoint that returns an HTTP 200 OK status if the Neon Postgres connection is active and the `rag_metadata` table is accessible.
-   **FR-007**: The FastAPI application MUST expose a `GET /db/status/qdrant` utility endpoint that returns an HTTP 200 OK status if the Qdrant Cloud connection is active and the `book_vectors` collection is accessible and correctly configured.
-   **FR-008**: The schema definition for Neon Postgres and the collection configuration for Qdrant MUST implicitly or explicitly establish `chunk_id` as the primary key/identifier in Neon that directly corresponds to the `id` field of a vector point in Qdrant.

### Key Entities

-   **RAG Metadata**: Represents the structured information about text chunks from the "Physical AI & Humanoid Robotics" book, stored in Neon Postgres.
    -   Key attributes (examples): `chunk_id` (unique identifier), `document_id`, `chapter_title`, `page_number`, `text_snippet_preview`. The schema will be minimal.
-   **Book Vectors**: Represents the high-dimensional vector embeddings of text content from the book, stored in Qdrant.
    -   Key attributes: Each vector point will have a unique `id` (corresponding to Neon's `chunk_id`) and a `payload` containing metadata (e.g., `chunk_id` for inverse lookup) and the vector itself.

## Success Criteria

### Measurable Outcomes

-   **SC-001**: The `GET /db/status/neon` endpoint returns an HTTP 200 OK response within 500 milliseconds (p95 latency) after the FastAPI application has started.
-   **SC-002**: The `GET /db/status/qdrant` endpoint returns an HTTP 200 OK response within 500 milliseconds (p95 latency) after the FastAPI application has started.
-   **SC-003**: The `book_vectors` collection in Qdrant is initialized with the correct vector size (e.g., 1536 for `text-embedding-3-small`) upon application startup, as verified by Qdrant client inspection.
-   **SC-004**: A test data insertion into Neon Postgres and Qdrant (via development scripts) correctly links metadata to vector points using `chunk_id` as the common identifier.

## Assumptions

-   **ASS-001**: The specification intentionally includes explicit mentions of technologies (FastAPI, Neon Postgres, Qdrant, Pydantic) and specific implementation details. This is an accepted deviation from strict technology-agnosticism for specifications, as it directly aligns with the project's mandated technology stack as defined in the project constitution (v2.0.0). The project prioritizes successful integration of the mandated stack, even if it leads to a more technical specification.

## Clarifications

### Session 2025-12-07

-   **Q**: Should the specification strictly adhere to a generalized, technology-agnostic phrasing, which would conflict with the detailed requirements of the existing, mandated stack? Or should we proceed with the current technical specification which is immediately executable but violates the abstract principle of agnosticism?
    **A**: Proceed with the current technical specification. The project's success is tied to the successful integration of the mandated, non-negotiable stack. This acknowledges a deviation from a general guideline in favor of project-specific technical mandates.