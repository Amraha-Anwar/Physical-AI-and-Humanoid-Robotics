# P1.3 Database Connection: Schema Definition & API Layer - Tasks

**Feature Branch**: `001-db-schema-api`  
**Goal**: Establish robust and testable connections to Neon Postgres and Qdrant Cloud within the FastAPI backend, complete with necessary data models and verification endpoints.

---

## Phase 1: Setup

*   - [ ] T001 Create the top-level `backend/` directory in the project root.
*   - [ ] T002 Navigate to `backend/` and initialize poetry: `poetry init --no-interaction`.
*   - [ ] T003 Install core Python dependencies: `poetry add fastapi pydantic qdrant-client "psycopg2-binary" python-dotenv uvicorn`.

## Phase 2: Foundational Tasks

This phase establishes the core models and client implementations that are prerequisites for verifying database connections.

*   - [X] T004 Create `backend/models.py` file.
*   - [X] T005 Define Pydantic models `QdrantConfig` and `NeonConfig` in `backend/models.py`, ensuring sensitive data (API keys, connection strings) are read from environment variables.
*   - [X] T006 Define Pydantic model `RagChunk` in `backend/models.py`, including `chunk_id` as a string type to be used as the primary key in Neon and point ID in Qdrant.
*   - [X] T007 [P] Create `backend/qdrant_client.py` file.
*   - [X] T008 [P] Implement `initialize_qdrant_client(config: QdrantConfig)` function in `backend/qdrant_client.py` to connect to Qdrant Cloud using `qdrant-client` library.
*   - [X] T009 [P] Add logic to `initialize_qdrant_client` in `backend/qdrant_client.py` to check for the existence of the `book_vectors` collection and create it if missing, configuring `vector_size=1536` and `distance=COSINE`.
*   - [X] T010 [P] Create `backend/neon_client.py` file.
*   - [X] T011 [P] Implement `initialize_neon_db(config: NeonConfig)` function in `backend/neon_client.py` to establish a connection to Neon Postgres using `psycopg2-binary`.
*   - [X] T012 [P] Add DDL logic within `initialize_neon_db` in `backend/neon_client.py` to create the `rag_metadata` table if it doesn't exist, matching the fields defined in the `RagChunk` Pydantic model.

## Phase 3: User Story 1 - Verify Neon Postgres Connection [US1]

**Goal**: Confirm that the FastAPI application can successfully establish a connection to the Neon Postgres database and initialize the `rag_metadata` table.
**Independent Test**: Running the `/db/status/neon` endpoint returns a success response (HTTP 200 OK).

*   - [X] T013 [US1] Create `backend/dependencies.py` file.
*   - [X] T014 [US1] Implement `get_neon_db()` FastAPI dependency function in `backend/dependencies.py` that utilizes `initialize_neon_db` and returns a database connection.
*   - [X] T015 [US1] Create `backend/main.py` file to house the FastAPI application.
*   - [X] T016 [US1] Implement the `GET /db/status/neon` endpoint in `backend/main.py`. This endpoint should use `Depends(get_neon_db)` and return an HTTP 200 OK if the dependency successfully provides a connection.
*   - [X] T017 [US1] Configure FastAPI application startup hook (`on_startup`) in `backend/main.py` to call a `setup_db_clients` function (to be defined) that initializes the global Neon client.

## Phase 4: User Story 2 - Verify Qdrant Cloud Connection [US2]

**Goal**: Confirm that the FastAPI application can successfully establish a connection to the Qdrant Cloud instance and initialize the `book_vectors` collection with the correct vector size.
**Independent Test**: Running the `/db/status/qdrant` endpoint returns a success response (HTTP 200 OK).

*   - [ ] T018 [US2] Implement `get_qdrant_client()` FastAPI dependency function in `backend/dependencies.py` that utilizes `initialize_qdrant_client` and returns a Qdrant client instance.
*   - [ ] T019 [US2] Implement the `GET /db/status/qdrant` endpoint in `backend/main.py`. This endpoint should use `Depends(get_qdrant_client)` and return an HTTP 200 OK if the dependency successfully provides a client.
*   - [ ] T020 [US2] Update the `setup_db_clients` function in `backend/dependencies.py` to also initialize the global Qdrant client. (This task implies T017 is completed and setup_db_clients is already configured for Neon, now expanding for Qdrant)

## Phase 5: User Story 3 - Schema Alignment Verification [US3]

**Goal**: Ensure that the Neon Postgres and Qdrant schemas are aligned, where the Neon `chunk_id` serves as the foreign key/identifier for the Qdrant vector point ID.
**Independent Test**: An integration test verifies the `chunk_id` linkage during data insertion.

*   - [ ] T021 [US3] Create `tests/` directory at the project root.
*   - [ ] T022 [US3] Create `tests/integration_test.py` file.
*   - [ ] T023 [US3] Write an integration test in `tests/integration_test.py` that inserts a sample `RagChunk` into Neon and a corresponding vector point into Qdrant, verifying that the `chunk_id` from Neon correctly becomes the `id` for the Qdrant point.
*   - [ ] T024 [US3] Update `pyproject.toml` to include `pytest` for testing.

## Phase 6: Polish & Cross-Cutting Concerns

*   - [ ] T025 Create an `.env.example` file in the `backend/` directory with placeholders for all required environment variables (`NEON_POSTGRES_CONNECTION_STRING`, `QDRANT_HOST`, `QDRANT_API_KEY`).
*   - [ ] T026 Add comprehensive docstrings and type hints to all functions and classes in `backend/models.py`, `backend/qdrant_client.py`, `backend/neon_client.py`, `backend/dependencies.py`, and `backend/main.py`.
*   - [ ] T027 Review and refine error handling for database connection failures and table/collection creation, ensuring informative log messages.

---

## Dependencies

This section outlines the completion order of user stories.

1.  **User Story 1 (Verify Neon Postgres Connection)** -> **User Story 2 (Verify Qdrant Cloud Connection)** -> **User Story 3 (Schema Alignment Verification)**

## Parallel Execution Opportunities

*   **Between Foundational Tasks (T007-T012)**: Creation and implementation of Qdrant client (`backend/qdrant_client.py`) and Neon client (`backend/neon_client.py`) can be developed in parallel.
*   **Between User Story 1 & 2 Dependencies/Endpoints (T014-T016 and T018-T019)**: Once foundational tasks are complete, the implementation of `get_neon_db` and `get_qdrant_client` dependencies, and their respective status endpoints, can be done in parallel.

## Implementation Strategy

The implementation will follow an iterative approach, prioritizing foundational components before integrating them into FastAPI endpoints.
1.  Start with defining data models and basic client connections.
2.  Implement and verify Neon Postgres connection and schema creation.
3.  Implement and verify Qdrant Cloud connection and collection creation.
4.  Integrate both clients into FastAPI using dependency injection.
5.  Add verification endpoints to confirm connectivity.
6.  Develop integration tests to validate schema alignment and overall functionality.
