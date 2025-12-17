# Tasks: Diagnose and Fix RAG Chatbot Runtime Failure

**Feature Branch**: `007-fix-rag-runtime`
**Plan**: [Implementation Plan](plan.md)
**Spec**: [Feature Specification](spec.md)

## Phase 1: Robust Environment Loading & CORS
**Goal**: Prevent startup crashes caused by missing environment variables and ensure the server can successfully receive requests by fixing CORS configuration.
**Independent Test**: Server starts without crashing; `curl` requests to root or health endpoints return 200/404 (not connection refused).

### Implementation Tasks
- [ ] T001 [US1] Add pre-startup environment variable validation in `backend/main.py` (check for NEON_POSTGRES_CONNECTION_STRING, QDRANT_HOST, QDRANT_API_KEY, GEMINI_API_KEY, BASE_URL)
- [ ] T002 [US1] Refactor `backend/main.py` to simplify CORS middleware (remove duplicates, allow `*` or `http://localhost:3000` explicitly)
- [ ] T003 [US1] Add logging statement at the start of the chat router function in `backend/api/query.py` (or relevant router file) to confirm request receipt

## Phase 2: Refactor Agent Instantiation
**Goal**: Eliminate module-level side effects that cause import-time crashes and ensure the agent runner is correctly injected.
**Independent Test**: Importing `backend.query.agents` in a python shell does not crash even if env vars are missing.

### Implementation Tasks
- [ ] T004 [US1] Refactor `backend/query/agents.py` to move global `client = OpenAI(...)` instantiation inside a factory function or class method
- [ ] T005 [US1] Update `backend/dependencies.py` to instantiate the agent runner using the new factory function and ensure correct dependency injection
- [ ] T006 [US1] Add logging in the router function in `backend/api/query.py` to verify the injected agent object is valid (not None)

## Phase 3: Validate Agent Invocation & Cleanup
**Goal**: Verify the end-to-end request flow and remove diagnostic artifacts.
**Independent Test**: Chatkit frontend receives a text response; backend logs show full execution trace.

### Implementation Tasks
- [ ] T007 [US1] Verify and ensure `agent.run(message)` is correctly called in the router function in `backend/api/query.py`
- [ ] T008 [US1] Remove diagnostic logging statements added in T003 and T006 after verification

## Dependencies
- Phase 1 must complete before Phase 2 to ensure the server can actually start and load the refactored code.
- Phase 2 must complete before Phase 3 to ensure there is a valid agent object to invoke.

## Implementation Strategy
- **Surgical Fixes**: Changes are limited to `main.py`, `dependencies.py`, `query/agents.py`, and `api/query.py`.
- **Validation-Driven**: Each phase unlocks the next level of the request lifecycle (Startup -> Injection -> Execution).
