# Implementation Plan - Diagnose and Fix RAG Chatbot Runtime Failure

**Feature**: Diagnose and Fix RAG Chatbot Runtime Failure (`007-fix-rag-runtime`)
**Status**: In Progress

## Technical Context

The RAG chatbot backend (FastAPI) is failing to handle requests from the frontend, reporting "Failed to fetch" (connection refused/closed) and showing no logs.
Research indicates the root cause is likely a startup crash due to missing environment variables causing `RuntimeError` during client initialization in `dependencies.py` and `agents.py`. A secondary issue is restrictive CORS configuration.

**Dependencies**:
- FastAPI
- OpenAI SDK (used for Gemini)
- Neon (Postgres)
- Qdrant (Vector DB)
- Python `dotenv`

**Unknowns**:
- None. Root cause identified as startup crash + CORS.

## Constitution Check

- **I. RAG Accuracy**: Fix ensures the agent is actually invoked.
- **II. Interactive UX**: Fix enables the frontend to work.
- **III. Scalability**: Fix maintains serverless compatibility by handling env vars correctly.
- **IV. Code Integrity**: Fix follows FastAPI and Python best practices (no global side effects).

## Plan Phases

### Phase 1: Robust Environment Loading & CORS (T087)
**Goal**: Prevent startup crashes and ensure the server can receive requests.

- [ ] **T087.1 Env Validation**: Add a pre-startup check in `main.py` to verify `NEON_POSTGRES_CONNECTION_STRING`, `QDRANT_HOST`, `QDRANT_API_KEY`, `GEMINI_API_KEY`, `BASE_URL` exist. Log clear errors to stderr if missing.
- [ ] **T087.2 Fix CORS**: Update `backend/main.py` to simplify CORS middleware. Allow `*` for dev or ensure `http://localhost:3000` is explicitly allowed. Remove redundant middleware.
- [ ] **T087.3 Endpoint Logging**: Add a log statement at the start of `/api/chat` to confirm request receipt.

### Phase 2: Refactor Agent Instantiation (T088)
**Goal**: Remove dangerous module-level side effects in `backend/query/agents.py`.

- [ ] **T088.1 Refactor agents.py**: Move the global `client = OpenAI(...)` inside a function or class (e.g., `get_agent_runner`).
- [ ] **T088.2 Dependency Injection**: Ensure `dependencies.py` correctly instantiates the agent runner using `get_agent_runner` and passes it to the router.
- [ ] **T088.3 Object Logging**: Log the successful injection of the agent object in the router.

### Phase 3: Validate Agent Invocation (T089)
**Goal**: Verify the full request flow.

- [ ] **T089.1 Invoke Agent**: Verify `agent.run(message)` is called in the router.
- [ ] **T089.2 Cleanup**: Once verified, remove temporary diagnostic logs.

## Verification Plan

### Automated Tests
- N/A (Runtime fix verified via manual execution and logs).

### Manual Verification
1. Start server `uvicorn main:app`.
2. Verify "Application startup complete" log (no crash).
3. Send POST request via curl or Frontend.
4. Verify backend logs show request processing.
5. Verify frontend receives text response.