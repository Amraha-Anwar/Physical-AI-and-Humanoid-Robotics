# Research: Diagnose and Fix RAG Chatbot Runtime Failure

## Status
- **Date**: 2025-12-17
- **Investigator**: Gemini
- **Goal**: Confirm root cause of "Failed to fetch" errors and lack of backend logs.

## Findings

### 1. Root Cause Analysis
The investigation strongly suggests that the **server is failing to start entirely**, which explains both the "Failed to fetch" (connection refused) and the lack of logs (server never runs).

- **Startup Crash**: The application initializes critical services (`setup_db_clients` in `backend/dependencies.py`) and module-level clients (`backend/query/agents.py`) immediately on startup or import.
- **Missing Env Vars**: These initializers strictly require environment variables:
    - `NEON_POSTGRES_CONNECTION_STRING`
    - `QDRANT_HOST` / `QDRANT_API_KEY`
    - `GEMINI_API_KEY`
    - `BASE_URL`
- **RuntimeError**: If any of these are missing, the code raises `RuntimeError` or fails on `os.environ[]` lookups, causing the Uvicorn process to exit immediately.

### 2. Secondary Issues
- **CORS Config**: `backend/main.py` has a restrictive CORS policy (allow_origins=["http://localhost:3000"]) and potentially redundant middleware declarations. This could block requests even if the server starts, if the frontend URL doesn't match exactly.
- **Module-Level Side Effects**: `backend/query/agents.py` initializes an `OpenAI` client at the *module level* (global scope). This is dangerous because simply importing the file will crash the app if env vars aren't set, making it hard to debug or test components in isolation.

## Decisions & Recommendations

### Decision 1: Robust Environment Loading
- **Action**: Add a check at the very beginning of `main.py` (or a pre-startup script) to validate the presence of `.env` file and critical variables. Print a clear error message to `stderr` if missing.

### Decision 2: Lazy Initialization
- **Action**: Refactor `backend/query/agents.py` to avoid module-level client instantiation. Move client creation inside a dependency provider or function call (`get_agent_runner` or similar). This prevents import-time crashes.

### Decision 3: Fix CORS
- **Action**: Simplify and correct the CORS middleware in `main.py` to be more permissive for development (allow `*` or ensure the exact frontend origin is configured) and remove duplicates.

### Decision 4: Diagnostic Logging
- **Action**: As per the user's plan, adding logging at the start of the router function is good, but we also need logging *during startup* to catch the crash before requests even happen.

## Alternatives Considered
- **Mocking**: We could mock the clients for dev, but the goal is to fix the *real* runtime.
- **Docker**: Containerizing might enforce env vars, but the issue is code-level fragility.
