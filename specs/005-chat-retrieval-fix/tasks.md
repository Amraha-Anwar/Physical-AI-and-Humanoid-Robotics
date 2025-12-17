# Implementation Tasks: Chat Retrieval Fix & Chatkit Integration

**Feature**: Chat Retrieval Fix & Chatkit Integration
**Status**: Draft
**Spec**: [specs/005-chat-retrieval-fix/spec.md](../spec.md)
**Plan**: [specs/005-chat-retrieval-fix/plan.md](../plan.md)

## Task List

### Task 0: Fix Agent Runner Sync/Async Mismatch [CRITICAL]
- [x] **Description**: In `backend/query/agents.py`, replace the `AsyncOpenAI` client with the synchronous `OpenAI` client. The `Runner.run_sync` method requires a synchronous client to function correctly; currently, it receives coroutines from the async client and fails silently.
- **Files Affected**: `backend/query/agents.py`.
- **Explicit Non-Effects**: No changes to agent instructions, tools, or other logic.
- **Acceptance Criteria**:
    - `AsyncOpenAI` import is replaced with `OpenAI`.
    - `client` instantiation uses `OpenAI(...)`.
    - Agent execution logs show successful completion instead of silence/hanging.

### Task 1: Tune Backend Retrieval Limit
- [x] **Description**: In `backend/query/query_service.py`, modify the `_execute_retrieval` method to increase the Qdrant query limit from 5 to 10.
- **Files Affected**: `backend/query/query_service.py`.
- **Explicit Non-Effects**: No changes to retrieval logic, embedding, or ranking. No changes to `agents.py`.
- **Acceptance Criteria**:
    - `limit` parameter in `qdrant_client.query_points` is set to 10.
    - `test_query.py` still runs successfully.

### Task 2: Install Chatkit Frontend Dependency
- [x] **Description**: Add `@chatscope/chat-ui-kit-react` to the frontend dependencies.
- **Files Affected**: `frontend/package.json`.
- **Explicit Non-Effects**: No changes to other packages.
- **Acceptance Criteria**:
    - `npm install` runs successfully.
    - Package is listed in `package.json`.

### Task 3: Create ChatkitWidget Component
- [x] **Description**: Create `frontend/src/components/ChatkitWidget.js` that implements a chat interface using `@chatscope/chat-ui-kit-react`. It must handle sending messages to `POST /api/query` and displaying responses. It should be styled minimally to match the previous widget (floating button).
- **Files Affected**: `frontend/src/components/ChatkitWidget.js` (NEW).
- **Explicit Non-Effects**: No changes to existing theme files yet.
- **Acceptance Criteria**:
    - Component renders a floating chat button.
    - Component opens a chat window on click.
    - Sends correct JSON to `/api/query`.
    - Displays "User" and "Assistant" messages.

### Task 4: Integrate ChatkitWidget into Root
- [x] **Description**: Modify `frontend/src/theme/Root.js` to import and render `ChatkitWidget` instead of the legacy `RAGChatWidget`.
- **Files Affected**: `frontend/src/theme/Root.js`.
- **Explicit Non-Effects**: No changes to Docusaurus config or other theme components.
- **Acceptance Criteria**:
    - The new chat widget appears on the documentation site.
    - The old chat widget is gone.

### Task 5: Remove Legacy Chat SDK
- [x] **Description**: Remove `frontend/src/theme/RAGChatWidget/ChatSDK.js` and `frontend/src/theme/RAGChatWidget/index.js` (and the folder if empty).
- **Files Affected**: `frontend/src/theme/RAGChatWidget/ChatSDK.js`, `frontend/src/theme/RAGChatWidget/index.js`.
- **Explicit Non-Effects**: No other files deleted.
- **Acceptance Criteria**:
    - Files are deleted.
    - Build (`npm run build`) still succeeds (verifying no dead imports).

### Task 6: Final Validation
- [x] **Description**: Run the full stack (backend + frontend) and verify the chat flow.
- **Files Affected**: None.
- **Explicit Non-Effects**: No code changes.
- **Acceptance Criteria**:
    - Chat bubble opens.
    - "Hello" query returns a response.
    - Network tab shows call to `/api/query`.
    - Backend logs show retrieval of up to 10 chunks.
