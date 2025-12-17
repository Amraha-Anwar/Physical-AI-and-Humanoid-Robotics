# Architectural Plan: Chat Retrieval Fix & Chatkit Integration

**Feature**: Chat Retrieval Fix & Chatkit Integration
**Status**: Draft
**Spec**: [specs/005-chat-retrieval-fix/spec.md](../spec.md)

## 1. High-Level Architecture Overview

The objective is to refine the RAG system's recall by tuning the backend retrieval parameter and to modernize the frontend interaction by replacing the custom `chatsSDK.js`/`RAGChatWidget` with a standard `Chatkit` component integration.

### Backend Tuning
-   **Current State**: `QueryService._execute_retrieval` fetches a hardcoded `limit=5` chunks from Qdrant.
-   **New State**: `QueryService._execute_retrieval` will be adjusted to fetch `limit=10` chunks to ensure broader context coverage for the Agent Reasoning layer. The `RetrievalTool` wrapper in `agents.py` will remain the same but benefit from this increased recall.

### Frontend Integration
-   **Current State**: `frontend/src/theme/Root.js` renders a custom `RAGChatWidget` which uses `ChatSDK.js` to call `POST /api/query`.
-   **New State**: `frontend/src/theme/Root.js` will render a new `ChatkitWidget` component.
-   **Component**: The new `ChatkitWidget` (located in `frontend/src/components/ChatkitWidget.js`) will implement the UI using standard React patterns (or the `Chatkit` library if available/mocked as standard React) and directly call `fetch('/api/query')` without the legacy `ChatSDK.js` abstraction.

## 2. Integration & Interaction Flow

### Backend: Retrieval Tuning
1.  **Call**: `OrchestratorAgent` calls `retrieve_context`.
2.  **Execution**: `QueryService._execute_retrieval` queries Qdrant with `limit=10`.
3.  **Result**: Up to 10 relevant chunks are returned to the agent context.

### Frontend: Chatkit Replacement
1.  **User Action**: User opens the chat bubble on the Docusaurus site.
2.  **Component**: `ChatkitWidget` renders.
3.  **Message Send**: User types a message. `ChatkitWidget` sends a standard `fetch` POST request to `/api/query` with JSON payload `{ query_text: "...", session_id: "..." }`.
4.  **Response Handling**: The widget receives the JSON response `{ answer: "..." }` and displays it in the chat history.
5.  **State**: Session ID is managed locally within the widget (e.g., in `localStorage` or React state).

## 3. Implementation Details

### File Impacts (Additive/Modification)

1.  **`backend/query/query_service.py`** (MODIFY)
    -   Change `limit=5` to `limit=10` in `_execute_retrieval`.

2.  **`frontend/package.json`** (MODIFY)
    -   Add `@chatscope/chat-ui-kit-react` (or standard `react-chat-widget` equivalent if "Chatkit" implies that, otherwise we build a standard React component named `Chatkit` as per "Chatkit" typically referring to a library like ChatScope or Pusher, but here we will use a standard clean React implementation if no specific library is mandated, OR use `@chatscope/chat-ui-kit-react` which is the standard open-source "Chat UI Kit").
    -   *Decision*: We will use `@chatscope/chat-ui-kit-react` as it is the most common "Chatkit" for React.

3.  **`frontend/src/components/ChatkitWidget.js`** (NEW)
    -   Implement the chat interface using `@chatscope/chat-ui-kit-react`.
    -   Handle `onSend` to call `/api/query`.
    -   Handle styling to match existing theme (minimal overrides).

4.  **`frontend/src/theme/Root.js`** (MODIFY)
    -   Import `ChatkitWidget`.
    -   Replace `RAGChatWidget` with `ChatkitWidget`.

5.  **`frontend/src/theme/RAGChatWidget/ChatSDK.js`** (DELETE/DEPRECATE)
    -   File will be removed or left unused.

### Configuration
-   No new env vars required.

## 4. Risk Assessment

-   **Blast Radius**: **LOW**.
    -   Backend change is a simple parameter tune.
    -   Frontend change is isolated to the chat overlay component. Main documentation site is unaffected.
-   **UX Consistency**:
    -   *Risk*: New widget might look different.
    -   *Mitigation*: We will apply basic CSS styling to the `ChatkitWidget` to match the previous widget's floating button position and color scheme.
-   **Performance**:
    -   *Risk*: Fetching 10 chunks might slightly increase latency.
    -   *Mitigation*: Negligible impact for <20 chunks. Neon fetch is fast.

## 5. Verification Plan

1.  **Backend Test**: Run `test_query.py` and inspect logs to see 10 chunks returned (if relevant).
2.  **Frontend Test**:
    -   Start Docusaurus (`npm start`).
    -   Open Chat.
    -   Send message "Hello".
    -   Verify response appears.
    -   Check Browser Network tab to ensure request goes to `/api/query` and response is handled.
