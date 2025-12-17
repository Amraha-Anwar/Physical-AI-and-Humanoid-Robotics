# Feature Specification: Chat Retrieval Fix & Chatkit Integration

**Feature Branch**: `005-chat-retrieval-fix`
**Created**: 2025-12-14
**Status**: Draft
**Input**: User description: Ensure agents retrieve all relevant context from Qdrant & Neon, replace chatsSDK.js with proper Chatkit integration, and maintain all existing UI, layout, backend logic, and RAG flow.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Context-Aware Chat Response (Priority: P1)

As a user, I want the chat interface to provide answers grounded in the full context of my query, so that I receive accurate and complete information from the knowledge base.

**Why this priority**: The primary value of the RAG system is accurate retrieval. Current retrieval tuning is required to ensure *all* relevant context is fetched.

**Independent Test**: Send a query known to require multiple chunks of information (e.g., "What are the three laws of robotics mentioned in the text?"). Verify the response contains all parts and the retrieval logs show multiple chunks being fetched.

**Acceptance Scenarios**:

1. **Given** a user query requiring broad context, **When** the agent processes it, **Then** it MUST retrieve multiple relevant chunks from Qdrant/Neon and synthesize them.
2. **Given** the retrieval wrapper, **When** invoked, **Then** it MUST NOT fail silently or truncate essential results.

### User Story 2 - Chatkit Frontend Integration (Priority: P1)

As a user, I want to interact with the system using a robust, standard chat interface (Chatkit), so that I have a reliable and maintainable user experience without custom SDK glitches.

**Why this priority**: Replacing `chatsSDK.js` is a mandatory correction to improve maintainability and stability.

**Independent Test**: Open the chat page in the browser. Verify the chat UI renders correctly, messages can be sent, and responses are received and displayed without console errors related to `chatsSDK`.

**Acceptance Scenarios**:

1. **Given** the frontend application, **When** I load the chat page, **Then** the Chatkit component MUST load instead of the legacy custom chat implementation.
2. **Given** the Chatkit interface, **When** I send a message, **Then** it MUST correctly format the request to the `POST /api/query` endpoint and display the agent's response.

---

### Edge Cases

- **Network Failure**: Chatkit should handle API timeout or failure gracefully (e.g., retry or error message).
- **Empty Retrieval**: If Qdrant returns nothing, Chatkit should display the agent's "I don't know" response clearly.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST replace the usage of `chatsSDK.js` in the frontend with the **Chatkit** library (or a standard React equivalent if "Chatkit" refers to a specific proprietary tool, assuming standard React integration pattern).
- **FR-002**: The frontend MUST communicate with the existing `POST /api/query` endpoint without requiring changes to the endpoint's contract.
- **FR-003**: The backend Agent Retrieval Tool (`retrieve_context`) MUST be tuned/verified to ensure it returns *all* relevant chunks provided by `QueryService`, not just the top 1 or a summary, to the Reasoning Agent.
- **FR-004**: The UI layout and styling MUST remain identical to the previous version; only the chat component logic/internal component is swapped.
- **FR-005**: All RAG logic (Qdrant/Neon) MUST remain intact; only the wrapper invocation or parameters (like `limit`) in the agent layer may be adjusted if necessary for better recall.

### Key Entities

- **Chat Interface**: The frontend component user interacts with.
- **Agent API**: The `POST /api/query` endpoint.
- **Retrieval Context**: The list of text chunks fetched from the DB.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: `chatsSDK.js` is removed from the codebase or disconnected from the active UI.
- **SC-002**: User queries in the new Chatkit UI receive responses indistinguishable in quality/format from the previous backend tests.
- **SC-003**: Retrieval logs show the Agent receiving full context (up to the token limit) when necessary.
- **SC-004**: Frontend console is free of errors related to message passing.

### Assumptions

- "Chatkit" refers to a standard React chat library or the existing project's intended replacement component. If a specific library `chatkit` is meant, it will be installed; otherwise, we assume standard React component integration.
- The `POST /api/query` endpoint is stable and working (verified in feature 004).