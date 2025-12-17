# Feature Specification: Chatkit UI + Backend Fix

**Feature Branch**: `006-chatkit-ui-backend-fix`
**Created**: 2025-12-14
**Status**: Draft
**Input**: User description: Apply chat CSS from "frontend/components/chatbot.css", update Chatkit message send logic to call FastAPI backend, fix frontend → backend integration, and maintain existing agent-based RAG logic.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Styled Chat Interface (Priority: P1)

As a user, I want the chat widget to visually match the previous chatbot style (floating button, colors), so that the user experience is consistent and polished.

**Why this priority**: Visual consistency is a mandatory requirement. The raw Chatkit style needs to be adapted.

**Independent Test**: Open the chat widget. Verify it uses the correct primary colors, fonts, and positioning as defined in `chatbot.css`.

**Acceptance Scenarios**:

1. **Given** the Docusaurus site is loaded, **When** I view the page, **Then** I see a floating chat button styled according to `chatbot.css`.
2. **Given** the chat window is open, **When** I look at the header and message bubbles, **Then** they MUST reflect the project's color scheme (e.g., `var(--ifm-color-primary)`).

### User Story 2 - Functional RAG Chat (Priority: P1)

As a user, I want my questions sent from the chat widget to be answered by the backend agent, so that I can get information from the knowledge base.

**Why this priority**: This is the core functionality. The frontend must correctly call the `POST /api/query` endpoint.

**Independent Test**: Send a message "What is a humanoid robot?". Verify the network request goes to `/api/query` and a relevant answer appears in the chat window.

**Acceptance Scenarios**:

1. **Given** the chat widget, **When** I type a message and hit send, **Then** the UI MUST display the user message immediately.
2. **Given** the message is sent, **When** the backend responds, **Then** the UI MUST display the assistant's response in a new bubble.
3. **Given** a network error, **When** the request fails, **Then** the UI MUST display a friendly error message.

---

### Edge Cases

- **Long Responses**: Chat bubbles should wrap text correctly and handle markdown/newlines if the backend returns them (though plain text is expected for now).
- **Session Continuity**: The session ID should persist across the user's session on the page (already implemented, but verifying).

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The `ChatkitWidget` component MUST import and apply styles from `frontend/components/chatbot.css` (or equivalent custom CSS file) to match the legacy design.
- **FR-002**: The `ChatkitWidget` MUST use `fetch` to POST to `/api/query` with the payload `{ query_text: string, session_id: string }`.
- **FR-003**: The `ChatkitWidget` MUST handle the backend response format `{ answer: string, context: string }` and display `answer`.
- **FR-004**: The integration MUST NOT modify the backend `query_service.py` or `agents.py` logic; it purely fixes the frontend-to-backend wiring and styling.

### Key Entities

- **ChatWidget**: The React component using Chatkit.
- **Backend API**: The FastAPI `POST /api/query` endpoint.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Chat widget styling matches the provided `chatbot.css` reference (colors, button shape).
- **SC-002**: 100% of valid user queries sent via the widget result in a backend call and response display.
- **SC-003**: No regressions in Docusaurus build or runtime.

### Assumptions

- `frontend/components/chatbot.css` exists or its content is provided/can be inferred from previous context (we created `ChatkitWidget.module.css` previously, we will now align it with the user's request for `chatbot.css`).
- The backend is running on port 8000 and Docusaurus on port 3000.