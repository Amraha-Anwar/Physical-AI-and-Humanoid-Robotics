# P3.1 Docusaurus Chat Component: React UI and API Hookup - Tasks

**Feature Branch**: `001-docusaurus-chat-ui`  
**Goal**: Develop a reusable React component compatible with Docusaurus's MDX structure that provides the user interface for the RAG chatbot and integrates the two-mode query system (standard query and highlight-to-query) with the backend FastAPI endpoint.

---

## Phase 1: Setup and Basic Component Structure

*   - [ ] T001 Create `frontend/src/components/RAGChatComponent.tsx` file.
*   - [ ] T002 Define TypeScript interfaces for `ConversationMessage` and `ChatState` (as per `data-model.md`) within or alongside `frontend/src/components/RAGChatComponent.tsx`.
*   - [ ] T003 Initialize React state hooks (`useState`) for `messages`, `currentQuery`, `highlightedText`, `isLoading`, `error` in `frontend/src/components/RAGChatComponent.tsx`.

## Phase 2: User Story 1 - Docusaurus Compatibility & Basic Rendering [US1]

**Goal**: The component is successfully embedded within a Docusaurus MDX page and renders correctly.
**Independent Test**: Embed the component in a test MDX file and verify it renders without errors.

*   - [ ] T004 [US1] Implement basic JSX structure (divs for chat container, messages display, input area) in `frontend/src/components/RAGChatComponent.tsx`.
*   - [ ] T005 [US1] Embed the `RAGChatComponent` in a test MDX file (e.g., `frontend/docs/intro/01-foundations.mdx`) for initial rendering verification.

## Phase 3: User Story 3 - Highlight Listener & `context_snippet` Preparation [US3]

**Goal**: Implement a client-side listener that captures highlighted text and prepares it to be sent as the `context_snippet` parameter.
**Independent Test**: Highlight text on the Docusaurus page and verify `highlightedText` state updates correctly.

*   - [ ] T006 [US3] Implement `useEffect` hook in `frontend/src/components/RAGChatComponent.tsx` to attach `selectionchange` event listener to `document`.
*   - [ ] T007 [US3] In the `selectionchange` handler, capture `window.getSelection().toString()` and update `highlightedText` state in `frontend/src/components/RAGChatComponent.tsx`.
*   - [ ] T008 [US3] Implement `useEffect` cleanup function to remove the `selectionchange` listener when the component unmounts in `frontend/src/components/RAGChatComponent.tsx`.
*   - [ ] T009 [US3] Add a UI element to `frontend/src/components/RAGChatComponent.tsx` to visually display the `highlightedText` as a cue (e.g., a small tag above the input).

## Phase 4: User Story 2 - Standard Text Input & API Connectivity [US2]

**Goal**: The component provides a standard text input field for general queries and can successfully send a test query to the FastAPI endpoint and display the response.
**Independent Test**: Enter text in the input box and send. Verify the request hits the FastAPI endpoint *without* the `context_snippet` parameter and the bot responds.

*   - [ ] T010 [US2] Implement text input field and a "Send" button in `frontend/src/components/RAGChatComponent.tsx`, ensuring proper state binding (`currentQuery`).
*   - [ ] T011 [US2] Implement `handleSendMessage` asynchronous function in `frontend/src/components/RAGChatComponent.tsx` to:
    *   Construct `RAGQueryRequest` payload from `currentQuery` and `highlightedText`.
    *   Use `fetch` to send a POST request to `http://localhost:8000/api/rag/query` (update URL as needed).
    *   Handle `isLoading` and `error` states during the API call.
    *   Parse `RAGQueryResponse` from the backend.
*   - [ ] T012 [US2] Configure Docusaurus `docusaurus.config.js` to proxy `/api` requests to the FastAPI backend (e.g., `target: 'http://localhost:8000'`) to handle CORS during local development.

## Phase 5: User Story 4 - Conversation History & State Management [US4]

**Goal**: The chat interface correctly manages and displays the conversation history (user query, bot response).
**Independent Test**: Interact with the chat component and verify that all user and bot messages are correctly added to the display.

*   - [ ] T013 [US4] Implement logic within `handleSendMessage` to add user query to `messages` state before the API call.
*   - [ ] T014 [US4] Implement logic within `handleSendMessage` to add bot response to `messages` state after a successful API call.
*   - [ ] T015 [US4] Render `messages` state in the message display area of `frontend/src/components/RAGChatComponent.tsx`, differentiating between user and bot messages.

## Phase 6: Polish & Cross-Cutting Concerns

*   - [ ] T016 Apply CSS Modules styling by creating `frontend/src/components/RAGChatComponent.module.css` and linking it to `RAGChatComponent.tsx`, implementing basic, theme-consistent styling.
*   - [ ] T017 Add comprehensive comments and type annotations to `frontend/src/components/RAGChatComponent.tsx`.
*   - [ ] T018 Create `frontend/src/components/tests/RAGChatComponent.test.tsx` for unit tests covering state management, event handling (e.g., selection change, input change), and API call mocks.
*   - [ ] T019 Update `frontend/package.json` to include any necessary testing libraries (e.g., `react-testing-library`, `jest-dom`) and configure testing scripts.
*   - [ ] T020 Implement basic error display in `frontend/src/components/RAGChatComponent.tsx` for `error` state.

---

## Dependencies

This section outlines the completion order of user stories.

1.  **Phase 1 (Setup and Basic Component Structure)** -> **Phase 2 (User Story 1 - Docusaurus Compatibility & Basic Rendering)** -> **Phase 3 (User Story 3 - Highlight Listener & `context_snippet` Preparation)** -> **Phase 4 (User Story 2 - Standard Text Input & API Connectivity)** -> **Phase 5 (User Story 4 - Conversation History & State Management)** -> **Phase 6 (Polish & Cross-Cutting Concerns)**

## Parallel Execution Opportunities

*   **Phase 2 (US1)** and **Phase 3 (US3)** can be developed in parallel up to the point of API integration, as their core logic (rendering vs. selection listener) is independent.
*   **Styling (T016)** can be worked on throughout the component's development as UI elements are added.
*   **Testing (T018-T019)** can begin early with unit tests for state management and individual functions.

## Implementation Strategy

The implementation will follow a layered approach, starting with the basic component structure and incrementally adding functionality:
1.  Set up the React component file and initial state management.
2.  Implement the basic UI and verify Docusaurus rendering.
3.  Add the highlight listener and its associated UI cues.
4.  Implement the API interaction logic, including conditional payload construction and error handling.
5.  Integrate conversation history management.
6.  Finalize styling, add comprehensive testing, and documentation.
