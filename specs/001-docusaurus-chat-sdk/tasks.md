# Implementation Tasks: Docusaurus ChatKit SDK Integration

**Feature Branch**: `001-docusaurus-chat-sdk`
**Spec**: [spec.md](spec.md)
**Plan**: [plan.md](plan.md)

## Phase 1: Setup
*Initialize project structure and dependencies.*

- [x] T001 Create component directory structure in `frontend/src/theme/RAGChatWidget`
- [x] T002 Create CSS module file `frontend/src/theme/RAGChatWidget/styles.module.css` with initial empty class definitions
- [x] T003 Create `frontend/src/theme/Root.js` to serve as the global mount point (swizzling Root)

## Phase 2: Foundational
*Implement core SDK logic and API communication.*

**Goal**: Establish reliable communication with the backend.
**Independent Test**: Can manually invoke `ChatSDK.sendMessage()` from browser console and see network request/response.

- [x] T004 Create `frontend/src/theme/RAGChatWidget/ChatSDK.js` class structure with singleton pattern
- [x] T005 Implement `sessionId` management in `ChatSDK.js` (generate UUIDv4 on first load, persist to localStorage)
- [x] T006 Implement `sendMessage` method in `ChatSDK.js` to POST to `/api/query` adhering to `RAGQueryRequest` contract

## Phase 3: User Story 1 - Basic RAG Q&A (Priority: P1)
*Implement the chat UI and connect it to the SDK.*

**Goal**: Users can open chat, type message, and see response.
**Independent Test**: Open widget, send "Hello", verify response in UI.

- [x] T007 [US1] Create `frontend/src/theme/RAGChatWidget/index.js` with basic React component structure (ChatWidget)
- [x] T008 [US1] Implement `useState` hooks in `index.js` for `messages`, `isLoading`, and `isOpen`
- [x] T009 [US1] Implement UI structure: Trigger button (floating), Chat Window, Message List, Input Area in `index.js`
- [x] T010 [US1] Style the component using `styles.module.css` (floating button, chat window layout, message bubbles)
- [x] T011 [US1] Integrate `ChatSDK.sendMessage` in `index.js` handleSubmit function
- [x] T012 [US1] Render `evaluation_scores` (faithfulness/relevance) in assistant message bubbles
- [x] T013 [US1] Mount `<ChatWidget />` inside `frontend/src/theme/Root.js` to make it global

## Phase 4: User Story 2 - Context-Aware Querying (Priority: P2)
*Implement text selection capture and context injection.*

**Goal**: Highlighted text is sent as `selected_context`.
**Independent Test**: Highlight text, send query, verify `selected_context` in network payload.

- [x] T014 [US2] Add `mouseup` event listener in `ChatSDK.js` (or `index.js`) to capture `window.getSelection().toString()`
- [x] T015 [US2] Update `ChatSDK.sendMessage` signature to accept `selectedContext` or pull from internal state
- [x] T016 [US2] Update `index.js` to pass captured text selection to `ChatSDK.sendMessage`
- [x] T017 [US2] Visual feedback: Add a small indicator in the Chat UI when text is currently selected (e.g., "Context selected")


## Phase 6: T067 Implement Floating 'Ask' Popup for Text Selection

**Goal**: Implement a persistent, floating "Ask" button that appears near user-selected text, offering a direct UX flow for context-based questioning.
**Independent Test**: Selecting text on the Docusaurus page should make a floating "Ask" button appear. Clicking it should open the chat, send the query with context, and display the response.

- [x] T067.1 Modify RAGChatWidget to manage the state and screen coordinates (x, y) of a floating 'Ask' button element.
- [x] T067.2 Implement improved logic in the mouseup event listener to calculate the position of the selected text and show the floating button only when a non-empty selection is made.
- [x] T067.3 Implement the click handler for the 'Ask' button. This handler must first open the chatbot widget, then pass the captured `selected_context` to the ChatSDK, and immediately trigger the sending of the message to the backend.
- [x] T067.4 Add a visual cue in the chatbot's input field once a selection is captured, allowing the user to type a question or hit send directly.

## Dependencies

1. **T003** (Root) enables global persistence for **T013**.
2. **T006** (SDK Logic) is required for **T011** (UI Integration).
3. **T014** (Selection Capture) is required for **T016** (Context Integration).

## Implementation Strategy

1. **Skeleton First**: Build the SDK class (T004-T006) and empty UI component (T001-T003).
2. **Basic Chat**: Implement the UI and connect to SDK (T007-T013).
3. **Context Feature**: Add the selection listener and payload update (T014-T017).
4. **Polish**: Styles and error handling (T010, T018).
