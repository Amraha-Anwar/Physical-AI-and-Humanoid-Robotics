# Architectural Plan: Chatkit UI + Backend Fix

**Feature**: Chatkit UI + Backend Fix
**Status**: Draft
**Spec**: [specs/006-chatkit-ui-backend-fix/spec.md](../spec.md)

## 1. High-Level Architecture Overview

The objective is to finalize the frontend chat widget by applying the project's custom styling (`chatbot.css`) to the new `ChatkitWidget` component and ensuring the backend integration is robust.

We will adapt `chatbot.css` into `ChatkitWidget.module.css` to style the widget's container, toggle button, and window. We will also add CSS overrides to force the `@chatscope/chat-ui-kit-react` inner components (messages, input) to match the visual theme defined in `chatbot.css` (e.g., specific colors for user/assistant messages).

The `ChatkitWidget.js` logic will be verified to ensure it correctly constructs the POST request to FastAPI.

## 2. Integration & Interaction Flow

### Frontend Styling
1.  **Container**: The floating button and main window container will use `chatbot.css` classes (`chatWidgetContainer`, `chatTrigger`, `chatWindow`).
2.  **Inner UI**: The Chatkit components (`MessageList`, `Message`, `MessageInput`) will be styled via CSS overrides in the module file to match the `chatbot.css` aesthetic (colors, fonts).

### Backend Integration
1.  **Send**: `ChatkitWidget` captures input -> `fetch('POST /api/query')`.
2.  **Payload**: `{ "query_text": "...", "session_id": "..." }`.
3.  **Response**: Frontend expects `{ "answer": "..." }`.
4.  **Display**: Answer is appended to the message list.

## 3. Implementation Details

### File Impacts (Modification)

1.  **`frontend/src/components/ChatkitWidget.module.css`** (MODIFY)
    -   Replace content with adapted `chatbot.css`.
    -   Add specific overrides for `.cs-message--outgoing`, `.cs-message--incoming`, `.cs-message-input` to match the custom look (using `!important` or high specificity where needed to override library styles).

2.  **`frontend/src/components/ChatkitWidget.js`** (MODIFY)
    -   Update class names to match `chatbot.css` (e.g., `chatToggleButton` -> `chatTrigger`).
    -   Verify `fetch` logic (already correct, but will double-check error handling).

3.  **`frontend/src/components/chatbot.css`** (REFERENCE/DELETE)
    -   This file serves as the source of truth for styles but will be consolidated into the module CSS.

### Configuration
-   No changes to `docusaurus.config.js`.

## 4. Risk Assessment

-   **Blast Radius**: **LOW**. Changes are strictly within the Chatkit component and its styles.
-   **CSS Conflicts**:
    -   *Risk*: Chatkit library styles might be stubborn.
    -   *Mitigation*: Use specific CSS module selectors and variable overrides (Chatkit uses SCSS variables usually, but CSS overrides work).

## 5. Verification Plan

1.  **Visual Test**:
    -   Button matches `chatbot.css` (gradient, shadow).
    -   Window background matches `chatbot.css` (blur, dark mode).
    -   User messages use the primary color gradient.
2.  **Functional Test**:
    -   Send message -> Verify backend response.
    -   Verify session persistence (reloading page keeps session ID).
