# Implementation Tasks: Chatkit UI + Backend Fix

**Feature**: Chatkit UI + Backend Fix
**Status**: Draft
**Spec**: [specs/006-chatkit-ui-backend-fix/spec.md](../spec.md)
**Plan**: [specs/006-chatkit-ui-backend-fix/plan.md](../plan.md)

## Task List

### Task 1: Update Chatkit CSS Module
- [x] **Description**: Replace the content of `frontend/src/components/ChatkitWidget.module.css` with the styles from `frontend/src/components/chatbot.css`. Add CSS overrides for Chatkit-specific classes (e.g., `.cs-message`, `.cs-message-input`) to force them to match the project's visual theme (primary colors, dark mode backgrounds) as defined in `chatbot.css`.
- **Files Affected**: `frontend/src/components/ChatkitWidget.module.css`.
- **Explicit Non-Effects**: No changes to `ChatkitWidget.js` yet. No changes to global CSS.
- **Acceptance Criteria**:
    - `ChatkitWidget.module.css` contains all styles from `chatbot.css`.
    - Includes specific overrides for `@chatscope` classes to align colors/fonts.

### Task 2: Update Chatkit Component Structure & Classes
- [x] **Description**: Modify `frontend/src/components/ChatkitWidget.js` to use the CSS classes defined in the updated module file. Specifically, rename `chatToggleButton` to `chatTrigger`, `chatWindow` to `chatWindow` (if not already), and ensure the container uses `chatWidgetContainer`. Verify the DOM structure supports the new styles.
- **Files Affected**: `frontend/src/components/ChatkitWidget.js`.
- **Explicit Non-Effects**: No changes to backend integration logic yet.
- **Acceptance Criteria**:
    - Component uses `styles.chatTrigger`, `styles.chatWindow`, `styles.chatWidgetContainer`.
    - Rendered widget matches the look of the previous manual chatbot (floating button, dark window).

### Task 3: Verify & Tune Backend Integration
- [x] **Description**: In `frontend/src/components/ChatkitWidget.js`, review the `handleSend` function. Ensure it sends the correct payload `{ query_text, session_id }` to `/api/query` and correctly parses the response `{ answer }`. Add error handling for network failures to display a user-friendly error message in the chat.
- **Files Affected**: `frontend/src/components/ChatkitWidget.js`.
- **Explicit Non-Effects**: No changes to `backend/query/query_service.py`.
- **Acceptance Criteria**:
    - `handleSend` correctly `fetch`es from `/api/query`.
    - User messages are added to state immediately.
    - Assistant responses from backend are added to state upon receipt.
    - Error messages appear in chat on failure.

### Task 4: Final Validation
- [x] **Description**: Run the Docusaurus frontend and FastAPI backend. Open the chat widget, send a test query ("Hello"), and verify both the visual styling (colors, layout) and the functional response (agent answer).
- **Files Affected**: None (Verification step).
- **Explicit Non-Effects**: No code changes.
- **Acceptance Criteria**:
    - Chat widget opens with correct styling.
    - Message "Hello" receives an answer.
    - No console errors.
