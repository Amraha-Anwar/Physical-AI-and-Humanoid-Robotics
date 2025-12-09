# Quickstart: Testing the Chat Widget

## Prerequisites
1. Backend running: `cd backend && uvicorn main:app --reload` (Port 8000)
2. Frontend running: `cd frontend && npm start` (Port 3000)

## Manual Verification

### 1. Widget Visibility
- Open `http://localhost:3000`.
- Verify a chat bubble icon appears in the bottom right corner.
- Click to open the widget.

### 2. Sending a Message
- Type "Hello" in the input.
- Press Enter.
- Verify "Hello" appears in the chat history.
- Verify a response is received from the bot (if backend is connected).

### 3. Context Selection
- Highlight some text on the documentation page.
- Open the chat widget.
- Type "Explain this".
- Verify (via Network tab in DevTools) that the POST request to `/api/query` includes the highlighted text in `selected_context`.
