# Data Model: RAG Chatbot Runtime

## Entities

### ChatRequest
Represents the message payload sent from the frontend to the backend.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| message | string | Yes | The user's input text. |
| context | string | No | Optional selected text context (highlight-to-query). |

### ChatResponse
Represents the AI's response returned to the frontend.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| response | string | Yes | The generated answer from the agent. |
| status | string | Yes | "success" or "error". |

## Validation Rules
- `message` must not be empty.
- `context` is optional but helpful for specific queries.
