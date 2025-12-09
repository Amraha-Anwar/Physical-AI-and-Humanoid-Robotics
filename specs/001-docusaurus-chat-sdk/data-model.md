# Data Model: Frontend Chat State

## Entities

### Message
Represents a single message in the conversation history.

| Field | Type | Description |
|-------|------|-------------|
| `id` | `string` | Unique ID (UUIDv4) |
| `role` | `string` | 'user' or 'assistant' |
| `content` | `string` | The text content of the message |
| `timestamp` | `number` | Unix timestamp |
| `metadata` | `object` | Optional. Contains `evaluation_scores` for assistant messages. |

### ChatSession
Represents the current state of the user's conversation.

| Field | Type | Description |
|-------|------|-------------|
| `sessionId` | `string` | Persistent UUID for the backend session. |
| `messages` | `Message[]` | Ordered list of messages. |
| `isLoading` | `boolean` | True if waiting for API response. |
| `isOpen` | `boolean` | True if the chat widget is expanded. |
| `selectedContext` | `string` | Currently selected text on the page (if any). |

## State Management
- `sessionId` is persisted in `localStorage` key `rag_session_id`.
- `messages` are held in React state (ephemeral for now, per spec focus on SDK structure, but could be persisted).
