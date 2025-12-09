# Data Model: API Integration

## Entities

### RAGQueryRequest
Represents the input payload for the RAG Chatbot.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `query_text` | `str` | Yes | The user's natural language question. |
| `session_id` | `str` | Yes | Unique identifier for the chat session (UUID). |
| `selected_context` | `str` | No (Optional) | Text highlighted by the user to focus the answer. Defaults to `None`. |

### RAGQueryResponse
Represents the output payload from the RAG Chatbot.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `answer` | `str` | Yes | The generated answer from the LLM. |
| `session_id` | `str` | Yes | The session ID echoed back (or generated). |
| `evaluation_scores` | `Dict[str, float]` | Yes | Scores for 'relevance' and 'faithfulness'. |

## Relationships
- One `RAGQueryRequest` produces one `RAGQueryResponse`.
- `session_id` links the request/response to a persistent conversation history in the `HistoryService`.
