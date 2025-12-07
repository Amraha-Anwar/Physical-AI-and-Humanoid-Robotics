# Quickstart: API Data Models

## Usage

### 1. Request with Standard Retrieval
Send a POST request to `/query` with `query_text` and `session_id`.

```bash
curl -X POST "http://localhost:8000/query" \
     -H "Content-Type: application/json" \
     -d '{
           "query_text": "What is URDF?",
           "session_id": "12345-abcde"
         }'
```

### 2. Request with Selected Context (Highlight)
Include `selected_context` to force the bot to answer based *only* on that text.

```bash
curl -X POST "http://localhost:8000/query" \
     -H "Content-Type: application/json" \
     -d '{
           "query_text": "Explain this code.",
           "session_id": "12345-abcde",
           "selected_context": "class Robot: def __init__(self): pass"
         }'
```

### 3. Response Format
The API returns a JSON object matching `RAGQueryResponse`.

```json
{
  "answer": "URDF stands for Unified Robot Description Format...",
  "session_id": "12345-abcde",
  "evaluation_scores": {
    "relevance": 0.95,
    "faithfulness": 0.98
  }
}
```

