# Research: API Data Models & Logic Update

**Feature**: Define API Data Models (T050)
**Date**: 2025-12-07

## Decisions

### 1. Pydantic Models Location
- **Decision**: Create `backend/api/models.py` to house `RAGQueryRequest` and `RAGQueryResponse`.
- **Rationale**: Centralizes data contracts, avoiding circular imports and keeping `api/query.py` clean.
- **Alternatives**: Keep in `api/query.py` (clutters route logic), or `models.py` in root (too broad).

### 2. Handling `selected_context`
- **Decision**: If `selected_context` is provided in the request, the `QueryService` will **bypass** the retrieval step (Qdrant/Neon) and use the provided text as the sole context for the LLM.
- **Rationale**: Constitution Principle II ("Interactive UX & Selection Integrity") states the chatbot must answer based *only* on the user-selected text if provided.
- **Implications**: The `answer_question_async` method signature needs to accept this optional argument.

### 3. API Contract
- **Decision**: The `/query` endpoint will strictly enforce the `RAGQueryRequest` schema.
- **Rationale**: Ensures the frontend sends all necessary data (session_id, query, optional context) in a structured way.

## Unknowns Resolved
- **Logic for Context**: Confirmed via Constitution that selected context takes precedence (exclusively).
