# Implementation Plan: Define API Data Models for Frontend Integration

**Branch**: `001-api-data-models` | **Date**: 2025-12-07 | **Spec**: [Link](spec.md)
**Input**: Feature specification from `/specs/001-api-data-models/spec.md`

## Summary

Implement strict Pydantic models (`RAGQueryRequest` and `RAGQueryResponse`) in `backend/api/models.py` to define the API contract. Refactor `backend/api/query.py` and `backend/query/query_service.py` to use these models and support the optional `selected_context` field, ensuring that if context is selected, it takes precedence over retrieval.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: `fastapi`, `pydantic`
**Storage**: None (Logic flow only)
**Testing**: `pytest`
**Target Platform**: Local/Serverless (FastAPI)
**Project Type**: Backend API

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **RAG Accuracy**: Maintained/Enhanced by allowing specific context selection.
- **Interactive UX**: Directly supports the highlight-to-query feature via `selected_context`.
- **Code Integrity**: Improves codebase by explicit typing and separation of concerns.

## Project Structure

### Documentation (this feature)

```text
specs/001-api-data-models/
├── plan.md              # This file
├── research.md          # Logic decisions
├── data-model.md        # Entity definitions
├── quickstart.md        # Usage guide
└── contracts/           # OpenAPI spec
    └── api.yaml
```

### Source Code (repository root)

```text
backend/
├── api/
│   ├── models.py        # NEW: Pydantic models
│   └── query.py         # MODIFIED: Endpoint signature
└── query/
    └── query_service.py # MODIFIED: Logic for selected_context
```

**Structure Decision**: Standard FastAPI structure with `models.py` for shared data contracts.

## Complexity Tracking

N/A - Standard Implementation.