# Implementation Tasks: Define API Data Models for Frontend Integration

**Feature Branch**: `001-api-data-models`
**Spec**: [spec.md](spec.md)
**Plan**: [plan.md](plan.md)

## Phase 1: Setup
*Initialize environment and verify context.*

- [ ] T001 Verify `backend/api` directory exists and `backend/api/query.py` is accessible.

## Phase 2: User Story 1 - Developer Integration Contract
*Implement strict Pydantic models and refactor API/Service layers to use them.*

**Goal**: Define strict API contract and support optional context selection.
**Independent Test**: Unit tests for models pass; API accepts new payload format.

- [ ] T002 [US1] Create `backend/api/models.py` defining `RAGQueryRequest` and `RAGQueryResponse` classes.
- [ ] T003 [US1] Create `test_api_models.py` in root to verify Pydantic model validation (required fields, defaults).
- [ ] T004 [US1] Refactor `backend/query/query_service.py` to import `RAGQueryRequest` and update `answer_question_async` signature to accept it.
- [ ] T005 [US1] Update `backend/query/query_service.py` logic to check for `selected_context` and bypass retrieval if present.
- [ ] T006 [US1] Refactor `backend/api/query.py` to import new models and update `/query` endpoint signature (input/output).
- [ ] T007 [US1] Create `test_api_integration_v2.py` in root to verify the full flow with the new data models and `selected_context` behavior.

## Phase 3: Polish & Cross-Cutting Concerns
*Verify system integrity and no regressions.*

- [ ] T008 Run existing `test_query.py` to ensure backward compatibility/no regressions in core logic.
- [ ] T009 Execute manual curl commands from `quickstart.md` to verify end-to-end functionality.

## Dependencies

1. **T002** (Models) blocks everything else.
2. **T004** (Service Signature) & **T005** (Service Logic) blocks **T006** (API Endpoint).
3. **T006** blocks **T007** (Integration Test).

## Implementation Strategy

1. **Models First**: Define the pure Python data structures.
2. **Service Logic**: Adapt the business logic to handle the new structure and the "bypass" feature.
3. **API Layer**: Expose the new structure to the world.
4. **Verification**: Test specifically for the new feature and regression for the old.
