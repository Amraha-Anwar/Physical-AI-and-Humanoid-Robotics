---
id: 002
title: plan api model integration
stage: plan
date: 2025-12-07
surface: agent
model: gemini-2.0-flash-exp
feature: api-data-models
branch: 001-api-data-models
user: user
command: /sp.plan
labels: ["plan", "api", "refactor"]
links:
  spec: specs/001-api-data-models/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-api-data-models/plan.md
 - specs/001-api-data-models/research.md
 - specs/001-api-data-models/data-model.md
 - specs/001-api-data-models/contracts/api.yaml
 - specs/001-api-data-models/quickstart.md
tests:
 - none
---

## Prompt

/sp.plan T051 Plan API Model Integration

Goal: Create an architectural plan to implement the defined RAGQueryRequest and RAGQueryResponse Pydantic models and integrate them into the existing API endpoint and Query Service logic.

Constraint: Adhere strictly to the safety protocol: the implementation must be purely additive and must not break any existing feature (such as chat history or evaluation scoring). The integration must correctly handle the optional `selected_context` field.

Plan Steps:
1. **Model Definition:** Implement the `RAGQueryRequest` and `RAGQueryResponse` Pydantic models in `backend/api/models.py` as defined in the specification.
2. **API Endpoint Refactor:** Update the `@router.post("/query")` endpoint in `backend/api/query.py` to:
    - Use the new `RAGQueryRequest` model for the incoming request body.
    - Annotate the return type as `RAGQueryResponse`.
    - Pass the entire `RAGQueryRequest` object to the `QueryService.answer_question_async` method.
3. **QueryService Refactor:** Update the signature of `QueryService.answer_question_async` in `backend/query/query_service.py` to accept the `RAGQueryRequest` object. Adjust the internal logic to extract necessary data (`query_text`, `session_id`) and to handle the optional `selected_context` field when retrieving or compressing context.

## Response snapshot

Executed `/sp.plan` workflow.
1.  **Setup**: Loaded spec and constitution.
2.  **Phase 0**: Created `research.md` confirming logic to prioritize `selected_context`.
3.  **Phase 1**:
    *   Created `data-model.md` defining `RAGQueryRequest` and `RAGQueryResponse`.
    *   Created `contracts/api.yaml` (OpenAPI).
    *   Created `quickstart.md`.
4.  **Phase 2**: Created `plan.md` outlining the implementation structure.
5.  **Context**: Updated `GEMINI.md` with new plan details.

## Outcome

- ✅ Impact: Clear architectural plan and API contracts established.
- 🧪 Tests: N/A (Plan phase)
- 📁 Files: Plan, Research, Data Model, Contracts, Quickstart.
- 🔁 Next prompts: /sp.tasks to break down the plan into executable steps.
- 🧠 Reflection: The constraint to use `selected_context` exclusively (from Constitution) guided the architectural decision to bypass retrieval when present.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none
