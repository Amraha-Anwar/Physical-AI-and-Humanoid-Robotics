---
id: 003
title: generate implementation tasks for api model integration
stage: tasks
date: 2025-12-07
surface: agent
model: gemini-2.0-flash-exp
feature: api-data-models
branch: 001-api-data-models
user: user
command: /sp.tasks
labels: ["tasks", "planning"]
links:
  spec: specs/001-api-data-models/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-api-data-models/tasks.md
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

Generated `specs/001-api-data-models/tasks.md` containing 9 actionable tasks organized into Setup, User Story 1, and Polish phases.
Tasks strictly follow the checklist format and cover model creation, service refactoring, endpoint updates, and comprehensive testing (unit + integration + regression).

## Outcome

- ✅ Impact: Clear, actionable implementation roadmap ready for execution.
- 🧪 Tests: N/A (Tasks phase)
- 📁 Files: specs/001-api-data-models/tasks.md
- 🔁 Next prompts: /sp.checklists to create the pre-implementation checklist.
- 🧠 Reflection: The dependency chain is simple: Models -> Service -> API -> Tests.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none
