# Implementation Tasks: Agent-based RAG Orchestration

**Feature**: Agent-based RAG Orchestration
**Status**: Draft
**Spec**: [specs/004-agent-rag-orchestration/spec.md](../spec.md)
**Plan**: [specs/004-agent-rag-orchestration/plan.md](../plan.md)

## Task List

### Task 1: Environment & Dependency Validation
- **Description**: Verify that the `openai` SDK is available in the environment and that the `GEMINI_API_KEY` is accessible. Create a small verification script (temporary) to confirm the SDK can communicate with the Gemini endpoint using the key.
- **Files Affected**: None (temporary script only).
- **Explicit Non-Effects**: No changes to `backend/requirements.txt` (assuming `openai` is standard, if not, minimal add). No changes to existing env vars.
- **Acceptance Criteria**:
    - `openai` import works.
    - `GEMINI_API_KEY` is loaded.
    - A test call to the LLM returns a 200 OK response.

### Task 2: Create Agent Definitions Module
- **Description**: Create the new file `backend/query/agents.py`. Define the `OrchestratorAgent`, `RetrievalTool` (skeleton), and `ReasoningAgent` classes/functions using the OpenAI Agents SDK structure. Configure the `openai.Client` within this module to use `GEMINI_API_KEY` and the Gemini base URL.
- **Files Affected**: `backend/query/agents.py` (NEW).
- **Explicit Non-Effects**: No changes to `backend/query/query_service.py` yet.
- **Acceptance Criteria**:
    - `backend/query/agents.py` exists.
    - `openai.Client` is configured for Gemini.
    - `OrchestratorAgent` class is defined.
    - `RetrievalTool` class/function signature is defined.
    - `ReasoningAgent` class is defined.

### Task 3: Extract Retrieval Logic for Tool Wrapping
- **Description**: In `backend/query/query_service.py`, identify the existing logic that queries Qdrant and fetches from Postgres. Refactor this specific logic into a private method `_execute_retrieval(self, query: str)` within `QueryService` *without changing its behavior*. This allows the `RetrievalTool` to call it cleanly.
- **Files Affected**: `backend/query/query_service.py`.
- **Explicit Non-Effects**: No changes to the actual Qdrant/Postgres query logic (just moving it to a method). No changes to `answer_question_async` signature yet.
- **Acceptance Criteria**:
    - `_execute_retrieval` method exists in `QueryService`.
    - `answer_question_async` still works (temporarily calling the new method).

### Task 4: Implement Retrieval Tool & Reasoning Logic
- **Description**: In `backend/query/agents.py`, implement the `RetrievalTool` to accept a `QueryService` instance (or callback) and call the `_execute_retrieval` method. Implement the `ReasoningAgent`'s system prompt to take context and answer grounded questions.
- **Files Affected**: `backend/query/agents.py`.
- **Explicit Non-Effects**: No changes to `query_service.py`.
- **Acceptance Criteria**:
    - `RetrievalTool` correctly invokes the passed retrieval function.
    - `ReasoningAgent` is configured with a strict "grounded answer only" system prompt.

### Task 5: Implement Agent Orchestration Flow
- **Description**: In `backend/query/agents.py`, implement the `create_rag_swarm` (or equivalent orchestration function) that:
    1. Instantiates the Orchestrator Agent.
    2. Equips it with the `RetrievalTool`.
    3. Defines the handoff/flow to the `ReasoningAgent`.
    4. Returns a runner/executor object.
- **Files Affected**: `backend/query/agents.py`.
- **Explicit Non-Effects**: No changes to existing business logic.
- **Acceptance Criteria**:
    - `create_rag_swarm` returns a callable object that takes a user query and returns a final string.

### Task 6: Integrate Agents into Query Service
- **Description**: Modify `backend/query/query_service.py`'s `answer_question_async` method to:
    1. Instantiate the agent swarm using `create_rag_swarm`.
    2. Pass `self._execute_retrieval` to the swarm factory.
    3. Execute the swarm with the user's query.
    4. Return the result.
    *Crucial*: Remove the old manual chain (embedding -> Qdrant -> LLM -> LLM) *only after* the agent flow is wired.
- **Files Affected**: `backend/query/query_service.py`.
- **Explicit Non-Effects**: No changes to `backend/api/query.py`. No changes to frontend.
- **Acceptance Criteria**:
    - `answer_question_async` now uses the Agent Runner.
    - Old imperative logic is removed/commented out.

### Task 7: Validation & Regression Testing
- **Description**: Run the existing `test_query.py` (or manually test via API) to ensure `POST /api/query` still returns valid answers from the vector store.
- **Files Affected**: None (running tests).
- **Explicit Non-Effects**: No code changes.
- **Acceptance Criteria**:
    - RAG queries return accurate results.
    - Logs show agent invocation.
    - No 500 errors.
