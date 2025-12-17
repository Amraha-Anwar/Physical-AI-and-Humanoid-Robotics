# Architectural Plan: Agent-based RAG Orchestration

**Feature**: Agent-based RAG Orchestration
**Status**: Draft
**Spec**: [specs/004-agent-rag-orchestration/spec.md](../spec.md)

## 1. High-Level Architecture Overview

The objective is to refactor the internal execution flow of the RAG system to use the **OpenAI Agents SDK** for orchestration, replacing the imperative function chaining currently in `backend/query/query_service.py`.

The external API contract (`POST /api/query`) will remain identical. The `QueryService` will act as the entry point, but instead of manually calling Qdrant and LLMs, it will instantiate and run an **Orchestrator Agent**.

### Current Flow (Imperative)
`API -> QueryService -> QdrantClient -> Postgres (Context Fetch) -> LLM (Compress) -> LLM (Answer) -> Response`

### New Flow (Agent-Driven)
`API -> QueryService -> [Orchestrator Agent] -> [Retrieval Agent (Tool)] -> [Reasoning Agent] -> Response`

All agents will be powered by **Gemini models** accessed via the OpenAI Agents SDK compatibility layer, using `GEMINI_API_KEY`.

## 2. Agent Responsibility & Interaction Flow

We will define three primary logical components, mapped to SDK constructs.

### A. Orchestrator Agent (The "Manager")
- **Role**: Entry point for the user query.
- **Responsibility**: 
  1. Receive the user's question.
  2. Decide to call the **Retrieval Agent** to get context.
  3. Pass the question + retrieved context to the **Reasoning Agent**.
  4. Return the final answer to `QueryService`.
- **SDK Mapping**: A standard `Agent` configured with instructions to coordinate the RAG process.

### B. Retrieval Agent (The "Tool User")
- **Role**: Interface to the Vector Store.
- **Responsibility**: 
  1. Receive a search query.
  2. Execute the existing logic: `qdrant_client.query_points` -> Fetch chunks from Postgres.
  3. Return the raw text chunks.
- **SDK Mapping**: This will effectively be a **Tool** (function) provided to the Orchestrator, rather than a standalone conversational agent, to strictly follow the SDK's pattern for deterministic tool usage. The `QueryService`'s existing retrieval logic will be wrapped in a function `retrieve_context(query: str)` and exposed as a tool.

### C. Reasoning Agent (The "Synthesizer")
- **Role**: Generator of the final answer.
- **Responsibility**: 
  1. Receive `User Question` + `Retrieved Context`.
  2. Generate a grounded response using *only* the provided context.
- **SDK Mapping**: A standard `Agent` with a strict system prompt enforcing groundedness and hallucination prevention.

### Interaction Sequence
1. **User** sends POST request to `/api/query`.
2. **FastAPI** calls `QueryService.answer_question_async`.
3. **QueryService** instantiates the `Orchestrator Agent`.
4. **Orchestrator** receives the user message.
5. **Orchestrator** calls the `retrieve_context` tool (wrapping Qdrant+Postgres logic).
6. **Tool** executes existing retrieval code and returns text chunks.
7. **Orchestrator** passes context + question to **Reasoning Agent** (or handles generation itself if configured as a single powerful agent with tools - *Decision: We will use a dedicated Reasoning Agent for separation of concerns as requested*).
8. **Reasoning Agent** generates the response.
9. **QueryService** extracts the final text and returns it to the API.

## 3. Implementation Details

### Dependencies
- **OpenAI Agents SDK**: `openai` (official package).
- **Gemini Adapter**: Configuration of the `openai.Client` to point to Gemini's endpoint (if applicable) or usage of Google's official OpenAI-compatible transport if available. *Correction per Spec*: Use "Gemini-compatible configuration" of the OpenAI SDK.

### File Impacts (Additive Only)

We will create a new module to house the agent definitions to avoid cluttering `query_service.py` and ensure isolation.

1.  **`backend/query/agents.py`** (NEW)
    -   `create_rag_swarm()`: Factory function to setup the agents.
    -   `OrchestratorAgent` definition.
    -   `RetrievalTool` definition (wraps `QueryService` logic).
    -   `ReasoningAgent` definition.

2.  **`backend/query/query_service.py`** (MODIFY)
    -   Import `create_rag_swarm` / Agent definitions.
    -   Update `answer_question_async` to use the agent runner instead of the manual chain.
    -   *Crucial*: Extract the retrieval logic (Qdrant + Postgres fetch) into a helper method `_execute_retrieval` that can be wrapped by the `RetrievalTool`.

### Configuration
-   **API Key**: `os.getenv("GEMINI_API_KEY")` used to initialize the `openai.Client`.
-   **Base URL**: Set to Gemini's OpenAI-compatible endpoint (e.g., `https://generativelanguage.googleapis.com/v1beta/openai/`).

## 4. Risk Assessment

-   **Blast Radius**: **LOW**. 
    -   The changes are confined to the internal implementation of `QueryService`.
    -   The API contract is unchanged.
    -   Existing database interactions are preserved, just moved into a tool wrapper.
-   **Compatibility**: 
    -   *Risk*: OpenAI SDK compatibility with Gemini endpoints might have subtle differences in function calling formats.
    -   *Mitigation*: We will use standard tool definitions and simple prompts to minimize complexity.
-   **Performance**:
    -   *Risk*: Agent loops can introduce latency compared to linear code.
    -   *Mitigation*: Orchestrator instructions will be explicit to avoid unnecessary "thought loops".

## 5. Verification Plan

1.  **Unit Test**: Mock the `openai.Client` and verify that `QueryService` instantiates the agents and calls them.
2.  **Integration Test**: Run a query against the live dev environment (using the existing `test_query.py` or similar) and verify a valid response is returned from the Qdrant content.
3.  **Logs Inspection**: Verify logs show "Orchestrator starting", "Tool retrieval called", "Reasoning responding".
