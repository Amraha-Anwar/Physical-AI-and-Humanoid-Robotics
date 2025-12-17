# Feature Specification: Agent-based RAG Orchestration

**Feature Branch**: `004-agent-rag-orchestration`
**Created**: 2025-12-14
**Status**: Draft
**Input**: User description: Implement a compliant Agent-based RAG orchestration layer using the OFFICIAL OpenAI Agents SDK (Gemini-compatible configuration).

## User Scenarios & Testing *(mandatory)*

### User Story 1 - RAG Query Execution (Priority: P1)

As a user, I want to ask a question in the existing chat interface and receive a grounded response, so that I can get accurate information from the knowledge base without noticing the underlying architectural change.

**Why this priority**: This is the core functionality. The switch to Agents SDK must be seamless and maintain existing RAG quality.

**Independent Test**: Can be tested by sending a query to the chat endpoint and verifying the response is generated using the agent orchestration (via logs) and contains correct information.

**Acceptance Scenarios**:

1. **Given** the system is running with the new Agent Orchestration, **When** I send a query "What is a humanoid robot?", **Then** the Orchestrator Agent receives the query, delegates to the Retrieval Agent to fetch Qdrant data, passes it to the Reasoning Agent, and returns a coherent answer.
2. **Given** the RAG flow, **When** the Retrieval Agent is called, **Then** it MUST use the existing Qdrant vector store and logic, not a new implementation.
3. **Given** the agent configuration, **When** agents are initialized, **Then** they MUST utilize the `GEMINI_API_KEY` via the official OpenAI Agents SDK configuration.

---

### Edge Cases

- **Service Unavailability**: What happens if Qdrant or the LLM is down? The Agents SDK should handle or bubble up errors gracefully, preserving existing error handling behavior where possible.
- **Empty Retrieval**: What happens if Qdrant returns no results? The Reasoning Agent should handle "no context" scenarios appropriately (e.g., "I don't have enough information").

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST implement an **Orchestrator Agent** responsible for receiving user queries and managing the flow.
- **FR-002**: System MUST implement a **Retrieval Agent** that wraps the *existing* Qdrant query logic (no new vector logic).
- **FR-003**: System MUST implement a **Reasoning Agent** that takes user query + retrieved context and generates the final response.
- **FR-004**: All agents MUST be instantiated and managed using the **Official OpenAI Agents SDK** classes and functions.
- **FR-005**: The `GEMINI_API_KEY` MUST be used for LLM inference, configured strictly according to the official SDK documentation for Gemini compatibility.
- **FR-006**: The implementation MUST NOT introduce any custom SDK wrappers (e.g., `chatsSDK.js`) or undocumented agent patterns.
- **FR-007**: The **Context7 MCP Server** connection MUST remain active and functional within the agent environment if applicable (or at least not be broken).
- **FR-008**: The implementation MUST NOT modify existing UI, routes, schemas, or database configurations (Neon/Postgres, Qdrant setup).
- **FR-009**: Agent invocation MUST be explicit in the backend code (FastAPI), replacing any previous direct function calls for RAG generation.

### Key Entities

- **Orchestrator Agent**: The central controller using the SDK.
- **Retrieval Tool**: A tool definition (compatible with the SDK) that wraps the existing Qdrant search function.
- **Context**: The data retrieved from Qdrant and passed between agents.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: RAG queries return results with equivalent or better relevance/accuracy compared to the previous implementation.
- **SC-002**: 100% of the RAG orchestration logic is handled via the OpenAI Agents SDK; no legacy orchestration code remains in the active path.
- **SC-003**: No changes are detected in the frontend UI or API contract (endpoints remain the same).
- **SC-004**: The system successfully initializes with `GEMINI_API_KEY` without errors.
- **SC-005**: Zero custom "agent framework" or "chatbot SDK" files exist in the codebase; only official library usage is present.

### Assumptions

- The OpenAI Agents SDK is compatible with the current Python environment.
- The `GEMINI_API_KEY` has sufficient permissions for the models used by the SDK.
- Existing Qdrant and Neon connections are healthy.