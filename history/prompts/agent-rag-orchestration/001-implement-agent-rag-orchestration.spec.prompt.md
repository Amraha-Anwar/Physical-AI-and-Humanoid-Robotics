---
id: 001
title: implement-agent-rag-orchestration
stage: spec
date: 2025-12-14
surface: agent
model: gemini-2.0-flash
feature: agent-rag-orchestration
branch: 004-agent-rag-orchestration
user: User
command: /sp.specify
labels: ["rag", "agents", "openai-sdk", "spec"]
links:
  spec: specs/004-agent-rag-orchestration/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/004-agent-rag-orchestration/spec.md
 - specs/004-agent-rag-orchestration/checklists/requirements.md
tests:
 - none
---

## Prompt

objective: Implement a compliant Agent-based RAG orchestration layer using the OFFICIAL OpenAI Agents SDK (Gemini-compatible configuration), WITHOUT altering any existing UI, layout, routes, schemas, APIs, vector logic, database logic, or frontend behavior. non_goals: - DO NOT redesign UI or Chat UI components - DO NOT replace or refactor existing FastAPI endpoints - DO NOT remove or modify Qdrant integration - DO NOT remove or modify Neon/Postgres integration - DO NOT change embeddings, chunking, or retrieval logic - DO NOT introduce custom SDKs (e.g. chatsSDK.js) - DO NOT invent agent patterns or abstractions - DO NOT simulate Agents without SDK usage - DO NOT modify Context7 MCP server configuration - DO NOT break backward compatibility mandatory_corrections: - Replace any custom chatbot SDK implementation with: - OpenAI Agents SDK ONLY - Official documented agent creation patterns - Ensure agent orchestration explicitly exists - Ensure the RAG flow is agent-driven, not function-driven - Ensure Gemini API key usage follows official SDK configuration - Ensure agents consume retrieved Qdrant context properly required_tech_stack_enforcement: agents_layer: sdk: OpenAI Agents SDK (official) configuration: api_key: GEMINI_API_KEY source: Official SDK documentation ONLY inference_mode: agent_orchestration agent_types_required: - RAG Orchestrator Agent - Retrieval Agent (Qdrant) - Reasoning/Response Agent vector_store: provider: Qdrant status: MUST_REMAIN_UNCHANGED database: provider: Neon (Postgres) status: MUST_REMAIN_UNCHANGED backend: framework: FastAPI status: MUST_REMAIN_UNCHANGED mcp: provider: Context7 MCP Server status: MUST_REMAIN_CONNECTED agent_design_spec: orchestrator_agent: responsibility: - Receive user query - Delegate retrieval to Retrieval Agent - Provide retrieved context to Reasoning Agent - Return final answer constraints: - No business logic duplication - No UI coupling retrieval_agent: responsibility: - Query Qdrant using existing logic - Return ranked documents only constraints: - Must reuse existing vector search code - No embedding changes reasoning_agent: responsibility: - Generate final response using retrieved context constraints: - No hallucination - Context-grounded answers only implementation_rules: - Agents MUST be instantiated using official SDK classes/functions - Agent invocation MUST be explicit (no hidden wrappers) - No inline LLM calls outside Agents SDK - No hardcoded prompts outside agent definitions - All existing files remain untouched unless explicitly required - Any new files MUST be minimal and isolated validation_criteria: - RAG works exactly as before - UI behavior is identical - Agents are clearly present in code - chatsSDK.js or equivalent is removed or unused - OpenAI Agents SDK is verifiably used - Gemini API key is used via official SDK config - Context7 MCP server remains functional failure_conditions: - Any UI/layout change - Any custom SDK invention - Any undocumented agent pattern - Any replacement of working RAG logic - Any deviation from official documentation output_expectations: - Minimal diff - Agent code only - Clear agent orchestration - Fully spec-compliant implementation

## Response snapshot

Created feature branch 004-agent-rag-orchestration, wrote specification, and validated it against the checklist.

## Outcome

- ✅ Impact: Created feature spec for Agent-based RAG.
- 🧪 Tests: None run.
- 📁 Files: Created spec and checklist.
- 🔁 Next prompts: /sp.clarify or /sp.plan
- 🧠 Reflection: Followed strict constraints for OpenAI SDK usage.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
