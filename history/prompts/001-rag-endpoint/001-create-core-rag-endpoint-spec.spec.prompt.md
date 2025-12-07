---
id: 001
title: Create Core RAG Endpoint Spec
stage: spec
date: 2025-12-07
surface: agent
model: gemini-1.5-pro
feature: 001-rag-endpoint
branch: 001-rag-endpoint
user: user
command: /sp.specify

labels: ["spec", "rag-endpoint"]
links:
  spec: specs/001-rag-endpoint/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-endpoint/spec.md
  - specs/001-rag-endpoint/checklists/requirements.md
tests:
  - null
---

## Prompt

P2.3 Retrieval & Agent Logic: Core RAG FastAPI Endpoint Target module: P2.3 Retrieval & Agent Logic Focus: Implement the main FastAPI endpoint that orchestrates the Retrieval-Augmented Generation (RAG) process, including vector search in Qdrant, context lookup in Neon, prompt construction, and final response generation using the OpenAI Agents/ChatKit SDKs. Success criteria: - **Core RAG Endpoint**: A new FastAPI endpoint (`/api/rag/query`) accepts a user query and returns a contextual answer. - **Retrieval**: The system successfully uses the input query to generate an embedding and performs a fast **vector similarity search** against the Qdrant `book_vectors` collection, retrieving the top N most relevant chunk IDs. - **Context Lookup**: The retrieved chunk IDs are used to fetch the full text and metadata from the **Neon Postgres** `rag_metadata` table. - **Agent/Prompt Construction**: The full retrieved text context is combined with the user query into a single, comprehensive prompt template, instructing the OpenAI model to answer *only* based on the provided context. - **Highlight-to-Query Feature**: The endpoint can handle an optional `context_snippet` parameter, which is prioritized over the standard RAG search if provided, forcing the LLM to answer based on that snippet alone. - **Tool Adherence**: The final generation step uses the mandated **OpenAI Agents/ChatKit SDKs**. Constraints: - **Endpoint Structure**: The endpoint must be part of the existing FastAPI application in the `backend/` directory. - **Dependency Use**: Must rely on the existing database client dependencies (`get_qdrant_client`, `get_neon_db`) defined in P1.3. - **OpenAI Tooling**: Must use the **OpenAI Python SDK** for both query embedding and final response generation (mimicking Agent/ChatKit functionality). - **Retrieval Limit**: The search must retrieve a controlled number of chunks (e.g., N=5) to manage LLM context window size. Not building: - The Docusaurus frontend or user interface (Phase 3). - User authentication beyond basic API security (P1.2). - Any data ingestion logic (completed in P2.2).

## Response snapshot

The specification for "P2.3 Retrieval & Agent Logic: Core RAG FastAPI Endpoint" has been drafted and validated. It explicitly incorporates technical details and a user-focused approach, which aligns with the project's constitutional decision to prioritize specific technology integration over strict technology-agnosticism for this feature. The associated checklist has been updated to reflect these accepted deviations.

## Outcome

- ✅ Impact: A detailed feature specification for the core RAG FastAPI endpoint has been created, addressing all user requirements and documenting accepted deviations from general guidelines.
- 🧪 Tests: No automated tests were run during specification creation.
- 📁 Files: `specs/001-rag-endpoint/spec.md` and `specs/001-rag-endpoint/checklists/requirements.md` were created/modified.
- 🔁 Next prompts: The feature specification is ready for the planning phase.
- 🧠 Reflection: Successfully applied the clarified constitutional decision regarding technology-agnosticism, generating a specification that is both aligned with project mandates and ready for the next development stage.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
