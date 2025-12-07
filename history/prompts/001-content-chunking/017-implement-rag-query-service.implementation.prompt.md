---
id: 017
title: implement-rag-query-service
stage: implementation
date: 2025-12-07
surface: agent
model: gemini-2.0-flash-exp
feature: 001-content-chunking
branch: 001-content-chunking
user: user
command: /sp.implement
labels: ["implementation", "rag", "query-service"]
links:
  spec: specs/001-content-chunking/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/query/query_service.py
tests:
  - null
---

## Prompt

/sp.implement P3.1 Query Engine Setup: Execute Task T035

Goal: Implement the core service responsible for the Retrieval-Augmented Generation (RAG) workflow.

Execute:
- T035 Implement the RAG Query Service

Constraint:
- The code must be placed in `backend/query/query_service.py`.
- The class must be named `QueryService`.
- It must be initialized with the database clients (Qdrant, Neon) and the embedding client (OpenAI/Gemini).
- It must include an asynchronous method, `answer_question_async(query_text: str)`, that performs the following steps:
    1. **Embed the query:** Uses the EmbeddingClient to convert the user's `query_text` into a vector.
    2. **Retrieve chunks:** Uses the Qdrant client to search the `book_vectors` collection for the top relevant chunks (e.g., top 5) based on vector similarity.
    3. **Fetch context:** Uses the Neon client to retrieve the full metadata and text content for the retrieved chunks (using their IDs).
    4. **Generate answer:** Sends the original `query_text` and the retrieved context (full text) to an LLM (OpenAI/Gemini) to generate a final, grounded answer.

## Response snapshot

Implemented `QueryService` in `backend/query/query_service.py` with RAG workflow logic.

## Outcome

- ✅ Impact: Core RAG logic implemented (Embedding -> Qdrant -> Neon -> LLM).
- 🧪 Tests: None run.
- 📁 Files: Modified backend/query/query_service.py.
- 🔁 Next prompts: Continue with P3.1 tasks (API endpoint integration).
- 🧠 Reflection: Implemented the service using Gemini for both embedding and generation, keeping consistent with the project's current configuration. Used the specified clients and flow.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
