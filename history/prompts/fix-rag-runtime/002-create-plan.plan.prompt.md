id: 002
title: Plan RAG Runtime Fix
stage: plan
date: 2025-12-17
surface: agent
model: gemini-2.0-flash-exp
feature: fix-rag-runtime
branch: 007-fix-rag-runtime
user: user
command: /sp.plan
labels: [plan, bugfix, startup-crash, cors]
links:
  spec: specs/007-fix-rag-runtime/spec.md
  plan: specs/007-fix-rag-runtime/plan.md
  research: specs/007-fix-rag-runtime/research.md
  phr: history/prompts/fix-rag-runtime/002-create-plan.plan.prompt.md
files:
  - specs/007-fix-rag-runtime/plan.md
  - specs/007-fix-rag-runtime/research.md
  - specs/007-fix-rag-runtime/data-model.md
  - specs/007-fix-rag-runtime/quickstart.md
  - specs/007-fix-rag-runtime/contracts/chat-api.yaml
tests: []
---
/sp.plan S013 Diagnose and Fix RAG Chatbot Runtime Failure

# Plan Overview
The plan will proceed through three surgical phases to diagnose and fix the break in the request lifecycle: 1) Network and Routing verification, 2) Dependency Injection integrity check, and 3) Agent Runner Execution validation. This will be achieved by adding minimal logging and confirming the FastAPI wiring to successfully pass the agent object to the router function.

## Phase 1: Verify Network and Routing Integrity (T087)
Goal: Ensure the request successfully reaches the FastAPI router and is not blocked by CORS or an incorrect endpoint path.

* **T087.1 CORS Configuration:** Explicitly verify that the CORS middleware is correctly applied to the FastAPI application, allowing `*` origins (or the specific frontend URL) and the necessary methods (`POST`, `OPTIONS`).
* **T087.2 Endpoint Logging:** Add a minimal `print()` or `logger.info()` statement as the very first line inside the main chat endpoint function (`/api/chat` or similar) to confirm the request handler is executed.

## Phase 2: Check Dependency Injection and Agent Instantiation (T088)
Goal: Confirm the necessary agent runner object (using OpenAI Agents SDK, Gemini via base\_url) is correctly instantiated and injected into the router function.

* **T088.1 Dependency Audit:** Examine the function signature and body of the FastAPI router function. Ensure the agent object is being injected as a `Depends()` parameter.
* **T088.2 Constructor Verification:** Check the agent/client constructor function (where the `OpenAI` client is created). Ensure all required arguments are correctly mapped: `base_url`, `api_key`, and any necessary environment variables are loaded via `python-dotenv`.
* **T088.3 Object Logging:** Add logging immediately after the agent object is injected into the router function to confirm the agent object is not `None` or an uninitialized stub.

## Phase 3: Validate Agent Invocation Flow (T089)
Goal: Verify the agent runner function is being called correctly with the chat message payload.

* **T089.1 Runner Function Call:** Verify the line of code that invokes the agent runner (e.g., `agent.run(message)`) is correct and that the payload (`message`) is successfully extracted from the request body.
* **T089.2 Final Cleanup:** Remove all diagnostic logging added in T087 and T088 to restore the clean state after the fix is verified.

## Required Commit and Verification
* All changes must be committed to the new feature branch (`007-fix-rag-runtime`).
* Verification: The backend console must show log activity when a message is sent, and the frontend must display a successful response instead of "Failed to fetch."

RESPONSE_TEXT
Generated Implementation Plan for feature `007-fix-rag-runtime`.
Artifacts created:
- `research.md`: Identified startup crash (env vars) and CORS as root causes.
- `plan.md`: Defined 3-phase surgical fix (Env/CORS, Refactor, Validate).
- `data-model.md`, `contracts/`, `quickstart.md`.
Ready for tasks generation (`/sp.tasks`).
