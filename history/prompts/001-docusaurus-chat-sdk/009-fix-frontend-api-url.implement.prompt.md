---
id: 009
title: fix-frontend-api-url
stage: implement
date: 2025-12-07
surface: agent
model: gemini-2.0-flash-exp
feature: 001-docusaurus-chat-sdk
branch: 001-docusaurus-chat-sdk
user: user
command: /sp.implement
labels: ["fix", "frontend", "api"]
links:
  spec: specs/001-docusaurus-chat-sdk/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - frontend/src/theme/RAGChatWidget/ChatSDK.js
tests:
 - none
---

## Prompt

T065 Fix Missing Database Table: Create Chat History Table

Goal: Correct the API endpoint URL in the ChatSDK to ensure requests are routed from the Docusaurus frontend (port 3000) to the FastAPI backend (port 8000), resolving the silent network failure.

Execute:
- T065 Update the fetch URL in `frontend/src/theme/RAGChatWidget/ChatSDK.js`.

Constraint:
- Change the URL from a relative path (`/api/query`) to the absolute path of the backend: `http://localhost:8000/api/query`.

## Response snapshot

Implemented T065: Updated the fetch URL in `frontend/src/theme/RAGChatWidget/ChatSDK.js` from `/api/query` to `http://localhost:8000/api/query`.

## Outcome

- ✅ Impact: Frontend API calls should now correctly reach the FastAPI backend.
- 🧪 Tests: Manual verification by submitting queries from the chat widget.
- 📁 Files: Modified `frontend/src/theme/RAGChatWidget/ChatSDK.js`.
- 🔁 Next prompts: Verify the fix.
- 🧠 Reflection: Crucial fix for inter-service communication.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): N/A
