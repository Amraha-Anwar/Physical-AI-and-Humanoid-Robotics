id: 005
title: Implement Async Client Fix
stage: implement
date: 2025-12-17
surface: agent
model: gemini-2.0-flash-exp
feature: fix-rag-runtime
branch: 007-fix-rag-runtime
user: user
command: /sp.implement
labels: [implement, bugfix, async, openai]
links:
  spec: specs/007-fix-rag-runtime/spec.md
  plan: specs/007-fix-rag-runtime/plan.md
  tasks: specs/007-fix-rag-runtime/tasks.md
  phr: history/prompts/fix-rag-runtime/005-implement-async-fix.implement.prompt.md
files:
  - backend/query/agents.py
  - backend/query/query_service.py
  - backend/api/query.py
tests: []
---
/sp.implement T090 Fix Asynchronous Client Conflict in Agent SDK

RESPONSE_TEXT
Implemented T090 tasks to fix `TypeError: object ChatCompletion can't be used in 'await' expression`.
- **Async Switch**: Updated `backend/query/agents.py` to use `AsyncOpenAI`, `async def run_rag_pipeline`, and `await Runner.run`.
- **Async Callback**: Updated `backend/query/query_service.py` to pass an async retrieval function and await `run_rag_pipeline` directly, removing `asyncio.to_thread`.
- **Cleanup**: Removed diagnostic logs from `backend/api/query.py`.
Ready for verification.
