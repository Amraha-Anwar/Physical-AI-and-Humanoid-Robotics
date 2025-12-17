id: 006
title: Implement RAG Grounding
stage: implement
date: 2025-12-17
surface: agent
model: gemini-2.0-flash-exp
feature: fix-rag-runtime
branch: 007-fix-rag-runtime
user: user
command: /sp.implement
labels: [implement, bugfix, rag, grounding]
links:
  spec: specs/007-fix-rag-runtime/spec.md
  plan: specs/007-fix-rag-runtime/plan.md
  tasks: specs/007-fix-rag-runtime/tasks.md
  phr: history/prompts/fix-rag-runtime/006-implement-grounding.implement.prompt.md
files:
  - backend/query/agents.py
tests: []
---
/sp.implement T091 Enforce Contextual Grounding in RAG Agent

RESPONSE_TEXT
Implemented T091 tasks to enforce strict contextual grounding.
- **Restored Integrity**: Rewrote `backend/query/agents.py` to fix corruption from previous steps.
- **Strict Prompt**: Updated `Reasoning Agent` instructions to explicitly forbid external knowledge and require "I cannot find the answer" for missing context.
- **Verification**: Code structure is now correct and async-compatible.
Ready for testing.
