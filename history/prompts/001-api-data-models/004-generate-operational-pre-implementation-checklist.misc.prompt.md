---
id: 004
title: generate operational pre implementation checklist
stage: misc
date: 2025-12-07
surface: agent
model: gemini-2.0-flash-exp
feature: api-data-models
branch: 001-api-data-models
user: user
command: /sp.checklist
labels: ["checklist", "pre-implementation", "operational"]
links:
  spec: specs/001-api-data-models/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-api-data-models/checklists/pre-implementation.md
tests:
 - none
---

## Prompt

/sp.checklist T053 Generate Operational Pre-Implementation Checklist

Goal: Generate a comprehensive, operational checklist to verify the full readiness of the system (local codebase and external services) before beginning the implementation tasks for integrating the new API data models.

Constraint: The checklist must strictly adhere to the following operational criteria:
1.  **Nature:** Operational Verification (Checks if the system is ready to run).
2.  **Scope:** Full Stack (Includes checks for external services like Neon, Qdrant, and the LLM API).
3.  **Safety:** Include items to verify the baseline functional state of existing features (Regression Check).

## Response snapshot

Generated `specs/001-api-data-models/checklists/pre-implementation.md` with 9 checks covering:
- Local environment files and dependencies.
- Connectivity to external services (Neon, Qdrant, OpenAI).
- Baseline safety checks (passing existing tests).

## Outcome

- ✅ Impact: Ensures the environment is fully operational and safe before code changes begin.
- 🧪 Tests: N/A (Checklist generation)
- 📁 Files: specs/001-api-data-models/checklists/pre-implementation.md
- 🔁 Next prompts: /sp.checklists to run the checklist validation (manual step usually, or automated if tools allow).
- 🧠 Reflection: Explicitly separating "Requirements" vs "Operational" checklists is crucial for avoiding confusion.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none
