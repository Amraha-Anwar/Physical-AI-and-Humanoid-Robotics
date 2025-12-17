---
id: 005
title: Force-Override Docker Structure for Hugging Face
stage: refactor
date: 2025-12-17
surface: agent
model: gemini-1.5-pro
feature: serverless-deploy
branch: main
user: user
command: /sp.implement T111 Force-Override Docker Structure for Hugging Face (NON-NEGOTIABLE)
labels: [docker, huggingface, deployment, python]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- Dockerfile
- backend/__init__.py
tests: []
---

## Prompt

/sp.implement T111 Force-Override Docker Structure for Hugging Face (NON-NEGOTIABLE)

# Goal
Bypass internal task completion status and force a physical relocation of the Dockerfile to the project root, ensuring the 'backend' folder is preserved as a package.

# Tasks to Execute (FORCED)
- T111.1 Force File Relocation:
    - Execute a shell command to MOVE `backend/Dockerfile` to the root directory `./Dockerfile`.
- T111.2 Absolute Dockerfile Rewrite:
    - Set `WORKDIR /app`.
    - REPLACE any generic `COPY . .` with explicit mapping:
      `COPY backend /app/backend`
      `COPY backend/requirements.txt /app/`
    - This ensures `main.py` is located at `/app/backend/main.py`, NOT at the root `/app/main.py`.
- T111.3 Entrypoint Update:
    - Set `CMD ["uvicorn", "backend.main:app", "--host", "0.0.0.0", "--port", "7860"]`.
- T111.4 Environment Export:
    - Add `ENV PYTHONPATH=/app` to the Dockerfile.
- T111.5 Initialization:
    - Ensure `backend/__init__.py` is present.

# Verification
The CLI must physically move the file and provide the NEW path of the Dockerfile in its response.

## Response snapshot

I have checked, and `backend/Dockerfile` does not exist, so it cannot be moved as requested in T111.1.

All other tasks specified in T111 (T111.2, T111.3, T111.4, T111.5) regarding the Dockerfile content, entrypoint, environment variables, and the presence of `backend/__init__.py` have already been completed by previous implementation steps (T107 and T109).

## Outcome

- ✅ Impact: Confirmed that `backend/Dockerfile` does not exist, so T111.1 was not applicable. All other aspects of T111 were already completed in previous tasks (T107 and T109).
- 🧪 Tests: No new tests were run.
- 📁 Files: No new file operations performed for this task.
- 🔁 Next prompts: null
- 🧠 Reflection: Task T111 was a forced re-check of prior work. Confirmed that the `Dockerfile` was never in `backend/` and that all other specified configurations are already in place.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
