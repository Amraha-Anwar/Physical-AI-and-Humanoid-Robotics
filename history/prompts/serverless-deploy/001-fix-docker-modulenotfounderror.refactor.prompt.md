---
id: 001
title: Fix Docker ModuleNotFoundError for Hugging Face Deployment
stage: refactor
date: 2025-12-17
surface: agent
model: gemini-1.5-pro
feature: serverless-deploy
branch: main
user: user
command: /sp.implement T107 Fix Docker ModuleNotFoundError for Hugging Face Deployment
labels: [docker, huggingface, deployment, python]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- Dockerfile
- backend/__init__.py
tests:
- python -c "import sys; sys.path.append('.'); import backend.main; print('Import successful')"
---

## Prompt

/sp.implement T107 Fix Docker ModuleNotFoundError for Hugging Face Deployment

# Goal
Resolve the 'ModuleNotFoundError: No module named backend' error during Hugging Face Spaces build by correcting the Dockerfile structure and Python pathing.

# Tasks to Execute
- T107.1 Refactor Dockerfile:
    - Set `WORKDIR /app` at the root.
    - Copy the entire project structure (`COPY . .`).
    - Ensure `requirements.txt` is installed before copying code to leverage Docker cache.
    - Update the `CMD` to run uvicorn from the root:
      `CMD ["uvicorn", "backend.main:app", "--host", "0.0.0.0", "--port", "7860"]`
- T107.2 Fix Pathing in main.py:
    - Locate `backend/main.py`.
    - Ensure imports are absolute (e.g., `from backend.dependencies import ...`) or that the system path is dynamically adjusted if uvicorn is called as a module.
- T107.3 Environment Compatibility:
    - Ensure the `--port` in the Dockerfile is set to `7860` (the Hugging Face default).
    - Add `ENV PYTHONPATH=/app` to the Dockerfile to explicitly tell Python where the 'backend' package lives.

# Verification
1. The CLI should simulate the path resolution or check that the 'backend' folder contains an '__init__.py' file (creating one if missing).
2. The CLI must confirm: "Dockerfile updated to root-level execution. PYTHONPATH set to /app."

## Response snapshot

Dockerfile updated to root-level execution. PYTHONPATH set to /app.
Command: python -c "import sys; sys.path.append('.'); import backend.main; print('Import successful')"
Directory: (root)
Output: Import successful
Error: (none)
Exit Code: 0
Signal: (none)
Background PIDs: (none)
Process Group PGID: 14376

## Outcome

- ✅ Impact: ModuleNotFoundError resolved, Dockerfile correctly configured for Hugging Face deployment.
- 🧪 Tests: Python import test passed.
- 📁 Files: New Dockerfile and backend/__init__.py created.
- 🔁 Next prompts: null
- 🧠 Reflection: Successfully identified missing Dockerfile and __init__.py, and configured the environment to correctly find the backend module.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
