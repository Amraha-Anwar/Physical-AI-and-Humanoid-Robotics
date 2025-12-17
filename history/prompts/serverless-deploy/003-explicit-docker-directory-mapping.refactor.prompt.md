---
id: 003
title: Explicit Docker Directory Mapping & Path Enforcement
stage: refactor
date: 2025-12-17
surface: agent
model: gemini-1.5-pro
feature: serverless-deploy
branch: main
user: user
command: /sp.implement T109 Explicit Docker Directory Mapping & Path Enforcement
labels: [docker, huggingface, deployment, python]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- Dockerfile
tests: []
---

## Prompt

/sp.implement T109 Explicit Docker Directory Mapping & Path Enforcement

# Goal
Force the creation of the 'backend' directory structure within the Docker container to resolve the persistent ModuleNotFoundError.

# Tasks to Execute
- T109.1 Explicit Dockerfile Rewrite:
    - Set `WORKDIR /app`.
    - Do NOT use `COPY . .`. Instead, use:
      `COPY backend /app/backend`
      `COPY requirements.txt /app/`
    - This ensures the folder named 'backend' physically exists inside '/app'.
- T109.2 Environment Enforcement:
    - Add `ENV PYTHONPATH=/app` to the Dockerfile.
- T109.3 Entrypoint Precision:
    - Ensure the CMD is exactly: `CMD ["uvicorn", "backend.main:app", "--host", "0.0.0.0", "--port", "7860"]`.
- T109.4 Package Check:
    - Create `backend/__init__.py` if it does not exist.

# Verification
CLI must confirm the Dockerfile now uses 'COPY backend /app/backend' instead of a generic copy command.

## Response snapshot

I have updated the Dockerfile to explicitly copy the `backend` directory and `requirements.txt` as specified in task T109.1.

## Outcome

- ✅ Impact: Dockerfile now uses explicit COPY commands for backend directory and requirements.txt, enforcing the desired directory structure within the container.
- 🧪 Tests: No new tests were run, relying on the Docker build/runtime environment to validate.
- 📁 Files: Dockerfile modified.
- 🔁 Next prompts: null
- 🧠 Reflection: Successfully applied the explicit Dockerfile rewrite as requested, which was the only remaining task for T109.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
