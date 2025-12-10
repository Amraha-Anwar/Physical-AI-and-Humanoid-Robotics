---
id: 003
title: final-vercel-build-command-correction
stage: refactor
date: 2025-12-10
surface: agent
model: gemini-2.0-flash-exp
feature: none
branch: main
user: user
command: /sp.implement
labels: ["vercel", "fix", "deployment", "config"]
links:
  spec: null
  ticket: T085
  adr: null
  pr: null
files:
 - vercel.json
 - .gitignore
tests:
 - none
---

## Prompt

/sp.implement T085 Final Vercel Build Command Correction

Goal: Eliminate the persistent Vercel build error (Exit Code 1) by correcting the redundant 'cd frontend' instruction within the Vercel Build Command setting. This assumes the Vercel Project Root is correctly set to 'frontend'.

Execute:
- T085 Vercel Build Command Cleanup: Update the Vercel build command in your settings (or `vercel.json`) to the streamlined version:
    ```bash
    npm install && npm run build
    ```
- T085.1 Vercel Project Root Verification: Re-verify that the Project Root is manually set to `frontend` in your Vercel Project Settings.
- T085.2 Commit and Re-deploy: Since the previous attempt may have missed committing the `vercel.json` change, commit any file changes and push to GitHub to trigger a fresh deployment with the corrected command.

Constraint: Preserve all existing features, functions, logic, UI, layout, and content (T001-T076 verified features must remain untouched). Focus strictly on external deployment configuration.

## Response snapshot

Executed T085:
1. Updated `vercel.json`:
   - `buildCommand`: `npm install && npm run build` (removed `cd frontend` as it's redundant if Root Directory is `frontend`).
   - `outputDirectory`: `build` (corrected from `frontend/build` to match the relative path from the new root).
2. Updated `.gitignore`: Added `.vercel` and `frontend/build`.
3. Committed and Pushed: "Fix Vercel build command and output dir for frontend root".

## Outcome

- ✅ Impact: Triggered new Vercel deployment with streamlined configuration.
- 🧪 Tests: Relies on Vercel cloud build (user to verify in dashboard).
- 📁 Files: `vercel.json`, `.gitignore`.
- 🔁 Next prompts: Monitor deployment status.
- 🧠 Reflection: This configuration assumes the user has set the "Root Directory" to `frontend` in Vercel.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): N/A
