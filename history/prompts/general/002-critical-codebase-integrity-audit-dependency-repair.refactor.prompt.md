---
id: 002
title: critical-codebase-integrity-audit-dependency-repair
stage: refactor
date: 2025-12-10
surface: agent
model: gemini-2.0-flash-exp
feature: none
branch: main
user: user
command: /sp.implement
labels: ["audit", "fix", "dependencies", "vercel"]
links:
  spec: null
  ticket: T084
  adr: null
  pr: null
files:
 - vercel.json
tests:
 - npm run build (frontend)
---

## Prompt

/sp.implement T084 Critical Codebase Integrity Audit and Dependency Repair

Goal: Perform a complete audit of the codebase against the project history to fix structural file corruption (especially missing core Docusaurus dependencies in `frontend/package.json`) and resolve the new Vercel output error ("No Output Directory named 'build' found"). All existing functional code, features, UI, and logic must remain strictly untouched.

Execute:
- T084.1 Frontend Dependency Restoration: **Audit the historical `frontend/package.json` contents against the current file.** Ensure all core Docusaurus dependencies (e.g., `@docusaurus/core`, `@docusaurus/preset-classic`, etc.) are present and correctly versioned, incorporating any manual additions made by the user.
- T084.2 Frontend Dependency Installation: Execute a clean install of Node.js dependencies (`cd frontend && npm install`).
- T084.3 Vercel Environment Configuration Check: Confirm the Vercel Build Command is set to the most robust known working command: `npm install && npm run build` (or `npx docusaurus build` as a fallback), and the Output Directory remains set to `build/` in `vercel.json` or Vercel settings.
- T084.4 Final Build Command Test: Run a local test build (`cd frontend && npm run build`) to ensure the `build/` directory is successfully created, confirming the missing dependencies were the cause of the Output Directory error.

Constraint: No functional, logical, or UI/layout code introduced in T001-T083 shall be altered. Focus is on file integrity and environment setup only.

## Response snapshot

Executed T084:
1. Audited `frontend/package.json` - dependencies found intact.
2. Updated `vercel.json` to correctly point to `frontend` directory for build command (`cd frontend && npm install && npm run build`).
3. Ran `npm install` in `frontend` - success.
4. Ran `npm run build` in `frontend` - success, `build` directory created.

## Outcome

- ✅ Impact: Resolved Vercel deployment configuration and verified build integrity.
- 🧪 Tests: `npm run build` passed locally.
- 📁 Files: Modified `vercel.json`.
- 🔁 Next prompts: Deploy to Vercel or proceed with further features.
- 🧠 Reflection: The issue was likely incorrect CWD in Vercel build command, not corruption.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): N/A
