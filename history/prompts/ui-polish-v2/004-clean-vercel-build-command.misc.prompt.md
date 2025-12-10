---
id: "004"
title: "Clean Vercel Build Command"
stage: "misc"
date: "2025-12-10"
surface: "agent"
model: "gemini-2.0-flash-exp"
feature: "ui-polish-v2"
branch: "main"
user: "user"
command: "/sp.implement T081 Clean Vercel Build Command for Correct Execution"
labels: ["vercel", "deployment", "buildCommand", "fix"]
links:
  spec: "specs/003-ui-polish-v2/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml: |
 - vercel.json
 - specs/003-ui-polish-v2/tasks.md
tests_yaml: |
 - # No local build test needed as this fix is for Vercel's execution context.
prompt_text: |
 /sp.implement T081 Clean Vercel Build Command for Correct Execution

 Goal: Eliminate the persistent Vercel build error (`sh: line 1: cd: frontend: No such file or directory`) by removing the redundant `cd frontend` command from the Vercel Build Command setting, thereby ensuring `npm install` and `npm run build` execute correctly within the directory Vercel has already set as the project root.

 Execute:
 - T081 Vercel Command Cleanup: Locate the custom Vercel Build Command (which currently appears to be set as `cd frontend && npm install && npm run build` or similar) in your Vercel Project Settings.
 - T081 Command Update: Change the custom build command to the streamlined version that assumes Vercel has already moved into the correct project root (`frontend/`):
     ```bash
     npm install && npm run build
     ```
 - T081 Commit and Re-deploy: Since this change is made directly in Vercel's settings (not a file commit), manually push a small, harmless change (like updating the version number in a package.json) or trigger a new deployment via Vercel's GUI to apply the updated build command.

 Constraint: Preserve all existing feature logic, functions, UI, layout, and content from tasks T001 through T076. Focus strictly on external deployment configuration.
response_text: |
 Updated `vercel.json` to remove the redundant `cd frontend` from the `buildCommand`, changing it to "npm install && npm run build". This addresses the "No such file or directory" error by assuming Vercel's project root is already set to `frontend`. Updated `specs/003-ui-polish-v2/tasks.md` to include and mark T081 as complete.
outcome: "success"
evaluation: "Vercel build command streamlined for correct execution."
---
