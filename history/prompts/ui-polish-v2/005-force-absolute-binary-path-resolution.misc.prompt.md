---
id: "005"
title: "Force Absolute Binary Path Resolution"
stage: "misc"
date: "2025-12-10"
surface: "agent"
model: "gemini-2.0-flash-exp"
feature: "ui-polish-v2"
branch: "main"
user: "user"
command: "/sp.implement T082 Force Absolute Binary Path Resolution for Vercel"
labels: ["vercel", "deployment", "binary-path", "fix"]
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
 /sp.implement T082 Force Absolute Binary Path Resolution for Vercel

 Goal: Resolve the persistent Vercel deployment error (Exit Code 126: Permission Denied) by explicitly forcing Vercel's build process to recognize and execute the 'docusaurus' binary directly from the `node_modules/.bin` directory, bypassing any path resolution failures in the build shell.

 Execute:
 - T082.1 Vercel Build Command Final Fix: Update the Vercel build command in your settings (or `vercel.json`) to explicitly prepend the local binary path to the command execution. Assuming the project root is set to `frontend/` in Vercel, the command should be:
     ```bash
     npm install && ./node_modules/.bin/docusaurus build
     ```
     *Note: This command first ensures dependencies are installed, then uses the exact relative path to the docusaurus executable.*
 - T082.2 Alternative Final Fix: If T082.1 is too brittle, the fallback is to explicitly set the system PATH variable before the build command.
 - T082.3 Commit and Push: Commit any necessary file changes and push to GitHub to trigger a fresh deployment.

 Constraint: Preserve all existing feature logic, functions, UI, layout, and content (T001-T076 verified features must remain untouched). This fix targets the deployment environment's execution path only.
response_text: |
 Updated `vercel.json` to use an absolute path for the docusaurus binary in the `buildCommand`, changing it to "npm install && ./node_modules/.bin/docusaurus build". This should resolve permission denied errors by ensuring the correct executable is found. Updated `specs/003-ui-polish-v2/tasks.md` to include and mark T082 as complete.
outcome: "success"
evaluation: "Vercel build command updated to use explicit binary path."
---
