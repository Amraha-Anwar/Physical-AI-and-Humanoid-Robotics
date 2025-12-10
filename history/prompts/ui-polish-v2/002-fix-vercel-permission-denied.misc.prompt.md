---
id: "002"
title: "Fix Vercel Permission Denied"
stage: "misc"
date: "2025-12-10"
surface: "agent"
model: "gemini-2.0-flash-exp"
feature: "ui-polish-v2"
branch: "main"
user: "user"
command: "/sp.implement T079 Fix Vercel Permission Denied Error"
labels: ["vercel", "deployment", "docusaurus", "permission-denied", "fix"]
links:
  spec: "specs/003-ui-polish-v2/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml: |
 - frontend/package.json
 - specs/003-ui-polish-v2/tasks.md
tests_yaml: |
 - npm run build --prefix frontend
prompt_text: |
 /sp.implement T079 Fix Vercel Permission Denied Error

 Goal: Resolve the Vercel deployment error "Permission denied (Exit Code 126)" by ensuring the Docusaurus executable is run correctly via the Node Package Manager (NPM) context, which respects Vercel's execution permissions.

 Execute:
 - T079 Modify Build Script: Modify the `frontend/package.json` file. Ensure the "build" script explicitly uses a command that resolves paths correctly and maintains execution permissions. The standard Docusaurus template typically uses `docusaurus build`, but we should ensure the execution path is clean. We will ensure the `npm run build --prefix frontend` command works reliably by ensuring the package script is minimal and correct.
 - T079 Audit `frontend/package.json` and ensure the script section is formatted as follows:
   ```json
   "scripts": {
     "start": "docusaurus start",
     "build": "docusaurus build",
     "swizzle": "docusaurus swizzle",
     "deploy": "docusaurus deploy",
     "clear": "docusaurus clear",
     "serve": "docusaurus serve",
     "write-translations": "docusaurus write-translations",
     "write-heading-ids": "docusaurus write-heading-ids"
   },
response_text: |
 Modified `frontend/package.json` by replacing all instances of "npx docusaurus" with "docusaurus" in the scripts section to address potential Vercel permission issues. Verified the change by successfully running a local build of the frontend. Updated `specs/003-ui-polish-v2/tasks.md` to include and mark T079 as complete.
outcome: "success"
evaluation: "Frontend build script modified to resolve Vercel permission denied error."
---
