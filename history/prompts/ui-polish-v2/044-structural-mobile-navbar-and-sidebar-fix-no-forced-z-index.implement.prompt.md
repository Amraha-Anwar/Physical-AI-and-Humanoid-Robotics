---
id: 044
title: Structural Mobile Navbar and Sidebar Fix (No Forced Z-Index)
stage: implement
date: 2025-12-06
surface: agent
model: gemini-1.5-pro
feature: ui-polish-v2
branch: (not specified)
user: (not specified)
command: /sp.implement
labels: []
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/docusaurus.config.js
  - frontend/src/css/custom.css
  - specs/003-ui-polish-v2/tasks.md
tests: []
---

## Prompt

```
/sp.implement

Phase: Final Critical Mobile Layout Fix (Structural Correction)

Execute Task:
1. **Task: Structural Mobile Navbar and Sidebar Fix (No Forced Z-Index)**
   What to do: Perform a detailed structural CSS correction based on Docusaurus theme knowledge to fix the remaining mobile display issues without using arbitrary, high Z-Index values or changing the site title in the config.
    * **Constraint Adherence:** Do **NOT** change the book's full name ("Physical AI & Humanoid Robotics"). Do **NOT** use `z-index: 99999;`.
    * **Layering Fix (Sidebar Behind Hero):** Identify the primary CSS class for the mobile sidebar/drawer (e.g., `.navbar-sidebar`) and apply a **structurally correct `z-index` value** that is higher than the Navbar's base `z-index` (which is often around 100-1000) and higher than the Hero section (which is typically z-index 1). The fix must correctly place the sidebar **in front** of the Hero content.
    * **Navbar Clipping Fix (Title Cutting Off):** Adjust the **horizontal padding** and/or the **width management** of the Navbar container and the inner title/logo wrapper to ensure the full, original book title is visible on mobile screens without clipping or wrapping aggressively. This may require slightly reducing the logo size or increasing the Navbar's overall padding in mobile media queries.

   Acceptance: "The mobile sidebar opens layered **above** the Hero Section; the **full, original site title** is visible and contained within the Navbar on mobile viewports."
   Output: Updated `frontend/docusaurus.config.js` (for title change) and `frontend/src/css/custom.css` (for Z-Index and padding fixes).
```

## Response snapshot

Implemented structural mobile Navbar and sidebar fixes in `frontend/src/css/custom.css`, including setting a structurally correct z-index for the sidebar and adjusting padding and logo size for the Navbar. Reverted the site title change in `frontend/docusaurus.config.js`. The task was added to `specs/003-ui-polish-v2/tasks.md` and marked as complete.

## Outcome

- ✅ Impact: Implemented structural mobile layout fixes, adhering to constraints about title and z-index.
- 🧪 Tests: Manual verification on various mobile viewports is needed for visual confirmation.
- 📁 Files: `frontend/docusaurus.config.js`, `frontend/src/css/custom.css`, and `specs/003-ui-polish-v2/tasks.md` were modified.
- 🔁 Next prompts: User to confirm changes and proceed.
- 🧠 Reflection: Successfully applied structural CSS corrections and reverted configuration changes as per constraints.

## Evaluation notes (flywheel)

- Failure modes observed: `bash` command not recognized on Windows, requiring manual PHR creation.
- Graders run and results (if applicable): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Automate PHR creation on Windows for `powershell` environment.
