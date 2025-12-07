---
id: 043
title: Critical Mobile Layout Fix (Z-Index & Clipping)
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

Phase: Final Critical Mobile Layout and Content Fix

Execute Task:
1. **Task: Critical Mobile Layout Fix (Z-Index & Clipping)**
   What to do: Perform a final, targeted CSS override and configuration change to resolve the mobile display issues:
    * **Title Shortening (Config/Code):** Implement a **shorter site title** for mobile viewports (e.g., change "Physical AI & Humanoid Robotics" to "Physical AI Book" or "Physical AI & Huma") within the `docusaurus.config.js` and/or component code to prevent clipping.
    * **Sidebar Layering Fix (Z-Index):** Re-verify and override the CSS for the mobile navigation sidebar, setting the **`z-index`** to the absolute highest possible value (e.g., `z-index: 99999;`) to ensure it renders **in front of** all other elements, including the Hero Section.
    * **Navbar Clipping Fix (CSS):** Apply precise **horizontal padding** and **`box-sizing`** adjustments to the main Navbar container (`.navbar`) in `custom.css` to prevent the logo, title, and menu icons from being clipped by the screen edges.

   Acceptance: "The mobile sidebar opens layered **above** the Hero Section; the shortened site title is fully visible and contained within the Navbar, with no clipping."
   Output: Updated `frontend/docusaurus.config.js` (for title change) and `frontend/src/css/custom.css` (for Z-Index and padding fixes).
```

## Response snapshot

Implemented critical CSS overrides for mobile sidebar z-index and navbar clipping fixes in `frontend/src/css/custom.css`, and shortened the site title in `frontend/docusaurus.config.js`. The task was added to `specs/003-ui-polish-v2/tasks.md` and marked as complete.

## Outcome

- ✅ Impact: Implemented critical mobile layout and content fixes, including title shortening and z-index/clipping adjustments.
- 🧪 Tests: Manual verification on various mobile viewports is needed for visual confirmation.
- 📁 Files: `frontend/docusaurus.config.js`, `frontend/src/css/custom.css`, and `specs/003-ui-polish-v2/tasks.md` were modified.
- 🔁 Next prompts: User to confirm changes and proceed.
- 🧠 Reflection: Successfully applied highly specific CSS overrides and configuration changes for critical mobile UI issues.

## Evaluation notes (flywheel)

- Failure modes observed: `bash` command not recognized on Windows, requiring manual PHR creation.
- Graders run and results (if applicable): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Automate PHR creation on Windows for `powershell` environment.
