---
id: 022
title: Execute Setup Phase for UI Overhaul
stage: implement
date: 2025-12-06
surface: agent
model: gemini
feature: overhaul-homepage-ui
branch: 002-overhaul-homepage-ui
user: user
command: /sp.implement
labels: [implement, tailwind, docusaurus, setup]
links:
  spec: specs/002-overhaul-homepage-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - frontend/tailwind.config.js
 - frontend/src/plugins/tailwind-plugin.js
 - frontend/docusaurus.config.js
 - frontend/src/css/custom.css
 - specs/002-overhaul-homepage-ui/tasks.md
tests: []
---

## Prompt

/sp.implement

Phase: Setup & Global Styles

Execute Task:
1. **Task 1.1: Configure Tailwind CSS and Fonts**
   What to do:
   1. Install necessary npm packages (Tailwind CSS, PostCSS, Autoprefixer) within the `frontend/` directory.
   2. Create and configure `tailwind.config.js` in `frontend/` (disabling preflight).
   3. Update `frontend/docusaurus.config.js` to use a custom plugin that applies Tailwind via PostCSS.
   4. Define the **Playfair Display** (Header) and **Source Sans Pro** (Body) fonts via `@font-face` or configuration.
   Acceptance: "Tailwind CSS utility classes are available in React components; custom fonts are globally loaded and ready for use."
   Output: Updated `package.json`, `tailwind.config.js`, custom plugin file, and initial `frontend/src/css/custom.css` imports.

## Response snapshot

Executed Phase 1 tasks (Setup & Global Styles).
- Installed Tailwind dependencies (T001).
- Configured `tailwind.config.js` and created Docusaurus plugin (T002).
- Updated `docusaurus.config.js` to load the plugin.
- Defined global styles, fonts, and colors in `custom.css` (T003).

## Outcome

- ✅ Impact: Project is now configured for Tailwind development with correct branding.
- 🧪 Tests: `npm start` should now load Tailwind.
- 📁 Files: Modified/Created configuration and CSS files.
- 🔁 Next prompts: /sp.implement (Phase 2 Component Development)
- 🧠 Reflection: Bundled T003 with Setup phase as it was part of the logical setup unit requested by user.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
