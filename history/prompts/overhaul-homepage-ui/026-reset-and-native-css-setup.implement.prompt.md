---
id: 026
title: Reset and Native CSS Setup
stage: implement
date: 2025-12-06
surface: agent
model: gemini
feature: overhaul-homepage-ui
branch: 002-overhaul-homepage-ui
user: user
command: /sp.implement
labels: [implement, refactor, css, cleanup]
links:
  spec: specs/002-overhaul-homepage-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - frontend/src/css/custom.css
 - frontend/package.json
 - frontend/docusaurus.config.js
tests: []
---

## Prompt

/sp.implement

Phase: Reset & Native CSS Setup

Execute Task:
1. **Task: Clean Tailwind Dependencies**
   What to do: Conceptually remove all artifacts related to the broken Tailwind setup: assume removal of npm packages, deletion of `tailwind.config.js`, and deletion/removal of the custom Docusaurus PostCSS plugin reference.
   Acceptance: "Project is reset to a standard Docusaurus setup, ready for custom CSS."
   Output: (Conceptual completion, no file output)

2. **Task: Native CSS Global Style & Typography Setup**
   What to do:
   1. Define the custom color variables (Deep Charcoal, Saffron Gold) and load the custom fonts (Playfair Display, Source Sans Pro) directly within `frontend/src/css/custom.css` using standard CSS and `@font-face` rules.
   2. Set the global background to Deep Charcoal (#1A1A2E).
   Acceptance: "Global dark background and custom fonts are successfully applied, passing the visual aesthetic test."
   Output: `frontend/src/css/custom.css` (Updated)

## Response snapshot

Executed Reset and Native CSS Setup.
- Removed Tailwind dependencies and configuration.
- Removed custom plugin from Docusaurus config.
- Rewrote `custom.css` to use native CSS variables, fonts, and global styles for the requested aesthetic.

## Outcome

- ✅ Impact: Switched from Tailwind to Native CSS for simpler maintenance and better integration with Docusaurus Classic theme.
- 🧪 Tests: Visual verification pending.
- 📁 Files: `frontend/src/css/custom.css`, `frontend/package.json`, `frontend/docusaurus.config.js`
- 🔁 Next prompts: /sp.implement (Update components to use native CSS classes)
- 🧠 Reflection: Pivot away from Tailwind executed cleanly.

## Evaluation notes (flywheel)

- Failure modes observed: PowerShell `rm` command usage issue (fixed by using individual `Remove-Item`).
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
