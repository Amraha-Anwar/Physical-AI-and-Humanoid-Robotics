---
id: 019
title: Create Docusaurus Home Page Spec
stage: spec
date: 2025-12-06
surface: agent
model: gemini
feature: overhaul-homepage-ui
branch: 002-overhaul-homepage-ui
user: user
command: /sp.specify
labels: [spec, docusaurus, ui]
links:
  spec: specs/002-overhaul-homepage-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-overhaul-homepage-ui/spec.md
 - specs/002-overhaul-homepage-ui/checklists/requirements.md
tests: []
---

## Prompt

Docusaurus Home Page UI Overhaul Target Component: Docusaurus Home Page (Hero, Feature Cards, Footer) and Global Styles Focus: Implementing a high-end, futuristic, and premium aesthetic that aligns with the book's "Physical AI & Humanoid Robotics" subject matter. Success criteria: - **Visual Compliance:** The generated UI must strictly adhere to the black background, enhanced golden #F2A900 highlights, and white/gold shaded text aesthetic. - **Aesthetic Compliance:** Successfully implements the "Neo-Minimalist Tech" aesthetic: refined typography, subtle diffused glow/borders, and elevated design agency presentation. - **Image Integration:** The 3D glassy robot image must be prominently and elegantly integrated into the Hero Section. Image path: `/hero.png` (placed in `frontend/static/img/`). - **Styling Method:** Successfully implements **Tailwind CSS/Custom CSS** for component-level styling (utility-first approach). - **Typography:** Uses the specified custom serif/sans-serif font pairing (via @font-face rules if required) to deliver a classy, academic aesthetic. - **Component Design:** Generates functional, styled components for: 1) The Hero Section, 2) The Module Cards (Master the Stack). - **Component Design:** Generates functional designs for three primary areas: 1. **Hero Section:** Integrate the high-contrast 3D robot image into the background or as a main focal point. Use large, elegant serif and sans-serif typography (white and gold shaded). 2. **Module Cards (Master the Stack):** Design card components with dark backgrounds, softly rounded corners, and a **1px glowing gold/violet diffused border** instead of harsh lines. 3. **Navigation/Footer:** Ensure all navigational elements are clean, high-contrast, and use the gold accent for links/hovers. - **Responsiveness:** All components must be fully responsive and functional on mobile devices. Constraints: - **Project Structure:** All files and paths must respect the new root directory: `frontend/`. - **External Libraries:** **Tailwind CSS** for styling. The specification must include the necessary setup steps in the subsequent plan. - **Color Palette:** Primary Background: Deep Charcoal/Black. Primary Accent: #F2A900 (Saffron Gold). - **Color Mandate:** Must strictly use the defined palette: Background: Deep Charcoal/Black; Primary Accent: #F2A900 (Saffron Gold). - **Responsiveness:** All components must be fully responsive for mobile, tablet, and desktop views. Not building: - Documentation pages (only Home Page and global components). - Any complex database or backend logic.

## Response snapshot

Created specification for Docusaurus Home Page UI Overhaul. Validated against checklist.

## Outcome

- ✅ Impact: Created validated spec.
- 🧪 Tests: N/A
- 📁 Files: 2 files created.
- 🔁 Next prompts: /sp.clarify or /sp.plan
- 🧠 Reflection: Manual creation of files required due to script issues.

## Evaluation notes (flywheel)

- Failure modes observed: Script parsing error.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
