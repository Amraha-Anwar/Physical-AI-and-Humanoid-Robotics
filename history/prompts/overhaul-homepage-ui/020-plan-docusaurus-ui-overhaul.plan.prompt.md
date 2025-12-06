---
id: 020
title: Plan Docusaurus UI Overhaul
stage: plan
date: 2025-12-06
surface: agent
model: gemini
feature: overhaul-homepage-ui
branch: 002-overhaul-homepage-ui
user: user
command: /sp.plan
labels: [plan, architecture, tailwind, docusaurus]
links:
  spec: specs/002-overhaul-homepage-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-overhaul-homepage-ui/plan.md
 - specs/002-overhaul-homepage-ui/research.md
 - specs/002-overhaul-homepage-ui/data-model.md
 - specs/002-overhaul-homepage-ui/contracts/file-structure.md
 - specs/002-overhaul-homepage-ui/quickstart.md
tests: []
---

## Prompt

/sp.plan

Create: architecture sketch (of the Home Page sections), component structure, technical approach, quality validation.

Decisions needing documentation: list important choices with options and tradeoffs.
Testing strategy: validation checks based on acceptance criteria (e.g., color contrast, responsiveness).

---
## UI Implementation Plan Requirements

### 1. Technical Approach & Tailwind Setup 
The following steps must be executed before creating components:
1. **Install Dependencies (Relative to frontend/):** Install Tailwind CSS, PostCSS, and Autoprefixer.
2. **Configure Tailwind:** Create `tailwind.config.js` in the `frontend/` root. Must disable **preflight** to avoid overriding Docusaurus styles (`corePlugins: { preflight: false }`).
3. **Docusaurus Plugin:** Create a custom Docusaurus plugin (e.g., in `frontend/src/plugins/tailwind-plugin.js`) to configure PostCSS to load Tailwind CSS and Autoprefixer.
4. **Load CSS:** Update `frontend/src/css/custom.css` to import Tailwind base, components, and utilities.

### 2. Architecture Sketch (Home Page Layout)
* **Goal:** Sketch the visual layout of the Home Page, divided into three main sections: **Hero (Robot Focus)**, **Master the Stack (4x2 Cards)**, and **Bottom Fold (Links/Footer)**. Annotate the use of Gold (#F2A900) and Deep Charcoal (#1A1A2E).

### 3. Component Structure (Relative to frontend/)
* **A.** `frontend/src/css/custom.css`: Define custom color variables, font imports.
* **B.** `frontend/src/plugins/tailwind-plugin.js`: (Required for Tailwind integration).
* **C.** `frontend/src/pages/index.js`: Main Home Page layout using `import Layout from '@theme/Layout'`.
* **D.** `frontend/src/components/HeroSection.js`: React component for the main hero.
* **E.** `frontend/src/components/ModuleCard.js`: Reusable card component.

### 4. Decisions Needing Documentation
1. **Typography Pairing (Aesthetic Choice):** To achieve the "classy, academic, tech" look, the following Google Fonts will be used:
    * **Header (Serif):** **Playfair Display** (for elegance and class).
    * **Body/UI (Sans-serif):** **Source Sans Pro** (for clean readability in documentation).
2. **Image Path Confirmation:** The robot image will be accessed via the path `/img/hero.png` (due to Docusaurus static folder mapping).
3. **Gold Usage Strategy:** Gold (#F2A900) is reserved for: **Primary Buttons, Card Borders (on hover), Major Titles, and UI accents**. All body text remains White/Light Gray for readability.

### 5. Testing Strategy
* **Tailwind Check:** Verify a test component (e.g., a red button) correctly renders Tailwind utility classes in `frontend/src/pages/index.js`.
* **Color Contrast:** Verify white text against the Deep Charcoal background passes WCAG AA guidelines.
* **Responsiveness:** Ensure the 4x2 card layout collapses gracefully into a 2x4 or 1x8 layout on mobile breakpoints using Tailwind's responsive prefixes (e.g., `md:grid-cols-4`).

## Response snapshot

Created implementation plan and design artifacts for Docusaurus Home Page UI Overhaul.
- Defined Tailwind integration strategy via custom plugin.
- Established Component/Data Model for Hero and Cards.
- Created Quickstart guide for verification.

## Outcome

- ✅ Impact: Ready for task generation.
- 🧪 Tests: Defined in quickstart.md
- 📁 Files: plan.md, research.md, data-model.md, contracts/file-structure.md, quickstart.md
- 🔁 Next prompts: /sp.tasks
- 🧠 Reflection: User provided very specific requirements which sped up the research phase.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
