---
id: 021
title: Generate Tasks for UI Overhaul
stage: tasks
date: 2025-12-06
surface: agent
model: gemini
feature: overhaul-homepage-ui
branch: 002-overhaul-homepage-ui
user: user
command: /sp.tasks
labels: [tasks, planning, docusaurus, tailwind]
links:
  spec: specs/002-overhaul-homepage-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-overhaul-homepage-ui/tasks.md
tests: []
---

## Prompt

Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts.

Context for task generation: Task Structure for Docusaurus Home Page UI Overhaul The UI overhaul project breaks into 3 phases with 7 atomic tasks. Here's the breakdown: ### Phase 1: Setup & Global Styles (2 tasks, ~45 minutes) These tasks establish the Tailwind CSS foundation and global aesthetic. Task 1.1: Configure Tailwind CSS and Fonts Duration: 30 minutes Depends on: Nothing What to do: 1. Install necessary npm packages (Tailwind CSS, PostCSS, Autoprefixer) within the `frontend/` directory. 2. Create and configure `tailwind.config.js` in `frontend/` (disabling preflight). 3. Update `frontend/docusaurus.config.js` to use a custom plugin that applies Tailwind via PostCSS. 4. Define the **Playfair Display** (Header) and **Source Sans Pro** (Body) fonts via `@font-face` or configuration. Acceptance: "Tailwind CSS utility classes are available in React components; custom fonts are globally loaded and ready for use." Output: Updated `package.json`, `tailwind.config.js`, custom plugin file, and initial `frontend/src/css/custom.css` imports. Task 1.2: Define Global Black/Gold/White Palette Duration: 15 minutes Depends on: Task 1.1 What to do: Define the core color palette in the global CSS/Tailwind config: Deep Charcoal Background (#1A1A2E), Saffron Gold Accent (#F2A900), and White/Light Gray text. Set the primary background and text color in `custom.css`. Acceptance: "Global background and text colors are correctly applied; custom gold color variable is defined and accessible via Tailwind utilities." Output: Finalized `tailwind.config.js` color definitions and `frontend/src/css/custom.css` global rules. ### Phase 2: Component Development (3 tasks, ~90 minutes) These tasks focus on building the custom components defined in the plan. Task 2.1: Develop the Module Card Component Duration: 30 minutes Depends on: Task 1.2 What to do: Create the reusable `frontend/src/components/ModuleCard.js` component using Tailwind. Implement the **softly rounded, diffused gold border** effect (via CSS pseudo-elements or shadows) and the accent hover state. Acceptance: "The Module Card component adheres to the Black/Gold aesthetic, is reusable, and implements the premium diffused border effect." Output: `frontend/src/components/ModuleCard.js` Task 2.2: Develop the Hero Section Component Duration: 45 minutes Depends on: Task 2.1 What to do: Create the `frontend/src/components/HeroSection.js` component. Integrate the **3D glassy robot image** (path: `/img/hero.png`). Use the **Playfair Display** font for the main title and apply **white/gold shaded text** styling for the premium look. Acceptance: "Hero Section component is visually dominant, integrates the image elegantly, and uses the correct typography and gold accenting for the title." Output: `frontend/src/components/HeroSection.js` Task 2.3: Develop Navigation and Footer Styles Duration: 15 minutes Depends on: Task 2.2 What to do: Apply custom styles (via `custom.css` or Tailwind utilities) to the default Docusaurus Navbar and Footer components to integrate the Black/Gold theme, ensuring high contrast for links. Acceptance: "Navbar and Footer are themed with the black background and gold links; all navigational elements remain accessible and functional." Output: Updated global CSS/SCSS. ### Phase 3: Integration and Review (2 tasks, ~45 minutes) These tasks involve assembling the components and performing final quality checks. Task 3.1: Integrate Components into Home Page Duration: 30 minutes Depends on: Task 2.3 What to do: Assemble all developed components (Hero, Module Cards) into the main `frontend/src/pages/index.js` file, ensuring the overall Home Page layout matches the architectural sketch and the "Master the Stack" section uses the custom cards. Acceptance: "The complete Home Page renders without layout errors; all components are correctly nested and content is structured logically." Output: Final `frontend/src/pages/index.js` Task 3.2: Final QA and Responsiveness Check Duration: 15 minutes Depends on: Task 3.1 What to do: Review all generated code for WCAG AA color contrast compliance and test Tailwind's responsiveness utility classes (e.g., ensuring card layout adjusts correctly on mobile). Acceptance: "WCAG AA compliance is confirmed for text/background contrast; the layout is fully responsive and looks professional on all screen sizes." Output: Final verified component files.

## Response snapshot

Generated `specs/002-overhaul-homepage-ui/tasks.md` with 8 tasks covering Setup, Foundation, and 3 User Stories.
- Tasks are dependency-ordered.
- Each task follows the strict checklist format.
- Includes parallel execution opportunities.

## Outcome

- ✅ Impact: Clear execution path for UI implementation.
- 🧪 Tests: Defined per task/phase.
- 📁 Files: tasks.md
- 🔁 Next prompts: Execute tasks (e.g., T001)
- 🧠 Reflection: Mapped user's specific task breakdown into the required User Story phase structure successfully.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
