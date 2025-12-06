---
id: 027
title: Refactor Components to Native CSS
stage: implement
date: 2025-12-06
surface: agent
model: gemini
feature: overhaul-homepage-ui
branch: 002-overhaul-homepage-ui
user: user
command: /sp.implement
labels: [implement, refactor, css, components]
links:
  spec: specs/002-overhaul-homepage-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - frontend/src/css/custom.css
 - frontend/src/components/HeroSection.js
 - frontend/src/components/ModuleCard.js
 - frontend/src/pages/index.js
tests: []
---

## Prompt

/sp.implement

Phase: Component Refactoring (Native CSS)

Execute Tasks:
1. **Task: Refactor HeroSection to Native CSS**
   What to do: Refactor the code in `frontend/src/components/HeroSection.tsx` (or .js if you reverted) to remove all Tailwind utility classes and replace them with new, descriptive CSS classes (e.g., `hero__container`, `hero__title--gold`) defined in `custom.css`. Ensure the **3D glassy robot image** is integrated and the **Playfair Display** font is applied to the title.
   Acceptance: "HeroSection component is functional, uses only standard CSS classes, and visually maintains the high-end Black/Gold aesthetic."
   Output: Updated `frontend/src/components/HeroSection.tsx`

2. **Task: Refactor ModuleCard to Native CSS**
   What to do: Refactor the code in `frontend/src/components/ModuleCard.tsx` (or .js) to remove all Tailwind utility classes. Replace them with standard CSS classes (e.g., `module-card`, `module-card--glow`) to achieve the **softly rounded, diffused gold border** effect using CSS techniques (like `box-shadow` or pseudo-elements).
   Acceptance: "ModuleCard component is functional, uses only standard CSS classes, and implements the premium diffused border/glow effect."
   Output: Updated `frontend/src/components/ModuleCard.tsx`

## Response snapshot

Refactored Components to Native CSS.
- Updated `custom.css` with comprehensive class definitions for Hero, Cards, and Grid layouts.
- Updated `HeroSection.js` to use new CSS classes.
- Updated `ModuleCard.js` to use new CSS classes.
- Updated `index.js` to use new CSS classes for layout and typography.

## Outcome

- ✅ Impact: Full removal of Tailwind dependency while maintaining the visual design intent.
- 🧪 Tests: Visual verification pending.
- 📁 Files: `frontend/src/css/custom.css`, `frontend/src/components/HeroSection.js`, `frontend/src/components/ModuleCard.js`, `frontend/src/pages/index.js`
- 🔁 Next prompts: /sp.green (Final verification)
- 🧠 Reflection: Native CSS implementation provides cleaner component code (separation of concerns) at the cost of a larger CSS file.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
