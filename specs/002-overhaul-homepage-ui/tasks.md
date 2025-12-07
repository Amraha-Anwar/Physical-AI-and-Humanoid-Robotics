# Tasks: Docusaurus Home Page UI Overhaul

**Branch**: `002-overhaul-homepage-ui`
**Status**: Pending
**Total Tasks**: 8

## Phase 1: Setup (Project Initialization)

> **Goal**: Initialize Tailwind CSS and configure the environment for styling.
> **Test**: `npm start` runs without errors and Tailwind classes are generated.

- [x] T001 Install Tailwind dependencies in `frontend/package.json`
- [x] T002 Configure Tailwind and PostCSS plugin in `frontend/tailwind.config.js` and `frontend/src/plugins/tailwind-plugin.js`

## Phase 2: Foundational (Blocking Prerequisites)

> **Goal**: Establish the global design system (colors, fonts).
> **Test**: Global background is Deep Charcoal; fonts are loaded.

- [x] T003 Define global color palette and fonts in `frontend/src/css/custom.css`

## Phase 3: User Story 1 (Visitor Immersive First Impression)

> **Goal**: Implement the Hero section with the 3D robot image and premium typography.
> **Test**: Hero section renders with correct image and styled text.

### Implementation
- [x] T004 [US1] Develop HeroSection component in `frontend/src/components/HeroSection.js`

## Phase 4: User Story 2 (Module Exploration)

> **Goal**: Implement reusable Module Cards with the "Neo-Minimalist Tech" style.
> **Test**: Cards render with dark background, rounded corners, and hover effects.

### Implementation
- [x] T005 [US2] Develop ModuleCard component in `frontend/src/components/ModuleCard.js`

## Phase 5: User Story 3 (Integration & Mobile Responsiveness)

> **Goal**: Integrate all components into the homepage and ensure mobile compatibility.
> **Test**: Homepage is fully functional and responsive on mobile devices.

### Implementation
- [x] T006 [US3] Style Navigation and Footer in `frontend/src/css/custom.css`
- [x] T007 [US3] Integrate components into Home Page layout in `frontend/src/pages/index.js`

## Final Phase: Polish & Cross-Cutting Concerns

> **Goal**: Final quality assurance and polish.
> **Test**: WCAG AA compliance and visual verification.

- [x] T008 Final QA for color contrast and responsiveness in `frontend/src/css/custom.css`

## Dependencies

1. **T001** (Setup) -> **T002** -> **T003** (Foundation)
2. **T003** -> **T004** (US1), **T005** (US2), **T006** (US3) (Parallelizable)
3. **T004**, **T005** -> **T007** (Integration)
4. **T007**, **T006** -> **T008** (Polish)

## Parallel Execution

After Phase 2 (Foundation) is complete, the following can be developed in parallel:
- **US1**: Hero Section (T004)
- **US2**: Module Cards (T005)
- **US3**: Navbar/Footer Styling (T006)

## Implementation Strategy

1.  **Infrastructure First**: Get Tailwind running immediately to unblock styling.
2.  **Component Isolation**: Build Hero and Cards as isolated components first.
3.  **Integration**: Assemble the page only after components are verified.
4.  **Mobile-Last Verification**: Check responsiveness after structural integration.
