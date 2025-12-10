# Implementation Tasks: UI Polish and Structure Update (V2)

**Feature**: UI Polish and Structure Update (V2)
**Spec**: [specs/003-ui-polish-v2/spec.md](specs/003-ui-polish-v2/spec.md)

## Dependencies

- **Phase 1 (US1)**: Blocks all subsequent phases (visual foundation).
- **Phase 2 (US2)**: Blocks final polish; matrix component required for homepage assembly.
- **Phase 3 (US3)**: Independent of Phase 2, but typically done after main layout.

## Phase 1: User Story 1 - Global Aesthetic & Typography

**Goal**: Establish the new visual baseline with Montserrat/Inter typography and global theme variables.
**Independent Test**: Verify via dev tools that all headers use Montserrat and body uses Inter.

### Implementation Tasks

- [x] T001 [US1] Implement custom font pairing (Montserrat/Inter) in `frontend/src/css/custom.css`

## Phase 2: User Story 2 - Homepage Layout & Navigation

**Goal**: Redesign the Homepage with better flow, animations, and the new Learning Journey Matrix.
**Independent Test**: Verify homepage layout flow: Hero -> Smooth Divider -> Modules -> Matrix.

### Implementation Tasks

- [x] T002 [US2] Fix Hero-Module divider aesthetic with gradient/spacing in `frontend/src/css/custom.css`
- [x] T003 [P] [US2] Implement subtle animated graphics for Hero in `frontend/src/components/HeroSection.js`
- [x] T004 [P] [US2] Develop Learning Journey Matrix component in `frontend/src/components/LearningJourneyMatrix.js`
- [x] T005 [US2] Assemble Homepage with new sections in `frontend/src/pages/index.js`

## Phase 3: User Story 3 - Footer Navigation

**Goal**: Replace default footer with custom navigation structure.
**Independent Test**: Verify footer presence and link functionality on all pages.

### Implementation Tasks

- [x] T006 [P] [US3] Develop Custom Footer component in `frontend/src/theme/Footer/index.js`

## Phase 4: Final Polish & Validation

**Goal**: Ensure all changes meet quality standards and specific constraints.

### Implementation Tasks

- [x] T007 Perform final UI polish QA and verification using `specs/003-ui-polish-v2/quickstart.md`
- [x] T008 Implement Site Logo and Favicon in `frontend/docusaurus.config.js` and `frontend/src/css/custom.css`
- [x] T009 Final Logo/Favicon Configuration Overwrite and Build Check.
- [x] T010 Fix Mobile Sidebar Aesthetic, Layering, and Navbar Layout in `frontend/src/css/custom.css`.
- [x] T011 Fix Mobile Sidebar Z-Index and Navbar Clipping in `frontend/src/css/custom.css`.
- [x] T012 Critical Mobile Layout Fix (Z-Index & Clipping) in `frontend/docusaurus.config.js` and `frontend/src/css/custom.css`.
- [x] T013 Structural Mobile Navbar and Sidebar Fix (No Forced Z-Index) in `frontend/docusaurus.config.js` and `frontend/src/css/custom.css`.
- [x] T014 Structural Mobile Navbar and Sidebar Fix (Final Audit) in `frontend/src/css/custom.css`.
- [x] T015 Structural Mobile Navbar and Sidebar Fix (Final Audit) in `frontend/src/css/custom.css`.

## Implementation Strategy

- **MVP Scope**: Tasks T001, T002, T004, T005 (Core visual update + Matrix).
- **Parallel Execution**: T003, T004, and T006 can be developed in parallel by different developers or in separate steps.
- **Incremental Delivery**:
  1.  Fonts & Global Styles (US1)
  2.  Homepage Layout Fixes & Matrix (US2)
  3.  Footer Replacement (US3)

- [x] T078 Resolve Vercel Deployment 404 Error: Fix continuous 404 error by correcting Vercel configuration for Docusaurus.

- [x] T079 Fix Vercel Permission Denied Error: Adjust frontend build scripts to resolve permission denied error on Vercel.

- [x] T080 Final Vercel Configuration Override: Explicitly define project root and build command to fix Exit Code 126.

- [x] T081 Clean Vercel Build Command for Correct Execution: Remove redundant 'cd frontend' from build command.
