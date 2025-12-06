# Implementation Plan: UI Polish and Structure Update (V2)

**Branch**: `003-ui-polish-v2` | **Date**: 2025-12-06 | **Spec**: [specs/003-ui-polish-v2/spec.md](specs/003-ui-polish-v2/spec.md)
**Input**: Feature specification from `/specs/003-ui-polish-v2/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a comprehensive UI polish and structure update for the Docusaurus-based frontend. This includes integrating Montserrat and Inter fonts, redesigning the Home Page with a new Learning Journey Matrix section, refining the Hero section transition, and implementing a custom, navigable Footer. All changes will use native CSS/SCSS and React overrides to maintain a high-contrast Black/Gold theme.

## Technical Context

**Language/Version**: JavaScript (React 18+), CSS3
**Primary Dependencies**: Docusaurus v3+ (React-based static site generator)
**Storage**: N/A (Frontend only)
**Testing**: Manual verification of UI/UX (Responsive design, font loading)
**Target Platform**: Web (Responsive)
**Project Type**: Web Application (Docusaurus Frontend)
**Performance Goals**: CSS-only animations for minimal impact; fast font loading.
**Constraints**: No external UI frameworks; strict directory structure (`frontend/`); Black/Gold theme adherence.
**Scale/Scope**: Global theme updates + Homepage specific features + Global Footer.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Technical Accuracy & Integrity**: Passed. Uses standard Docusaurus patterns (Swizzling, React components).
- **Step-by-Step Pedagogy**: N/A for UI, but supports pedagogy by improving navigation ("Learning Journey Matrix").
- **Consistency & Tone**: Passed. Enforces consistent font usage and branding.
- **Format & Standardization**: Passed. Uses MDX/React components within Docusaurus structure.
- **Code Integrity**: Passed. Implementation uses valid CSS/JS.

## Project Structure

### Documentation (this feature)

```text
specs/003-ui-polish-v2/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
frontend/
├── src/
│   ├── components/
│   │   ├── HeroSection.js           # Update: Animation integration
│   │   ├── LearningJourneyMatrix.js # New: Matrix section component
│   │   └── ModuleCard.js            # Update: Visual polish if needed
│   ├── css/
│   │   └── custom.css               # Update: Global fonts, animations, divider styles
│   ├── pages/
│   │   └── index.js                 # Update: Layout structure
│   └── theme/
│       └── Footer/                  # New: Swizzled/Custom Footer component
│           └── index.js
└── static/
    └── img/                         # Assets for Hero/Matrix if needed
```

**Structure Decision**: Follows standard Docusaurus project structure, keeping custom components in `src/components` and global styles in `src/css`. Footer customization via `src/theme/Footer` (Swizzling).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| None | | |