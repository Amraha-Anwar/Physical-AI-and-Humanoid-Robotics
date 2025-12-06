# Implementation Plan: Docusaurus Home Page UI Overhaul

**Branch**: `002-overhaul-homepage-ui` | **Date**: 2025-12-06 | **Spec**: [specs/002-overhaul-homepage-ui/spec.md](../spec.md)
**Input**: Feature specification from `/specs/002-overhaul-homepage-ui/spec.md`

## Summary

Implement a "Physical AI" themed homepage update using Tailwind CSS. This involves setting up Tailwind within Docusaurus, defining a custom color palette (Deep Charcoal & Gold), and creating new React components (`HeroSection`, `ModuleCard`) to replace the default homepage content. The design focuses on a "Neo-Minimalist Tech" aesthetic with glassy effects and 3D imagery.

## Technical Context

**Language/Version**: JavaScript / React (Docusaurus standard)
**Primary Dependencies**: Docusaurus, Tailwind CSS, PostCSS, Autoprefixer
**Storage**: N/A (Static Site)
**Testing**: Visual Verification, Responsive Design Checks (Chrome DevTools)
**Target Platform**: Web (Responsive)
**Project Type**: Web / Documentation Site
**Performance Goals**: Fast LCP (Largest Contentful Paint) for Hero image; efficient CSS loading.
**Constraints**: Must use Tailwind CSS; Must adhere to Black/#F2A900 color palette.
**Scale/Scope**: Homepage and global styling only.

## Constitution Check

*GATE: Passed.*

- **Technical Accuracy**: Uses standard Tailwind integration for Docusaurus.
- **Step-by-Step Pedagogy**: N/A (This is a UI task, not content).
- **Consistency & Tone**: Visual tone aligns with "Physical AI" theme.
- **Format**: Standard Docusaurus structure.
- **Code Integrity**: Code will be standard React/CSS.

## Project Structure

### Documentation (this feature)

```text
specs/002-overhaul-homepage-ui/
├── plan.md              # This file
├── research.md          # Technical approach & decisions
├── data-model.md        # Component props
├── quickstart.md        # Testing instructions
├── contracts/           # File structure contract
└── tasks.md             # Implementation tasks (to be created)
```

### Source Code (repository root)

```text
frontend/
├── tailwind.config.js          # NEW: Tailwind config
├── src/
│   ├── plugins/
│   │   └── tailwind-plugin.js  # NEW: Docusaurus plugin
│   ├── css/
│   │   └── custom.css          # MODIFIED: CSS imports & vars
│   ├── components/
│   │   ├── HeroSection.js      # NEW: Hero component
│   │   └── ModuleCard.js       # NEW: Card component
│   └── pages/
│       └── index.js            # MODIFIED: Main entry point
└── static/
    └── img/
        └── hero.png            # EXISTING: Asset
```

**Structure Decision**: Standard Docusaurus "classic" theme structure extended with a local plugin for Tailwind.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Custom Plugin | Tailwind integration | Docusaurus doesn't support Tailwind natively; manual CSS is harder to maintain for this design. |