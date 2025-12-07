# File Structure Contract

**Feature**: Docusaurus Home Page UI Overhaul
**Root**: `frontend/`

This contract defines the required file structure and key exports for the UI overhaul.

## New Files

### `frontend/src/plugins/tailwind-plugin.js`
**Exports**:
- Default function returning a Docusaurus plugin object with `configurePostCss`.

### `frontend/src/components/HeroSection.js`
**Exports**:
- Default export: `HeroSection` (React Component)

### `frontend/src/components/ModuleCard.js`
**Exports**:
- Default export: `ModuleCard` (React Component)

### `tailwind.config.js` (in `frontend/` root)
**Exports**:
- `module.exports` containing Tailwind configuration.

## Modified Files

### `frontend/src/pages/index.js`
**Changes**:
- Import `HeroSection` and `ModuleCard`.
- Replace existing `HomepageHeader` and `HomepageFeatures`.
- Implement new layout structure.

### `frontend/src/css/custom.css`
**Changes**:
- `@tailwind base;`
- `@tailwind components;`
- `@tailwind utilities;`
- Overrides for Docusaurus Infima variables.
