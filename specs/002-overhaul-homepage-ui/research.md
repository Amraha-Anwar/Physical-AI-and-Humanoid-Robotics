# Research: Docusaurus Home Page UI Overhaul

**Feature**: Docusaurus Home Page UI Overhaul
**Date**: 2025-12-06

## Technical Approach

### 1. Tailwind CSS Integration
**Decision**: Use a custom Docusaurus plugin to inject Tailwind's PostCSS configuration.
**Rationale**: Docusaurus does not natively support Tailwind out of the box without configuration. A local plugin allows us to hook into the webpack/PostCSS loader chain without ejecting or maintaining a complex custom webpack config.
**Implementation Details**:
- Install `tailwindcss`, `postcss`, `autoprefixer`.
- Create `tailwind.config.js` with `corePlugins: { preflight: false }` to prevent conflict with Docusaurus base styles.
- Create `frontend/src/plugins/tailwind-plugin.js` to configure PostCSS.

### 2. Typography
**Decision**: Use Google Fonts: **Playfair Display** (Serif) for headers and **Source Sans Pro** (Sans-serif) for body text.
**Rationale**:
- **Playfair Display**: Adds the "classy, academic" aesthetic requested for the "Physical AI" theme.
- **Source Sans Pro**: Maintains high readability for documentation content, complementing the serif headers.
**Implementation**: Import via `@import` in `custom.css` or `<link>` in `docusaurus.config.js` (preferred CSS import for component encapsulation).

### 3. Gold Usage Strategy
**Decision**: Restrict Gold (#F2A900) to high-impact elements (Buttons, Borders on hover, Major Titles).
**Rationale**: Overuse of gold on a dark background can reduce readability and look "cheap". Using it as an accent creates the "premium/high-end" feel.
**Constraint**: Body text must remain White/Light Gray for accessibility (WCAG AA).

### 4. Image Integration
**Decision**: Serve `hero.png` from `frontend/static/img/` and reference as `/img/hero.png`.
**Rationale**: Docusaurus maps `static/` to the build root.

## Testing Strategy
1.  **Tailwind Verification**: Create a temporary "Test" component or element with `bg-red-500` to verify the class loads.
2.  **Responsive Design**: Test at breakpoints: Mobile (<640px), Tablet (768px), Desktop (1024px+).
3.  **Contrast Check**: Manual verification of text colors against the `#1A1A2E` (Deep Charcoal) background.
