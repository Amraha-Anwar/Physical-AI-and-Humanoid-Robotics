# Research Findings: UI Polish and Structure Update (V2)

**Feature**: UI Polish and Structure Update (V2)
**Status**: Complete

## Decisions

### 1. Typography Strategy
**Decision**: Use Google Fonts for **Montserrat** (Headers) and **Inter** (Body).
**Rationale**: These fonts provide the desired modern, readable aesthetic and are freely available.
**Implementation**: Import via CSS `@import` or standard HTML link in Docusaurus config, applied via CSS variables in `custom.css`.

### 2. Divider Implementation
**Decision**: Use a **CSS Gradient Mask** combined with increased padding.
**Rationale**: "Not look odd" requires a smooth visual transition rather than a hard line. A linear gradient fading from the Hero background color to the content background color creates a seamless effect.
**Alternatives Considered**: SVG Divider (too complex for simple cleanup), HR tag (too abrupt).

### 3. Footer Customization
**Decision**: Swizzle the Docusaurus `Footer` component.
**Rationale**: Docusaurus allows component overriding (swizzling). Since the requirements involve a completely new layout (Book Title + 4 specific Module columns), wrapping the existing footer is insufficient. We will replace the internal logic of the Footer while maintaining site consistency.
**Method**: Create `frontend/src/theme/Footer/index.js` to completely define the new structure.

### 4. Animation Technique
**Decision**: Native CSS Transitions and Keyframes.
**Rationale**: Meets constraint "No external UI frameworks". CSS is performant for simple "pulsing" or "fade-in" effects required for the Hero and Matrix.

## Best Practices

- **Docusaurus Swizzling**: Prefer "eject" (wrapping is technically safer but limiting for structure changes) or strictly overriding by placing the file in `src/theme`. We will use the `src/theme` file placement method which acts as an override.
- **CSS Variables**: Use CSS variables for colors (Black/Gold) to ensure theme consistency and easy updates.
- **Responsive Design**: Use media queries to stack the Learning Journey Matrix cards vertically on mobile devices.
