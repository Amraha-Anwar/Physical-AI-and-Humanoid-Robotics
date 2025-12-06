# Feature Specification: UI Polish and Structure Update (V2)

**Feature Branch**: `003-ui-polish-v2`  
**Created**: 2025-12-06  
**Status**: Draft  
**Input**: User description: "UI Polish and Structure Update (V2) Target Component: Global Theme, Home Page (Hero, New Section), and Footer. Focus: Implementing aesthetic polish, fixing layout issues, and enhancing navigation based on user feedback. Success criteria: - **Font Implementation:** Successfully implements the new pairing: **Montserrat (Header)** and **Inter (Body)** across the entire Docusaurus theme, ensuring excellent readability. - **Aesthetic Fix (Divider):** The visual separation between the Hero Section and the Module Cards ("Master the Stack") must be redesigned to be clean, elegant, and **not look odd** (e.g., subtle use of a gradient mask or increased vertical spacing/padding). - **New Section Integration:** Adds a new "Learning Journey Matrix" section (inspired by the reference site's matrix/tiers breakdown). This section must visually link the four book modules to high-level outcomes using gold highlights and a card-based structure. - **Footer Design:** Creates a new, elegant, and attractive Footer. It must include: 1. The **Book Title** prominently displayed. 2. Links to the start page of **all four Modules (1-4)**. 3. Uses the Black/Gold theme with appropriate contrast. - **Theme Maintenance:** All changes must uphold the Black/Gold/White aesthetic and high-contrast standards. Constraints: - **Project Structure:** All files and paths must respect the `frontend/` directory structure. - **Aesthetic Priority:** The visual impact and perceived quality must be significantly higher than the previous iteration. - **Implementation Method:** Must rely on Native CSS/SCSS and React component overrides (no external UI frameworks). Not building: - New documentation content (only structure and presentation changes). - Complex animation libraries (rely on CSS/SVG/simple React for animation)."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Global Aesthetic & Typography (Priority: P1)

As a visitor, I want to experience a cohesive and readable design with specific font pairings so that the content is easy to consume and looks professional.

**Why this priority**: Typography is fundamental to the site's readability and brand identity (Montserrat/Inter pairing).

**Independent Test**: Can be tested by inspecting the computed styles of headers and body text across various pages to verify Montserrat and Inter are applied.

**Acceptance Scenarios**:

1. **Given** any page on the site, **When** the page loads, **Then** all headers (H1-H6) display in Montserrat font.
2. **Given** any page on the site, **When** the page loads, **Then** all body text displays in Inter font.
3. **Given** the site theme, **When** viewed, **Then** the color palette strictly adheres to Black/Gold/White with high contrast.

---

### User Story 2 - Homepage Layout & Navigation (Priority: P1)

As a learner, I want a structured homepage with a clear learning path and smooth visual transitions so that I can easily navigate to the relevant modules.

**Why this priority**: The homepage is the entry point; the new "Learning Journey Matrix" and improved divider are critical for guiding users.

**Independent Test**: Can be tested by viewing the homepage, verifying the divider aesthetic, and checking the presence and functionality of the Learning Journey Matrix.

**Acceptance Scenarios**:

1. **Given** the homepage, **When** scrolling from Hero Section to Module Cards, **Then** the visual transition is clean and elegant (e.g., gradient mask or adequate spacing) without looking disjointed.
2. **Given** the homepage, **When** scrolling down, **Then** a "Learning Journey Matrix" section is visible.
3. **Given** the Learning Journey Matrix, **When** viewing the cards, **Then** they visually link to the four book modules using gold highlights.

---

### User Story 3 - Footer Navigation (Priority: P2)

As a user, I want a helpful footer with clear navigation links so that I can jump to specific modules from anywhere on the site.

**Why this priority**: Provides essential navigation fallback and branding presence at the bottom of pages.

**Independent Test**: Can be tested by scrolling to the bottom of any page and clicking the footer links.

**Acceptance Scenarios**:

1. **Given** the footer, **When** viewed, **Then** the Book Title is prominently displayed.
2. **Given** the footer, **When** checked, **Then** it contains links to the start pages of Module 1, Module 2, Module 3, and Module 4.
3. **Given** the footer, **When** viewed, **Then** it uses the Black/Gold theme with appropriate text contrast.

### Edge Cases

- What happens when viewed on mobile devices? (Responsive design must maintain aesthetics).
- How does the Learning Journey Matrix stack on small screens?
- Does the font pairing load correctly if web fonts fail (fallbacks)?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST implement **Montserrat** font for all headers and **Inter** font for all body text globally.
- **FR-002**: Homepage MUST feature a redesigned visual separation between Hero Section and Module Cards (clean, elegant, e.g., gradient or spacing).
- **FR-003**: Homepage MUST include a new **"Learning Journey Matrix"** section.
- **FR-004**: The Learning Journey Matrix MUST display card-based links to all four book modules.
- **FR-005**: The Learning Journey Matrix MUST use **gold highlights** for visual emphasis.
- **FR-006**: Site MUST implement a new Footer containing the **Book Title** and links to **Modules 1, 2, 3, and 4**.
- **FR-007**: Footer MUST utilize the **Black/Gold** color theme.
- **FR-008**: All UI changes MUST be implemented using **Native CSS/SCSS** and **React component overrides** only (No external UI frameworks).
- **FR-009**: All files MUST be located within the `frontend/` directory structure.

### Key Entities *(include if feature involves data)*

- **Learning Journey Matrix**: A UI component representing the structured path through the 4 modules.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of headers use Montserrat and body text uses Inter (verified via browser dev tools).
- **SC-002**: Homepage contains "Learning Journey Matrix" with 4 distinct module entry points.
- **SC-003**: Footer contains exactly 5 key elements: Book Title + 4 Module Links.
- **SC-004**: 0% usage of external UI frameworks (Bootstrap, Tailwind, etc.) in the new implementation.
- **SC-005**: Visual theme consistently matches Black/Gold/White palette across Hero, Matrix, and Footer.