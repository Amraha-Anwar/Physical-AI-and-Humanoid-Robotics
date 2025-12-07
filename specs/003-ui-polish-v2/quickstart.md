# Quickstart: UI Polish Verification

**Feature**: UI Polish and Structure Update (V2)

## Setup

1.  Navigate to `frontend/` directory.
2.  Install dependencies (if not already): `npm install`.
3.  Start development server: `npm start`.

## Verification Steps

### 1. Typography Check
1.  Open the homepage (`http://localhost:3000`).
2.  Inspect any Header (H1, H2). Verify font-family is **Montserrat**.
3.  Inspect any paragraph text. Verify font-family is **Inter**.

### 2. Visual Inspection
1.  **Hero Section**: Observe the "pulsing" animation on the robot image.
2.  **Divider**: Scroll down from Hero. Verify smooth transition to the cards (no "odd" gap).
3.  **Learning Matrix**: Scroll past "Master the Stack". Verify the "Learning Journey Matrix" exists with 4 cards.
4.  **Footer**: Scroll to bottom. Verify custom footer with Book Title and 4 Module links.

### 3. Navigation Test
1.  Click on a card in "Learning Journey Matrix". Verify it navigates to the correct module docs.
2.  Click on a link in the Footer. Verify it navigates to the correct module docs.
