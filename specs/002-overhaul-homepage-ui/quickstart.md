# Quickstart: Testing the UI Overhaul

**Feature**: Docusaurus Home Page UI Overhaul

## Prerequisites
- Node.js 18+
- `frontend/` directory as the working root for commands.

## Setup

1.  **Navigate to frontend**:
    ```bash
    cd frontend
    ```

2.  **Install Dependencies**:
    ```bash
    npm install
    ```
    *Note: This will install the new `tailwindcss`, `postcss`, and `autoprefixer` packages defined in `package.json`.*

## Running the Dev Server

1.  **Start Docusaurus**:
    ```bash
    npm start
    ```
2.  **Open Browser**:
    Navigate to `http://localhost:3000`

## Verification Steps

1.  **Visual Check**:
    - Confirm background is Deep Charcoal (`#1A1A2E`).
    - Confirm Hero Image (`hero.png`) is visible.
    - Confirm Headlines use "Playfair Display" (Serif).

2.  **Tailwind Validation**:
    - Inspect a Module Card.
    - Verify it has Tailwind utility classes (e.g., `rounded-xl`, `hover:border-gold`).
    - Verify the computed styles match the Tailwind utilities.

3.  **Responsiveness**:
    - Resize browser to mobile width (< 640px).
    - Confirm cards stack vertically.
    - Confirm navigation hamburger menu works and uses Gold accents.
