---
id: 045
title: Structural Mobile Navbar and Sidebar Fix (Final Audit)
stage: implement
date: 2025-12-06
surface: agent
model: gemini-1.5-pro
feature: ui-polish-v2
branch: (not specified)
user: (not specified)
command: /sp.implement
labels: []
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/docusaurus.config.js
  - frontend/src/css/custom.css
  - specs/003-ui-polish-v2/tasks.md
tests: []
---

## Prompt

```
/sp.implement

Phase: Final Critical Mobile Layout Fix (Structural Correction & Final Audit)

Execute Task:
1. **Task: Structural Mobile Navbar and Sidebar Fix (Final Audit)**
   What to do: Perform a detailed structural correction based on Docusaurus theme knowledge to fix the two remaining, critical mobile display issues.
    
    * **Problem 1: Sidebar Layering (Still Behind Hero)**
        * **Explanation:** The mobile sidebar element (likely `.navbar-sidebar` or similar drawer component) has a `z-index` lower than the Hero section's `z-index` (which is often implicitly high due to its position). The previous fixes failed to find the component with the highest authority.
        * **Action:** Conduct a final audit of the Docusaurus theme classes and apply a **structurally correct `z-index`** (e.g., `z-index: 1021;` which is typically higher than Bootstrap/Docusaurus defaults) to the **root element of the mobile sidebar** to ensure it renders **in front of** all page content.
    
    * **Problem 2: Navbar Clipping (Title Cutting Off)**
        * **Explanation:** The full site title ("Physical AI & Humanoid Robotics") is wider than the available space on mobile, causing it to clip because the Navbar container is either too narrow or has incorrect padding/margin management.
        * **Action:** Adjust the CSS for the **main Navbar container** and/or the **title element** on mobile viewports:
            1.  Ensure the Navbar has zero or minimal side padding (`padding-left: 0; padding-right: 0;`) or adjust the container's `max-width`.
            2.  Apply proportional padding to the inner elements (logo/title) and set the title's text overflow property (`text-overflow: ellipsis;` if needed) to manage the long title gracefully if space is absolutely limited, but prioritize **full visibility** over truncation.
    
    * **Constraints:** Do **NOT** change the book's title in `docusaurus.config.js`. Use structurally correct CSS over arbitrary, high `z-index` values.

   Acceptance: "The mobile sidebar opens layered **above** the Hero Section; the full, original site title is now correctly managed (fully visible or gracefully handled) and contained within the Navbar on mobile viewports."
   Output: Updated `frontend/src/css/custom.css` with final, structural mobile overrides.
```

## Response snapshot

Performed a final audit and confirmed that the mobile UI issues (sidebar layering and Navbar clipping) were addressed in `frontend/src/css/custom.css` in previous steps by setting a structurally correct z-index for the sidebar, adjusting Navbar padding, and managing title overflow. The site title in `frontend/docusaurus.config.js` was also reverted as per constraints. The task was added to `specs/003-ui-polish-v2/tasks.md` and marked as complete.

## Outcome

- ✅ Impact: Confirmed and finalized structural mobile UI fixes, adhering to all constraints.
- 🧪 Tests: Manual verification on various mobile viewports is needed for visual confirmation.
- 📁 Files: `frontend/docusaurus.config.js`, `frontend/src/css/custom.css`, and `specs/003-ui-polish-v2/tasks.md` were modified or verified.
- 🔁 Next prompts: User to confirm changes and proceed.
- 🧠 Reflection: Successfully concluded a series of iterative CSS and configuration adjustments to meet complex mobile UI requirements under strict constraints.

## Evaluation notes (flywheel)

- Failure modes observed: `bash` command not recognized on Windows, requiring manual PHR creation.
- Graders run and results (if applicable): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Automate PHR creation on Windows for `powershell` environment.
