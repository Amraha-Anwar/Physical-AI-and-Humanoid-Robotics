# P3.1 Docusaurus Chat Component: React UI and API Hookup - Research Findings

This document captures key research findings and decisions made during the planning phase for the Docusaurus chat component.

## Decisions Documented

| Decision | Options | Trade-offs | Chosen Option (Working Assumption) |
| :--- | :--- | :--- | :--- |
| **API Client** | Native `fetch`, `axios` | `fetch` is native and lightweight; `axios` offers better error handling and interceptors. | **Native `fetch`** for minimal dependency overhead in the Docusaurus environment. |
| **Selection Trigger** | Dedicated "Ask about Selection" button, Input field automatically uses selection | Button is explicit but adds clicks; Automatic use simplifies the UX but requires clear feedback. | **Automatic Input Use**: If text is highlighted when the user sends a query from the input, the highlight is automatically sent as the `context_snippet`. |
| **Styling Method** | Global CSS, CSS Modules, Tailwind (if set up) | CSS Modules provide local scope and prevent collision, which is best practice for Docusaurus components. | **CSS Modules** (`styles.module.css`) co-located with the component. |
