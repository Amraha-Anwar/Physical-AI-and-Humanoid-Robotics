# Implementation Plan: Docusaurus ChatKit SDK Integration

**Branch**: `001-docusaurus-chat-sdk` | **Date**: 2025-12-07 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/001-docusaurus-chat-sdk/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a persistent, floating RAG Chat Widget in the Docusaurus frontend. The widget will reside in `src/theme/Root.js` to ensure persistence across navigation. It will utilize a separate `ChatSDK` class to manage API communication with the `/api/query` endpoint, enforcing the `RAGQueryRequest` contract. It will also capture user text selection to support context-aware querying.

## Technical Context

**Language/Version**: JavaScript (ES6+), React 17+ (Docusaurus default)
**Primary Dependencies**: `react`, `@docusaurus/core`, `clsx` (for styling)
**Storage**: `localStorage` (for session ID)
**Testing**: Manual verification (Quickstart), Jest (if available in frontend, optional for this scope)
**Target Platform**: Web (Modern Browsers)
**Project Type**: Web Application (Frontend)
**Performance Goals**: <100ms UI response for typing, standard API latency for response.
**Constraints**: Must use `/api/query` endpoint. Must look native to Docusaurus.
**Scale/Scope**: Single component, globally mounted.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **RAG Accuracy**: N/A (Frontend only, passes data through).
- [x] **Interactive UX**: Supports highlight-to-query (FR-006).
- [x] **Scalability**: Client-side implementation does not add server load beyond API calls.
- [x] **Code Integrity**: Uses SDK structure as requested.

## Project Structure

### Documentation (this feature)

```text
specs/001-docusaurus-chat-sdk/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
└── tasks.md             # Phase 2 output
```

### Source Code (repository root)

```text
frontend/
├── src/
│   ├── theme/
│   │   ├── Root.js                 # Global mount point
│   │   └── RAGChatWidget/          # New Component Directory
│   │       ├── index.js            # Main Component
│   │       ├── styles.module.css   # Styles
│   │       └── ChatSDK.js          # Logic/API Layer
```

**Structure Decision**: Swizzling `Root` (creating `src/theme/Root.js`) is the chosen approach to ensure the chat widget persists across page navigation. The widget logic and UI will be encapsulated in `src/theme/RAGChatWidget`.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| None      |            |                                     |