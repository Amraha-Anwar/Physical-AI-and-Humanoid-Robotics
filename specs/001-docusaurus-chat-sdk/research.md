# Research: Docusaurus ChatKit SDK Integration

**Feature**: `001-docusaurus-chat-sdk`
**Status**: Completed

## 1. Component Placement & Persistence
**Question**: Where should the persistent chat widget live in the Docusaurus React tree?
**Options**:
- Option A: Swizzle `Layout`
- Option B: Swizzle `Root`
- Option C: Add to every markdown page manually (rejected)

**Decision**: **Option B (Swizzle `Root`)**
**Rationale**: The `Root` component sits at the top of the React tree and *never unmounts* during client-side navigation. This is critical for maintaining chat state (open/closed, message history) as the user navigates between documentation pages. `Layout` re-mounts on navigation, which would reset the chat state.

## 2. SDK Structure
**Question**: How to implement the "ChatKit-like" SDK?
**Decision**: Create a dedicated class/module `ChatService` in `src/services/ChatService.js` (or `src/theme/RAGChatWidget/ChatService.js` to keep it self-contained).
**Rationale**: Encapsulating the API logic separates concerns. The React component handles UI, the Service handles `fetch`, `localStorage` (session), and state logic.

## 3. Text Selection
**Question**: How to capture text selection reliably?
**Decision**: Use `document.getSelection().toString()` inside a `mouseup` event listener attached to `window` or the specific documentation container.
**Rationale**: Standard DOM API is sufficient. We must ensure it runs only in the browser (`ExecutionEnvironment.canUseDOM`) to avoid SSR build errors.

## 4. API Routing
**Question**: How to route to FastAPI?
**Decision**: Use `fetch('/api/query', ...)` directly.
**Rationale**: Docusaurus is served on `localhost:3000`, FastAPI on `localhost:8000`. We will need a proxy setup in `docusaurus.config.js` or standard CORS handling. *Assumption*: The prompt implies we are just hitting the endpoint. A proxy in Docusaurus (via Webpack dev server config) is cleaner to avoid CORS issues in dev.
*Constraint Check*: The prompt says "route... to the custom FastAPI RAG endpoint".

## 5. Styling
**Decision**: CSS Modules (`styles.module.css`) co-located with the component.
**Rationale**: Standard Docusaurus practice for component-scoped styling.
