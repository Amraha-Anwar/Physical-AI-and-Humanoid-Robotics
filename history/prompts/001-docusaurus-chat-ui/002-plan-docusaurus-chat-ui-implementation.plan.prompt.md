---
id: 002
title: Plan Docusaurus Chat UI Implementation
stage: plan
date: 2025-12-07
surface: agent
model: gemini-1.5-pro
feature: 001-docusaurus-chat-ui
branch: 001-docusaurus-chat-ui
user: user
command: /sp.plan

labels: ["plan", "docusaurus-chat-ui"]
links:
  spec: specs/001-docusaurus-chat-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-docusaurus-chat-ui/plan.md
  - specs/001-docusaurus-chat-ui/research.md
  - specs/001-docusaurus-chat-ui/data-model.md
  - specs/001-docusaurus-chat-ui/quickstart.md
  - specs/001-docusaurus-chat-ui/contracts/rag_chat_component_api.yaml
  - GEMINI.md
tests:
  - null
---

## Prompt

Create: architecture sketch, section structure, research approach, quality validation.
Decisions needing documentation: list important choices with options and tradeoffs.
Testing strategy: validation checks based on acceptance criteria.

Technical details:
- Use research-concurrent approach (research while writing, not all upfront)
- Follow APA citation style from Constitution
- Organize by phases: Research → Foundation → Analysis → Synthesis

---

### Specification: P3.1 Docusaurus Chat Component: React UI and API Hookup

#### 🏗️ Architecture Sketch (Component Integration)

* **Component Location**: Place the React component in `src/components/RAGChatComponent.tsx`.
* **Rendering**: Component is imported into a specific MDX page of the book.
* **Selection Listener**: A global listener on the document captures text selection.
* **API Handler**: Manages state, calls the FastAPI backend (ensuring CORS compliance), and handles API responses.

#### 🧱 Section Structure (Task Breakdown)
1.  **P3.1.1 React Component Setup**: Create the base `RAGChatComponent.tsx` file and set up state management for conversation history and loading status.
2.  **P3.1.2 Standard Input UI**: Design the basic chat interface, including the message display area and the text input field for standard queries.
3.  **P3.1.3 Highlight Listener Implementation**: Implement the `useEffect` hook to attach/detach a `selectionchange` event listener to `document`. This listener captures `window.getSelection().toString()`.
4.  **P3.1.4 API Handler (Fetch/Axios)**: Implement the asynchronous function to call the FastAPI `/api/rag/query` endpoint. This handler must be capable of sending **either** the `query` field **or** both the `query` and the `context_snippet`.
5.  **P3.1.5 Integration and UI/UX**: Embed the component in a test MDX file and finalize the styling to ensure a seamless fit with the Docusaurus theme.

#### 🔬 Research Approach
* **Selection API**: Confirm the reliable method for capturing text selection across major browsers using `document.addEventListener('selectionchange', ...)` and `window.getSelection()`. (Confirmed feasible based on search results).
* **CORS Handling**: Research Docusaurus proxy settings or necessary FastAPI CORS configurations required for local development and final deployment.

#### ✅ Quality Validation (Acceptance Criteria)
* **Functional Test (Mode 1)**: Enter text in the input box and send. Verify the request hits the FastAPI endpoint *without* the `context_snippet` parameter.
* **Functional Test (Mode 2)**: Highlight text on the page, then enter a query and send. Verify the request hits the FastAPI endpoint *with* the highlighted text correctly populated in the `context_snippet` parameter.
* **UI/UX**: The component maintains conversation state and displays messages clearly, including a loading indicator during API calls.

---

### ✍️ Decisions Needing Documentation
| Decision | Options | Trade-offs | Chosen Option (Working Assumption) |
| :--- | :--- | :--- | :--- |
| **API Client** | Native `fetch`, `axios` | `fetch` is native and lightweight; `axios` offers better error handling and interceptors. | **Native `fetch`** for minimal dependency overhead in the Docusaurus environment. |
| **Selection Trigger** | Dedicated "Ask about Selection" button, Input field automatically uses selection | Button is explicit but adds clicks; Automatic use simplifies the UX but requires clear feedback. | **Automatic Input Use**: If text is highlighted when the user sends a query from the input, the highlight is automatically sent as the `context_snippet`. |
| **Styling Method** | Global CSS, CSS Modules, Tailwind (if set up) | CSS Modules provide local scope and prevent collision, which is best practice for Docusaurus components. | **CSS Modules** (`styles.module.css`) co-located with the component. |

### 🧪 Testing Strategy
1.  **Unit Test (Selection)**: Mock the DOM's `window.getSelection()` object to return a test string and verify the component's state correctly captures it on a simulated `selectionchange` event.
2.  **Integration Test (CORS)**: Run the Docusaurus dev server and the FastAPI dev server concurrently. Test the API call to ensure no CORS or network errors occur.
3.  **E2E Test**: Manually perform both Mode 1 (General Query) and Mode 2 (Highlight Query) interaction flows to confirm the full RAG cycle is triggered correctly by the frontend.

---

## Constitution Check

The plan for "P3.1 Docusaurus Chat Component: React UI and API Hookup" aligns exceptionally well with the project's constitution (v2.0.0).

*   **I. RAG Accuracy & Contextual Relevance**: The component directly interfaces with the backend RAG endpoint (P2.3), ensuring that user queries result in contextual answers derived from the book's content.
*   **II. Interactive UX & Selection Integrity**: This phase directly implements the core requirements for the user interface. The chat component provides the "Interactive UX," and the "Highlight Listener" with its `context_snippet` prioritization explicitly addresses the "Selection Integrity" mandate, allowing users to ask questions directly about selected text.
*   **III. Scalability & Component Reliability**: The component leverages standard React development practices and native web APIs (`fetch`), minimizing frontend complexity and ensuring it integrates smoothly with the scalable FastAPI backend. Frontend component itself will be relatively lightweight.
*   **IV. Code Integrity & Tool Adherence**: The plan adheres to using **React/TypeScript** and standard web APIs, which are common and established technologies. The decision to use **CSS Modules** aligns with Docusaurus best practices for styling, preventing conflicts and promoting maintainability. The component's placement in the appropriate Docusaurus components directory ensures proper integration.

**Key Standards**:
*   **API Security**: The plan explicitly includes research into **CORS Handling** and verification via integration tests, addressing the security implications of cross-origin requests between the frontend and backend.
*   **Deployment**: The component is designed for **Docusaurus compatibility**, which targets a static site generation and aligns with lightweight, cost-effective serverless deployment environments.

No explicit violations of the constitution were identified. The plan effectively translates the constitutional principles and project requirements into a concrete frontend implementation strategy.

---
