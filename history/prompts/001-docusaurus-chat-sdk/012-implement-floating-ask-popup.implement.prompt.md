---
id: 012
title: implement-floating-ask-popup
stage: implement
date: 2025-12-07
surface: agent
model: gemini-2.0-flash-exp
feature: 001-docusaurus-chat-sdk
branch: 001-docusaurus-chat-sdk
user: user
command: /sp.implement
labels: ["feature", "frontend", "ux"]
links:
  spec: specs/001-docusaurus-chat-sdk/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - frontend/src/theme/RAGChatWidget/index.js
 - frontend/src/theme/RAGChatWidget/styles.module.css
 - specs/001-docusaurus-chat-sdk/tasks.md
tests:
 - none
---

## Prompt

T067 Implement Floating 'Ask' Popup for Text Selection

Goal: Enhance the Context-Aware Querying feature by implementing a persistent, floating "Ask" popup that appears near user-selected text, offering a direct UX flow for context-based questioning as per the user's design requirement.

Execute:
- T067.1: Modify RAGChatWidget to manage the state and screen coordinates (x, y) of a floating 'Ask' button element.
- T067.2: Implement improved logic in the mouseup event listener to calculate the position of the selected text and show the floating button only when a non-empty selection is made.
- T067.3: Implement the click handler for the 'Ask' button. This handler must first open the chatbot widget, then pass the captured `selected_context` to the ChatSDK, and immediately trigger the sending of the message to the backend.
- T067.4: Add a visual cue in the chatbot's input field once a selection is captured, allowing the user to type a question or hit send directly.

## Response snapshot

Completed all sub-tasks for T067:
- T067.1: Managed state and coordinates for the floating "Ask" button in `index.js`.
- T067.2: Implemented logic in `mouseup` event listener for position and visibility of the button.
- T067.3: Implemented click handler for "Ask" button to open chat, send context.
- T067.4: Utilized existing `selectedContextIndicator` as visual cue.

## Outcome

- ✅ Impact: Implemented the floating "Ask" popup for enhanced context-aware querying UX.
- 🧪 Tests: Manual verification required for visual and functional aspects.
- 📁 Files: Modified `frontend/src/theme/RAGChatWidget/index.js`, `frontend/src/theme/RAGChatWidget/styles.module.css`, and `specs/001-docusaurus-chat-sdk/tasks.md`.
- 🔁 Next prompts: Verify the new functionality.
- 🧠 Reflection: Successfully added a significant UX improvement for context-based interactions.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): N/A
