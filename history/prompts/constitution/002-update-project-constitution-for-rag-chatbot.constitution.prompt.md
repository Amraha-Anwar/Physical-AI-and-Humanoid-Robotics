---
id: 002
title: Update Project Constitution for RAG Chatbot
stage: constitution
date: 2025-12-07
surface: agent
model: gemini-1.5-pro
feature: none
branch: main
user: user
command: /sp.constitution

labels: ["constitution", "rag", "chatbot", "principles", "standards"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

Project: **Integrated RAG Chatbot Development for "Physical AI & Humanoid Robotics" Book**

Core principles:
- **I. RAG Accuracy & Contextual Relevance**: The chatbot must retrieve and generate answers *exclusively* based on the content of the "Physical AI & Humanoid Robotics" book. Responses must be technically accurate and directly traceable to the source text.
- **II. Interactive UX & Selection Integrity**: The chatbot interface must seamlessly integrate into the Docusaurus site. It must robustly support the *specific* requirement of answering questions based *only* on a user-selected text snippet (highlight-to-query).
- **III. Scalability & Component Reliability**: All components (FastAPI, Neon/Postgres, Qdrant) must be configured for serverless-compatible, low-latency, and high-availability operation, utilizing the free-tier constraints effectively.
- **IV. Code Integrity & Tool Adherence**: All backend code (Python/FastAPI) and vector processing logic must adhere to best practices for the specified OpenAI Assistants/ChatKit SDKs, and cleanly manage the interaction between the Vector DB (Qdrant) and the relational DB (Neon).

Key standards:
- **Embedding Model**: Use a standard, high-performance OpenAI embedding model (e.g., `text-embedding-3-small`).
- **Chunking Strategy**: Implement a context-aware chunking strategy (e.g., recursive text splitting) to optimize RAG retrieval performance.
- **API Security**: Implement basic API key management for the FastAPI server (e.g., API key, rate limiting).
- **Deployment**: Final deployment must be suitable for a lightweight, cost-effective serverless environment (e.g., Vercel, Render, or similar for FastAPI).
- **Database Schema**: A minimal schema in Neon Postgres must be defined to manage metadata (e.g., document ID, chapter title, chunk ID).

Constraints & Technology Stack (Additions to existing stack):
- **Core Technology**: Retrieval-Augmented Generation (RAG).
- **LLM/Agents**: OpenAI Agents/ChatKit SDKs.
- **Backend/API**: **FastAPI** (Python).
- **Relational/Metadata DB**: **Neon Serverless Postgres**.
- **Vector DB**: **Qdrant Cloud Free Tier**.
- **Frontend Integration**: JavaScript/React component embedded in the existing Docusaurus site.

Success criteria:
- The RAG pipeline is successfully built, deployed, and functional.
- The chatbot can accurately answer general questions about the book's content.
- The chatbot correctly processes and responds to questions based *only* on a specific user-highlighted text block.
- All specified mandatory technologies are successfully integrated and operational under their respective free/serverless constraints.

## Response snapshot

Updated the project constitution (`.specify/memory/constitution.md`) to reflect the new focus on RAG Chatbot development for the 'Physical AI & Humanoid Robotics' book. This included updating the core principles, adding new sections for key standards and success criteria, and incrementing the constitution version to 2.0.0.

## Outcome

- ✅ Impact: Project constitution successfully updated with new RAG chatbot focus.
- 🧪 Tests: No automated tests were run for this documentation update.
- 📁 Files: One file was modified: `.specify/memory/constitution.md`.
- 🔁 Next prompts: The user can now proceed with further development based on the updated constitution.
- 🧠 Reflection: The process successfully updated the constitution, but the bash script for PHR creation failed. Implemented PHR creation with agent-native tools.

## Evaluation notes (flywheel)

- Failure modes observed: Bash script for PHR creation failed in PowerShell environment.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
