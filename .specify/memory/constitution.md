<!--
SYNC IMPACT REPORT
==================
Version: 1.0.0 -> 2.0.0
Project Name: "Physical AI & Humanoid Robotics" Book -> Integrated RAG Chatbot Development for "Physical AI & Humanoid Robotics" Book
Principles Modified:
- I. Technical Accuracy & Integrity -> I. RAG Accuracy & Contextual Relevance
- II. Step-by-Step Pedagogy -> II. Interactive UX & Selection Integrity
- III. Consistency & Tone -> III. Scalability & Component Reliability
- IV. Format & Standardization -> IV. Code Integrity & Tool Adherence
- V. Code Integrity -> Removed (Integrated into new principles)
Added Sections:
- Key Standards
- Success Criteria
Removed Sections:
- Development Workflow
Templates Checked:
- .specify/templates/plan-template.md (✅ No changes needed)
- .specify/templates/spec-template.md (✅ No changes needed)
- .specify/templates/tasks-template.md (✅ No changes needed)
-->

# Integrated RAG Chatbot Development for "Physical AI & Humanoid Robotics" Book Constitution

## Core Principles

### I. RAG Accuracy & Contextual Relevance
The chatbot must retrieve and generate answers *exclusively* based on the content of the "Physical AI & Humanoid Robotics" book. Responses must be technically accurate and directly traceable to the source text. This is critical to maintaining the educational integrity and reliability of the information provided.

### II. Interactive UX & Selection Integrity
The chatbot interface must seamlessly integrate into the Docusaurus site. It must robustly support the *specific* requirement of answering questions based *only* on a user-selected text snippet (highlight-to-query). The user experience should be intuitive, and the selection mechanism must be precise and reliable.

### III. Scalability & Component Reliability
All components (FastAPI, Neon/Postgres, Qdrant) must be configured for serverless-compatible, low-latency, and high-availability operation, utilizing the free-tier constraints effectively. This ensures the chatbot remains responsive and accessible as usage grows without incurring prohibitive costs.

### IV. Code Integrity & Tool Adherence
All backend code (Python/FastAPI) and vector processing logic must adhere to best practices for the specified OpenAI Assistants/ChatKit SDKs, and cleanly manage the interaction between the Vector DB (Qdrant) and the relational DB (Neon). Code must be well-documented, maintainable, and follow idiomatic conventions for each technology.

## Key Standards

### Embedding Model
Use a standard, high-performance OpenAI embedding model (e.g., `text-embedding-3-small`). This choice is crucial for ensuring the quality and relevance of vector embeddings used in RAG.

### Chunking Strategy
Implement a context-aware chunking strategy (e.g., recursive text splitting) to optimize RAG retrieval performance. Effective chunking is vital for maximizing the accuracy and relevance of retrieved document segments.

### API Security
Implement basic API key management for the FastAPI server (e.g., API key, rate limiting). This provides essential protection against unauthorized access and abuse, even within a serverless environment.

### Deployment
Final deployment must be suitable for a lightweight, cost-effective serverless environment (e.g., Vercel, Render, or similar for FastAPI). This aligns with the scalability and reliability principles, ensuring efficient resource utilization.

### Database Schema
A minimal schema in Neon Postgres must be defined to manage metadata (e.g., document ID, chapter title, chunk ID). This metadata is essential for organizing, querying, and maintaining the integrity of the RAG system's data sources.

## Constraints & Technology Stack

### Core Technology
Retrieval-Augmented Generation (RAG).

### LLM/Agents
OpenAI Agents/ChatKit SDKs.

### Backend/API
**FastAPI** (Python).

### Relational/Metadata DB
**Neon Serverless Postgres**.

### Vector DB
**Qdrant Cloud Free Tier**.

### Frontend Integration
JavaScript/React component embedded in the existing Docusaurus site.

## Success Criteria

- The RAG pipeline is successfully built, deployed, and functional.
- The chatbot can accurately answer general questions about the book's content.
- The chatbot correctly processes and responds to questions based *only* on a specific user-highlighted text block.
- All specified mandatory technologies are successfully integrated and operational under their respective free/serverless constraints.

## Governance

**Version**: 2.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07