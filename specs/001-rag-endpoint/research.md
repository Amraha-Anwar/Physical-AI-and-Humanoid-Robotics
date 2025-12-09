# P2.3 Retrieval & Agent Logic: Core RAG FastAPI Endpoint - Research Findings

This document captures key research findings and decisions made during the planning phase for the core RAG FastAPI endpoint.

## Decisions Documented

| Decision | Options | Trade-offs | Chosen Option (Working Assumption) |
| :--- | :--- | :--- | :--- |
| **LLM Model** | `gpt-3.5-turbo`, `gpt-4-turbo` | `gpt-3.5-turbo` is faster/cheaper; `gpt-4-turbo` is better at complex reasoning and instruction following (better for RAG). | **`gpt-4-turbo`** for higher quality, context-aware RAG responses, adhering to the quality focus. |
| **Qdrant Retrieval Strategy** | Simple Top-k Search, Contextual Re-ranking | Simple Top-k is fast; Re-ranking is more accurate but adds latency. | **Simple Top-N (N=5) Search** for initial low-latency implementation, as quality relies more on chunking (P2.1) and prompt (P2.3.4). |
| **Highlight Implementation** | Bypass RAG, Include in RAG | Bypassing is simpler/faster but requires dedicated prompt; Including maintains one prompt structure. | **Router/Bypass Logic**: If `context_snippet` is present, it directly becomes the *only* context, bypassing the Qdrant search entirely. |
