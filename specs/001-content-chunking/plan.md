# P2.1 Content Pre-processing: Extraction and Chunking - Implementation Plan

## Overview

This plan details the development of a standalone Python script responsible for pre-processing the "Physical AI & Humanoid Robotics" book's MDX source files. The script will handle content extraction, cleaning, metadata harvesting, and context-aware chunking, ultimately producing structured data (`RagChunk` objects) ready for vectorization and database ingestion.

---

### Specification: P2.1 Content Pre-processing: Extraction and Chunking

#### 🏗️ Architecture Sketch (Data Flow)

*   **Input**: Directory of Docusaurus MDX files (Markdown content with possible React/HTML components).
*   **Parser/Cleaner**: A dedicated function to read the MDX, strip frontmatter, ignore non-text components (like `<ReactComponent />`), and normalize the remaining Markdown.
*   **Chunking Logic**: A **Recursive Character Text Splitter** to intelligently split the clean text, prioritizing structural separators (e.g., `#` then `##` then `

` then `.`).
*   **Metadata Attachment**: A function to assign the chapter and section headers to each resulting chunk, linking back to the `RagChunk` schema.
*   **Output**: A final list of `RagChunk` objects serialized as a JSON file, ready for P2.2.

#### 🧱 Section Structure (Task Breakdown)
1.  **P2.1.1 Environment Setup**: Create the processing directory (`scripts/`) and install specialized MDX parsing and text splitting libraries (e.g., `python-markdown`, `LangChain`).
2.  **P2.1.2 MDX Cleaning Function**: Implement the core function to read an MDX file, remove frontmatter, and strip Docusaurus/React components, returning pure Markdown text.
3.  **P2.1.3 Metadata Harvester**: Implement logic to traverse the MDX structure and extract the current Chapter Title and Section Heading for context preservation before chunking.
4.  **P2.1.4 Recursive Chunking Implementation**: Configure and apply a recursive text splitter. Define optimal `chunk_size` and `chunk_overlap` parameters based on LLM context windows and RAG best practices.
5.  **P2.1.5 Final Output Generation**: Implement the main script loop that processes all book files, maps the cleaned, chunked data and metadata to the Pydantic `RagChunk` model, and writes the complete structured dataset to a single output JSON file.

#### 🔬 Research Approach
*   **Optimal Chunking Strategy**: Research effective `chunk_size` and `chunk_overlap` for technical/academic texts to maximize contextual recall for the LLM.
*   **MDX Parsing**: Identify the best Python library/method for handling MDX/Markdown while gracefully ignoring embedded React components.

#### ✅ Quality Validation (Acceptance Criteria)
*   **Integrity Check**: The total character count of the input MDX files (excluding frontmatter) must approximately match the total character count of the text in the final JSON chunks.
*   **Metadata Check**: At least 95% of the chunks must have valid, non-empty `chapter_title` and `section_heading` values.
*   **Size Check**: No single chunk text exceeds a specified maximum length (e.g., 512 tokens), ensuring it fits within the context window limits.

---

### ✍️ Decisions Needing Documentation
| Decision | Options | Trade-offs | Chosen Option (Working Assumption) |
| :--- | :--- | :--- | :--- |
| **MDX Parsing Tool** | Custom Regex/String Split, `markdown-it-py`, `mistune` | Regex is fragile; specialized parsers are more robust but add dependencies. | **Use a robust Python Markdown parser (e.g., `markdown-it-py`) combined with targeted pre-cleaning** to strip frontmatter/components. |
| **Chunking Type** | Simple Fixed-Size, Sentence Splitter, Recursive Splitter | Fixed-size loses context; Sentence splitter is better but ignores structure; Recursive Splitter preserves semantic hierarchy. | **Recursive Character Text Splitter** (e.g., from LangChain library) to prioritize splitting on structural elements (`#`, `##`, `

`). |
| **Chunk Size/Overlap** | Varies (e.g., 256/50, 512/100) | Smaller chunks increase retrieval accuracy but lose context; larger chunks retain context but might pull irrelevant info. | **Chunk Size: 512 tokens (approx. 2000 characters), Overlap: 100 characters**. (Subject to validation during P2.1.4). |

### 🧪 Testing Strategy
1.  **Unit Tests (TDD)**: Create small, representative sample MDX files containing frontmatter and React components. Test the cleaning function (P2.1.2) to ensure it returns only the pure text content.
2.  **Integration Test**: Run the full script on a few chapters of the actual book. Manually review 50 random generated chunks to verify: (a) no control characters/tags remain, and (b) the associated metadata (chapter/section) is accurate.
3.  **Statistical Validation**: Check the distribution of chunk lengths to ensure they adhere to the defined size constraints (Max 512 tokens).

---

## Constitution Check

The plan for "P2.1 Content Pre-processing: Extraction and Chunking" aligns well with the project's constitution (v2.0.0).

*   **I. RAG Accuracy & Contextual Relevance**: This phase is foundational for RAG accuracy. Effective pre-processing and chunking ensure the quality of data fed into the vectorization and retrieval steps.
*   **III. Scalability & Component Reliability**: The script is designed as a standalone, offline process, which contributes to overall system reliability by offloading heavy computation from the FastAPI server. The chosen chunking strategy and tools support efficient processing.
*   **IV. Code Integrity & Tool Adherence**: The plan specifies using Python and well-regarded libraries (e.g., `markdown-it-py`, LangChain), and explicitly mandates the use of the `RagChunk` Pydantic model from `backend/models.py`, ensuring schema consistency and adherence to established project patterns.
*   **Key Standards (Chunking Strategy, Database Schema)**: The plan directly addresses the requirement for a context-aware chunking strategy and ensures metadata extraction aligns with the `RagChunk` schema.
*   **Constraints & Technology Stack**: The plan respects the constraint of building a standalone Python script, operating independently of the FastAPI server but integrating with `backend/models.py`.

No explicit violations of the constitution were identified. The technical specifics within the plan are in line with the previously clarified and accepted deviation from strict technology-agnosticism.

---
