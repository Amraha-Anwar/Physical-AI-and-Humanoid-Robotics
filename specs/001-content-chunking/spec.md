# Feature Specification: P2.1 Content Pre-processing: Extraction and Chunking

**Feature Branch**: `001-content-chunking`  
**Created**: 2025-12-07  
**Status**: Draft  
**Input**: User description: "P2.1 Content Pre-processing: Extraction and Chunking Target module: P2.1 Content Pre-processing Focus: Develop a standalone Python script to read the Docusaurus-compatible MDX source files of the book, clean the content, apply an effective chunking strategy, and output the results in a structured format suitable for vectorization. Success criteria: - **MDX Handling**: The script successfully parses and extracts raw text from multiple MDX files, ignoring Docusaurus-specific frontmatter and React/HTML components. - **Context Preservation**: The chunking strategy (e.g., Recursive Character Text Splitting) retains logical context, ensuring chunks are small enough for embedding but large enough to answer questions. - **Metadata Extraction**: The script accurately extracts and assigns metadata (Chapter Title, Section Heading) to each generated chunk, directly aligning with the `RagChunk` schema defined in P1.3. - **Output Format**: The final processed data is output as a clean, structured JSON file (or list of objects) ready for ingestion by the vectorization script (P2.2). Constraints: - **Input Source**: Must handle `.mdx` files with varying levels of Markdown complexity (headings, lists, code blocks). - **Tooling**: Use common, non-proprietary Python libraries for text processing (e.g., LangChain TextSplitters, Markdown parsers). - **Environment**: Script must be executable independently of the FastAPI server but utilize the Pydantic models from `backend/models.py`. Not building: - The actual vectorization or Qdrant/Neon insertion logic (P2.2). - Any API endpoint in FastAPI; this is a preparatory data processing script."

## User Scenarios & Testing

### User Story 1 - Process MDX Files and Extract Raw Text (Priority: P1)
As a data engineer, I want to reliably parse Docusaurus MDX files, ignoring irrelevant components and frontmatter, to obtain clean raw text, so that the content is ready for chunking and metadata extraction.
**Why this priority**: This is the fundamental first step in processing the book content.
**Independent Test**: Running the script with a sample MDX file and verifying the output raw text contains only the expected content.
**Acceptance Scenarios**:
1.  **Given** a valid `.mdx` file containing Docusaurus frontmatter and React/HTML components, **When** the content pre-processing script processes this file, **Then** the script successfully extracts only the raw text, excluding frontmatter and component markup.

### User Story 2 - Chunk Content with Context Preservation (Priority: P1)
As a data engineer, I want to segment the extracted raw text into meaningful chunks that preserve logical context, so that each chunk is suitable for embedding and can be used to answer questions effectively.
**Why this priority**: Effective chunking is crucial for RAG system performance and accuracy.
**Independent Test**: Running the script and inspecting the generated chunks to ensure they are logically coherent and of appropriate size.
**Acceptance Scenarios**:
1.  **Given** clean raw text extracted from an MDX file, **When** the content pre-processing script applies its chunking strategy, **Then** the text is broken into chunks such that related sentences and paragraphs remain together, and individual chunks are neither too long nor too short (e.g., within a specified token/character range).

### User Story 3 - Extract and Assign Metadata (Priority: P1)
As a data engineer, I want to automatically extract relevant metadata such as chapter title and section headings and assign it to each generated chunk, directly aligning with the `RagChunk` schema, so that the chunks can be properly contextualized and indexed.
**Why this priority**: Accurate metadata is essential for the `RagChunk` schema and for effective retrieval.
**Independent Test**: Running the script and verifying that each generated chunk object includes correct `chunk_id`, `chapter_title`, and `section_heading` (or similar derived metadata).
**Acceptance Scenarios**:
1.  **Given** processed chunks of text, **When** the content pre-processing script assigns metadata, **Then** each chunk object includes a unique `chunk_id`, the correct `chapter_title`, and any relevant `section_heading` derived from the document structure, matching the `RagChunk` schema.

### User Story 4 - Output Structured Data for Vectorization (Priority: P2)
As a data engineer, I want the processed and chunked content, along with its metadata, to be output in a clean, structured JSON format, so that it can be easily consumed by the subsequent vectorization script (P2.2).
**Why this priority**: Facilitates seamless integration with the next stage of the RAG pipeline.
**Independent Test**: Running the script and inspecting the final output file to ensure it is valid JSON and contains all expected fields for each chunk.
**Acceptance Scenarios**:
1.  **Given** fully processed and chunked content with assigned metadata, **When** the script completes its execution, **Then** a structured JSON file (or similar iterable structure) is generated, where each element represents a `RagChunk` object fully compatible with the `RagChunk` Pydantic model.

---

### Edge Cases

-   What happens if an MDX file is malformed or contains unexpected syntax? (The script should gracefully handle parsing errors, potentially skipping the file or logging warnings, without crashing.)
-   How does the script handle very short MDX files (e.g., only frontmatter, or very little content)? (The script should still attempt to process them and produce either an empty or minimal set of chunks with appropriate metadata.)
-   What if a document has no explicit chapter title or section headings? (The script should use a fallback mechanism, e.g., the filename or a default value for the chapter title, and omit section headings.)
-   How does the script manage very long lines or code blocks during chunking? (The chunking strategy should ensure long lines/code blocks are handled without creating excessively large or truncated chunks, potentially by splitting code blocks or treating them as distinct units.)
-   What if the output directory is not writable? (The script should report an error and exit gracefully.)

## Requirements

### Functional Requirements

-   **FR-001**: The script MUST read `.mdx` files from a specified input directory (e.g., `frontend/docs/`).
-   **FR-002**: The script MUST parse `.mdx` content to extract raw text, ignoring Docusaurus-specific frontmatter (YAML block at the top) and embedded React/HTML components (e.g., `<p>...</p>`, `<HeroSection />`).
-   **FR-003**: The script MUST implement a context-aware chunking strategy (e.g., Recursive Character Text Splitting) to segment the extracted text.
-   **FR-004**: The chunking strategy MUST ensure chunks are within configurable size limits (e.g., character count or token count) while attempting to preserve semantic boundaries (e.g., not splitting sentences or paragraphs unnaturally).
-   **FR-005**: The script MUST extract metadata (e.g., Chapter Title from frontmatter or first heading, Section Heading from Markdown headings) from the `.mdx` files.
-   **FR-006**: The script MUST assign the extracted metadata, along with a unique `chunk_id` (UUID), to each generated text chunk, aligning with the `RagChunk` Pydantic model from `backend/models.py`.
-   **FR-007**: The script MUST output the processed data as a list of `RagChunk` objects (or a JSON representation thereof) to a specified output file.
-   **FR-008**: The script MUST be runnable as a standalone Python application from the project root.
-   **FR-009**: The script MUST import and utilize the `RagChunk` Pydantic model from `backend/models.py` to ensure schema consistency.
-   **FR-010**: The script MUST use common, non-proprietary Python libraries for text processing (e.g., `markdown-it-py` for parsing, LangChain `RecursiveCharacterTextSplitter` for chunking).

### Key Entities

-   **MDX Document**: The input source file (Docusaurus-compatible Markdown with JSX) from the `frontend/docs/` directory.
-   **Raw Text**: The clean text content extracted from MDX files, stripped of non-textual elements, ready for chunking.
-   **Text Chunk**: A segment of the raw text, generated by the chunking strategy, intended for vectorization.
-   **Metadata**: Structured information associated with each Text Chunk (e.g., `chapter_title`, `section_heading`, `document_id`, `page_number`).
-   **RagChunk Object**: A Pydantic model instance (`backend/models.py.RagChunk`) encapsulating a Text Chunk and its Metadata, ready for output as a list.

## Success Criteria

### Measurable Outcomes

-   **SC-001**: The script successfully processes all `.mdx` files in the `frontend/docs/` directory without crashing or reporting unhandled exceptions.
-   **SC-002**: For 95% of processed `.mdx` files, the extracted raw text correctly excludes Docusaurus frontmatter and React/HTML components (verified by automated parsing tests or manual spot checks).
-   **SC-003**: The average length of generated text chunks falls within a predefined range (e.g., 200-500 characters) with a standard deviation below a specified threshold (e.g., 100 characters), indicating consistent chunk sizing.
-   **SC-004**: For 98% of generated `RagChunk` objects, the `chapter_title` and any detected `section_heading` metadata are accurately extracted and assigned, matching the original MDX document structure.
-   **SC-005**: The script outputs a valid JSON file (`output.json`) where each entry is a valid JSON object conforming to the `RagChunk` Pydantic model schema.
-   **SC-006**: The script completes processing of all `.mdx` files in `frontend/docs/` within a reasonable time frame (e.g., under 30 seconds for 50 average-sized documents).