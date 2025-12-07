# P2.1 Content Pre-processing: Extraction and Chunking - Tasks

**Feature Branch**: `001-content-chunking`  
**Goal**: Develop a robust, standalone Python script to extract, clean, and chunk the book's MDX content, associating necessary metadata for RAG ingestion.

---

## Phase 1: Setup

*   - [ ] T001 Create `scripts/` directory at the project root.
*   - [ ] T002 Create `scripts/content_processor/` directory.
*   - [ ] T003 Initialize poetry in `scripts/content_processor/`: `poetry init --no-interaction`.
*   - [ ] T004 Install Python dependencies in `scripts/content_processor/`: `poetry add python-markdown-it markdown-it-py linkify-it-py mdit-py-plugins python-frontmatter langchain-text-splitters pydantic`.

## Phase 2: User Story 1 - Process MDX Files and Extract Raw Text [US1]

**Goal**: Reliably parse Docusaurus MDX files, ignoring irrelevant components and frontmatter, to obtain clean raw text.
**Independent Test**: Running the script with a sample MDX file and verifying the output raw text contains only the expected content.

*   - [ ] T005 [US1] Create `scripts/content_processor/mdx_parser.py` file.
*   - [ ] T006 [US1] Implement a function `strip_frontmatter(mdx_content: str)` in `scripts/content_processor/mdx_parser.py` to remove YAML frontmatter.
*   - [ ] T007 [US1] Implement a function `strip_react_html_components(markdown_content: str)` in `scripts/content_processor/mdx_parser.py` to remove embedded React/HTML components.
*   - [ ] T008 [US1] Implement `extract_raw_markdown(mdx_filepath: Path)` function in `scripts/content_processor/mdx_parser.py` that orchestrates frontmatter stripping and component removal.

## Phase 3: User Story 3 - Extract and Assign Metadata [US3]

**Goal**: Automatically extract relevant metadata such as chapter title and section headings and assign it to each generated chunk, directly aligning with the `RagChunk` schema.
**Independent Test**: Running the script and verifying that each generated chunk object includes correct `chunk_id`, `chapter_title`, and `section_heading` (or similar derived metadata).

*   - [ ] T009 [P] [US3] Create `scripts/content_processor/metadata_extractor.py` file.
*   - [ ] T010 [P] [US3] Implement `get_document_id(mdx_filepath: Path)` in `scripts/content_processor/metadata_extractor.py` to generate a unique ID for each MDX document.
*   - [ ] T011 [P] [US3] Implement `get_chapter_title(mdx_filepath: Path, raw_markdown: str)` in `scripts/content_processor/metadata_extractor.py` to extract chapter title (from frontmatter or first heading).
*   - [ ] T012 [P] [US3] Implement a mechanism `track_section_heading(markdown_parser_event)` in `scripts/content_processor/metadata_extractor.py` to track the current section heading during parsing.
*   - [ ] T013 [P] [US3] Ensure `backend/models.py` is accessible by adding `sys.path.append('../../backend')` or equivalent in `scripts/content_processor/metadata_extractor.py` or main script.

## Phase 4: User Story 2 - Chunk Content with Context Preservation [US2]

**Goal**: Segment the extracted raw text into meaningful chunks that preserve logical context.
**Independent Test**: Running the script and inspecting the generated chunks to ensure they are logically coherent and of appropriate size.

*   - [ ] T014 [US2] Create `scripts/content_processor/chunking_strategy.py` file.
*   - [ ] T015 [US2] Implement `get_text_splitter()` function in `scripts/content_processor/chunking_strategy.py` that returns a configured `RecursiveCharacterTextSplitter` (chunk size 2000 chars, overlap 100 chars, with custom separators).
*   - [ ] T016 [US2] Implement `chunk_text(raw_text: str, splitter)` function in `scripts/content_processor/chunking_strategy.py` to apply the splitter and generate text chunks.

## Phase 5: User Story 4 - Output Structured Data for Vectorization [US4]

**Goal**: Output the processed and chunked content, along with its metadata, in a clean, structured JSON format.
**Independent Test**: Running the script and inspecting the final output file to ensure it is valid JSON and contains all expected fields for each chunk.

*   - [ ] T017 [US4] Create `scripts/content_processor/main_processor.py` file.
*   - [ ] T018 [US4] Implement a main processing loop in `scripts/content_processor/main_processor.py` that:
    *   Finds all MDX files in `frontend/docs/`.
    *   For each file, extracts raw markdown using `mdx_parser.py`.
    *   Extracts metadata using `metadata_extractor.py`.
    *   Chunks the raw markdown using `chunking_strategy.py`.
    *   For each chunk, creates a `RagChunk` Pydantic object, populating all fields including `chunk_id` (e.g., UUID.uuid4() or hash).
*   - [ ] T019 [US4] Implement logic in `scripts/content_processor/main_processor.py` to serialize the list of `RagChunk` objects to a JSON file (e.g., `processed_rag_chunks.json`).

## Phase 6: Polish & Cross-Cutting Concerns

*   - [ ] T020 Add comprehensive docstrings and type hints to all functions and classes in `scripts/content_processor/`.
*   - [ ] T021 Implement basic command-line argument parsing for input directory and output file in `scripts/content_processor/main_processor.py`.
*   - [ ] T022 Implement error handling for file I/O operations and parsing errors in `scripts/content_processor/`.
*   - [ ] T023 Create `scripts/content_processor/sample.mdx` for unit/integration testing.
*   - [ ] T024 Create `scripts/content_processor/test_processor.py` for unit tests for MDX cleaning, metadata extraction, and chunking.
*   - [ ] T025 Update `pyproject.toml` in `scripts/content_processor/` to include `pytest` for testing.

---

## Dependencies

This section outlines the completion order of user stories.

1.  **User Story 1 (Process MDX Files and Extract Raw Text)** -> **User Story 3 (Extract and Assign Metadata)** -> **User Story 2 (Chunk Content with Context Preservation)** -> **User Story 4 (Output Structured Data for Vectorization)**

## Parallel Execution Opportunities

*   **Phase 2: User Story 1 (Process MDX Files and Extract Raw Text)** and **Phase 3: User Story 3 (Extract and Assign Metadata)**: These phases can be developed somewhat in parallel once the initial MDX parsing is functional. Metadata extraction can leverage the same parsing libraries.
*   **Within Phase 3 (Metadata Extraction)**: Tasks T009-T012 can be developed in parallel, focusing on different aspects of metadata harvesting.

## Implementation Strategy

The implementation will follow an iterative approach, prioritizing core content extraction and cleaning, followed by metadata enrichment, then chunking, and finally output generation.
1.  Set up the isolated script environment and install necessary libraries.
2.  Implement robust MDX parsing and cleaning to get raw text.
3.  Develop metadata extraction logic to correctly identify chapter and section information.
4.  Integrate the chunking strategy to segment the cleaned text while preserving context.
5.  Orchestrate the entire process in a main script, mapping to the `RagChunk` model and outputting structured JSON.
6.  Implement comprehensive testing and polish.
