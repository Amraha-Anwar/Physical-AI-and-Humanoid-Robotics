# P2.1 Content Pre-processing: Extraction and Chunking - Data Model

This document outlines the data flow and conceptual models involved in the content pre-processing, extraction, and chunking script. The primary output is a collection of `RagChunk` objects, which aligns with the schema defined in P1.3.

## 1. Input Data Model

**Entity**: `MDX Document`

**Description**: The source of the book content. These are Docusaurus-compatible Markdown files that may contain embedded JSX (React components) and YAML frontmatter.

**Key Characteristics**:
*   File format: `.mdx`
*   Location: `frontend/docs/` directory.
*   Content: Markdown, JSX, YAML frontmatter.

## 2. Intermediate Data Models

### 2.1 Raw Text

**Entity**: `Raw Text`

**Description**: The clean textual content derived from an `MDX Document` after stripping all non-textual elements (frontmatter, JSX components, Docusaurus-specific tags). This is the input for the chunking process.

**Key Characteristics**:
*   Pure text content.
*   Retains Markdown formatting (e.g., headings, lists) relevant for chunking.

### 2.2 Metadata

**Entity**: `Metadata`

**Description**: Structured information extracted from the `MDX Document` and associated with individual `Text Chunks`. This metadata provides context for each chunk.

**Key Attributes**:
*   `chapter_title` (String): Derived from the document's structure (e.g., frontmatter or main heading).
*   `section_heading` (String, Optional): Derived from Markdown headings within the document.
*   `document_id` (String): A unique identifier for the source MDX file/document.
*   `page_number` (Integer, Optional): Conceptual page number if applicable, or derived from chunk order.

## 3. Output Data Model

**Entity**: `Text Chunk`

**Description**: A segment of `Raw Text` that has been intelligently split to preserve contextual meaning. Each `Text Chunk` is combined with its `Metadata` to form a `RagChunk` object.

**Key Characteristics**:
*   Optimized for embedding (within size limits).
*   Retains semantic integrity.

**Entity**: `RagChunk Object`

**Description**: The final structured output for each processed text segment. This entity directly corresponds to the `RagChunk` Pydantic model defined in `backend/models.py` (from P1.3).

**Attributes (as per `backend/models.py.RagChunk`)**:
*   `chunk_id` (str): Unique identifier for the text chunk (e.g., UUID).
*   `document_id` (str): Identifier for the source MDX document.
*   `chapter_title` (str): Title of the chapter/document.
*   `page_number` (int): Page number or logical sequence number for the chunk.
*   `text_snippet_preview` (str): The actual text content of the chunk.
*   `created_at` (datetime, optional): Timestamp of creation.

**Relationship**:
Each `MDX Document` is transformed into multiple `RagChunk Objects` after cleaning, chunking, and metadata assignment.
