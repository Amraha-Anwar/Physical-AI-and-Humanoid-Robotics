# P2.1 Content Pre-processing: Extraction and Chunking - Quickstart Guide

This guide provides a rapid setup and execution process for the content pre-processing script.

## Prerequisites

*   Python 3.9+
*   Poetry (for dependency management)
*   Access to the book's MDX source files (located in `frontend/docs/`).

## 1. Setup Environment

Create a new directory for the script, e.g., `scripts/content_processor/`, or use an existing `scripts/` directory if available.

## 2. Install Dependencies

Navigate to the `scripts/content_processor/` directory (or wherever the script will reside) and install the required Python packages. For example, using poetry:

```bash
cd scripts/content_processor/
poetry init --no-interaction # If not already initialized
poetry add python-markdown-it markdown-it-py linkify-it-py mdit-py-plugins python-frontmatter langchain-text-splitters
# Ensure backend/models.py is accessible via PYTHONPATH if not installed as a package
```

## 3. Prepare Input MDX Files

Ensure your MDX files are located in `frontend/docs/`. The script will typically expect to read from this location or a configurable path.

## 4. Run the Content Pre-processing Script

Once the script (e.g., `process_mdx.py`) is implemented, execute it from the project root or its designated script directory.

```bash
# Example command (actual command may vary based on script implementation)
python scripts/content_processor/process_mdx.py --input-dir frontend/docs/ --output-file processed_rag_chunks.json
```

## 5. Verify Output

After execution, check the generated `processed_rag_chunks.json` file.

### 5.1 Manual Inspection

*   Open the `processed_rag_chunks.json` file.
*   Verify that it's a valid JSON array.
*   Inspect a few `RagChunk` objects to ensure:
    *   `chunk_id` is a unique identifier.
    *   `chapter_title` and `text_snippet_preview` are present and accurate.
    *   No Docusaurus frontmatter or React/HTML components are visible in `text_snippet_preview`.
    *   The `text_snippet_preview` represents a coherent chunk of text.

### 5.2 Schema Validation

(This step requires the `RagChunk` Pydantic model to be accessible)

You can write a small Python script to load the JSON output and validate it against the `RagChunk` Pydantic model from `backend/models.py`.

```python
import json
from backend.models import RagChunk # Assuming backend/ is in PYTHONPATH

with open('processed_rag_chunks.json', 'r', encoding='utf-8') as f:
    data = json.load(f)

# Validate each entry
for item in data:
    try:
        RagChunk(**item)
        print(f"Validated chunk: {item.get('chunk_id', 'N/A')}")
    except Exception as e:
        print(f"Validation error for item: {item.get('chunk_id', 'N/A')} - {e}")

print("Validation complete.")
```

This quickstart will help confirm the script's basic functionality and output quality.
