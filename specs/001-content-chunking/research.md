# P2.1 Content Pre-processing: Extraction and Chunking - Research Findings

This document captures key research findings and decisions made during the planning phase for content pre-processing, extraction, and chunking.

## Decisions Documented

| Decision | Options | Trade-offs | Chosen Option (Working Assumption) |
| :--- | :--- | :--- | :--- |
| **MDX Parsing Tool** | Custom Regex/String Split, `markdown-it-py`, `mistune` | Regex is fragile; specialized parsers are more robust but add dependencies. | **Use a robust Python Markdown parser (e.g., `markdown-it-py`) combined with targeted pre-cleaning** to strip frontmatter/components. |
| **Chunking Type** | Simple Fixed-Size, Sentence Splitter, Recursive Splitter | Fixed-size loses context; Sentence splitter is better but ignores structure; Recursive Splitter preserves semantic hierarchy. | **Recursive Character Text Splitter** (e.g., from LangChain library) to prioritize splitting on structural elements (`#`, `##`, `\n\n`). |
| **Chunk Size/Overlap** | Varies (e.g., 256/50, 512/100) | Smaller chunks increase retrieval accuracy but lose context; larger chunks retain context but might pull irrelevant info. | **Chunk Size: 512 tokens (approx. 2000 characters), Overlap: 100 characters**. (Subject to validation during P2.1.4). |
