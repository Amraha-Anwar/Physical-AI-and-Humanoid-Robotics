import os
import re
from typing import List, Dict
from backend.ingestion.models import InputContent, Chapter, TextSection

class DocusaurusLoader:
    def __init__(self, docs_root: str):
        self.docs_root = docs_root

    def load(self) -> InputContent:
        chapters = []
        for root, dirs, files in os.walk(self.docs_root):
            for file in files:
                if file.endswith((".md", ".mdx")):
                    file_path = os.path.join(root, file)
                    relative_path = os.path.relpath(file_path, self.docs_root)
                    
                    with open(file_path, "r", encoding="utf-8") as f:
                        content = f.read()
                    
                    # Strip frontmatter
                    content = self._strip_frontmatter(content)
                    
                    # Parse sections based on headers
                    sections = self._parse_markdown_sections(content)
                    
                    if sections:
                        chapters.append(Chapter(
                            chapter_title=relative_path, # Use file path as chapter title
                            sections=sections
                        ))
        
        return InputContent(
            document_id="docusaurus-docs",
            chapters=chapters
        )

    def _strip_frontmatter(self, text: str) -> str:
        # Simple YAML frontmatter stripper
        if text.startswith("---"):
            try:
                _, _, body = text.split("---", 2)
                return body.strip()
            except ValueError:
                return text # Malformed or no closing ---
        return text

    def _parse_markdown_sections(self, text: str) -> List[TextSection]:
        # Split by H1, H2, H3 headers
        # Regex to find headers: ^#{1,3}\s+(.*)
        # We want to capture the header and the content following it.
        
        lines = text.split('\n')
        sections = []
        current_title = "Introduction"
        current_text = []
        
        header_pattern = re.compile(r'^(#{1,3})\s+(.+)$')
        
        for line in lines:
            match = header_pattern.match(line)
            if match:
                # If we have accumulated text, save the previous section
                if current_text:
                    sections.append(TextSection(
                        section_title=current_title,
                        text="\n".join(current_text).strip()
                    ))
                
                # Start new section
                current_title = match.group(2).strip()
                current_text = [] # Don't include the header in the text body? 
                # Task says "keep related topics together". 
                # Usually beneficial to include the header in the chunk text or metadata.
                # Here we put it in section_title. 
                # Let's also keep the header in the text for context if needed, but the model has section_title.
                current_text.append(line) 
            else:
                current_text.append(line)
        
        # Append the last section
        if current_text:
            sections.append(TextSection(
                section_title=current_title,
                text="\n".join(current_text).strip()
            ))
            
        return sections
