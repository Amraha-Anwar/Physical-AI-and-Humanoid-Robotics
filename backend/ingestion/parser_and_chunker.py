import uuid
from datetime import datetime
from typing import List, Generator

from models import RagChunk
from ingestion.models import InputContent, Chapter, TextSection

class ContentProcessor:
    """
    Processes raw input content, breaking it down into RAG-ready chunks.
    """
    def __init__(self, chunk_size: int = 512, chunk_overlap: int = 50):
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap

    def _chunk_text(self, text: str, document_id: str, chapter_title: str, page_number: int) -> Generator[RagChunk, None, None]:
        """
        Splits a given text into smaller chunks with overlap.
        """
        words = text.split()
        if len(words) <= self.chunk_size:
            yield RagChunk(
                chunk_id=str(uuid.uuid4()),
                document_id=document_id,
                chapter_title=chapter_title,
                page_number=page_number,
                text_snippet_preview=text,
                created_at=datetime.utcnow()
            )
            return

        for i in range(0, len(words), self.chunk_size - self.chunk_overlap):
            chunk_words = words[i:i + self.chunk_size]
            chunk_text = " ".join(chunk_words)
            yield RagChunk(
                chunk_id=str(uuid.uuid4()),
                document_id=document_id,
                chapter_title=chapter_title,
                page_number=page_number,
                text_snippet_preview=chunk_text,
                created_at=datetime.utcnow()
            )

    def process_content(self, raw_content: InputContent) -> List[RagChunk]:
        """
        Iterates through the content structure and chunks text into RagChunk objects.
        """
        all_chunks: List[RagChunk] = []
        document_id = raw_content.document_id

        for chapter in raw_content.chapters:
            chapter_title = chapter.chapter_title
            for page_idx, section in enumerate(chapter.sections): # Assuming page_number can be represented by section index
                full_text = section.text
                page_number = page_idx + 1 # Simple page numbering per chapter
                
                for chunk in self._chunk_text(full_text, document_id, chapter_title, page_number):
                    all_chunks.append(chunk)
        return all_chunks
