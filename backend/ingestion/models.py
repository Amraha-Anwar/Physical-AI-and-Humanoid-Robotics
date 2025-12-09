from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime

# Assuming RagChunk is in backend/models.py, we'll import it
from backend.models import RagChunk

class TextSection(BaseModel):
    section_title: Optional[str] = None
    text: str

class Chapter(BaseModel):
    chapter_title: str
    sections: List[TextSection]

class InputContent(BaseModel):
    """
    Model reflecting the expected book structure for ingestion.
    """
    document_id: str
    chapters: List[Chapter]

class EmbeddableChunk(RagChunk):
    """
    Extends RagChunk with the embedding vector.
    """
    embedding_vector: List[float]
