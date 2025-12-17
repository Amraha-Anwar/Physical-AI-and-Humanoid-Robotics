from pydantic import BaseModel, SecretStr
from typing import Optional, List
from datetime import datetime

class QdrantConfig(BaseModel):
    host: str
    port: Optional[int] = None
    api_key: SecretStr
    vector_size: int = 768

class NeonConfig(BaseModel):
    connection_string: SecretStr
    db_name: Optional[str] = None
    user: Optional[str] = None
    password: Optional[SecretStr] = None
    host: Optional[str] = None
    port: int = 5432

class RagChunk(BaseModel):
    chunk_id: str # UUID or similar unique identifier
    document_id: str
    chapter_title: str
    page_number: int
    text_snippet_preview: str
    created_at: Optional[datetime] = None

    class Config:
        from_attributes = True # For SQLAlchemy ORM integration
