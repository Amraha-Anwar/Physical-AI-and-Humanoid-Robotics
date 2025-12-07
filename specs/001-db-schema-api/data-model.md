# P1.3 Database Connection: Schema Definition & API Layer - Data Model

This document defines the conceptual data models for the RAG metadata and vector embeddings, along with configuration models for database connections.

## 1. RAG Metadata Model (Neon Postgres)

**Entity**: `RagChunk`

**Description**: Represents a segmented piece of text (chunk) from the "Physical AI & Humanoid Robotics" book, along with its associated metadata. This data will be stored in the `rag_metadata` table in Neon Postgres.

**Attributes**:

*   `chunk_id` (UUID/String, Primary Key): A unique identifier for the text chunk. This will also serve as the link to the corresponding vector in Qdrant.
*   `document_id` (UUID/String): Identifier for the source document (e.g., specific book edition, chapter).
*   `chapter_title` (String): The title of the chapter the chunk belongs to.
*   `page_number` (Integer): The page number from which the chunk was extracted.
*   `text_snippet_preview` (String): A short excerpt of the text chunk for display or quick reference. The full text may be stored elsewhere or referenced.
*   `created_at` (Timestamp): Timestamp of when the record was created.

**Pydantic Model (Conceptual)**:

```python
from pydantic import BaseModel
from typing import Optional
from datetime import datetime

class RagChunk(BaseModel):
    chunk_id: str # UUID or similar unique identifier
    document_id: str
    chapter_title: str
    page_number: int
    text_snippet_preview: str
    created_at: Optional[datetime] = None

    class Config:
        orm_mode = True # For SQLAlchemy ORM integration
```

## 2. Book Vectors Model (Qdrant Cloud)

**Entity**: `BookVector`

**Description**: Represents the high-dimensional vector embedding of a `RagChunk`'s text content, stored in the `book_vectors` collection in Qdrant Cloud.

**Attributes**:

*   `id` (String/UUID): The unique identifier for the vector point, which directly corresponds to `RagChunk.chunk_id`.
*   `vector` (List[Float]): The numerical representation (embedding) of the text chunk. Vector size will be 1536.
*   `payload` (Dictionary): Additional metadata associated with the vector, primarily including `chunk_id` for inverse lookups and potentially other derived attributes.

**Qdrant Point Structure (Conceptual)**:

```python
# Represents a point in Qdrant
class QdrantPoint:
    id: str # Corresponds to RagChunk.chunk_id
    vector: list[float] # Vector embedding, size 1536
    payload: dict = {
        "chunk_id": str,
        # Potentially other metadata for filtering/faceting
    }
```

## 3. Configuration Models

### 3.1 Qdrant Configuration

**Entity**: `QdrantConfig`

**Description**: Configuration parameters required to connect to Qdrant Cloud.

**Attributes**:

*   `host` (String): Qdrant service host.
*   `port` (Integer, Optional): Qdrant service port (if applicable).
*   `api_key` (String, Secret): API key for Qdrant Cloud authentication.
*   `vector_size` (Integer): Expected vector size for the collection (e.g., 1536).

**Pydantic Model (Conceptual)**:

```python
from pydantic import BaseModel, SecretStr

class QdrantConfig(BaseModel):
    host: str
    port: Optional[int] = None
    api_key: SecretStr
    vector_size: int = 1536 # Default for text-embedding-ada-002 / v3-small
```

### 3.2 Neon Postgres Configuration

**Entity**: `NeonConfig`

**Description**: Configuration parameters required to connect to Neon Postgres.

**Attributes**:

*   `connection_string` (String, Secret): The full connection string for Neon Postgres.
*   `db_name` (String, Optional): Database name (if not in connection string).
*   `user` (String, Optional): Database user (if not in connection string).
*   `password` (String, Secret, Optional): Database password (if not in connection string).
*   `host` (String, Optional): Database host (if not in connection string).
*   `port` (Integer, Optional): Database port (if not in connection string, default 5432).

**Pydantic Model (Conceptual)**:

```python
from pydantic import BaseModel, SecretStr
from typing import Optional

class NeonConfig(BaseModel):
    connection_string: SecretStr
    db_name: Optional[str] = None
    user: Optional[str] = None
    password: Optional[SecretStr] = None
    host: Optional[str] = None
    port: int = 5432
```
