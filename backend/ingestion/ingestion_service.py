import logging
import uuid
from typing import List
from psycopg2.extensions import connection as PgConnection # type: ignore
from qdrant_client import AsyncQdrantClient
from qdrant_client.http.models import PointStruct, models
from datetime import datetime

from backend.ingestion.embeddings_client import EmbeddingClient
from backend.ingestion.parser_and_chunker import ContentProcessor
from backend.ingestion.models import InputContent, EmbeddableChunk
from backend.models import RagChunk # Import RagChunk from backend.models

logger = logging.getLogger(__name__)

class IngestionService:
    """
    Service to coordinate content processing, embedding, and storage.
    """
    def __init__(self,
                 embedding_client: EmbeddingClient,
                 content_processor: ContentProcessor,
                 # neon_db_dependency: Callable[[], Generator[PgConnection, None, None]],
                 qdrant_client: AsyncQdrantClient):
        self.embedding_client = embedding_client
        self.content_processor = content_processor
        # self.get_neon_db = neon_db_dependency # Store the dependency callable
        self.qdrant_client = qdrant_client
        self.qdrant_collection_name = "book_vectors" # Ensure this matches where collection is created

    async def ingest_content_async(self, raw_content: InputContent, neon_conn: PgConnection) -> List[EmbeddableChunk]:
        """
        Performs end-to-end ingestion: chunking, embedding, and saving to databases.
        """
        processed_chunks: List[RagChunk] = self.content_processor.process_content(raw_content)
        embeddable_chunks: List[EmbeddableChunk] = []

        for chunk in processed_chunks:
            try:
                embedding = await self.embedding_client.generate_embedding(chunk.text_snippet_preview)
                embeddable_chunk = EmbeddableChunk(**chunk.dict(), embedding_vector=embedding)
                embeddable_chunks.append(embeddable_chunk)
            except Exception as e:
                logger.error(f"Failed to generate embedding for chunk {chunk.chunk_id}: {e}", exc_info=True)
                # Decide whether to skip this chunk or re-raise
                continue

        logger.info(f"ContentProcessor generated {len(processed_chunks)} chunks.")
        logger.info(f"Successfully generated embeddings for {len(embeddable_chunks)} chunks.")

        await self._save_to_neon(embeddable_chunks, neon_conn)
        await self._save_to_qdrant(embeddable_chunks)

        return embeddable_chunks

    async def _save_to_neon(self, chunks: List[EmbeddableChunk], neon_conn: PgConnection):
        """
        Saves chunk metadata to the Neon Postgres database.
        """
        insert_query = """
        INSERT INTO rag_metadata (chunk_id, document_id, chapter_title, page_number, text_snippet_preview, created_at)
        VALUES (%s, %s, %s, %s, %s, %s)
        ON CONFLICT (chunk_id) DO NOTHING;
        """
        with neon_conn.cursor() as cur:
            for chunk in chunks:
                try:
                    cur.execute(insert_query, (
                        chunk.chunk_id,
                        chunk.document_id,
                        chunk.chapter_title,
                        chunk.page_number,
                        chunk.text_snippet_preview,
                        chunk.created_at
                    ))
                except Exception as e:
                    logger.error(f"Failed to save chunk {chunk.chunk_id} to Neon: {e}", exc_info=True)
            neon_conn.commit()
        logger.info(f"Saved {len(chunks)} chunks to Neon Postgres.")

    async def _save_to_qdrant(self, chunks: List[EmbeddableChunk]):
        """
        Saves embeddable chunks to the Qdrant vector store.
        """
        points = []
        for chunk in chunks:
            # Qdrant expects payload as a dict
            payload = chunk.dict()
            # Remove embedding_vector from payload as it's passed separately
            embedding_vector = payload.pop("embedding_vector")

            points.append(PointStruct(
                id=str(uuid.uuid4()),  # Qdrant requires a unique ID for each point
                vector=embedding_vector,
                payload=payload
            ))
        
        try:
            operation_info = await self.qdrant_client.upsert(
                collection_name=self.qdrant_collection_name,
                wait=True,
                points=points
            )
            logger.info(f"Saved {len(points)} points to Qdrant. Status: {operation_info.status}")
        except Exception as e:
            logger.error(f"Failed to save chunks to Qdrant: {e}", exc_info=True)
            raise
