from fastapi import APIRouter, Depends, HTTPException
from starlette.responses import JSONResponse
import logging

from backend.ingestion.models import InputContent # Added import for InputContent
from backend.ingestion.ingestion_service import IngestionService
from backend.dependencies import get_ingestion_service, get_neon_db
from psycopg2.extensions import connection as PgConnection # Import PgConnection

logger = logging.getLogger(__name__)

router = APIRouter()

@router.post("/ingestion", response_model=dict, status_code=200)
async def ingest_content_endpoint(
    raw_content: InputContent,
    ingestion_service: IngestionService = Depends(get_ingestion_service),
    neon_conn: PgConnection = Depends(get_neon_db) # Add neon_conn dependency
):
    """
    Endpoint to initiate the content ingestion workflow.
    Receives raw book content, chunks it, generates embeddings,
    and saves to both Neon (metadata) and Qdrant (vectors).
    """
    logger.info(f"Received ingestion request for document_id: {raw_content.document_id}")
    try:
        processed_chunks = await ingestion_service.ingest_content_async(raw_content, neon_conn) # Pass neon_conn
        return JSONResponse(content={
            "message": "Content ingestion completed successfully.",
            "document_id": raw_content.document_id,
            "chunks_processed": len(processed_chunks)
        })
    except Exception as e:
        logger.error(f"Error during content ingestion for document_id {raw_content.document_id}: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Failed to ingest content: {e}")
