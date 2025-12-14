import os
import psycopg2
import functools
from typing import Generator, Optional
from psycopg2.extensions import connection as PgConnection
from qdrant_client import AsyncQdrantClient
import logging
from fastapi import Depends

from backend.models import NeonConfig, QdrantConfig
from backend.neon_client import initialize_neon_db
from backend.qdrant_client import initialize_qdrant_client

from backend.ingestion.embeddings_client import EmbeddingClient
from backend.ingestion.parser_and_chunker import ContentProcessor
from backend.ingestion.ingestion_service import IngestionService
from backend.query.query_service import QueryService
from backend.query.history_service import HistoryService
from backend.evaluation.evaluation_service import EvaluationService
import openai

# Configure basic logging for visibility
logger = logging.getLogger(__name__)

# Global variables to hold the DB connections/clients
_qdrant_client: Optional[AsyncQdrantClient] = None
_embedding_client: Optional[EmbeddingClient] = None
_content_processor: Optional[ContentProcessor] = None
_ingestion_service: Optional[IngestionService] = None
_query_service: Optional[QueryService] = None
_history_service: Optional[HistoryService] = None
_evaluation_service: Optional[EvaluationService] = None

def get_neon_config() -> NeonConfig:
    """Loads Neon configuration from environment variables."""
    conn_string = os.getenv("NEON_POSTGRES_CONNECTION_STRING")
    if not conn_string:
        raise ValueError("NEON_POSTGRES_CONNECTION_STRING environment variable not set.")
    return NeonConfig(connection_string=conn_string)

@functools.lru_cache()
def _get_cached_neon_config() -> NeonConfig:
    return get_neon_config()

def get_qdrant_config() -> QdrantConfig:
    """Loads Qdrant configuration from environment variables."""
    host = os.getenv("QDRANT_HOST")
    api_key = os.getenv("QDRANT_API_KEY")
    vector_size_str = os.getenv("QDRANT_VECTOR_SIZE", "768")

    if not host or not api_key:
        raise ValueError("QDRANT_HOST and QDRANT_API_KEY environment variables must be set.")
    
    try:
        vector_size = int(vector_size_str)
    except ValueError:
        raise ValueError("QDRANT_VECTOR_SIZE must be an integer.")

    return QdrantConfig(host=host, api_key=api_key, vector_size=vector_size)

@functools.lru_cache()
def _get_cached_qdrant_config() -> QdrantConfig:
    return get_qdrant_config()

async def setup_db_clients():
    """
    Initializes global database clients (Qdrant, Embedding, ContentProcessor) and services.
    Ensures Neon DB tables are created. This function should be called on application startup.
    """
    global _qdrant_client
    global _embedding_client
    global _content_processor
    global _ingestion_service
    global _query_service
    global _history_service
    global _evaluation_service

    logger.info("Initializing database clients and services...")
    
    try:
        # Initialize Neon DB (DDL only, no global connection)
        neon_config = _get_cached_neon_config()
        initialize_neon_db(neon_config) # Just run DDL
        logger.info("Neon Postgres DDL ensured.")

        # Initialize Qdrant Client
        qdrant_config = _get_cached_qdrant_config()
        _qdrant_client = await initialize_qdrant_client(qdrant_config)
        logger.info("Qdrant client initialized.")

        # Initialize EmbeddingClient
        _embedding_client = EmbeddingClient()
        logger.info("EmbeddingClient initialized.")

        # Initialize ContentProcessor
        _content_processor = ContentProcessor()
        logger.info("ContentProcessor initialized.")

        # Initialize IngestionService (without neon_conn in constructor)
        _ingestion_service = IngestionService(
            embedding_client=_embedding_client,
            content_processor=_content_processor,
            qdrant_client=_qdrant_client
        )
        logger.info("IngestionService initialized.")
        
        # Initialize HistoryService (without neon_conn in constructor)
        _history_service = HistoryService()
        logger.info("HistoryService initialized.")
        
        # Initialize QueryService (without neon_conn in constructor)
        _query_service = QueryService(
            embedding_client=_embedding_client,
            qdrant_client=_qdrant_client,
            history_service=_history_service
        )
        logger.info("QueryService initialized.")

        # Initialize EvaluationService (without neon_conn in constructor)
        openai_client = openai.AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        _evaluation_service = EvaluationService(
            openai_client=openai_client,
        )
        logger.info("EvaluationService initialized.")


    except Exception as e:
        logger.critical(f"Failed to initialize one or more database clients or services: {e}", exc_info=True)
        # Depending on desired behavior, could re-raise or attempt graceful shutdown
        raise RuntimeError("Database client and service initialization failed.") from e


def get_neon_db() -> Generator[PgConnection, None, None]:
    """
    FastAPI dependency that provides a Neon Postgres database connection.
    A new connection is created for each request and closed after the request is finished.
    """
    neon_config = _get_cached_neon_config()
    conn_string = neon_config.connection_string.get_secret_value()
    conn = None
    try:
        conn = psycopg2.connect(conn_string)
        conn.autocommit = True # Ensure DDL statements are committed immediately
        yield conn
    finally:
        if conn:
            conn.close()


def get_qdrant_client() -> AsyncQdrantClient:
    """
    FastAPI dependency that provides a Qdrant client instance.
    The client is initialized once globally and reused.
    """
    global _qdrant_client
    if _qdrant_client is None:
        # This path should ideally not be hit if setup_db_clients is called on startup
        logger.warning("Qdrant client not initialized during startup. Cannot initialize async client in sync dependency.")
        raise RuntimeError("Qdrant client not initialized.")
    return _qdrant_client

def get_ingestion_service(
    qdrant_client: AsyncQdrantClient = Depends(get_qdrant_client)
) -> IngestionService:
    """
    FastAPI dependency that provides an IngestionService instance.
    """
    global _ingestion_service
    global _embedding_client
    global _content_processor

    if _ingestion_service is None or _embedding_client is None or _content_processor is None:
        logger.warning("IngestionService or its components not initialized during startup. Initializing on first request.")
        # Re-initialize components if not already done (should be done by setup_db_clients)
        _embedding_client = EmbeddingClient()
        _content_processor = ContentProcessor()
        _ingestion_service = IngestionService(
            embedding_client=_embedding_client,
            content_processor=_content_processor,
            qdrant_client=qdrant_client
        )
    return _ingestion_service

def get_query_service(
    qdrant_client: AsyncQdrantClient = Depends(get_qdrant_client)
) -> QueryService:
    """
    FastAPI dependency that provides a QueryService instance.
    """
    global _query_service
    global _embedding_client
    global _history_service

    if _query_service is None or _embedding_client is None:
        logger.warning("QueryService or its components not initialized during startup. Initializing on first request.")
        _embedding_client = EmbeddingClient()
        if _history_service is None:
             _history_service = HistoryService() # Constructor no longer takes neon_conn
        
        _query_service = QueryService(
            embedding_client=_embedding_client,
            qdrant_client=qdrant_client,
            history_service=_history_service # Constructor no longer takes neon_conn
        )
    return _query_service

def get_evaluation_service(
) -> EvaluationService:
    """
    FastAPI dependency that provides an EvaluationService instance.
    """
    global _evaluation_service
    if _evaluation_service is None:
        logger.warning("EvaluationService not initialized during startup. Initializing on first request.")
        openai_client = openai.AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        _evaluation_service = EvaluationService(
            openai_client=openai_client
        )
    return _evaluation_service