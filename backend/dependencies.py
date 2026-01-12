import os
import psycopg2
import functools
from typing import Generator, Optional
from psycopg2.extensions import connection as PgConnection
from qdrant_client import AsyncQdrantClient
import logging
from fastapi import Depends

from models import NeonConfig, QdrantConfig
from neon_client import initialize_neon_db
from qdrant_service import initialize_qdrant_client

from ingestion.embeddings_client import EmbeddingClient
from ingestion.parser_and_chunker import ContentProcessor
from ingestion.ingestion_service import IngestionService
from query.query_service import QueryService
from query.history_service import HistoryService
from evaluation.evaluation_service import EvaluationService
import openai

# -------------------------------------------------------------------
# Logging
# -------------------------------------------------------------------
logger = logging.getLogger(__name__)

# -------------------------------------------------------------------
# Global singletons
# -------------------------------------------------------------------
_qdrant_client: Optional[AsyncQdrantClient] = None
_embedding_client: Optional[EmbeddingClient] = None
_content_processor: Optional[ContentProcessor] = None
_ingestion_service: Optional[IngestionService] = None
_query_service: Optional[QueryService] = None
_history_service: Optional[HistoryService] = None
_evaluation_service: Optional[EvaluationService] = None


# -------------------------------------------------------------------
# Neon (Postgres) config
# -------------------------------------------------------------------
def get_neon_config() -> NeonConfig:
    conn_string = os.getenv("NEON_POSTGRES_CONNECTION_STRING")
    if not conn_string:
        raise ValueError("NEON_POSTGRES_CONNECTION_STRING environment variable not set.")
    return NeonConfig(connection_string=conn_string)


@functools.lru_cache()
def _get_cached_neon_config() -> NeonConfig:
    return get_neon_config()


# -------------------------------------------------------------------
# Qdrant config (CLOUD – REST ONLY)
# -------------------------------------------------------------------
def get_qdrant_config() -> QdrantConfig:
    """
    Qdrant Cloud configuration.
    IMPORTANT:
    - Use HTTPS URL
    - NO port
    - NO gRPC
    """
    host = os.getenv("QDRANT_HOST")
    api_key = os.getenv("QDRANT_API_KEY")
    vector_size_str = os.getenv("QDRANT_VECTOR_SIZE", "768")

    if not host or not api_key:
        raise ValueError("QDRANT_HOST and QDRANT_API_KEY must be set.")

    try:
        vector_size = int(vector_size_str)
    except ValueError:
        raise ValueError("QDRANT_VECTOR_SIZE must be an integer.")

    return QdrantConfig(
        host=host,
        api_key=api_key,
        vector_size=vector_size,
    )


@functools.lru_cache()
def _get_cached_qdrant_config() -> QdrantConfig:
    return get_qdrant_config()


# -------------------------------------------------------------------
# Startup initialization
# -------------------------------------------------------------------
async def setup_db_clients():
    global _qdrant_client
    global _embedding_client
    global _content_processor
    global _ingestion_service
    global _query_service
    global _history_service
    global _evaluation_service

    logger.info("Initializing database clients and services...")

    try:
        # Neon DB (DDL only)
        neon_config = _get_cached_neon_config()
        initialize_neon_db(neon_config)
        logger.info("Neon Postgres DDL ensured.")

        # Qdrant Cloud (REST)
        qdrant_config = _get_cached_qdrant_config()
        _qdrant_client = await initialize_qdrant_client(qdrant_config)
        logger.info("Qdrant client initialized.")

        # Core components
        _embedding_client = EmbeddingClient()
        _content_processor = ContentProcessor()
        _history_service = HistoryService()

        # Services
        _ingestion_service = IngestionService(
            embedding_client=_embedding_client,
            content_processor=_content_processor,
            qdrant_client=_qdrant_client,
        )

        _query_service = QueryService(
            embedding_client=_embedding_client,
            qdrant_client=_qdrant_client,
            history_service=_history_service,
        )

        api_key = os.getenv("GEMINI_API_KEY") or os.getenv("OPENAI_API_KEY")
        base_url = (
            "https://generativelanguage.googleapis.com/v1beta/openai/"
            if os.getenv("GEMINI_API_KEY")
            else None
        )

        openai_client = openai.AsyncOpenAI(
            api_key=api_key,
            base_url=base_url,
        )

        _evaluation_service = EvaluationService(openai_client=openai_client)

        logger.info("All database clients and services initialized successfully.")

    except Exception as e:
        logger.critical(
            "Failed to initialize one or more database clients or services",
            exc_info=True,
        )
        raise RuntimeError("Database client and service initialization failed.") from e


# -------------------------------------------------------------------
# FastAPI dependencies
# -------------------------------------------------------------------
def get_neon_db() -> Generator[PgConnection, None, None]:
    neon_config = _get_cached_neon_config()
    conn_string = neon_config.connection_string.get_secret_value()
    conn = None
    try:
        conn = psycopg2.connect(conn_string)
        conn.autocommit = True
        yield conn
    finally:
        if conn:
            conn.close()


def get_qdrant_client() -> AsyncQdrantClient:
    if _qdrant_client is None:
        raise RuntimeError("Qdrant client not initialized.")
    return _qdrant_client


def get_ingestion_service(
    qdrant_client: AsyncQdrantClient = Depends(get_qdrant_client),
) -> IngestionService:
    global _ingestion_service
    global _embedding_client
    global _content_processor

    if _ingestion_service is None:
        _embedding_client = _embedding_client or EmbeddingClient()
        _content_processor = _content_processor or ContentProcessor()
        _ingestion_service = IngestionService(
            embedding_client=_embedding_client,
            content_processor=_content_processor,
            qdrant_client=qdrant_client,
        )

    return _ingestion_service


def get_query_service(
    qdrant_client: AsyncQdrantClient = Depends(get_qdrant_client),
) -> QueryService:
    global _query_service
    global _embedding_client
    global _history_service

    if _query_service is None:
        _embedding_client = _embedding_client or EmbeddingClient()
        _history_service = _history_service or HistoryService()
        _query_service = QueryService(
            embedding_client=_embedding_client,
            qdrant_client=qdrant_client,
            history_service=_history_service,
        )

    return _query_service


def get_evaluation_service() -> EvaluationService:
    global _evaluation_service

    if _evaluation_service is None:
        api_key = os.getenv("GEMINI_API_KEY") or os.getenv("OPENAI_API_KEY")
        base_url = (
            "https://generativelanguage.googleapis.com/v1beta/openai/"
            if os.getenv("GEMINI_API_KEY")
            else None
        )

        openai_client = openai.AsyncOpenAI(
            api_key=api_key,
            base_url=base_url,
        )

        _evaluation_service = EvaluationService(openai_client=openai_client)

    return _evaluation_service
