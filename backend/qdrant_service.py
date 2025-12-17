from qdrant_client import AsyncQdrantClient
from qdrant_client.http.models import Distance, VectorParams
from backend.models import QdrantConfig
import logging

logger = logging.getLogger(__name__)

async def initialize_qdrant_client(config: QdrantConfig, collection_name: str = "book_vectors") -> AsyncQdrantClient:
    """
    Initializes and returns an AsyncQdrant client.
    Connects to Qdrant Cloud using the provided configuration.
    Ensures the specified collection exists, creating it if necessary.
    """
    qdrant_client = AsyncQdrantClient(
        url=config.host,
        port=config.port,
        api_key=config.api_key.get_secret_value(),
        prefer_grpc=True
    )

    # Ensure the collection exists
    try:
        # Check if collection already exists
        if await qdrant_client.collection_exists(collection_name=collection_name):
             logger.info(f"Collection '{collection_name}' already exists.")
        else:
            logger.info(f"Collection '{collection_name}' not found. Creating new collection.")
            await qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(size=config.vector_size, distance=Distance.COSINE),
            )
            logger.info(f"Collection '{collection_name}' created successfully with vector_size={config.vector_size}.")

    except Exception as e:
        logger.error(f"Error ensuring Qdrant collection '{collection_name}' exists: {e}")
        raise

    return qdrant_client