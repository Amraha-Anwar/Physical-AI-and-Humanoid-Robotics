from qdrant_client import AsyncQdrantClient
from qdrant_client.models import Distance, VectorParams
from models import QdrantConfig
import logging

logger = logging.getLogger(__name__)

COLLECTION_NAME = "book_vectors"

async def initialize_qdrant_client(config: QdrantConfig) -> AsyncQdrantClient:
    """
    Initialize and return a Qdrant client configured for Qdrant Cloud.
    """
    try:
        # Extract plain string values from SecretStr if needed
        api_key = config.api_key
        if hasattr(api_key, 'get_secret_value'):
            api_key = api_key.get_secret_value()
        
        host = config.host
        if hasattr(host, 'get_secret_value'):
            host = host.get_secret_value()
        
        # For Qdrant Cloud, use the HTTPS URL directly
        client = AsyncQdrantClient(
            url=host,
            api_key=api_key,  # Now it's a plain string
            timeout=60,
            prefer_grpc=False,
        )
        
        logger.info(f"Attempting to connect to Qdrant at: {host}")
        
        # Check if collection exists
        try:
            logger.info(f"Validating Qdrant collection '{COLLECTION_NAME}'...")
            await client.get_collection(collection_name=COLLECTION_NAME)
            logger.info(f"Collection '{COLLECTION_NAME}' exists and is accessible.")
        except Exception as e:
            if "404" in str(e) or "Not Found" in str(e) or "not found" in str(e).lower():
                logger.warning(f"Collection '{COLLECTION_NAME}' not found. Creating it...")
                
                await client.create_collection(
                    collection_name=COLLECTION_NAME,
                    vectors_config=VectorParams(
                        size=config.vector_size,
                        distance=Distance.COSINE
                    )
                )
                logger.info(f"Collection '{COLLECTION_NAME}' created successfully.")
            else:
                logger.error(f"Failed to access collection: {e}")
                raise
        
        return client
        
    except Exception as e:
        logger.error(f"Qdrant initialization failed: {e}", exc_info=True)
        raise