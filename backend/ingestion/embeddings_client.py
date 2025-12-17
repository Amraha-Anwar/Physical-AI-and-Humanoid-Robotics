import os
from typing import List
from tenacity import retry, wait_random_exponential, stop_after_attempt
import logging
import cohere
import asyncio

logger = logging.getLogger(__name__)

class EmbeddingClient:
    """
    Client to interact with the Cohere API for text embedding generation.
    """
    def __init__(self):
        api_key = os.getenv("COHERE_API_KEY")
        if not api_key:
            logger.error("COHERE_API_KEY environment variable not set in EmbeddingClient.")
            raise ValueError("COHERE_API_KEY environment variable not set.")
        
        # Initialize Cohere AsyncClient
        self.client = cohere.AsyncClient(api_key)
        self.model = "embed-english-v3.0"
        
        logger.info(f"EmbeddingClient initialized with Cohere model: {self.model}")

    @retry(wait=wait_random_exponential(min=1, max=60), stop=stop_after_attempt(5))
    async def generate_embedding(self, text: str, input_type: str = "search_query") -> List[float]:
        """
        Generates an embedding vector using the Cohere API.
        
        Args:
            text: The text to embed.
            input_type: "search_document" (for ingestion) or "search_query" (for retrieval).
        """
        # Let exceptions propagate so tenacity can retry
        response = await self.client.embed(
            texts=[text],
            model=self.model,
            input_type=input_type,
            embedding_types=["float"]
        )
        return response.embeddings.float[0]
