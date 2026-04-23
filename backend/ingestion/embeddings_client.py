import os
from typing import List
from tenacity import retry, wait_random_exponential, stop_after_attempt
import logging
import openai

logger = logging.getLogger(__name__)

# ACTION REQUIRED: Qdrant collection may need to be recreated with vector_size=1536
# (text-embedding-3-small produces 1536-dimensional vectors; current QDRANT_VECTOR_SIZE
# default in dependencies.py is 768.)

class EmbeddingClient:
    """
    Client to interact with the OpenAI API for text embedding generation.
    """
    def __init__(self):
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            logger.error("OPENAI_API_KEY environment variable not set in EmbeddingClient.")
            raise ValueError("OPENAI_API_KEY environment variable not set.")

        # Initialize OpenAI AsyncClient (openai 2.x SDK)
        self.client = openai.AsyncOpenAI(api_key=api_key)
        self.model = "text-embedding-3-small"

        logger.info(f"EmbeddingClient initialized with OpenAI model: {self.model}")

    @retry(wait=wait_random_exponential(min=1, max=60), stop=stop_after_attempt(5))
    async def generate_embedding(self, text: str, input_type: str = "search_query") -> List[float]:
        """
        Generates an embedding vector using the OpenAI API.

        Args:
            text: The text to embed.
            input_type: Retained for API compatibility with prior Cohere-based client
                ("search_document" for ingestion, "search_query" for retrieval).
                OpenAI embeddings do not differentiate by input type, so this
                argument is accepted but not forwarded to the API.
        """
        # Let exceptions propagate so tenacity can retry
        response = await self.client.embeddings.create(
            input=[text],
            model=self.model,
        )
        return response.data[0].embedding
