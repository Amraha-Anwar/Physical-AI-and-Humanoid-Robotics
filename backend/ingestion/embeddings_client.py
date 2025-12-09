import os
import google.generativeai as genai
from typing import List
from tenacity import retry, wait_random_exponential, stop_after_attempt
import logging

logger = logging.getLogger(__name__)

class EmbeddingClient:
    """
    Client to interact with the Google Gemini API for text embedding generation.
    """
    def __init__(self):
        api_key = os.getenv("GOOGLE_API_KEY")
        if not api_key:
            logger.error("GOOGLE_API_KEY environment variable not set in EmbeddingClient.")
            raise ValueError("GOOGLE_API_KEY environment variable not set.")
        logger.info("GOOGLE_API_KEY successfully loaded for EmbeddingClient.")
        
        genai.configure(api_key=api_key)
        self.model = "gemini-embedding-001"
        logger.info(f"EmbeddingClient initialized with model: {self.model}")

    @retry(wait=wait_random_exponential(min=1, max=60), stop=stop_after_attempt(5))
    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generates a static dummy embedding vector for testing purposes, bypassing external API calls.
        """
        logger.warning("Generating DUMMY embedding vector. External API call bypassed for testing.")
        # Return a static list of 768 floats (standard dimensionality for Gemini embedding model)
        return [0.0] * 768