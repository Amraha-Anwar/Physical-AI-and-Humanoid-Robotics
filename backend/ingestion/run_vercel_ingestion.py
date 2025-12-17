import asyncio
import logging
import os
from dotenv import load_dotenv
import psycopg2

from dependencies import get_neon_config, get_qdrant_config
from qdrant_service import initialize_qdrant_client
from ingestion.embeddings_client import EmbeddingClient
from ingestion.parser_and_chunker import ContentProcessor
from ingestion.ingestion_service import IngestionService
from ingestion.vercel_crawler import VercelCrawler

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def main():
    logger.info("Starting Vercel Ingestion Task (T103)...")
    
    # 1. Load Environment
    load_dotenv()
    
    try:
        # 2. Setup Clients
        neon_config = get_neon_config()
        neon_conn = psycopg2.connect(neon_config.connection_string.get_secret_value())
        neon_conn.autocommit = True
        logger.info("Connected to Neon DB.")

        qdrant_config = get_qdrant_config()
        qdrant_client = await initialize_qdrant_client(qdrant_config)
        
        # 3. Clear Existing Collection (T103.2)
        collection_name = "book_vectors"
        if await qdrant_client.collection_exists(collection_name):
            logger.info(f"Clearing collection '{collection_name}'...")
            await qdrant_client.delete_collection(collection_name)
            # Re-initialize to create it empty
            await initialize_qdrant_client(qdrant_config, collection_name)
            logger.info("Collection recreated empty.")
        
        embedding_client = EmbeddingClient()
        content_processor = ContentProcessor()
        
        ingestion_service = IngestionService(
            embedding_client=embedding_client,
            content_processor=content_processor,
            qdrant_client=qdrant_client
        )

        # 4. Crawl Content (T103.1)
        crawler = VercelCrawler()
        input_content = crawler.crawl()
        logger.info(f"Crawled {len(input_content.chapters)} pages from Vercel.")

        # 5. Ingest (T103.2)
        await ingestion_service.ingest_content_async(input_content, neon_conn)
        
        # 6. Verify
        count_result = await qdrant_client.count(collection_name)
        logger.info(f"Final Point Count in Qdrant: {count_result.count}")

    except Exception as e:
        logger.error(f"Ingestion failed: {e}", exc_info=True)
    finally:
        if 'neon_conn' in locals() and neon_conn:
            neon_conn.close()

if __name__ == "__main__":
    asyncio.run(main())
