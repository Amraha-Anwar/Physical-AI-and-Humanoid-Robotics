import psycopg2
from models import NeonConfig, RagChunk # Import RagChunk
from psycopg2.extensions import connection as PgConnection
import logging

logger = logging.getLogger(__name__)

def initialize_neon_db(config: NeonConfig):
    """
    Initializes and returns a connection to Neon Postgres.
    Ensures the 'rag_metadata' table exists, creating it if necessary.
    """
    conn = None # Initialize conn to None
    try:
        conn_string = config.connection_string.get_secret_value()
        conn = psycopg2.connect(conn_string)
        conn.autocommit = True # Ensure DDL statements are committed immediately
        logger.info("Successfully connected to Neon Postgres for DDL operations.")

        # Add DDL logic to create rag_metadata table if it doesn't exist
        with conn.cursor() as cur:
            create_table_query = f"""
            CREATE TABLE IF NOT EXISTS rag_metadata (
                chunk_id TEXT PRIMARY KEY,
                document_id TEXT NOT NULL,
                chapter_title TEXT NOT NULL,
                page_number INTEGER,
                text_snippet_preview TEXT NOT NULL,
                created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
            );
            """
            cur.execute(create_table_query)
            logger.info("Ensured 'rag_metadata' table exists in Neon Postgres.")

            # Create chat_history table
            create_chat_history_table_query = f"""
            CREATE TABLE IF NOT EXISTS chat_history (
                id SERIAL PRIMARY KEY,
                session_id TEXT NOT NULL,
                user_query TEXT NOT NULL,
                llm_response TEXT NOT NULL,
                timestamp TIMESTAMP DEFAULT NOW()
            );
            CREATE INDEX IF NOT EXISTS idx_session_id ON chat_history (session_id);
            """
            cur.execute(create_chat_history_table_query)
            logger.info("Ensured 'chat_history' table and index exist in Neon Postgres.")

    except Exception as e:
        logger.error(f"Error connecting to or initializing Neon Postgres: {e}")
        raise
    finally:
        if conn:
            conn.close() # Close connection after DDL operations