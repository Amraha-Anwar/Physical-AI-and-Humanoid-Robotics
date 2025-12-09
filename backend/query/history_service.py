import logging
from typing import List, Dict, Any
from psycopg2.extensions import connection as PgConnection
from psycopg2.extras import RealDictCursor

logger = logging.getLogger(__name__)

class HistoryService:
    """
    Service to handle storage and retrieval of chat history in Neon PostgreSQL.
    """
    def __init__(self):
        pass # No connection held at init

    async def save_history_async(self, neon_conn: PgConnection, session_id: str, user_query: str, llm_response: str) -> None:
        """
        Stores the new user/LLM turn in the database.
        """
        query = """
        INSERT INTO chat_history (session_id, user_query, llm_response)
        VALUES (%s, %s, %s)
        """
        try:
            with neon_conn.cursor() as cur:
                cur.execute(query, (session_id, user_query, llm_response))
            neon_conn.commit()
            logger.info(f"Saved chat history for session {session_id}")
        except Exception as e:
            neon_conn.rollback()
            logger.error(f"Failed to save chat history: {e}", exc_info=True)
            # Re-raise or handle? Depending on requirements. Logging is safe.

    async def get_history_async(self, neon_conn: PgConnection, session_id: str) -> List[Dict[str, Any]]:
        """
        Retrieves the full history for a given session ID, ordered by timestamp.
        """
        query = """
        SELECT user_query, llm_response, timestamp
        FROM chat_history
        WHERE session_id = %s
        ORDER BY timestamp ASC
        """
        try:
            with neon_conn.cursor(cursor_factory=RealDictCursor) as cur:
                cur.execute(query, (session_id,))
                rows = cur.fetchall()
                # rows will be a list of RealDictRow, which behaves like a dict
                return [dict(row) for row in rows]
        except Exception as e:
            logger.error(f"Failed to retrieve chat history: {e}", exc_info=True)
            return []
