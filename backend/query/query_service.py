import logging
import google.generativeai as genai
import openai
import os
from typing import List, Dict, Any
from psycopg2.extensions import connection as PgConnection
from qdrant_client import AsyncQdrantClient
from backend.ingestion.embeddings_client import EmbeddingClient
from backend.query.history_service import HistoryService
# --- NEW IMPORT ---
from backend.api.models import RAGQueryRequest # Assuming this is the correct path after Task 1

logger = logging.getLogger(__name__)

class QueryService:
    """
    Service to handle RAG queries: embedding, retrieval, context fetching, and answer generation.
    """
    def __init__(self,
                 embedding_client: EmbeddingClient,
                 qdrant_client: AsyncQdrantClient,
                 history_service: HistoryService): # Removed neon_conn
        self.embedding_client = embedding_client
        self.qdrant_client = qdrant_client
        self.history_service = history_service
        self.qdrant_collection_name = "book_vectors"
        self.openai_client = openai.AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.openai_model = "gpt-3.5-turbo"

    # --- REFACTORED SIGNATURE ---
    async def answer_question_async(self, request: RAGQueryRequest, neon_conn: PgConnection) -> Dict[str, str]:
        """
        End-to-end RAG pipeline:
        1. Retrieve history
        2. Embed query (with history and selected context)
        3. Retrieve chunks from Qdrant
        4. Fetch context from Neon
        5. Generate answer using LLM
        6. Save history
        """
        # --- EXTRACTING FIELDS FROM REQUEST OBJECT ---
        session_id = request.session_id
        query_text = request.query_text
        selected_context = request.selected_context # New field
        
        logger.info(f"Received query: {query_text} for session: {session_id}. Selected Context: {bool(selected_context)}")

        # 0. Retrieve History
        history_records = await self.history_service.get_history_async(neon_conn, session_id)
        history_context = ""
        if history_records:
            # Format: "Conversation History: [Turn 1]; [Turn 2]; ..."
            # Turn: "User: <query> Assistant: <response>"
            turns = [f"User: {rec['user_query']} Assistant: {rec['llm_response']}" for rec in history_records]
            history_context = "Conversation History: " + "; ".join(turns) + ". "

        # 1. Embed the query (with history context AND selected context)
        
        # --- NEW LOGIC: PRIORITIZE SELECTED CONTEXT ---
        if selected_context:
            # If the user selected text, prioritize it by setting it as the main context for retrieval
            retrieval_input = f"CONTEXT FROM PAGE: {selected_context}. QUESTION: {query_text}"
        else:
            # Otherwise, use the standard history-augmented query
            retrieval_input = query_text
        
        # The full prompt for embedding combines history and the retrieval input
        full_query_text = f"{history_context}{retrieval_input}"
        # ----------------------------------------------
        
        query_vector = await self.embedding_client.generate_embedding(full_query_text)
        if not query_vector:
            logger.warning("Embedding generation failed or returned empty. Cannot proceed.")
            return {"answer": "I'm sorry, I couldn't process your query at this time.", "context": ""}

        # 2. Retrieve chunks from Qdrant
        try:
            search_results = await self.qdrant_client.query_points(
                collection_name=self.qdrant_collection_name,
                query=query_vector,
                limit=5
            )
        except Exception as e:
            logger.error(f"Qdrant search failed: {e}", exc_info=True)
            return {"answer": "I encountered an error while searching for information.", "context": ""}
        
        if not search_results:
            logger.info("No relevant chunks found in Qdrant.")
            return {"answer": "I couldn't find any relevant information in the knowledge base to answer your question.", "context": ""}

        relevant_chunk_ids = [point.payload.get('chunk_id') for point in search_results.points if point.payload]
        relevant_chunk_ids = [cid for cid in relevant_chunk_ids if cid]

        if not relevant_chunk_ids:
              logger.warning("Relevant points found but payload missing chunk_id.")
              return {"answer": "I found some information, but it seems corrupted.", "context": ""}

        # 3. Fetch context from Neon
        context_texts = await self._fetch_context_from_neon(relevant_chunk_ids, neon_conn)
        
        if not context_texts:
              logger.warning("Chunk IDs found in Qdrant but no content found in Neon.")
              pass

        # 4. Generate answer
        # Compress context using LLM (uses the original query_text)
        compressed_context = await self._compress_context_async(query_text, context_texts)
        if not compressed_context:
            logger.warning("Context compression failed or returned empty. Using original context.")
            
        final_context = [compressed_context] if compressed_context else context_texts
        context_str_used = "\n\n".join(final_context)

        # The LLM generates the answer using the full query including history (full_query_text)
        answer = await self._generate_answer_with_llm(full_query_text, final_context)
        
        # 5. Save History
        # --- USING NEW EXTRACTED VARIABLES ---
        await self.history_service.save_history_async(neon_conn, session_id, query_text, answer) 
        
        return {"answer": answer, "context": context_str_used}

    async def _compress_context_async(self, query_text: str, retrieved_texts: List[str]) -> str:
        """
        Synthesizes/compresses the list of retrieved texts into a smaller, highly relevant context string based on the query_text.
        """
        if not retrieved_texts:
            return ""

        combined_text = "\n\n".join(retrieved_texts)
        
        messages = [
            {"role": "system", "content": "You are a helpful assistant. Your task is to compress the following context by extracting only the sentences or phrases that are directly relevant to the user's query. Do not add any new information. Keep it concise."},
            {"role": "user", "content": f"Query: {query_text}\n\nContext:\n{combined_text}\n\nCompressed Context:"}
        ]

        try:
            response = await self.openai_client.chat.completions.create(
                model=self.openai_model,
                messages=messages,
                temperature=0.0
            )
            return response.choices[0].message.content
        except Exception as e:
            logger.error(f"Context compression failed: {e}", exc_info=True)
            return ""

    async def _fetch_context_from_neon(self, chunk_ids: List[str]) -> List[str]:
        """
        Retrieves text content for the given chunk IDs from Neon.
        """
        if not chunk_ids:
            return []

        # Prepare query with ANY operator for list matching
        query = """
        SELECT text_snippet_preview 
        FROM rag_metadata 
        WHERE chunk_id = ANY(%s)
        """
        
        fetched_texts = []
        try:
            with self.neon_conn.cursor() as cur:
                cur.execute(query, (chunk_ids,))
                rows = cur.fetchall()
                # row[0] is text_snippet_preview
                fetched_texts = [row[0] for row in rows if row[0]]
        except Exception as e:
            logger.error(f"Failed to fetch context from Neon: {e}", exc_info=True)
            return []
            
        return fetched_texts

    async def _generate_answer_with_llm(self, query: str, context_chunks: List[str]) -> str:
        """
        Generates an answer using the LLM (OpenAI) based on the context.
        """
        if not context_chunks:
            return "I couldn't retrieve enough context to answer your question accurately."

        context_str = "\n\n".join(context_chunks)
        
        messages = [
            {"role": "system", "content": "You are a helpful assistant for a Humanoid Robotics knowledge base. Use the following retrieved context to answer the user's question. If the answer is not in the context, say you don't know."},
            {"role": "user", "content": f"Context:\n{context_str}\n\nUser Question:\n{query}\n\nAnswer:"}
        ]

        try:
            # Using the OpenAI chat completions API
            response = await self.openai_client.chat.completions.create(
                model=self.openai_model,
                messages=messages,
                temperature=0.0 # Keeping temperature low for factual consistency
            )
            return response.choices[0].message.content
        except openai.APIError as e:
            logger.error(f"OpenAI API error: {e}", exc_info=True)
            return f"LLM Generation Failed: [OpenAI API Error] {e}"
        except Exception as e:
            logger.error(f"LLM generation failed: {e}", exc_info=True)
            return f"LLM Generation Failed: [{type(e).__name__}] Error"