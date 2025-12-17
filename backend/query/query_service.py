import logging
import google.generativeai as genai
import openai
from agents import AsyncOpenAI, OpenAIChatCompletionsModel
import os
from typing import List, Dict, Any
from psycopg2.extensions import connection as PgConnection
from qdrant_client import AsyncQdrantClient
from ingestion.embeddings_client import EmbeddingClient
from query.history_service import HistoryService
from api.models import RAGQueryRequest
from query.agents import run_rag_pipeline
from dotenv import load_dotenv
import asyncio

load_dotenv()

logger = logging.getLogger(__name__)

class QueryService:
    """
    Service to handle RAG queries: embedding, retrieval, context fetching, and answer generation.
    """
    def __init__(self,
                 embedding_client: EmbeddingClient,
                 qdrant_client: AsyncQdrantClient,
                 history_service: HistoryService):
        self.embedding_client = embedding_client
        self.qdrant_client = qdrant_client
        self.history_service = history_service
        self.qdrant_collection_name = "book_vectors"
        
        # Configure OpenAI Client to use Gemini
        gemini_api_key =os.getenv("GEMINI_API_KEY")
        gemini_base_url =os.getenv("BASE_URL")


        client = AsyncOpenAI(
            api_key=gemini_api_key,
            base_url=gemini_base_url,
        )

        model = OpenAIChatCompletionsModel(
            model="gemini-2.5-flash",
            openai_client=client,
        )

    async def answer_question_async(self, request: RAGQueryRequest, neon_conn: PgConnection) -> Dict[str, str]:
        """
        End-to-end RAG pipeline using Agent Orchestration.
        """
        session_id = request.session_id
        query_text = request.query_text
        selected_context = request.selected_context
        
        logger.info(f"Received query: {query_text} for session: {session_id}. Selected Context: {bool(selected_context)}")

        # Retrieve history
        history_records = await self.history_service.get_history_async(neon_conn, session_id)
        history_context = ""
        if history_records:
            turns = [f"User: {rec['user_query']} Assistant: {rec['llm_response']}" for rec in history_records]
            history_context = "Conversation History: " + "; ".join(turns) + ". "

        # Prepare full query for agent
        if selected_context:
            retrieval_input = f"CONTEXT FROM PAGE: {selected_context}. QUESTION: {query_text}"
        else:
            retrieval_input = query_text
        
        full_query_text = f"{history_context}{retrieval_input}"

        # Run RAG agent pipeline
        retrieved_chunks: List[str] = []  # Store retrieved context for debugging

        async def retrieval_fn_async(q: str) -> List[str]:
            nonlocal retrieved_chunks
            chunks = await self._execute_retrieval(q, neon_conn)
            retrieved_chunks = chunks # Capture for logging/history
            return chunks

        answer = await run_rag_pipeline(full_query_text, retrieval_fn_async)

        logger.info(f"Retrieved Chunks: {retrieved_chunks}")  # Debug: check what context is retrieved

        # Save history
        await self.history_service.save_history_async(neon_conn, session_id, query_text, answer)

        return {"answer": answer, "context": "\n\n".join(retrieved_chunks) or "Context managed by Agent Orchestrator."}

    async def _execute_retrieval(self, query_text_for_embedding: str, neon_conn: PgConnection) -> List[str]:
        """
        Executes the retrieval pipeline: Embedding -> Qdrant -> Neon.
        """
        # Use input_type="search_query" for retrieval
        query_vector = await self.embedding_client.generate_embedding(query_text_for_embedding, input_type="search_query")
        if not query_vector:
            logger.warning("Embedding generation failed or returned empty. Cannot proceed.")
            return []
        
        # Log vector check (T106.1)
        logger.info(f"Query Vector (first 10 elements): {query_vector[:10]}")

        try:
                    search_results = await self.qdrant_client.query_points(
                        collection_name=self.qdrant_collection_name, # Confirmed 'book_vectors' in __init__
                        query=query_vector,
                        limit=10,
                        score_threshold=0.0 # T102.1: Allow all matches for verification
                    )
        except Exception as e:
                logger.error(f"Qdrant search failed: {e}", exc_info=True)
                return []

        if not search_results.points:
            logger.info("No relevant chunks found in Qdrant.")
            return []

        # T102.1: Extract text from 'text' payload key (preferred) or fetch from Neon if missing
        context_texts = []
        chunk_ids_to_fetch = []

        for point in search_results.points:
            if point.payload and 'text' in point.payload:
                context_texts.append(point.payload['text'])
            elif point.payload and 'chunk_id' in point.payload:
                chunk_ids_to_fetch.append(point.payload['chunk_id'])
        
        if chunk_ids_to_fetch:
            fetched_texts = await self._fetch_context_from_neon(chunk_ids_to_fetch, neon_conn)
            context_texts.extend(fetched_texts)

        return context_texts

    async def _fetch_context_from_neon(self, chunk_ids: List[str], neon_conn: PgConnection) -> List[str]:
        """
        Retrieves text content for the given chunk IDs from Neon.
        """
        if not chunk_ids:
            return []

        query = """
        SELECT text_snippet_preview 
        FROM rag_metadata 
        WHERE chunk_id = ANY(%s)
        """
        
        fetched_texts = []
        try:
            with neon_conn.cursor() as cur:
                cur.execute(query, (chunk_ids,))
                rows = cur.fetchall()
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
            response = await self.openai_client.chat.completions.create(
                model=self.openai_model,
                messages=messages,
                temperature=0.0
            )
            return response.choices[0].message.content
        except Exception as e:
            logger.error(f"LLM generation failed: {e}", exc_info=True)
            return f"LLM Generation Failed: [{type(e).__name__}] Error"
