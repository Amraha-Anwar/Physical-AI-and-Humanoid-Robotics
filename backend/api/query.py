import uuid
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field
from typing import Optional, Dict
from backend.dependencies import get_query_service, get_evaluation_service, get_neon_db
from psycopg2.extensions import connection as PgConnection # Import PgConnection
import logging
from backend.api.models import RAGQueryRequest, RAGQueryResponse
from backend.query.query_service import QueryService

logger = logging.getLogger(__name__)

router = APIRouter()

# --- REMOVED: Old local QueryRequest/QueryResponse classes are removed/replaced ---

@router.post("/query", response_model=RAGQueryResponse) # <-- CHANGED RESPONSE MODEL
async def query_knowledge_base(
    request: RAGQueryRequest, # <-- CHANGED INPUT MODEL
    query_service: QueryService = Depends(get_query_service),
    evaluation_service: get_evaluation_service = Depends(get_evaluation_service),
    neon_conn: PgConnection = Depends(get_neon_db) # Add neon_conn dependency
):
    """
    Endpoint to query the RAG system.
    Accepts a user query, retrieves relevant context, and generates an answer, including optional selected context.
    """
    if not request.query_text.strip():
        raise HTTPException(status_code=400, detail="Query text cannot be empty.")

    # Generate session_id if not provided
    # NOTE: Accessing fields via the new request object
    session_id = request.session_id if request.session_id else str(uuid.uuid4())

    try:
        # --- CORE LOGIC CHANGE: Passing the entire request object ---
        # NOTE: query_service.answer_question_async must be updated next to accept 'request'
        result = await query_service.answer_question_async(request, neon_conn) # <--- PASSING FULL REQUEST OBJECT

        answer = result["answer"]
        context_used = result.get("context", "")
        
        # Evaluate RAG (uses request.query_text)
        try:
            eval_scores = await evaluation_service.evaluate_rag_async(request.query_text, context_used, answer)
        except Exception as e:
            logger.error(f"Evaluation failed (non-blocking): {e}")
            eval_scores = {"relevance": 0.0, "grounding": 0.0}

        # --- CONFORMING TO NEW RAGQueryResponse MODEL ---
        return RAGQueryResponse(
            answer=answer, 
            session_id=session_id, # Use the determined session_id
            evaluation_scores=eval_scores,
        )
    except Exception as e:
        logger.error(f"Error processing query: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="An error occurred while processing your request.")