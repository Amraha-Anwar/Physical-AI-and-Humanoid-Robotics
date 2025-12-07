import logging
import os
from typing import Dict, Any, List
import openai
from psycopg2.extensions import connection as PgConnection

logger = logging.getLogger(__name__)

class EvaluationService:
    """
    Service to score the quality of the RAG system's retrieval (relevance) and generation (faithfulness) processes.
    """
    def __init__(self, openai_client: openai.AsyncOpenAI): # Removed neon_conn
        self.openai_client = openai_client
        self.openai_model = "gpt-3.5-turbo" # Using a consistent model for evaluation

    async def evaluate_rag_async(self, question: str, context: str, answer: str) -> Dict[str, float]:
        """
        Evaluates the RAG pipeline by calculating context relevance and answer faithfulness scores.
        """
        relevance_score = await self._calculate_context_relevance(question, context)
        faithfulness_score = await self._calculate_answer_faithfulness(context, answer)

        return {
            "context_relevance_score": relevance_score,
            "answer_faithfulness_score": faithfulness_score
        }

    async def _calculate_context_relevance(self, question: str, context: str) -> float:
        """
        Scores how well the provided context relates to the question (0-10 scale).
        """
        messages = [
            {"role": "system", "content": "You are an expert evaluator. Score the relevance of the following context to the user's question on a scale from 0 to 10, where 0 is irrelevant and 10 is highly relevant. Return ONLY the number."},
            {"role": "user", "content": f"Question: {question}\n\nContext:\n{context}\n\nRelevance Score:"}
        ]
        return await self._get_score_from_llm(messages)

    async def _calculate_answer_faithfulness(self, context: str, answer: str) -> float:
        """
        Scores how well the final answer is supported ONLY by the provided context (0-10 scale).
        """
        messages = [
            {"role": "system", "content": "You are an expert evaluator. Score the faithfulness of the answer based ONLY on the provided context on a scale from 0 to 10, where 0 is hallucinated/unsupported and 10 is fully supported by the context. Return ONLY the number."},
            {"role": "user", "content": f"Context:\n{context}\n\nAnswer:\n{answer}\n\nFaithfulness Score:"}
        ]
        return await self._get_score_from_llm(messages)

    async def _get_score_from_llm(self, messages: List[Dict[str, str]]) -> float:
        """
        Helper method to call LLM and parse the score.
        """
        try:
            response = await self.openai_client.chat.completions.create(
                model=self.openai_model,
                messages=messages,
                temperature=0.0
            )
            content = response.choices[0].message.content.strip()
            # Attempt to extract a number from the response
            try:
                score = float(content)
                return max(0.0, min(10.0, score)) # Clamp between 0 and 10
            except ValueError:
                logger.warning(f"Could not parse score from LLM response: {content}")
                return 0.0 # Default fallback
        except Exception as e:
            logger.error(f"Evaluation failed: {e}", exc_info=True)
            return 0.0
