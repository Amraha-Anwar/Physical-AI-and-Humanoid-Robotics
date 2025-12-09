from pydantic import BaseModel
from typing import Optional, Dict

# The models defined in the specification (T050)
class RAGQueryRequest(BaseModel):
    query_text: str
    session_id: Optional[str] = None
    selected_context: Optional[str] = None

class RAGQueryResponse(BaseModel):
    answer: str
    session_id: str
    evaluation_scores: Optional[Dict[str, float]] = None