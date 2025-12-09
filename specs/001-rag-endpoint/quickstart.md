# P2.3 Retrieval & Agent Logic: Core RAG FastAPI Endpoint - Quickstart Guide

This guide provides a rapid setup and testing process for the core RAG FastAPI endpoint.

## Prerequisites

*   Python 3.9+
*   Poetry (for dependency management)
*   **OpenAI API Key**: Set as an environment variable (`OPENAI_API_KEY`).
*   **P1.3 Database Setup**: Ensure Neon Postgres and Qdrant Cloud are provisioned and accessible, and their connection details are configured.
*   **P2.2 Data Ingestion**: Ensure the Qdrant `book_vectors` collection and Neon `rag_metadata` table are populated with data from the P2.2 indexing script.

## 1. Setup Environment Variables

Ensure your `.env` file (in `backend/`) contains the following variables:

```
OPENAI_API_KEY="your_openai_api_key"
NEON_POSTGRES_CONNECTION_STRING="your_neon_postgres_connection_string"
QDRANT_HOST="your_qdrant_cloud_host"
QDRANT_API_KEY="your_qdrant_cloud_api_key"
```

## 2. Install Dependencies

Navigate to the `backend/` directory and install the required Python packages:

```bash
cd backend/
poetry install # Ensure openai, qdrant-client, psycopg2-binary, etc. are installed
```

## 3. Run the FastAPI Application

Start the FastAPI application. Ensure your environment variables are loaded (e.g., by running `poetry run uvicorn main:app --reload`).

```bash
poetry run uvicorn main:app --reload
```

The application will typically be available at `http://127.0.0.1:8000`.

## 4. Test the Core RAG Endpoint

Use `curl` or a tool like Postman/Insomnia to send requests to the `/api/rag/query` endpoint.

### 4.1 Test General RAG Query

Send a `POST` request with a user query.

```bash
curl -X POST "http://127.00.1:8000/api/rag/query" \
     -H "Content-Type: application/json" \
     -d '{
           "query": "What are the main components of ROS 2 architecture?",
           "context_snippet": null
         }'
```

**Expected Output**: A JSON response with a contextual `answer` based on the book's content.

### 4.2 Test Highlight-to-Query Feature

Send a `POST` request with a user query and a specific `context_snippet`.

```bash
curl -X POST "http://127.0.0.1:8000/api/rag/query" \
     -H "Content-Type: application/json" \
     -d '{
           "query": "Explain how TF (Transformations) works.",
           "context_snippet": "TF provides a standardized way to keep track of multiple coordinate frames and transform data between them. It is crucial for robotics applications to manage sensor data, robot kinematics, and navigation."
         }'
```

**Expected Output**: A JSON response with an `answer` that is strictly based on the provided `context_snippet`.

## 5. (Optional) Performance and Quality Checks

*   **Latency**: Monitor response times to ensure they meet the SC-001 (under 2 seconds for p95 latency).
*   **Relevance**: For general RAG queries, manually evaluate the answer's relevance and grounding in the book's content.
*   **Context Prioritization**: For highlight queries, ensure the LLM strictly adheres to the snippet.

This quickstart will help confirm the basic functionality and quality of the RAG endpoint.
